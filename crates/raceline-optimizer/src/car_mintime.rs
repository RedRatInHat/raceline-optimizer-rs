use crate::contracts::{
    Point2, SectionsTrackViewV1, TrackAreaContractV1, TrajectoryResultSeriesV1,
};
use crate::dense_frenet::{
    build_dense_section_frame_sample_from_geometry, DenseSectionFrameHermiteSampler,
    DenseSectionFrameInput, DenseSectionFrameSample,
};
use crate::json::JsonValue;
use crate::mintime::{
    mintime_result_to_json, solve_result_visualization_json, MintimeBackend, MintimeGeometryInput,
    MintimeNlpDimensions, MintimeNlpLayout, MintimeProgressCallback, MintimeProgressEvent,
    MintimeSolveRequestV1, MintimeSolveResult,
};
use crate::section_frame::{
    pure_frenet_path_factor, section_frame_progress_from_derivatives, signed_max_abs,
    velocity_heading_curvature_1pm,
};
use crate::section_geometry::{SectionFrameGeometry, SectionFrameMapView};
use crate::solver_api::{SolverApiError, SolverCancelToken};
use crate::station_generation::{
    generate_station_geometry, parse_station_options, validate_station_topology,
    StationGenerationRequestV1,
};
use crate::vehicle_dynamics::{
    car_double_track_dynamics, directional_longitudinal_tire_capacity_n, CarDoubleTrackControl,
    CarDoubleTrackDynamics, CarDoubleTrackParams, CarDoubleTrackState, CarDoubleTrackTireForces,
    TireLoadSensitivityMode, VehicleDynamicsModelFamily,
};
use std::collections::BTreeSet;
use std::ffi::c_void;
use std::ptr;

pub const OLD_CAR_MINTIME_SOLVER_ID: &str = "old_car_mintime";
const MIN_CAR_MINTIME_STATION_COUNT: usize = 20;
const CAR_STATE_LEN: usize = 5;
const CAR_CONTROL_LEN: usize = 4;
const CAR_COLLOCATION_DEGREE: usize = 3;
const CAR_DENSE_FRENET_SAMPLES_PER_INTERVAL: usize = 10;
const CAR_STATE_SCALE: [f64; CAR_STATE_LEN] = [50.0, 0.5, 1.0, 5.0, 1.0];
const CAR_CONTROL_SCALE: [f64; CAR_CONTROL_LEN] = [0.5, 7500.0, 20000.0, 5000.0];
const STATE_V_MPS: usize = 0;
const STATE_BETA_RAD: usize = 1;
const STATE_OMEGA_Z_RADPS: usize = 2;
const STATE_N_M: usize = 3;
const STATE_XI_RAD: usize = 4;
const CONTROL_DELTA_RAD: usize = 0;
const CONTROL_F_DRIVE_N: usize = 1;
const CONTROL_F_BRAKE_N: usize = 2;
const CONTROL_GAMMA_Y_N: usize = 3;
const CAR_DRIVE_BRAKE_MUTEX_LOWER_N2: f64 = -20_000.0;
const CAR_MINTIME_DEFAULT_MAX_ITER: i32 = 5000;
const CAR_MINTIME_DEFAULT_TOL: f64 = 1e-4;
const CAR_MINTIME_DEFAULT_ACCEPTABLE_TOL: f64 = 1e-4;
const CAR_MINTIME_DEFAULT_ACCEPTABLE_ITER: i32 = 5;
const CAR_MINTIME_PREVIEW_EVAL_PERIOD: u32 = 25;
const CAR_MINTIME_DEFAULT_WIDTH_OPT_M: f64 = 2.25;
const CAR_MINTIME_KART_WIDTH_OPT_M: f64 = 1.45;
const CAR_MINTIME_DEFAULT_PENALTY_DELTA: f64 = 10.0;
const CAR_MINTIME_DEFAULT_PENALTY_F: f64 = 0.01;
const CAR_MINTIME_DEFAULT_PENALTY_DELTA_DD: f64 = 0.03;
const CAR_MINTIME_DEFAULT_PENALTY_F_DD: f64 = 0.01;
const CAR_MINTIME_DEFAULT_PENALTY_N_DD: f64 = 0.01;
const CAR_MINTIME_DEFAULT_PENALTY_XI_DD: f64 = 0.05;
const CAR_MINTIME_DEFAULT_PENALTY_ENDPOINT_C1_DN: f64 = 0.0;
const CAR_MINTIME_DEFAULT_ENDPOINT_C1_DN_SCALE: f64 = 1.0;
const CAR_MINTIME_DEFAULT_PENALTY_ENDPOINT_C1_HEADING: f64 = 0.0;
const CAR_MINTIME_DEFAULT_ENDPOINT_C1_HEADING_SCALE_RAD: f64 = 1.0;
const CAR_MINTIME_DEFAULT_PENALTY_ENDPOINT_HEADING_JUMP: f64 = 0.0;
const CAR_MINTIME_DEFAULT_ENDPOINT_HEADING_JUMP_SCALE_RAD: f64 = 1.0;
const CAR_MINTIME_DEFAULT_PENALTY_ENDPOINT_D2N_JUMP: f64 = 0.0;
const CAR_MINTIME_DEFAULT_ENDPOINT_D2N_JUMP_SCALE: f64 = 1.0;
const CAR_MINTIME_DEFAULT_PREPEAK_GRIP_MARGIN: f64 = 0.98;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CarMintimeFormulationMode {
    PrepeakGripV1,
    LegacyFullPacejka,
}

impl CarMintimeFormulationMode {
    fn parse(raw: &str) -> Result<Self, String> {
        match raw.trim().to_ascii_lowercase().as_str() {
            "" | "default" | "prepeak_grip_v1" | "prepeak-grip-v1" => {
                Ok(Self::PrepeakGripV1)
            }
            "legacy" | "legacy_full_pacejka" | "full_pacejka" => {
                Ok(Self::LegacyFullPacejka)
            }
            other => Err(format!(
                "unsupported car_mintime_formulation_mode: {other}; expected prepeak_grip_v1 or legacy_full_pacejka"
            )),
        }
    }

    fn as_str(self) -> &'static str {
        match self {
            Self::PrepeakGripV1 => "prepeak_grip_v1",
            Self::LegacyFullPacejka => "legacy_full_pacejka",
        }
    }

    fn uses_prepeak_grip_domain(self) -> bool {
        matches!(self, Self::PrepeakGripV1)
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct CarMintimeSolveOptions {
    pub strict_collocation_tire_envelope: bool,
    pub strict_collocation_normal_load: bool,
    pub strict_collocation_kamm: bool,
    pub strict_collocation_power: bool,
    pub formulation_mode: CarMintimeFormulationMode,
    pub prepeak_grip_margin: f64,
    pub max_iter: i32,
    pub tol: f64,
    pub acceptable_tol: f64,
    pub acceptable_iter: i32,
    pub ipopt_print_level: i32,
    pub penalty_delta: f64,
    pub penalty_f: f64,
    pub penalty_delta_dd: f64,
    pub penalty_f_dd: f64,
    pub penalty_n_dd: f64,
    pub penalty_xi_dd: f64,
    pub penalty_endpoint_c1_dn: f64,
    pub endpoint_c1_dn_scale: f64,
    pub penalty_endpoint_c1_heading: f64,
    pub endpoint_c1_heading_scale_rad: f64,
    pub penalty_endpoint_heading_jump: f64,
    pub endpoint_heading_jump_scale_rad: f64,
    pub penalty_endpoint_d2n_jump: f64,
    pub endpoint_d2n_jump_scale: f64,
    pub ipopt_dll_path: Option<std::path::PathBuf>,
}

impl Default for CarMintimeSolveOptions {
    fn default() -> Self {
        Self {
            strict_collocation_tire_envelope: true,
            strict_collocation_normal_load: true,
            strict_collocation_kamm: true,
            strict_collocation_power: true,
            formulation_mode: CarMintimeFormulationMode::PrepeakGripV1,
            prepeak_grip_margin: CAR_MINTIME_DEFAULT_PREPEAK_GRIP_MARGIN,
            max_iter: CAR_MINTIME_DEFAULT_MAX_ITER,
            tol: CAR_MINTIME_DEFAULT_TOL,
            acceptable_tol: CAR_MINTIME_DEFAULT_ACCEPTABLE_TOL,
            acceptable_iter: CAR_MINTIME_DEFAULT_ACCEPTABLE_ITER,
            ipopt_print_level: 0,
            penalty_delta: CAR_MINTIME_DEFAULT_PENALTY_DELTA,
            penalty_f: CAR_MINTIME_DEFAULT_PENALTY_F,
            penalty_delta_dd: CAR_MINTIME_DEFAULT_PENALTY_DELTA_DD,
            penalty_f_dd: CAR_MINTIME_DEFAULT_PENALTY_F_DD,
            penalty_n_dd: CAR_MINTIME_DEFAULT_PENALTY_N_DD,
            penalty_xi_dd: CAR_MINTIME_DEFAULT_PENALTY_XI_DD,
            penalty_endpoint_c1_dn: CAR_MINTIME_DEFAULT_PENALTY_ENDPOINT_C1_DN,
            endpoint_c1_dn_scale: CAR_MINTIME_DEFAULT_ENDPOINT_C1_DN_SCALE,
            penalty_endpoint_c1_heading: CAR_MINTIME_DEFAULT_PENALTY_ENDPOINT_C1_HEADING,
            endpoint_c1_heading_scale_rad: CAR_MINTIME_DEFAULT_ENDPOINT_C1_HEADING_SCALE_RAD,
            penalty_endpoint_heading_jump: CAR_MINTIME_DEFAULT_PENALTY_ENDPOINT_HEADING_JUMP,
            endpoint_heading_jump_scale_rad: CAR_MINTIME_DEFAULT_ENDPOINT_HEADING_JUMP_SCALE_RAD,
            penalty_endpoint_d2n_jump: CAR_MINTIME_DEFAULT_PENALTY_ENDPOINT_D2N_JUMP,
            endpoint_d2n_jump_scale: CAR_MINTIME_DEFAULT_ENDPOINT_D2N_JUMP_SCALE,
            ipopt_dll_path: None,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct CarMintimeNlpSeed {
    pub layout: MintimeNlpLayout,
    pub dimensions: MintimeNlpDimensions,
    pub model_track_area: TrackAreaContractV1,
    pub station_s_m: Vec<f64>,
    pub centerline_xy_m: Vec<Point2>,
    pub kappa_1pm: Vec<f64>,
    pub ref_tangent_xy: Vec<Point2>,
    pub ref_left_normal_xy: Vec<Point2>,
    pub section_dir_xy: Vec<Point2>,
    pub section_dir_derivative_xy: Vec<Point2>,
    pub width_left_m: Vec<f64>,
    pub width_right_m: Vec<f64>,
    pub initial_guess: Vec<f64>,
    pub lower_bounds: Vec<f64>,
    pub upper_bounds: Vec<f64>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct CarMintimeNlpProblem {
    pub seed: CarMintimeNlpSeed,
    pub params: CarDoubleTrackParams,
    pub options: CarMintimeSolveOptions,
    pub objective_weights: CarMintimeObjectiveWeights,
    pub constraints: Vec<CarMintimeConstraintRow>,
    pub constraint_lower_bounds: Vec<f64>,
    pub constraint_upper_bounds: Vec<f64>,
    pub jacobian_pattern: Vec<(i32, i32)>,
    jacobian_columns: Vec<CarMintimeJacobianColumnEntries>,
    pub initial_diagnostics: CarMintimeNlpDiagnostics,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CarMintimeObjectiveWeights {
    pub penalty_delta: f64,
    pub penalty_f: f64,
    pub penalty_delta_dd: f64,
    pub penalty_f_dd: f64,
    pub penalty_n_dd: f64,
    pub penalty_xi_dd: f64,
    pub penalty_endpoint_c1_dn: f64,
    pub endpoint_c1_dn_scale: f64,
    pub penalty_endpoint_c1_heading: f64,
    pub endpoint_c1_heading_scale_rad: f64,
    pub penalty_endpoint_heading_jump: f64,
    pub endpoint_heading_jump_scale_rad: f64,
    pub penalty_endpoint_d2n_jump: f64,
    pub endpoint_d2n_jump_scale: f64,
}

impl Default for CarMintimeObjectiveWeights {
    fn default() -> Self {
        Self {
            penalty_delta: CAR_MINTIME_DEFAULT_PENALTY_DELTA,
            penalty_f: CAR_MINTIME_DEFAULT_PENALTY_F,
            penalty_delta_dd: CAR_MINTIME_DEFAULT_PENALTY_DELTA_DD,
            penalty_f_dd: CAR_MINTIME_DEFAULT_PENALTY_F_DD,
            penalty_n_dd: CAR_MINTIME_DEFAULT_PENALTY_N_DD,
            penalty_xi_dd: CAR_MINTIME_DEFAULT_PENALTY_XI_DD,
            penalty_endpoint_c1_dn: CAR_MINTIME_DEFAULT_PENALTY_ENDPOINT_C1_DN,
            endpoint_c1_dn_scale: CAR_MINTIME_DEFAULT_ENDPOINT_C1_DN_SCALE,
            penalty_endpoint_c1_heading: CAR_MINTIME_DEFAULT_PENALTY_ENDPOINT_C1_HEADING,
            endpoint_c1_heading_scale_rad: CAR_MINTIME_DEFAULT_ENDPOINT_C1_HEADING_SCALE_RAD,
            penalty_endpoint_heading_jump: CAR_MINTIME_DEFAULT_PENALTY_ENDPOINT_HEADING_JUMP,
            endpoint_heading_jump_scale_rad: CAR_MINTIME_DEFAULT_ENDPOINT_HEADING_JUMP_SCALE_RAD,
            penalty_endpoint_d2n_jump: CAR_MINTIME_DEFAULT_PENALTY_ENDPOINT_D2N_JUMP,
            endpoint_d2n_jump_scale: CAR_MINTIME_DEFAULT_ENDPOINT_D2N_JUMP_SCALE,
        }
    }
}

impl CarMintimeObjectiveWeights {
    #[must_use]
    pub fn from_options(options: &CarMintimeSolveOptions) -> Self {
        Self {
            penalty_delta: options.penalty_delta.max(0.0),
            penalty_f: options.penalty_f.max(0.0),
            penalty_delta_dd: options.penalty_delta_dd.max(0.0),
            penalty_f_dd: options.penalty_f_dd.max(0.0),
            penalty_n_dd: options.penalty_n_dd.max(0.0),
            penalty_xi_dd: options.penalty_xi_dd.max(0.0),
            penalty_endpoint_c1_dn: options.penalty_endpoint_c1_dn.max(0.0),
            endpoint_c1_dn_scale: options.endpoint_c1_dn_scale.max(1e-9),
            penalty_endpoint_c1_heading: options.penalty_endpoint_c1_heading.max(0.0),
            endpoint_c1_heading_scale_rad: options.endpoint_c1_heading_scale_rad.max(1e-9),
            penalty_endpoint_heading_jump: options.penalty_endpoint_heading_jump.max(0.0),
            endpoint_heading_jump_scale_rad: options.endpoint_heading_jump_scale_rad.max(1e-9),
            penalty_endpoint_d2n_jump: options.penalty_endpoint_d2n_jump.max(0.0),
            endpoint_d2n_jump_scale: options.endpoint_d2n_jump_scale.max(1e-9),
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
struct CarMintimeJacobianColumnEntries {
    variable_index: usize,
    entries: Vec<CarMintimeJacobianEntry>,
}

#[derive(Clone, Debug, PartialEq)]
struct CarMintimeJacobianEntry {
    pattern_index: usize,
    row_index: usize,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct CarMintimeStructuredDerivative {
    analytic: f64,
    numeric_term: CarMintimeNumericDerivativeTerm,
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum CarMintimeNumericDerivativeTerm {
    None,
    FullConstraint,
    CollocationDynamicsRhs {
        interval: usize,
        point: usize,
        state_index: usize,
    },
    ControlRateSigma {
        interval: usize,
        numerator: f64,
        ds: f64,
    },
    LateralLoadTransferVehiclePart {
        interval: usize,
    },
}

#[derive(Clone, Debug, PartialEq)]
pub struct CarMintimeNlpDiagnostics {
    pub objective_initial_s: f64,
    pub constraint_count: usize,
    pub max_initial_abs_residual: f64,
    pub worst_initial_constraint_label: String,
}

#[derive(Clone, Debug, PartialEq)]
pub enum CarMintimeConstraintRow {
    CollocationDynamics {
        interval: usize,
        point: usize,
        state_name: &'static str,
    },
    Continuity {
        interval: usize,
        state_name: &'static str,
    },
    Dynamics {
        interval: usize,
        state_name: &'static str,
    },
    PowerLimit {
        interval: usize,
    },
    CollocationPowerLimit {
        interval: usize,
        point: usize,
    },
    NormalLoad {
        interval: usize,
        wheel: &'static str,
    },
    CollocationNormalLoad {
        interval: usize,
        point: usize,
        wheel: &'static str,
    },
    TireEllipse {
        interval: usize,
        wheel: &'static str,
    },
    CollocationTireEllipse {
        interval: usize,
        point: usize,
        wheel: &'static str,
    },
    SlipPrepeak {
        interval: usize,
        wheel: &'static str,
    },
    CollocationSlipPrepeak {
        interval: usize,
        point: usize,
        wheel: &'static str,
    },
    LateralLoadTransfer {
        interval: usize,
    },
    DriveBrakeMutex {
        interval: usize,
    },
    ControlRate {
        interval: usize,
        control_name: &'static str,
    },
}

impl CarMintimeConstraintRow {
    #[must_use]
    pub fn label(&self) -> String {
        match self {
            Self::CollocationDynamics {
                interval,
                point,
                state_name,
            } => format!("colloc_{state_name}_{interval}_{point}"),
            Self::Continuity {
                interval,
                state_name,
            } => format!("continuity_{state_name}_{interval}"),
            Self::Dynamics {
                interval,
                state_name,
            } => format!("dyn_{state_name}_{interval}"),
            Self::PowerLimit { interval } => format!("power_limit_{interval}"),
            Self::CollocationPowerLimit { interval, point } => {
                format!("colloc_power_limit_{interval}_{point}")
            }
            Self::NormalLoad { interval, wheel } => format!("normal_load_{wheel}_{interval}"),
            Self::CollocationNormalLoad {
                interval,
                point,
                wheel,
            } => format!("colloc_normal_load_{wheel}_{interval}_{point}"),
            Self::TireEllipse { interval, wheel } => format!("tire_{wheel}_{interval}"),
            Self::CollocationTireEllipse {
                interval,
                point,
                wheel,
            } => format!("colloc_tire_{wheel}_{interval}_{point}"),
            Self::SlipPrepeak { interval, wheel } => {
                format!("slip_prepeak_{wheel}_{interval}")
            }
            Self::CollocationSlipPrepeak {
                interval,
                point,
                wheel,
            } => format!("colloc_slip_prepeak_{wheel}_{interval}_{point}"),
            Self::LateralLoadTransfer { interval } => format!("load_transfer_{interval}"),
            Self::DriveBrakeMutex { interval } => format!("drive_brake_mutex_{interval}"),
            Self::ControlRate {
                interval,
                control_name,
            } => format!("control_rate_{control_name}_{interval}"),
        }
    }

    #[must_use]
    pub fn family(&self) -> &'static str {
        match self {
            Self::CollocationDynamics { .. } => "collocation",
            Self::Continuity { .. } => "continuity",
            Self::Dynamics { .. } => "dynamics",
            Self::PowerLimit { .. } | Self::CollocationPowerLimit { .. } => "power",
            Self::NormalLoad { .. } | Self::CollocationNormalLoad { .. } => "normal_load",
            Self::TireEllipse { .. } | Self::CollocationTireEllipse { .. } => "tire",
            Self::SlipPrepeak { .. } | Self::CollocationSlipPrepeak { .. } => "slip_prepeak",
            Self::LateralLoadTransfer { .. } => "load_transfer",
            Self::DriveBrakeMutex { .. } => "drive_brake_mutex",
            Self::ControlRate { .. } => "control_rate",
        }
    }
}

impl CarMintimeNlpProblem {
    #[must_use]
    pub fn decision_variable_count(&self) -> usize {
        self.seed.dimensions.decision_variable_count()
    }

    #[must_use]
    pub fn constraint_count(&self) -> usize {
        self.constraints.len()
    }

    #[must_use]
    pub fn objective(&self, x: &[f64]) -> f64 {
        car_mintime_collocation_objective_s(&self.seed, self.params, x)
            + car_mintime_regularization_objective_s(
                &self.seed,
                self.params,
                self.objective_weights,
                x,
            )
    }

    #[must_use]
    pub fn constraint_values(&self, x: &[f64]) -> Vec<f64> {
        self.constraints
            .iter()
            .map(|row| self.constraint_value(row, x))
            .collect()
    }

    #[must_use]
    pub fn constraint_value(&self, row: &CarMintimeConstraintRow, x: &[f64]) -> f64 {
        car_mintime_constraint_value_from(&self.seed, self.params, x, row)
    }

    #[must_use]
    pub fn jacobian_values_numeric(&self, x: &[f64]) -> Vec<f64> {
        let mut values = vec![0.0; self.jacobian_pattern.len()];
        let mut plus = x.to_vec();
        let mut minus = x.to_vec();

        for column in &self.jacobian_columns {
            let variable_index = column.variable_index;
            let h = 1e-6 * x[variable_index].abs().max(1.0);
            plus[variable_index] = x[variable_index] + h;
            minus[variable_index] = x[variable_index] - h;

            for entry in &column.entries {
                values[entry.pattern_index] = (self
                    .constraint_value(&self.constraints[entry.row_index], &plus)
                    - self.constraint_value(&self.constraints[entry.row_index], &minus))
                    / (2.0 * h);
            }

            plus[variable_index] = x[variable_index];
            minus[variable_index] = x[variable_index];
        }

        values
    }

    #[must_use]
    pub fn jacobian_values_structured_numeric(&self, x: &[f64]) -> Vec<f64> {
        let mut values = vec![0.0; self.jacobian_pattern.len()];
        let mut plus = x.to_vec();
        let mut minus = x.to_vec();

        for column in &self.jacobian_columns {
            let variable_index = column.variable_index;
            let mut needs_numeric = false;

            for entry in &column.entries {
                let derivative = car_mintime_constraint_derivative_structured(
                    &self.seed,
                    self.params,
                    &self.constraints[entry.row_index],
                    x,
                    variable_index,
                );
                values[entry.pattern_index] = derivative.analytic;
                if derivative.numeric_term != CarMintimeNumericDerivativeTerm::None {
                    needs_numeric = true;
                }
            }

            if !needs_numeric {
                continue;
            }

            let h = 1e-6 * x[variable_index].abs().max(1.0);
            plus[variable_index] = x[variable_index] + h;
            minus[variable_index] = x[variable_index] - h;

            for entry in &column.entries {
                let derivative = car_mintime_constraint_derivative_structured(
                    &self.seed,
                    self.params,
                    &self.constraints[entry.row_index],
                    x,
                    variable_index,
                );
                if derivative.numeric_term == CarMintimeNumericDerivativeTerm::None {
                    continue;
                }

                values[entry.pattern_index] += (car_mintime_numeric_derivative_term_value(
                    &self.seed,
                    self.params,
                    &plus,
                    &self.constraints[entry.row_index],
                    derivative.numeric_term,
                ) - car_mintime_numeric_derivative_term_value(
                    &self.seed,
                    self.params,
                    &minus,
                    &self.constraints[entry.row_index],
                    derivative.numeric_term,
                )) / (2.0 * h);
            }

            plus[variable_index] = x[variable_index];
            minus[variable_index] = x[variable_index];
        }

        values
    }

    pub fn objective_gradient_numeric(&self, x: &[f64], grad: &mut [f64]) {
        let mut plus = x.to_vec();
        let mut minus = x.to_vec();

        for index in 0..x.len() {
            let h = 1e-6 * x[index].abs().max(1.0);
            plus[index] = x[index] + h;
            minus[index] = x[index] - h;
            grad[index] = (self.objective(&plus) - self.objective(&minus)) / (2.0 * h);
            plus[index] = x[index];
            minus[index] = x[index];
        }
    }

    pub fn objective_gradient_structured_numeric(&self, x: &[f64], grad: &mut [f64]) {
        grad.fill(0.0);
        let mut plus = x.to_vec();
        let mut minus = x.to_vec();

        for interval in 0..self.seed.dimensions.interval_count {
            for point in 0..CAR_COLLOCATION_DEGREE {
                let offset = collocation_state_offset(&self.seed, interval, point);
                for state_index in 0..CAR_STATE_LEN {
                    let variable_index = offset + state_index;
                    let h = 1e-6 * x[variable_index].abs().max(1.0);
                    plus[variable_index] = x[variable_index] + h;
                    minus[variable_index] = x[variable_index] - h;
                    grad[variable_index] = (car_mintime_collocation_objective_term_s(
                        &self.seed,
                        self.params,
                        &plus,
                        interval,
                        point + 1,
                    ) - car_mintime_collocation_objective_term_s(
                        &self.seed,
                        self.params,
                        &minus,
                        interval,
                        point + 1,
                    )) / (2.0 * h);
                    plus[variable_index] = x[variable_index];
                    minus[variable_index] = x[variable_index];
                }
            }
        }

        add_car_mintime_regularization_gradient(
            &self.seed,
            self.params,
            self.objective_weights,
            x,
            grad,
        );
    }

    #[must_use]
    pub fn to_series(&self, x: &[f64]) -> TrajectoryResultSeriesV1 {
        let count = self.seed.dimensions.station_count;
        let mut s_m = Vec::with_capacity(count);
        let mut x_m = Vec::with_capacity(count);
        let mut y_m = Vec::with_capacity(count);
        let mut heading_rad = Vec::with_capacity(count);
        let mut kappa = Vec::with_capacity(count);
        let mut v_mps = Vec::with_capacity(count);
        let mut ax_mps2 = Vec::with_capacity(count);
        let mut ay_mps2 = Vec::with_capacity(count);
        let mut utilization_cornering = Vec::with_capacity(count);
        let mut utilization_longitudinal = Vec::with_capacity(count);
        let mut utilization_combined = Vec::with_capacity(count);

        let points = (0..count)
            .map(|station| station_xy_from(&self.seed, x, station))
            .collect::<Vec<_>>();

        for station in 0..count {
            let interval = station.min(self.seed.dimensions.interval_count.saturating_sub(1));
            let state = car_state_from(&self.seed, x, station);
            let dynamics = car_mintime_dynamics_from(&self.seed, self.params, x, interval);
            let (ax_velocity_mps2, ay_velocity_mps2) =
                velocity_frame_acceleration(dynamics.ax_mps2, dynamics.ay_mps2, state.beta_rad);
            let tire_forces = dynamics.tire_forces;
            let combined = [
                tire_forces
                    .wheel_ellipse_utilization(self.params, "fl")
                    .sqrt(),
                tire_forces
                    .wheel_ellipse_utilization(self.params, "fr")
                    .sqrt(),
                tire_forces
                    .wheel_ellipse_utilization(self.params, "rl")
                    .sqrt(),
                tire_forces
                    .wheel_ellipse_utilization(self.params, "rr")
                    .sqrt(),
            ]
            .into_iter()
            .fold(0.0, f64::max);
            let longitudinal = [
                tire_forces.fx_fl_n.abs() / tire_forces.fz_fl_n.abs().max(1e-9),
                tire_forces.fx_fr_n.abs() / tire_forces.fz_fr_n.abs().max(1e-9),
                tire_forces.fx_rl_n.abs() / tire_forces.fz_rl_n.abs().max(1e-9),
                tire_forces.fx_rr_n.abs() / tire_forces.fz_rr_n.abs().max(1e-9),
            ]
            .into_iter()
            .fold(0.0, f64::max);
            let cornering = [
                tire_forces.fy_fl_n.abs() / tire_forces.fz_fl_n.abs().max(1e-9),
                tire_forces.fy_fr_n.abs() / tire_forces.fz_fr_n.abs().max(1e-9),
                tire_forces.fy_rl_n.abs() / tire_forces.fz_rl_n.abs().max(1e-9),
                tire_forces.fy_rr_n.abs() / tire_forces.fz_rr_n.abs().max(1e-9),
            ]
            .into_iter()
            .fold(0.0, f64::max);

            s_m.push(self.seed.station_s_m[station]);
            x_m.push(points[station][0]);
            y_m.push(points[station][1]);
            heading_rad.push(path_heading_rad(
                &points,
                station,
                seed_is_closed(&self.seed),
            ));
            kappa.push(kappa_1pm(&self.seed, station));
            v_mps.push(state.v_mps);
            ax_mps2.push(ax_velocity_mps2);
            ay_mps2.push(ay_velocity_mps2);
            utilization_cornering.push(cornering);
            utilization_longitudinal.push(longitudinal);
            utilization_combined.push(combined);
        }

        TrajectoryResultSeriesV1 {
            s_m,
            x_m,
            y_m,
            heading_rad,
            kappa_1pm: kappa,
            v_mps,
            ax_mps2,
            ay_mps2,
            utilization_cornering,
            utilization_longitudinal,
            utilization_combined,
            station_index: Some((0..count).map(|value| value as i64).collect()),
        }
    }
}

fn velocity_frame_acceleration(ax_body_mps2: f64, ay_body_mps2: f64, beta_rad: f64) -> (f64, f64) {
    let sin_beta = beta_rad.sin();
    let cos_beta = beta_rad.cos();
    (
        ax_body_mps2 * cos_beta + ay_body_mps2 * sin_beta,
        -ax_body_mps2 * sin_beta + ay_body_mps2 * cos_beta,
    )
}

pub struct CarDoubleTrackMintimeBackend;

fn cancel_error(message: &str) -> SolverApiError {
    SolverApiError::new("solve.cancelled", message)
}

fn is_cancelled(cancel_token: Option<&dyn SolverCancelToken>) -> bool {
    cancel_token.is_some_and(SolverCancelToken::is_cancelled)
}

impl CarDoubleTrackMintimeBackend {
    fn solve_with_cancel<'a>(
        &self,
        request: MintimeSolveRequestV1,
        mut progress: Option<MintimeProgressCallback<'a>>,
        cancel_token: Option<&'a dyn SolverCancelToken>,
    ) -> Result<MintimeSolveResult, SolverApiError> {
        if is_cancelled(cancel_token) {
            return Err(cancel_error("car mintime cancelled before preprocessing"));
        }
        emit_progress(
            &mut progress,
            MintimeProgressEvent {
                phase: "preprocessing".to_owned(),
                iteration: None,
                progress: Some(0.0),
                stage: None,
                stage_index: None,
                stage_count: None,
                stage_progress: Some(0.0),
                overall_progress: None,
                preview_source: None,
                message: Some("solve.phase.preprocessing".to_owned()),
                preview_trajectory_result: None,
                best_lap_time_s: None,
                model_track_area: None,
            },
        );
        let params = CarDoubleTrackParams::from_profile(&request.vehicle_dynamics_profile)
            .map_err(|message| SolverApiError::new("solve.invalidRequest", message))?;
        let options = CarMintimeSolveOptions::try_from_request(&request)?;
        let seed = build_car_mintime_nlp_seed(&request, params)?;
        if is_cancelled(cancel_token) {
            return Err(cancel_error(
                "car mintime cancelled after station preprocessing",
            ));
        }
        let problem = build_car_mintime_nlp_problem_with_options(seed, params, options.clone())?;

        emit_progress(
            &mut progress,
            MintimeProgressEvent {
                phase: "preprocessing".to_owned(),
                iteration: None,
                progress: Some(1.0),
                stage: None,
                stage_index: None,
                stage_count: None,
                stage_progress: Some(1.0),
                overall_progress: None,
                preview_source: None,
                message: Some("solve.phase.preprocessing".to_owned()),
                preview_trajectory_result: None,
                best_lap_time_s: None,
                model_track_area: Some(problem.seed.model_track_area.clone()),
            },
        );
        emit_progress(
            &mut progress,
            MintimeProgressEvent {
                phase: "running".to_owned(),
                iteration: None,
                progress: None,
                stage: Some("full_model".to_owned()),
                stage_index: Some(1),
                stage_count: Some(1),
                stage_progress: None,
                overall_progress: None,
                preview_source: None,
                message: Some(format!(
                    "solve.phase.running.objective_initial_s={:.6}",
                    problem.initial_diagnostics.objective_initial_s
                )),
                preview_trajectory_result: None,
                best_lap_time_s: Some(problem.initial_diagnostics.objective_initial_s),
                model_track_area: None,
            },
        );
        if is_cancelled(cancel_token) {
            return Err(cancel_error("car mintime cancelled before optimizer start"));
        }

        solve_car_mintime_with_ipopt(problem, options, progress, cancel_token)
    }
}

impl MintimeBackend for CarDoubleTrackMintimeBackend {
    fn solver_id(&self) -> &'static str {
        OLD_CAR_MINTIME_SOLVER_ID
    }

    fn solve(
        &self,
        request: MintimeSolveRequestV1,
        progress: Option<MintimeProgressCallback<'_>>,
    ) -> Result<MintimeSolveResult, SolverApiError> {
        self.solve_with_cancel(request, progress, None)
    }
}

impl CarMintimeSolveOptions {
    #[must_use]
    pub fn from_request(request: &MintimeSolveRequestV1) -> Self {
        Self::try_from_request(request).unwrap_or_default()
    }

    pub fn try_from_request(request: &MintimeSolveRequestV1) -> Result<Self, SolverApiError> {
        let mut options = Self::default();

        options.strict_collocation_tire_envelope =
            mintime_option_bool(request, "strict_collocation_tire_envelope").unwrap_or(true);
        options.strict_collocation_normal_load = options.strict_collocation_tire_envelope
            && mintime_option_bool(request, "strict_collocation_normal_load").unwrap_or(true);
        options.strict_collocation_kamm = options.strict_collocation_tire_envelope
            && mintime_option_bool(request, "strict_collocation_kamm").unwrap_or(true);
        options.strict_collocation_power = options.strict_collocation_tire_envelope
            && mintime_option_bool(request, "strict_collocation_power").unwrap_or(true);
        options.formulation_mode = mintime_option_str(request, "car_mintime_formulation_mode")
            .map(CarMintimeFormulationMode::parse)
            .transpose()
            .map_err(|message| SolverApiError::new("solve.invalidRequest", message))?
            .unwrap_or(CarMintimeFormulationMode::PrepeakGripV1);
        if let Some(value) = mintime_option_f64(request, "car_prepeak_grip_margin") {
            if !value.is_finite() || !(0.0..=1.0).contains(&value) || value == 0.0 {
                return Err(SolverApiError::new(
                    "solve.invalidRequest",
                    "car_prepeak_grip_margin must be finite and in (0, 1]",
                ));
            }
            options.prepeak_grip_margin = value;
        }
        if let Some(value) = mintime_option_f64(request, "max_iter") {
            options.max_iter = value.round() as i32;
        }
        if let Some(value) = mintime_option_f64(request, "tol") {
            options.tol = value;
        }
        if let Some(value) = mintime_option_f64(request, "acceptable_tol") {
            options.acceptable_tol = value;
        }
        if let Some(value) = mintime_option_f64(request, "acceptable_iter") {
            options.acceptable_iter = value.round() as i32;
        }
        if let Some(value) = mintime_option_f64(request, "ipopt_print_level") {
            options.ipopt_print_level = value.round() as i32;
        }
        if let Some(value) = mintime_option_f64(request, "penalty_delta") {
            options.penalty_delta = value;
        }
        if let Some(value) = mintime_option_f64(request, "penalty_f") {
            options.penalty_f = value;
        }
        if let Some(value) = mintime_option_f64(request, "penalty_delta_dd") {
            options.penalty_delta_dd = value;
        }
        if let Some(value) = mintime_option_f64(request, "penalty_f_dd") {
            options.penalty_f_dd = value;
        }
        if let Some(value) = mintime_option_f64(request, "penalty_n_dd") {
            options.penalty_n_dd = value;
        }
        if let Some(value) = mintime_option_f64(request, "penalty_xi_dd") {
            options.penalty_xi_dd = value;
        }
        if let Some(value) = mintime_option_f64(request, "penalty_endpoint_c1_dn") {
            options.penalty_endpoint_c1_dn = value;
        }
        if let Some(value) = mintime_option_f64(request, "endpoint_c1_dn_scale") {
            options.endpoint_c1_dn_scale = value;
        }
        if let Some(value) = mintime_option_f64(request, "penalty_endpoint_c1_heading") {
            options.penalty_endpoint_c1_heading = value;
        }
        if let Some(value) = mintime_option_f64(request, "endpoint_c1_heading_scale_rad") {
            options.endpoint_c1_heading_scale_rad = value;
        }
        if let Some(value) = mintime_option_f64(request, "penalty_endpoint_heading_jump") {
            options.penalty_endpoint_heading_jump = value;
        }
        if let Some(value) = mintime_option_f64(request, "endpoint_heading_jump_scale_rad") {
            options.endpoint_heading_jump_scale_rad = value;
        }
        if let Some(value) = mintime_option_f64(request, "penalty_endpoint_d2n_jump") {
            options.penalty_endpoint_d2n_jump = value;
        }
        if let Some(value) = mintime_option_f64(request, "endpoint_d2n_jump_scale") {
            options.endpoint_d2n_jump_scale = value;
        }
        if let Some(path) = mintime_option_str(request, "ipopt_dll_path") {
            options.ipopt_dll_path = Some(path.into());
        }

        options.max_iter = options.max_iter.max(1);
        options.acceptable_iter = options.acceptable_iter.max(0);
        options.penalty_delta = options.penalty_delta.max(0.0);
        options.penalty_f = options.penalty_f.max(0.0);
        options.penalty_delta_dd = options.penalty_delta_dd.max(0.0);
        options.penalty_f_dd = options.penalty_f_dd.max(0.0);
        options.penalty_n_dd = options.penalty_n_dd.max(0.0);
        options.penalty_xi_dd = options.penalty_xi_dd.max(0.0);
        options.penalty_endpoint_c1_dn = options.penalty_endpoint_c1_dn.max(0.0);
        options.endpoint_c1_dn_scale = options.endpoint_c1_dn_scale.max(1e-9);
        options.penalty_endpoint_c1_heading = options.penalty_endpoint_c1_heading.max(0.0);
        options.endpoint_c1_heading_scale_rad = options.endpoint_c1_heading_scale_rad.max(1e-9);
        options.penalty_endpoint_heading_jump = options.penalty_endpoint_heading_jump.max(0.0);
        options.endpoint_heading_jump_scale_rad = options.endpoint_heading_jump_scale_rad.max(1e-9);
        options.penalty_endpoint_d2n_jump = options.penalty_endpoint_d2n_jump.max(0.0);
        options.endpoint_d2n_jump_scale = options.endpoint_d2n_jump_scale.max(1e-9);
        Ok(options)
    }
}

fn mintime_option_bool(request: &MintimeSolveRequestV1, key: &str) -> Option<bool> {
    request
        .solve_options
        .iter()
        .find(|(entry_key, _)| entry_key == key)
        .and_then(|(_, value)| json_bool(value))
}

fn json_bool(value: &JsonValue) -> Option<bool> {
    match value {
        JsonValue::Bool(value) => Some(*value),
        JsonValue::String(value) => match value.trim().to_ascii_lowercase().as_str() {
            "1" | "true" | "yes" | "on" => Some(true),
            "0" | "false" | "no" | "off" => Some(false),
            _ => None,
        },
        _ => None,
    }
}

fn mintime_option_f64(request: &MintimeSolveRequestV1, key: &str) -> Option<f64> {
    request
        .solve_options
        .iter()
        .find(|(entry_key, _)| entry_key == key)
        .and_then(|(_, value)| value.as_f64())
}

fn mintime_option_str<'a>(request: &'a MintimeSolveRequestV1, key: &str) -> Option<&'a str> {
    request
        .solve_options
        .iter()
        .find(|(entry_key, _)| entry_key == key)
        .and_then(|(_, value)| value.as_str())
}

fn upsert_metadata(metadata: &mut Vec<(String, JsonValue)>, key: &str, value: JsonValue) {
    if let Some((_, existing)) = metadata.iter_mut().find(|(entry_key, _)| entry_key == key) {
        *existing = value;
    } else {
        metadata.push((key.to_owned(), value));
    }
}

struct CarMintimeIpoptNlp<'a> {
    problem: CarMintimeNlpProblem,
    variable_scales: Vec<f64>,
    constraint_scales: Vec<f64>,
    progress: Option<MintimeProgressCallback<'a>>,
    cancel_token: Option<&'a dyn SolverCancelToken>,
    objective_eval_count: u32,
    last_preview_eval_count: u32,
    last_ipopt_iteration: Option<u32>,
}

fn solve_car_mintime_with_ipopt<'a>(
    problem: CarMintimeNlpProblem,
    options: CarMintimeSolveOptions,
    progress: Option<MintimeProgressCallback<'a>>,
    cancel_token: Option<&'a dyn SolverCancelToken>,
) -> Result<MintimeSolveResult, SolverApiError> {
    solve_car_mintime_with_ipopt_initial(problem, options, progress, cancel_token, None)
}

fn solve_car_mintime_with_ipopt_initial<'a>(
    problem: CarMintimeNlpProblem,
    options: CarMintimeSolveOptions,
    progress: Option<MintimeProgressCallback<'a>>,
    cancel_token: Option<&'a dyn SolverCancelToken>,
    initial_x: Option<Vec<f64>>,
) -> Result<MintimeSolveResult, SolverApiError> {
    let variable_scales = car_mintime_decision_variable_scales(&problem.seed);
    let constraint_scales = car_mintime_constraint_scales(&problem.constraints, problem.params);
    let mut nlp = Box::new(CarMintimeIpoptNlp {
        problem,
        variable_scales,
        constraint_scales,
        progress,
        cancel_token,
        objective_eval_count: 0,
        last_preview_eval_count: 0,
        last_ipopt_iteration: None,
    });
    let physical_initial_x = initial_x.unwrap_or_else(|| nlp.problem.seed.initial_guess.clone());
    if physical_initial_x.len() != nlp.problem.decision_variable_count() {
        return Err(SolverApiError::new(
            "solve.invalidRequest",
            format!(
                "car mintime initial vector length {} does not match decision variable count {}",
                physical_initial_x.len(),
                nlp.problem.decision_variable_count()
            ),
        ));
    }
    let mut x = physical_initial_x
        .iter()
        .zip(&nlp.variable_scales)
        .map(|(value, scale)| value / scale)
        .collect::<Vec<_>>();
    let mut lower_x = nlp
        .problem
        .seed
        .lower_bounds
        .iter()
        .zip(&nlp.variable_scales)
        .map(|(value, scale)| value / scale)
        .collect::<Vec<_>>();
    let mut upper_x = nlp
        .problem
        .seed
        .upper_bounds
        .iter()
        .zip(&nlp.variable_scales)
        .map(|(value, scale)| value / scale)
        .collect::<Vec<_>>();
    let mut lower_g = nlp
        .problem
        .constraint_lower_bounds
        .iter()
        .zip(&nlp.constraint_scales)
        .map(|(value, scale)| value / scale)
        .collect::<Vec<_>>();
    let mut upper_g = nlp
        .problem
        .constraint_upper_bounds
        .iter()
        .zip(&nlp.constraint_scales)
        .map(|(value, scale)| value / scale)
        .collect::<Vec<_>>();
    let variable_count = i32::try_from(nlp.problem.decision_variable_count()).map_err(|_| {
        SolverApiError::new("solve.invalidRequest", "too many car mintime variables")
    })?;
    let constraint_count = i32::try_from(nlp.problem.constraint_count()).map_err(|_| {
        SolverApiError::new("solve.invalidRequest", "too many car mintime constraints")
    })?;
    let jacobian_count = i32::try_from(nlp.problem.jacobian_pattern.len()).map_err(|_| {
        SolverApiError::new(
            "solve.invalidRequest",
            "too many car mintime jacobian entries",
        )
    })?;
    let library_path = crate::ipopt::default_library_path(options.ipopt_dll_path);
    let ipopt = crate::ipopt::IpoptApi::load(&library_path).map_err(native_backend_error)?;

    unsafe {
        let problem = (ipopt.create_problem)(
            variable_count,
            lower_x.as_mut_ptr(),
            upper_x.as_mut_ptr(),
            constraint_count,
            lower_g.as_mut_ptr(),
            upper_g.as_mut_ptr(),
            jacobian_count,
            0,
            0,
            car_eval_f_cb,
            car_eval_g_cb,
            car_eval_grad_f_cb,
            car_eval_jac_g_cb,
            Some(car_eval_h_cb),
        );
        if problem.is_null() {
            return Err(native_backend_error(
                "CreateIpoptProblem returned null".to_owned(),
            ));
        }
        let _guard = crate::ipopt::IpoptProblemGuard::new(problem, ipopt.free_problem);
        ipopt
            .add_int(problem, "print_level", options.ipopt_print_level)
            .map_err(native_backend_error)?;
        ipopt
            .add_int(problem, "max_iter", options.max_iter)
            .map_err(native_backend_error)?;
        ipopt
            .add_num(problem, "tol", options.tol)
            .map_err(native_backend_error)?;
        ipopt
            .add_num(problem, "acceptable_tol", options.acceptable_tol)
            .map_err(native_backend_error)?;
        ipopt
            .add_int(problem, "acceptable_iter", options.acceptable_iter)
            .map_err(native_backend_error)?;
        ipopt
            .add_str(problem, "hessian_approximation", "limited-memory")
            .map_err(native_backend_error)?;
        ipopt
            .add_str(problem, "mu_strategy", "adaptive")
            .map_err(native_backend_error)?;
        if let Some(set_intermediate_callback) = ipopt.set_intermediate_callback {
            set_intermediate_callback(problem, car_intermediate_cb);
        }

        let mut g = vec![0.0; constraint_count as usize];
        let mut objective = 0.0;
        let initial_lap_time_s = car_mintime_collocation_objective_s(
            &nlp.problem.seed,
            nlp.problem.params,
            &physical_initial_x,
        );
        nlp.emit_preview(&physical_initial_x, initial_lap_time_s, true);
        let user_data = (&mut *nlp) as *mut CarMintimeIpoptNlp as *mut c_void;
        let status_code = (ipopt.solve)(
            problem,
            x.as_mut_ptr(),
            g.as_mut_ptr(),
            &mut objective,
            ptr::null_mut(),
            ptr::null_mut(),
            ptr::null_mut(),
            user_data,
        );
        if is_cancelled(nlp.cancel_token) {
            return Err(cancel_error("car mintime cancelled during optimizer solve"));
        }
        let status = crate::ipopt::status_name(status_code);
        if crate::ipopt::status_is_success(status_code) {
            emit_progress(
                &mut nlp.progress,
                MintimeProgressEvent {
                    phase: "postprocessing".to_owned(),
                    iteration: nlp.last_ipopt_iteration,
                    progress: Some(0.0),
                    stage: None,
                    stage_index: None,
                    stage_count: None,
                    stage_progress: Some(0.0),
                    overall_progress: None,
                    preview_source: None,
                    message: Some("solve.phase.postprocessing".to_owned()),
                    preview_trajectory_result: None,
                    best_lap_time_s: None,
                    model_track_area: None,
                },
            );
        }
        let physical_solution_x = nlp.physical_from_scaled(&x);
        let final_objective = nlp.problem.objective(&physical_solution_x);
        let final_lap_time_s = car_mintime_collocation_objective_s(
            &nlp.problem.seed,
            nlp.problem.params,
            &physical_solution_x,
        );
        let final_constraints = nlp.problem.constraint_values(&physical_solution_x);
        let (max_final_bound_violation, worst_final_bound_violation_label) =
            max_constraint_bound_violation_for_values(
                &nlp.problem.constraints,
                &final_constraints,
                &nlp.problem.constraint_lower_bounds,
                &nlp.problem.constraint_upper_bounds,
            );
        let final_bound_violation_report =
            top_constraint_bound_violation_report(&nlp.problem, &physical_solution_x, 8);
        if !crate::ipopt::status_is_success(status_code) {
            return Err(native_backend_error(format!(
                "Ipopt solve failed with status {status} ({status_code}); final_objective_s={final_objective:.9}; max_final_bound_violation={max_final_bound_violation:.9}; worst_final_bound={worst_final_bound_violation_label}; final_bound_violation_report={final_bound_violation_report}"
            )));
        }
        let trajectory_result = nlp.problem.to_series(&physical_solution_x);
        let trajectory_dense = car_dense_trajectory_json(
            &nlp.problem.seed,
            nlp.problem.params,
            &physical_solution_x,
            CAR_DENSE_FRENET_SAMPLES_PER_INTERVAL,
        );
        let section_frame_coherence_audit =
            car_section_frame_coherence_audit_json(&nlp.problem.seed);
        let lap_time_estimate_s = Some(final_lap_time_s);
        let closed =
            nlp.problem.seed.dimensions.interval_count == nlp.problem.seed.dimensions.station_count;
        let visualization = solve_result_visualization_json(&trajectory_result, closed);
        let model_track_area = nlp.problem.seed.model_track_area.clone();
        let mut diagnostics = car_mintime_diagnostics_json(
            &nlp.problem,
            status,
            status_code,
            final_objective,
            final_lap_time_s,
            max_final_bound_violation,
            &worst_final_bound_violation_label,
            &final_bound_violation_report,
            &physical_solution_x,
            &trajectory_result,
            closed,
        );
        append_json_object_field(
            &mut diagnostics,
            "section_frame_coherence_audit",
            section_frame_coherence_audit,
        );

        emit_progress(
            &mut nlp.progress,
            MintimeProgressEvent {
                phase: "completed".to_owned(),
                iteration: nlp.last_ipopt_iteration,
                progress: Some(1.0),
                stage: Some("full_model".to_owned()),
                stage_index: Some(1),
                stage_count: Some(1),
                stage_progress: Some(1.0),
                overall_progress: Some(1.0),
                preview_source: Some("full_model".to_owned()),
                message: Some("solve.phase.completed".to_owned()),
                preview_trajectory_result: Some(trajectory_result.clone()),
                best_lap_time_s: lap_time_estimate_s,
                model_track_area: None,
            },
        );

        Ok(MintimeSolveResult {
            runtime: "rust_car_mintime_ipopt".to_owned(),
            status: status.to_owned(),
            lap_time_estimate_s,
            trajectory_result,
            trajectory_dense: Some(trajectory_dense),
            trajectory_contract: Some(car_trajectory_contract_json()),
            model_track_area,
            visualization,
            diagnostics,
            warnings: Vec::new(),
        })
    }
}

unsafe extern "C" fn car_eval_f_cb(
    _n: i32,
    x: *mut f64,
    _new_x: bool,
    obj_value: *mut f64,
    user_data: *mut c_void,
) -> bool {
    let nlp = &mut *(user_data as *mut CarMintimeIpoptNlp);
    if is_cancelled(nlp.cancel_token) {
        return false;
    }
    let scaled_values = std::slice::from_raw_parts(x, nlp.problem.decision_variable_count());
    let values = nlp.physical_from_scaled(scaled_values);
    let objective = nlp.problem.objective(&values);
    *obj_value = objective;
    let lap_time_s =
        car_mintime_collocation_objective_s(&nlp.problem.seed, nlp.problem.params, &values);
    nlp.emit_preview(&values, lap_time_s, false);
    true
}

unsafe extern "C" fn car_intermediate_cb(
    _alg_mod: i32,
    iter_count: i32,
    obj_value: f64,
    _inf_pr: f64,
    _inf_du: f64,
    _mu: f64,
    _d_norm: f64,
    _regularization_size: f64,
    _alpha_du: f64,
    _alpha_pr: f64,
    _ls_trials: i32,
    user_data: *mut c_void,
) -> bool {
    if user_data.is_null() {
        return false;
    }
    let nlp = &mut *(user_data as *mut CarMintimeIpoptNlp);
    if is_cancelled(nlp.cancel_token) {
        return false;
    }
    nlp.emit_optimizer_iteration(iter_count, obj_value);
    true
}

unsafe extern "C" fn car_eval_grad_f_cb(
    _n: i32,
    x: *mut f64,
    _new_x: bool,
    grad_f: *mut f64,
    user_data: *mut c_void,
) -> bool {
    let nlp = &*(user_data as *const CarMintimeIpoptNlp);
    if is_cancelled(nlp.cancel_token) {
        return false;
    }
    let scaled_values = std::slice::from_raw_parts(x, nlp.problem.decision_variable_count());
    let values = nlp.physical_from_scaled(scaled_values);
    let grad = std::slice::from_raw_parts_mut(grad_f, nlp.problem.decision_variable_count());
    nlp.problem
        .objective_gradient_structured_numeric(&values, grad);
    nlp.scale_gradient_to_ipopt_variables(grad);
    true
}

unsafe extern "C" fn car_eval_g_cb(
    _n: i32,
    x: *mut f64,
    _new_x: bool,
    _m: i32,
    g: *mut f64,
    user_data: *mut c_void,
) -> bool {
    let nlp = &*(user_data as *const CarMintimeIpoptNlp);
    if is_cancelled(nlp.cancel_token) {
        return false;
    }
    let scaled_values = std::slice::from_raw_parts(x, nlp.problem.decision_variable_count());
    let values = nlp.physical_from_scaled(scaled_values);
    let constraints = std::slice::from_raw_parts_mut(g, nlp.problem.constraint_count());
    for ((target, value), scale) in constraints
        .iter_mut()
        .zip(nlp.problem.constraint_values(&values))
        .zip(&nlp.constraint_scales)
    {
        *target = value / scale;
    }
    true
}

unsafe extern "C" fn car_eval_jac_g_cb(
    _n: i32,
    x: *mut f64,
    _new_x: bool,
    _m: i32,
    _nele_jac: i32,
    i_row: *mut i32,
    j_col: *mut i32,
    values: *mut f64,
    user_data: *mut c_void,
) -> bool {
    let nlp = &*(user_data as *const CarMintimeIpoptNlp);
    if is_cancelled(nlp.cancel_token) {
        return false;
    }
    if values.is_null() {
        let rows = std::slice::from_raw_parts_mut(i_row, nlp.problem.jacobian_pattern.len());
        let cols = std::slice::from_raw_parts_mut(j_col, nlp.problem.jacobian_pattern.len());
        for (index, (row, col)) in nlp.problem.jacobian_pattern.iter().copied().enumerate() {
            rows[index] = row;
            cols[index] = col;
        }
        return true;
    }
    let scaled_x_values = std::slice::from_raw_parts(x, nlp.problem.decision_variable_count());
    let x_values = nlp.physical_from_scaled(scaled_x_values);
    let jac_values = std::slice::from_raw_parts_mut(values, nlp.problem.jacobian_pattern.len());
    let mut physical_jacobian = nlp.problem.jacobian_values_structured_numeric(&x_values);
    nlp.scale_jacobian_to_ipopt_variables(&mut physical_jacobian);
    for (target, value) in jac_values.iter_mut().zip(physical_jacobian) {
        *target = value;
    }
    true
}

unsafe extern "C" fn car_eval_h_cb(
    _n: i32,
    _x: *mut f64,
    _new_x: bool,
    _obj_factor: f64,
    _m: i32,
    _lambda: *mut f64,
    _new_lambda: bool,
    _nele_hess: i32,
    _i_row: *mut i32,
    _j_col: *mut i32,
    _values: *mut f64,
    _user_data: *mut c_void,
) -> bool {
    true
}

fn native_backend_error(message: String) -> SolverApiError {
    SolverApiError::new("solve.nativeBackendUnavailable", message)
}

impl CarMintimeIpoptNlp<'_> {
    fn emit_optimizer_iteration(&mut self, iter_count: i32, objective: f64) {
        let iteration = u32::try_from(iter_count.max(0)).unwrap_or(u32::MAX);
        self.last_ipopt_iteration = Some(iteration);
        emit_progress(
            &mut self.progress,
            MintimeProgressEvent {
                phase: "running".to_owned(),
                iteration: Some(iteration),
                progress: None,
                stage: Some("full_model".to_owned()),
                stage_index: Some(1),
                stage_count: Some(1),
                stage_progress: None,
                overall_progress: None,
                preview_source: None,
                message: Some(format!("solve.phase.running.objective={objective:.6}")),
                preview_trajectory_result: None,
                best_lap_time_s: None,
                model_track_area: None,
            },
        );
    }

    fn physical_from_scaled(&self, scaled: &[f64]) -> Vec<f64> {
        scaled
            .iter()
            .zip(&self.variable_scales)
            .map(|(value, scale)| value * scale)
            .collect()
    }

    fn scale_gradient_to_ipopt_variables(&self, grad: &mut [f64]) {
        for (value, scale) in grad.iter_mut().zip(&self.variable_scales) {
            *value *= scale;
        }
    }

    fn scale_jacobian_to_ipopt_variables(&self, values: &mut [f64]) {
        for (value, (row, col)) in values.iter_mut().zip(&self.problem.jacobian_pattern) {
            if let Ok(row) = usize::try_from(*row) {
                if let Some(scale) = self.constraint_scales.get(row) {
                    *value /= scale;
                }
            }
            if let Ok(col) = usize::try_from(*col) {
                if let Some(scale) = self.variable_scales.get(col) {
                    *value *= scale;
                }
            }
        }
    }

    fn emit_preview(&mut self, x: &[f64], objective: f64, force: bool) {
        self.objective_eval_count = self.objective_eval_count.saturating_add(1);
        if !force
            && self
                .objective_eval_count
                .saturating_sub(self.last_preview_eval_count)
                < CAR_MINTIME_PREVIEW_EVAL_PERIOD
        {
            return;
        }
        self.last_preview_eval_count = self.objective_eval_count;
        let series = self.problem.to_series(x);
        emit_progress(
            &mut self.progress,
            MintimeProgressEvent {
                phase: "running".to_owned(),
                iteration: self.last_ipopt_iteration,
                progress: None,
                stage: Some("full_model".to_owned()),
                stage_index: Some(1),
                stage_count: Some(1),
                stage_progress: None,
                overall_progress: None,
                preview_source: Some("full_model".to_owned()),
                message: Some(format!("solve.phase.running.objective_s={objective:.6}")),
                preview_trajectory_result: Some(series),
                best_lap_time_s: Some(objective),
                model_track_area: None,
            },
        );
    }
}

pub fn build_car_mintime_nlp_seed(
    request: &MintimeSolveRequestV1,
    params: CarDoubleTrackParams,
) -> Result<CarMintimeNlpSeed, SolverApiError> {
    if request.station_count < MIN_CAR_MINTIME_STATION_COUNT {
        return Err(SolverApiError::new(
            "solve.invalidRequest",
            format!("station_count must be at least {MIN_CAR_MINTIME_STATION_COUNT}"),
        ));
    }

    let (sections, prepared_model_track_area) = match &request.geometry_input {
        MintimeGeometryInput::PreparedStationGeometry(prepared) => (
            prepared.sections_track_view.clone(),
            prepared.model_track_area(),
        ),
        // rust_solver_http_request.v1 is retained for offline fixtures and
        // historical CLI experiments. The mobile product always uses v2.
        MintimeGeometryInput::LegacyRawGeometry(track_area) => {
            let solve_options = JsonValue::Object(request.solve_options.clone());
            let mut station_options =
                parse_station_options(request.station_count, Some(&solve_options)).map_err(
                    |message| {
                        SolverApiError::new(
                            "solve.invalidRequest",
                            format!("invalid station options: {message}"),
                        )
                    },
                )?;
            station_options.sample_count = request.station_count;
            let station_request = StationGenerationRequestV1 {
                request_key: "legacy_car_station_request".to_owned(),
                request_id: request.request_id.clone(),
                project_id: request.project_id.clone(),
                station_count: request.station_count,
                count_mode: crate::station_generation::StationCountMode::Exact,
                track_area: track_area.clone(),
                station_options,
                station_options_hash: "legacy_station_options".to_owned(),
                source_ref: crate::contracts::StationSourceRefV1 {
                    project_id: request.project_id.clone(),
                    geometry_id: track_area.track_id.clone(),
                    geometry_content_hash: crate::contracts::station_geometry_content_hash_v1(
                        track_area,
                    ),
                    route_id: track_area.track_id.clone(),
                },
            };
            let station_result = generate_station_geometry(&station_request, None);
            (
                station_result.sections_track_view,
                station_result.model_track_area,
            )
        }
    };
    validate_sections_for_car_mintime(&sections, request.station_count)?;

    let closed = prepared_model_track_area.trajectory_mode == "closed";
    let layout = MintimeNlpLayout::for_family(VehicleDynamicsModelFamily::CarDynamics);
    let mut dimensions = layout.dimensions_for_station_count(sections.station_s_m.len(), closed);
    dimensions.collocation_state_variable_count =
        dimensions.interval_count * CAR_COLLOCATION_DEGREE * CAR_STATE_LEN;
    let solver_station_s_m = if closed {
        closed_chord_station_m(&sections.centerline_xy_m)
    } else {
        open_chord_station_m(&sections.centerline_xy_m)
    };
    let kappa_1pm = if closed {
        python_compatible_closed_kappa_1pm(&sections.centerline_xy_m, &solver_station_s_m)
    } else {
        open_kappa_1pm(&sections.centerline_xy_m, &solver_station_s_m)
    };
    let section_geometry = if closed {
        closed_section_geometry(
            &sections.centerline_xy_m,
            &solver_station_s_m,
            Some(&sections.section_dirs_xy),
        )
    } else {
        open_section_geometry(
            &sections.centerline_xy_m,
            &solver_station_s_m,
            Some(&sections.section_dirs_xy),
        )
    };
    let initial_states = initial_state_guesses_from_kappa(params, &kappa_1pm);
    let initial_controls = initial_control_guesses_from_kappa(&initial_states);
    let width_opt_m = car_mintime_width_opt_m(request);
    let half_width_opt_m = width_opt_m * 0.5;
    let mut initial_guess = Vec::with_capacity(dimensions.decision_variable_count());
    let mut lower_bounds = Vec::with_capacity(dimensions.decision_variable_count());
    let mut upper_bounds = Vec::with_capacity(dimensions.decision_variable_count());
    let open_start_speed_request = (!closed)
        .then(|| mintime_option_f64(request, "open_start_speed_mps"))
        .flatten();
    let open_start_speed_effective = open_start_speed_request.map(|value| value.max(0.1));
    let open_standing_start = open_start_speed_request.is_some_and(|value| value.abs() <= 1.0e-9);
    let open_finish_speed_request = (!closed)
        .then(|| mintime_option_f64(request, "open_finish_speed_mps"))
        .flatten();
    let open_finish_speed_effective = open_finish_speed_request.map(|value| value.max(0.1));

    for index in 0..dimensions.station_count {
        let width_left = sections.width_left_m[index].max(1e-3);
        let width_right = sections.width_right_m[index].max(1e-3);
        let (lower_n_m, upper_n_m) =
            car_mintime_n_bounds_m(width_left, width_right, half_width_opt_m);

        push_state_row(
            &mut initial_guess,
            &mut lower_bounds,
            &mut upper_bounds,
            initial_states[index],
            lower_n_m,
            upper_n_m,
            params.max_speed_mps,
        );
    }
    if let Some(speed_mps) = open_start_speed_effective {
        initial_guess[STATE_V_MPS] = speed_mps;
        lower_bounds[STATE_V_MPS] = speed_mps;
        upper_bounds[STATE_V_MPS] = speed_mps;
    }
    if let Some(speed_mps) = open_finish_speed_effective {
        let offset = state_offset(dimensions.station_count.saturating_sub(1)) + STATE_V_MPS;
        initial_guess[offset] = speed_mps;
        lower_bounds[offset] = speed_mps;
        upper_bounds[offset] = speed_mps;
    }
    if open_standing_start {
        for state_index in [STATE_BETA_RAD, STATE_OMEGA_Z_RADPS, STATE_N_M, STATE_XI_RAD] {
            fix_decision_variable(
                &mut initial_guess,
                &mut lower_bounds,
                &mut upper_bounds,
                state_offset(0) + state_index,
                0.0,
            );
        }
    }

    for interval in 0..dimensions.interval_count {
        push_control_row(
            &mut initial_guess,
            &mut lower_bounds,
            &mut upper_bounds,
            initial_controls[interval],
            params,
        );
    }
    if open_standing_start && dimensions.interval_count > 0 {
        for control_index in [CONTROL_DELTA_RAD, CONTROL_GAMMA_Y_N] {
            fix_decision_variable(
                &mut initial_guess,
                &mut lower_bounds,
                &mut upper_bounds,
                dimensions.state_variable_count + control_index,
                0.0,
            );
        }
    }

    for interval in 0..dimensions.interval_count {
        let next = if closed {
            (interval + 1) % dimensions.station_count.max(1)
        } else {
            (interval + 1).min(dimensions.station_count.saturating_sub(1))
        };
        for point in 1..=CAR_COLLOCATION_DEGREE {
            let tau = car_legendre_collocation_coefficients_degree3().tau[point];
            let width_left = lerp(
                sections.width_left_m[interval],
                sections.width_left_m[next],
                tau,
            )
            .max(1e-3);
            let width_right = lerp(
                sections.width_right_m[interval],
                sections.width_right_m[next],
                tau,
            )
            .max(1e-3);
            let (lower_n_m, upper_n_m) =
                car_mintime_n_bounds_m(width_left, width_right, half_width_opt_m);
            push_state_row(
                &mut initial_guess,
                &mut lower_bounds,
                &mut upper_bounds,
                lerp_state_guess(initial_states[interval], initial_states[next], tau),
                lower_n_m,
                upper_n_m,
                params.max_speed_mps,
            );
        }
    }
    let mut model_track_area = prepared_model_track_area;
    if let Some(prepared) = request.prepared_station_geometry() {
        upsert_metadata(
            &mut model_track_area.metadata,
            "station_geometry_source",
            "prepared_station_geometry".into(),
        );
        upsert_metadata(
            &mut model_track_area.metadata,
            "station_geometry_artifact_key",
            prepared.prepared_bundle_hash.clone().into(),
        );
        upsert_metadata(
            &mut model_track_area.metadata,
            "sections_track_view_hash",
            prepared.sections_track_view_hash.clone().into(),
        );
    }
    if let Some(requested) = open_start_speed_request {
        upsert_metadata(
            &mut model_track_area.metadata,
            "open_start_speed_requested_mps",
            JsonValue::from(requested),
        );
        upsert_metadata(
            &mut model_track_area.metadata,
            "open_start_speed_effective_mps",
            JsonValue::from(open_start_speed_effective.unwrap_or(requested)),
        );
        if open_standing_start {
            upsert_metadata(
                &mut model_track_area.metadata,
                "open_start_pose_locked",
                JsonValue::Bool(true),
            );
            upsert_metadata(
                &mut model_track_area.metadata,
                "open_start_first_lateral_control_locked",
                JsonValue::Bool(true),
            );
        }
    }
    if let Some(requested) = open_finish_speed_request {
        upsert_metadata(
            &mut model_track_area.metadata,
            "open_finish_speed_requested_mps",
            JsonValue::from(requested),
        );
        upsert_metadata(
            &mut model_track_area.metadata,
            "open_finish_speed_effective_mps",
            JsonValue::from(open_finish_speed_effective.unwrap_or(requested)),
        );
    }

    Ok(CarMintimeNlpSeed {
        layout,
        dimensions,
        model_track_area,
        station_s_m: solver_station_s_m,
        kappa_1pm,
        centerline_xy_m: sections.centerline_xy_m,
        ref_tangent_xy: section_geometry.ref_tangent_xy,
        ref_left_normal_xy: section_geometry.ref_left_normal_xy,
        section_dir_xy: section_geometry.section_dir_xy,
        section_dir_derivative_xy: section_geometry.section_dir_derivative_xy,
        width_left_m: sections.width_left_m,
        width_right_m: sections.width_right_m,
        initial_guess,
        lower_bounds,
        upper_bounds,
    })
}

pub fn build_car_mintime_nlp_problem(
    seed: CarMintimeNlpSeed,
    params: CarDoubleTrackParams,
) -> Result<CarMintimeNlpProblem, SolverApiError> {
    build_car_mintime_nlp_problem_with_options(seed, params, CarMintimeSolveOptions::default())
}

pub fn build_car_mintime_nlp_problem_with_options(
    seed: CarMintimeNlpSeed,
    params: CarDoubleTrackParams,
    options: CarMintimeSolveOptions,
) -> Result<CarMintimeNlpProblem, SolverApiError> {
    if seed.layout.state_columns.len() != CAR_STATE_LEN
        || seed.layout.control_columns.len() != CAR_CONTROL_LEN
    {
        return Err(SolverApiError::new(
            "solve.invalidRequest",
            "car mintime NLP layout does not match car double-track dimensions",
        ));
    }

    let constraints = car_mintime_constraint_rows(seed.dimensions, &options);
    let residuals = car_mintime_initial_constraint_residuals(&seed, &constraints, params);
    let objective_weights = CarMintimeObjectiveWeights::from_options(&options);
    let (constraint_lower_bounds, constraint_upper_bounds) =
        car_mintime_constraint_bounds(&constraints, params, &options);
    let jacobian_pattern = car_mintime_jacobian_pattern(&seed, &constraints)?;
    let jacobian_columns = car_mintime_jacobian_columns_by_variable(
        seed.dimensions.decision_variable_count(),
        &jacobian_pattern,
    );
    let (max_initial_abs_residual, worst_initial_constraint_label) =
        worst_constraint_residual(&constraints, &residuals);
    let objective_initial_s =
        car_mintime_collocation_objective_s(&seed, params, &seed.initial_guess)
            + car_mintime_regularization_objective_s(
                &seed,
                params,
                objective_weights,
                &seed.initial_guess,
            );
    let initial_diagnostics = CarMintimeNlpDiagnostics {
        objective_initial_s,
        constraint_count: constraints.len(),
        max_initial_abs_residual,
        worst_initial_constraint_label,
    };

    Ok(CarMintimeNlpProblem {
        seed,
        params,
        options,
        objective_weights,
        constraints,
        constraint_lower_bounds,
        constraint_upper_bounds,
        jacobian_pattern,
        jacobian_columns,
        initial_diagnostics,
    })
}

fn car_mintime_decision_variable_scales(seed: &CarMintimeNlpSeed) -> Vec<f64> {
    let mut scales = Vec::with_capacity(seed.dimensions.decision_variable_count());

    for _ in 0..seed.dimensions.station_count {
        scales.extend_from_slice(&CAR_STATE_SCALE);
    }
    for _ in 0..seed.dimensions.interval_count {
        scales.extend_from_slice(&CAR_CONTROL_SCALE);
    }
    for _ in 0..seed.dimensions.interval_count {
        for _ in 0..CAR_COLLOCATION_DEGREE {
            scales.extend_from_slice(&CAR_STATE_SCALE);
        }
    }

    scales
}

fn car_mintime_constraint_scales(
    rows: &[CarMintimeConstraintRow],
    params: CarDoubleTrackParams,
) -> Vec<f64> {
    rows.iter()
        .map(|row| match row {
            CarMintimeConstraintRow::CollocationDynamics { .. }
            | CarMintimeConstraintRow::Continuity { .. }
            | CarMintimeConstraintRow::Dynamics { .. }
            | CarMintimeConstraintRow::TireEllipse { .. }
            | CarMintimeConstraintRow::CollocationTireEllipse { .. }
            | CarMintimeConstraintRow::SlipPrepeak { .. }
            | CarMintimeConstraintRow::CollocationSlipPrepeak { .. } => 1.0,
            CarMintimeConstraintRow::PowerLimit { .. }
            | CarMintimeConstraintRow::CollocationPowerLimit { .. } => params.power_max_w.max(1.0),
            CarMintimeConstraintRow::NormalLoad { .. }
            | CarMintimeConstraintRow::CollocationNormalLoad { .. } => {
                (params.mass_kg * params.gravity_mps2 * 0.25).max(1.0)
            }
            CarMintimeConstraintRow::LateralLoadTransfer { .. } => {
                (params.mass_kg * params.gravity_mps2 * params.lateral_grip_level).max(1.0)
            }
            CarMintimeConstraintRow::DriveBrakeMutex { .. } => {
                CAR_DRIVE_BRAKE_MUTEX_LOWER_N2.abs().max(1.0)
            }
            CarMintimeConstraintRow::ControlRate { control_name, .. } => match *control_name {
                "delta_rad" => (params.steering_angle_max_rad / params.steering_response_s)
                    .abs()
                    .max(1.0),
                "f_drive_N" => (params.drive_force_max_n / params.throttle_response_s)
                    .abs()
                    .max(1.0),
                "f_brake_N" => (params.brake_force_max_n / params.brake_response_s)
                    .abs()
                    .max(1.0),
                "gamma_y_N" => {
                    (params.mass_kg * params.gravity_mps2 * params.lateral_grip_level).max(1.0)
                }
                _ => unreachable!("unknown car control {control_name}"),
            },
        })
        .collect()
}

fn car_mintime_constraint_rows(
    dimensions: MintimeNlpDimensions,
    options: &CarMintimeSolveOptions,
) -> Vec<CarMintimeConstraintRow> {
    let mut rows = Vec::with_capacity(dimensions.interval_count * 42);

    for interval in 0..dimensions.interval_count {
        for point in 1..=CAR_COLLOCATION_DEGREE {
            for state_name in ["v_mps", "beta_rad", "omega_z_radps", "n_m", "xi_rad"] {
                rows.push(CarMintimeConstraintRow::CollocationDynamics {
                    interval,
                    point,
                    state_name,
                });
            }
            if options.strict_collocation_normal_load {
                for wheel in ["fl", "fr", "rl", "rr"] {
                    rows.push(CarMintimeConstraintRow::CollocationNormalLoad {
                        interval,
                        point,
                        wheel,
                    });
                }
            }
            if options.strict_collocation_kamm {
                for wheel in ["fl", "fr", "rl", "rr"] {
                    rows.push(CarMintimeConstraintRow::CollocationTireEllipse {
                        interval,
                        point,
                        wheel,
                    });
                }
            }
            if options.formulation_mode.uses_prepeak_grip_domain() {
                for wheel in ["fl", "fr", "rl", "rr"] {
                    rows.push(CarMintimeConstraintRow::CollocationSlipPrepeak {
                        interval,
                        point,
                        wheel,
                    });
                }
            }
            if options.strict_collocation_power {
                rows.push(CarMintimeConstraintRow::CollocationPowerLimit { interval, point });
            }
        }
        for state_name in ["v_mps", "beta_rad", "omega_z_radps", "n_m", "xi_rad"] {
            rows.push(CarMintimeConstraintRow::Continuity {
                interval,
                state_name,
            });
        }
        rows.push(CarMintimeConstraintRow::PowerLimit { interval });
        if options.strict_collocation_normal_load {
            for wheel in ["fl", "fr", "rl", "rr"] {
                rows.push(CarMintimeConstraintRow::NormalLoad { interval, wheel });
            }
        }
        for wheel in ["fl", "fr", "rl", "rr"] {
            rows.push(CarMintimeConstraintRow::TireEllipse { interval, wheel });
        }
        if options.formulation_mode.uses_prepeak_grip_domain() {
            for wheel in ["fl", "fr", "rl", "rr"] {
                rows.push(CarMintimeConstraintRow::SlipPrepeak { interval, wheel });
            }
        }
        rows.push(CarMintimeConstraintRow::LateralLoadTransfer { interval });
        rows.push(CarMintimeConstraintRow::DriveBrakeMutex { interval });

        if interval > 0 {
            for control_name in ["delta_rad", "f_drive_N", "f_brake_N"] {
                rows.push(CarMintimeConstraintRow::ControlRate {
                    interval,
                    control_name,
                });
            }
        }
    }

    rows
}

fn car_mintime_constraint_bounds(
    rows: &[CarMintimeConstraintRow],
    params: CarDoubleTrackParams,
    options: &CarMintimeSolveOptions,
) -> (Vec<f64>, Vec<f64>) {
    let mut lower = Vec::with_capacity(rows.len());
    let mut upper = Vec::with_capacity(rows.len());

    for row in rows {
        match row {
            CarMintimeConstraintRow::CollocationDynamics { .. }
            | CarMintimeConstraintRow::Continuity { .. }
            | CarMintimeConstraintRow::Dynamics { .. }
            | CarMintimeConstraintRow::LateralLoadTransfer { .. } => {
                lower.push(0.0);
                upper.push(0.0);
            }
            CarMintimeConstraintRow::PowerLimit { .. }
            | CarMintimeConstraintRow::CollocationPowerLimit { .. } => {
                lower.push(f64::NEG_INFINITY);
                upper.push(params.power_max_w);
            }
            CarMintimeConstraintRow::NormalLoad { .. }
            | CarMintimeConstraintRow::CollocationNormalLoad { .. } => {
                lower.push(0.0);
                upper.push(f64::INFINITY);
            }
            CarMintimeConstraintRow::TireEllipse { .. }
            | CarMintimeConstraintRow::CollocationTireEllipse { .. } => {
                lower.push(0.0);
                upper.push(1.0);
            }
            CarMintimeConstraintRow::SlipPrepeak { .. }
            | CarMintimeConstraintRow::CollocationSlipPrepeak { .. } => {
                let margin = options.prepeak_grip_margin;
                lower.push(-margin);
                upper.push(margin);
            }
            CarMintimeConstraintRow::DriveBrakeMutex { .. } => {
                lower.push(CAR_DRIVE_BRAKE_MUTEX_LOWER_N2);
                upper.push(0.0);
            }
            CarMintimeConstraintRow::ControlRate { control_name, .. } => match *control_name {
                "delta_rad" => {
                    lower.push(-params.steering_angle_max_rad / params.steering_response_s);
                    upper.push(params.steering_angle_max_rad / params.steering_response_s);
                }
                "f_drive_N" => {
                    lower.push(f64::NEG_INFINITY);
                    upper.push(params.drive_force_max_n / params.throttle_response_s);
                }
                "f_brake_N" => {
                    lower.push(-params.brake_force_max_n / params.brake_response_s);
                    upper.push(f64::INFINITY);
                }
                "gamma_y_N" => {
                    lower.push(f64::NEG_INFINITY);
                    upper.push(f64::INFINITY);
                }
                _ => unreachable!("unknown car control {control_name}"),
            },
        }
    }

    (lower, upper)
}

fn car_mintime_jacobian_pattern(
    seed: &CarMintimeNlpSeed,
    rows: &[CarMintimeConstraintRow],
) -> Result<Vec<(i32, i32)>, SolverApiError> {
    let mut pattern = Vec::new();

    for (row_index, row) in rows.iter().enumerate() {
        let row_index = i32::try_from(row_index).map_err(|_| {
            SolverApiError::new("solve.invalidRequest", "too many car mintime constraints")
        })?;

        for col_index in car_mintime_constraint_columns(seed, row) {
            pattern.push((
                row_index,
                i32::try_from(col_index).map_err(|_| {
                    SolverApiError::new("solve.invalidRequest", "too many car mintime variables")
                })?,
            ));
        }
    }

    Ok(pattern)
}

fn car_mintime_jacobian_columns_by_variable(
    decision_variable_count: usize,
    pattern: &[(i32, i32)],
) -> Vec<CarMintimeJacobianColumnEntries> {
    let mut columns = vec![Vec::new(); decision_variable_count];

    for (pattern_index, (row_index, col_index)) in pattern.iter().copied().enumerate() {
        let Ok(row_index) = usize::try_from(row_index) else {
            continue;
        };
        let Ok(col_index) = usize::try_from(col_index) else {
            continue;
        };
        if col_index >= decision_variable_count {
            continue;
        }

        columns[col_index].push(CarMintimeJacobianEntry {
            pattern_index,
            row_index,
        });
    }

    columns
        .into_iter()
        .enumerate()
        .filter_map(|(variable_index, entries)| {
            (!entries.is_empty()).then_some(CarMintimeJacobianColumnEntries {
                variable_index,
                entries,
            })
        })
        .collect()
}

fn car_mintime_constraint_columns(
    seed: &CarMintimeNlpSeed,
    row: &CarMintimeConstraintRow,
) -> Vec<usize> {
    let mut columns = BTreeSet::new();
    match row {
        CarMintimeConstraintRow::CollocationDynamics {
            interval,
            point,
            state_name,
        } => {
            let state_index = state_index(state_name);
            columns.insert(state_offset(*interval) + state_index);
            insert_collocation_point_state_columns(seed, &mut columns, *interval, point - 1);
            insert_collocation_state_component_columns(seed, &mut columns, *interval, state_index);
            insert_control_columns(seed, &mut columns, *interval);
        }
        CarMintimeConstraintRow::Continuity {
            interval,
            state_name,
        } => {
            let state_index = state_index(state_name);
            columns.insert(state_offset(*interval) + state_index);
            columns.insert(state_offset(next_station_index(seed, *interval)) + state_index);
            insert_collocation_state_component_columns(seed, &mut columns, *interval, state_index);
        }
        CarMintimeConstraintRow::Dynamics { interval, .. } => {
            insert_state_columns(&mut columns, *interval);
            insert_state_columns(&mut columns, next_station_index(seed, *interval));
            insert_control_columns(seed, &mut columns, *interval);
        }
        CarMintimeConstraintRow::PowerLimit { interval } => {
            columns.insert(state_offset(next_station_index(seed, *interval)) + STATE_V_MPS);
            columns.insert(control_offset(seed, *interval) + CONTROL_F_DRIVE_N);
        }
        CarMintimeConstraintRow::CollocationPowerLimit { interval, point } => {
            columns.insert(collocation_state_offset(seed, *interval, point - 1) + STATE_V_MPS);
            columns.insert(control_offset(seed, *interval) + CONTROL_F_DRIVE_N);
        }
        CarMintimeConstraintRow::NormalLoad { interval, .. }
        | CarMintimeConstraintRow::TireEllipse { interval, .. }
        | CarMintimeConstraintRow::SlipPrepeak { interval, .. } => {
            insert_state_columns(&mut columns, next_station_index(seed, *interval));
            insert_control_columns(seed, &mut columns, *interval);
        }
        CarMintimeConstraintRow::CollocationNormalLoad {
            interval, point, ..
        }
        | CarMintimeConstraintRow::CollocationTireEllipse {
            interval, point, ..
        }
        | CarMintimeConstraintRow::CollocationSlipPrepeak {
            interval, point, ..
        } => {
            insert_collocation_point_state_columns(seed, &mut columns, *interval, point - 1);
            insert_control_columns(seed, &mut columns, *interval);
        }
        CarMintimeConstraintRow::LateralLoadTransfer { interval } => {
            insert_state_columns(&mut columns, next_station_index(seed, *interval));
            insert_control_columns(seed, &mut columns, *interval);
        }
        CarMintimeConstraintRow::DriveBrakeMutex { interval } => {
            columns.insert(control_offset(seed, *interval) + CONTROL_F_DRIVE_N);
            columns.insert(control_offset(seed, *interval) + CONTROL_F_BRAKE_N);
        }
        CarMintimeConstraintRow::ControlRate {
            interval,
            control_name,
        } => {
            let previous = interval.saturating_sub(1);
            columns.insert(control_offset(seed, *interval) + control_index(control_name));
            columns.insert(control_offset(seed, previous) + control_index(control_name));
            insert_state_columns(&mut columns, previous);
            insert_control_columns(seed, &mut columns, previous);
        }
    }

    columns.into_iter().collect()
}

fn car_mintime_initial_constraint_residuals(
    seed: &CarMintimeNlpSeed,
    rows: &[CarMintimeConstraintRow],
    params: CarDoubleTrackParams,
) -> Vec<f64> {
    rows.iter()
        .map(|row| car_mintime_constraint_value_from(seed, params, &seed.initial_guess, row))
        .collect()
}

fn car_mintime_constraint_value_from(
    seed: &CarMintimeNlpSeed,
    params: CarDoubleTrackParams,
    x: &[f64],
    row: &CarMintimeConstraintRow,
) -> f64 {
    match row {
        CarMintimeConstraintRow::CollocationDynamics {
            interval,
            point,
            state_name,
        } => car_mintime_collocation_dynamics_residual(
            seed,
            params,
            x,
            *interval,
            *point,
            state_index(state_name),
        ),
        CarMintimeConstraintRow::Continuity {
            interval,
            state_name,
        } => {
            car_mintime_collocation_continuity_residual(seed, x, *interval, state_index(state_name))
        }
        CarMintimeConstraintRow::Dynamics {
            interval,
            state_name,
        } => car_mintime_dynamics_residual(seed, params, x, *interval, state_index(state_name)),
        CarMintimeConstraintRow::PowerLimit { interval } => {
            car_state_from(seed, x, next_station_index(seed, *interval)).v_mps
                * car_control_from(seed, x, *interval).f_drive_n
        }
        CarMintimeConstraintRow::CollocationPowerLimit { interval, point } => {
            collocation_state_from(seed, x, *interval, point - 1).v_mps
                * car_control_from(seed, x, *interval).f_drive_n
        }
        CarMintimeConstraintRow::NormalLoad { interval, wheel } => {
            let dynamics = car_mintime_path_dynamics_from(seed, params, x, *interval);
            let (_, _, fz_n, _, _) = car_wheel_force_values(dynamics.tire_forces, params, wheel);
            fz_n
        }
        CarMintimeConstraintRow::CollocationNormalLoad {
            interval,
            point,
            wheel,
        } => {
            let dynamics =
                car_mintime_collocation_dynamics_from(seed, params, x, *interval, *point);
            let (_, _, fz_n, _, _) = car_wheel_force_values(dynamics.tire_forces, params, wheel);
            fz_n
        }
        CarMintimeConstraintRow::TireEllipse { interval, wheel } => {
            let dynamics = car_mintime_path_dynamics_from(seed, params, x, *interval);
            dynamics
                .tire_forces
                .wheel_ellipse_utilization(params, wheel)
        }
        CarMintimeConstraintRow::CollocationTireEllipse {
            interval,
            point,
            wheel,
        } => {
            let dynamics =
                car_mintime_collocation_dynamics_from(seed, params, x, *interval, *point);
            dynamics
                .tire_forces
                .wheel_ellipse_utilization(params, wheel)
        }
        CarMintimeConstraintRow::SlipPrepeak { interval, wheel } => {
            let dynamics = car_mintime_path_dynamics_from(seed, params, x, *interval);
            car_wheel_slip_rad(dynamics.tire_forces, wheel)
                / car_pacejka_peak_slip_rad(params, wheel)
        }
        CarMintimeConstraintRow::CollocationSlipPrepeak {
            interval,
            point,
            wheel,
        } => {
            let dynamics =
                car_mintime_collocation_dynamics_from(seed, params, x, *interval, *point);
            car_wheel_slip_rad(dynamics.tire_forces, wheel)
                / car_pacejka_peak_slip_rad(params, wheel)
        }
        CarMintimeConstraintRow::LateralLoadTransfer { interval } => {
            lateral_load_transfer_path_residual_from(seed, params, x, *interval)
        }
        CarMintimeConstraintRow::DriveBrakeMutex { interval } => {
            let control = car_control_from(seed, x, *interval);
            control.f_drive_n * control.f_brake_n
        }
        CarMintimeConstraintRow::ControlRate {
            interval,
            control_name,
        } => {
            let current = control_value_from(seed, x, *interval, control_index(control_name));
            let previous = control_value_from(
                seed,
                x,
                interval.saturating_sub(1),
                control_index(control_name),
            );
            let ds = interval_ds_m(seed, interval.saturating_sub(1)).max(1e-6);
            let sigma = sigma_dt_ds_from(seed, params, x, interval.saturating_sub(1));

            (current - previous) / (ds * sigma).max(1e-6)
        }
    }
}

fn car_mintime_constraint_derivative_structured(
    seed: &CarMintimeNlpSeed,
    params: CarDoubleTrackParams,
    row: &CarMintimeConstraintRow,
    x: &[f64],
    variable_index: usize,
) -> CarMintimeStructuredDerivative {
    match row {
        CarMintimeConstraintRow::CollocationDynamics {
            interval,
            point,
            state_name,
        } => {
            let state_index = state_index(state_name);
            let scale = CAR_STATE_SCALE[state_index];
            let coefficients = car_legendre_collocation_coefficients_degree3();
            let mut analytic = 0.0;

            if variable_index == state_offset(*interval) + state_index {
                analytic -= coefficients.c[0][*point] / scale;
            }

            for collocation_point in 0..CAR_COLLOCATION_DEGREE {
                if variable_index
                    == collocation_state_offset(seed, *interval, collocation_point) + state_index
                {
                    analytic -= coefficients.c[collocation_point + 1][*point] / scale;
                }
            }

            let collocation_point = point - 1;
            let collocation_offset = collocation_state_offset(seed, *interval, collocation_point);
            let control_offset = control_offset(seed, *interval);
            let numeric_term = if (collocation_offset..collocation_offset + CAR_STATE_LEN)
                .contains(&variable_index)
                || (control_offset..control_offset + CAR_CONTROL_LEN).contains(&variable_index)
            {
                CarMintimeNumericDerivativeTerm::CollocationDynamicsRhs {
                    interval: *interval,
                    point: *point,
                    state_index,
                }
            } else {
                CarMintimeNumericDerivativeTerm::None
            };

            CarMintimeStructuredDerivative {
                analytic,
                numeric_term,
            }
        }
        CarMintimeConstraintRow::Continuity {
            interval,
            state_name,
        } => {
            let state_index = state_index(state_name);
            let scale = CAR_STATE_SCALE[state_index];
            let coefficients = car_legendre_collocation_coefficients_degree3();

            if variable_index == state_offset(*interval) + state_index {
                return CarMintimeStructuredDerivative {
                    analytic: coefficients.d[0] / scale,
                    numeric_term: CarMintimeNumericDerivativeTerm::None,
                };
            }

            for collocation_point in 0..CAR_COLLOCATION_DEGREE {
                if variable_index
                    == collocation_state_offset(seed, *interval, collocation_point) + state_index
                {
                    return CarMintimeStructuredDerivative {
                        analytic: coefficients.d[collocation_point + 1] / scale,
                        numeric_term: CarMintimeNumericDerivativeTerm::None,
                    };
                }
            }

            let analytic = if variable_index
                == state_offset(next_station_index(seed, *interval)) + state_index
            {
                -1.0 / scale
            } else {
                0.0
            };

            CarMintimeStructuredDerivative {
                analytic,
                numeric_term: CarMintimeNumericDerivativeTerm::None,
            }
        }
        CarMintimeConstraintRow::PowerLimit { interval } => {
            let speed_index = state_offset(next_station_index(seed, *interval)) + STATE_V_MPS;
            let drive_index = control_offset(seed, *interval) + CONTROL_F_DRIVE_N;
            let analytic = if variable_index == speed_index {
                control_value_from(seed, x, *interval, CONTROL_F_DRIVE_N)
            } else if variable_index == drive_index {
                state_value_from(seed, x, next_station_index(seed, *interval), STATE_V_MPS)
            } else {
                0.0
            };

            CarMintimeStructuredDerivative {
                analytic,
                numeric_term: CarMintimeNumericDerivativeTerm::None,
            }
        }
        CarMintimeConstraintRow::CollocationPowerLimit { interval, point } => {
            let speed_index = collocation_state_offset(seed, *interval, point - 1) + STATE_V_MPS;
            let drive_index = control_offset(seed, *interval) + CONTROL_F_DRIVE_N;
            let analytic = if variable_index == speed_index {
                control_value_from(seed, x, *interval, CONTROL_F_DRIVE_N)
            } else if variable_index == drive_index {
                collocation_state_from(seed, x, *interval, point - 1).v_mps
            } else {
                0.0
            };

            CarMintimeStructuredDerivative {
                analytic,
                numeric_term: CarMintimeNumericDerivativeTerm::None,
            }
        }
        CarMintimeConstraintRow::DriveBrakeMutex { interval } => {
            let drive_index = control_offset(seed, *interval) + CONTROL_F_DRIVE_N;
            let brake_index = control_offset(seed, *interval) + CONTROL_F_BRAKE_N;
            let analytic = if variable_index == drive_index {
                control_value_from(seed, x, *interval, CONTROL_F_BRAKE_N)
            } else if variable_index == brake_index {
                control_value_from(seed, x, *interval, CONTROL_F_DRIVE_N)
            } else {
                0.0
            };

            CarMintimeStructuredDerivative {
                analytic,
                numeric_term: CarMintimeNumericDerivativeTerm::None,
            }
        }
        CarMintimeConstraintRow::LateralLoadTransfer { interval } => {
            let gamma_index = control_offset(seed, *interval) + CONTROL_GAMMA_Y_N;
            let analytic = if variable_index == gamma_index {
                -1.0
            } else {
                0.0
            };

            CarMintimeStructuredDerivative {
                analytic,
                numeric_term: CarMintimeNumericDerivativeTerm::LateralLoadTransferVehiclePart {
                    interval: *interval,
                },
            }
        }
        CarMintimeConstraintRow::ControlRate {
            interval,
            control_name,
        } => {
            let control_index = control_index(control_name);
            let previous = interval.saturating_sub(1);
            let current_index = control_offset(seed, *interval) + control_index;
            let previous_index = control_offset(seed, previous) + control_index;
            let current = control_value_from(seed, x, *interval, control_index);
            let previous_value = control_value_from(seed, x, previous, control_index);
            let ds = interval_ds_m(seed, previous).max(1e-6);
            let sigma = sigma_dt_ds_from(seed, params, x, previous);
            let denominator = (ds * sigma).max(1e-6);
            let mut analytic = 0.0;

            if variable_index == current_index {
                analytic += 1.0 / denominator;
            }
            if variable_index == previous_index {
                analytic -= 1.0 / denominator;
            }

            let state_offset = state_offset(previous);
            let control_offset = control_offset(seed, previous);
            let numeric_term = if (state_offset..state_offset + CAR_STATE_LEN)
                .contains(&variable_index)
                || (control_offset..control_offset + CAR_CONTROL_LEN).contains(&variable_index)
            {
                CarMintimeNumericDerivativeTerm::ControlRateSigma {
                    interval: previous,
                    numerator: current - previous_value,
                    ds,
                }
            } else {
                CarMintimeNumericDerivativeTerm::None
            };

            CarMintimeStructuredDerivative {
                analytic,
                numeric_term,
            }
        }
        _ => CarMintimeStructuredDerivative {
            analytic: 0.0,
            numeric_term: CarMintimeNumericDerivativeTerm::FullConstraint,
        },
    }
}

fn car_mintime_numeric_derivative_term_value(
    seed: &CarMintimeNlpSeed,
    params: CarDoubleTrackParams,
    x: &[f64],
    row: &CarMintimeConstraintRow,
    term: CarMintimeNumericDerivativeTerm,
) -> f64 {
    match term {
        CarMintimeNumericDerivativeTerm::None => 0.0,
        CarMintimeNumericDerivativeTerm::FullConstraint => {
            car_mintime_constraint_value_from(seed, params, x, row)
        }
        CarMintimeNumericDerivativeTerm::CollocationDynamicsRhs {
            interval,
            point,
            state_index,
        } => {
            car_mintime_collocation_dynamics_rhs_norm(seed, params, x, interval, point, state_index)
        }
        CarMintimeNumericDerivativeTerm::ControlRateSigma {
            interval,
            numerator,
            ds,
        } => {
            let sigma = sigma_dt_ds_from(seed, params, x, interval);
            numerator / (ds * sigma).max(1e-6)
        }
        CarMintimeNumericDerivativeTerm::LateralLoadTransferVehiclePart { interval } => {
            lateral_load_transfer_vehicle_part_from(seed, params, x, interval)
        }
    }
}

fn car_mintime_dynamics_residual(
    seed: &CarMintimeNlpSeed,
    params: CarDoubleTrackParams,
    x: &[f64],
    interval: usize,
    state_index: usize,
) -> f64 {
    let next = next_station_index(seed, interval);
    let next_value = state_value_from(seed, x, next, state_index);
    let current_value = state_value_from(seed, x, interval, state_index);
    let ds = interval_ds_m(seed, interval);
    let dynamics = car_mintime_dynamics_from(seed, params, x, interval);

    let derivative = match state_index {
        STATE_V_MPS => dynamics.dv_ds,
        STATE_BETA_RAD => dynamics.dbeta_ds,
        STATE_OMEGA_Z_RADPS => dynamics.domega_z_ds,
        STATE_N_M => dynamics.dn_ds,
        STATE_XI_RAD => dynamics.dxi_ds,
        _ => 0.0,
    };

    next_value - current_value - ds * derivative
}

fn car_mintime_collocation_dynamics_residual(
    seed: &CarMintimeNlpSeed,
    params: CarDoubleTrackParams,
    x: &[f64],
    interval: usize,
    point: usize,
    state_index: usize,
) -> f64 {
    let coefficients = car_legendre_collocation_coefficients_degree3();
    let ds = interval_ds_m(seed, interval);
    let dynamics = car_mintime_collocation_dynamics_from(seed, params, x, interval, point);
    let derivative_norm = car_normalized_derivative(dynamics);
    let mut xp_norm =
        coefficients.c[0][point] * state_norm_value_from(seed, x, interval, state_index);
    for collocation_point in 0..CAR_COLLOCATION_DEGREE {
        xp_norm += coefficients.c[collocation_point + 1][point]
            * collocation_state_norm_value_from(seed, x, interval, collocation_point, state_index);
    }

    ds * derivative_norm[state_index] - xp_norm
}

fn car_mintime_collocation_dynamics_rhs_norm(
    seed: &CarMintimeNlpSeed,
    params: CarDoubleTrackParams,
    x: &[f64],
    interval: usize,
    point: usize,
    state_index: usize,
) -> f64 {
    let ds = interval_ds_m(seed, interval);
    let dynamics = car_mintime_collocation_dynamics_from(seed, params, x, interval, point);
    ds * car_normalized_derivative(dynamics)[state_index]
}

fn car_mintime_collocation_continuity_residual(
    seed: &CarMintimeNlpSeed,
    x: &[f64],
    interval: usize,
    state_index: usize,
) -> f64 {
    let coefficients = car_legendre_collocation_coefficients_degree3();
    let mut end_state_norm =
        coefficients.d[0] * state_norm_value_from(seed, x, interval, state_index);
    for collocation_point in 0..CAR_COLLOCATION_DEGREE {
        end_state_norm += coefficients.d[collocation_point + 1]
            * collocation_state_norm_value_from(seed, x, interval, collocation_point, state_index);
    }

    end_state_norm - state_norm_value_from(seed, x, next_station_index(seed, interval), state_index)
}

fn car_mintime_collocation_objective_s(
    seed: &CarMintimeNlpSeed,
    params: CarDoubleTrackParams,
    x: &[f64],
) -> f64 {
    (0..seed.dimensions.interval_count)
        .map(|interval| {
            (1..=CAR_COLLOCATION_DEGREE)
                .map(|point| {
                    car_mintime_collocation_objective_term_s(seed, params, x, interval, point)
                })
                .sum::<f64>()
        })
        .sum()
}

fn car_mintime_collocation_objective_term_s(
    seed: &CarMintimeNlpSeed,
    params: CarDoubleTrackParams,
    x: &[f64],
    interval: usize,
    point: usize,
) -> f64 {
    let coefficients = car_legendre_collocation_coefficients_degree3();
    let ds = interval_ds_m(seed, interval);
    let dynamics = car_mintime_collocation_dynamics_from(seed, params, x, interval, point);

    ds * coefficients.b[point] * dynamics.sigma_dt_ds
}

fn car_mintime_regularization_objective_s(
    seed: &CarMintimeNlpSeed,
    params: CarDoubleTrackParams,
    weights: CarMintimeObjectiveWeights,
    x: &[f64],
) -> f64 {
    let count = seed.dimensions.interval_count;
    if count == 0 {
        return 0.0;
    }

    let delta = control_series(seed, x, CONTROL_DELTA_RAD, 1.0);
    // Penalize steering as a curvature command so the same weight is comparable
    // between kart, GT and formula wheelbases.
    let steering_curvature = steering_curvature_regularization_series(params, &delta);
    let force = drive_brake_regularization_series(seed, x);
    let n = state_series(x, count, STATE_N_M, 1.0);
    let xi = state_series(x, count, STATE_XI_RAD, 1.0);
    let closed = seed_is_closed(seed);

    weights.penalty_delta * first_difference_squared(&steering_curvature, closed)
        + weights.penalty_f * first_difference_squared(&force, closed)
        + weights.penalty_delta_dd * second_difference_squared(&steering_curvature, closed)
        + weights.penalty_f_dd * second_difference_squared(&force, closed)
        + weights.penalty_n_dd * second_difference_squared(&n, closed)
        + weights.penalty_xi_dd * second_difference_squared(&xi, closed)
        + car_endpoint_c1_dn_objective_s(seed, weights, x)
        + car_endpoint_c1_heading_objective_s(seed, weights, x)
        + car_endpoint_heading_jump_objective_s(seed, weights, x)
        + car_endpoint_d2n_jump_objective_s(seed, weights, x)
}

fn car_endpoint_c1_dn_objective_s(
    seed: &CarMintimeNlpSeed,
    weights: CarMintimeObjectiveWeights,
    x: &[f64],
) -> f64 {
    if weights.penalty_endpoint_c1_dn <= 0.0 || seed.dimensions.interval_count == 0 {
        return 0.0;
    }

    (0..seed.dimensions.interval_count)
        .filter(|interval| seed_is_closed(seed) || *interval + 1 < seed.dimensions.station_count)
        .map(|interval| car_endpoint_c1_dn_objective_term_s(seed, weights, x, interval))
        .sum()
}

fn car_endpoint_c1_dn_objective_term_s(
    seed: &CarMintimeNlpSeed,
    weights: CarMintimeObjectiveWeights,
    x: &[f64],
    left_interval: usize,
) -> f64 {
    if weights.penalty_endpoint_c1_dn <= 0.0 {
        return 0.0;
    }
    let residuals = car_endpoint_continuity_residuals(seed, x, left_interval);
    let normalizer = seed.dimensions.interval_count.max(1) as f64;
    let scale = weights.endpoint_c1_dn_scale.max(1e-9);
    let left = residuals.c1_kin_left / scale;
    let right = residuals.c1_kin_right / scale;
    weights.penalty_endpoint_c1_dn * (left * left + right * right) / normalizer
}

fn car_endpoint_c1_heading_objective_s(
    seed: &CarMintimeNlpSeed,
    weights: CarMintimeObjectiveWeights,
    x: &[f64],
) -> f64 {
    if weights.penalty_endpoint_c1_heading <= 0.0 || seed.dimensions.interval_count == 0 {
        return 0.0;
    }

    (0..seed.dimensions.interval_count)
        .filter(|interval| seed_is_closed(seed) || *interval + 1 < seed.dimensions.station_count)
        .map(|interval| car_endpoint_c1_heading_objective_term_s(seed, weights, x, interval))
        .sum()
}

fn car_endpoint_c1_heading_objective_term_s(
    seed: &CarMintimeNlpSeed,
    weights: CarMintimeObjectiveWeights,
    x: &[f64],
    left_interval: usize,
) -> f64 {
    if weights.penalty_endpoint_c1_heading <= 0.0 {
        return 0.0;
    }
    let residuals = car_endpoint_continuity_residuals(seed, x, left_interval);
    let normalizer = seed.dimensions.interval_count.max(1) as f64;
    let scale = weights.endpoint_c1_heading_scale_rad.max(1e-9);
    let left = residuals.c1_heading_left_rad / scale;
    let right = residuals.c1_heading_right_rad / scale;
    weights.penalty_endpoint_c1_heading * (left * left + right * right) / normalizer
}

fn car_endpoint_heading_jump_objective_s(
    seed: &CarMintimeNlpSeed,
    weights: CarMintimeObjectiveWeights,
    x: &[f64],
) -> f64 {
    if weights.penalty_endpoint_heading_jump <= 0.0 || seed.dimensions.interval_count == 0 {
        return 0.0;
    }

    (0..seed.dimensions.interval_count)
        .filter(|interval| seed_is_closed(seed) || *interval + 1 < seed.dimensions.station_count)
        .map(|interval| car_endpoint_heading_jump_objective_term_s(seed, weights, x, interval))
        .sum()
}

fn car_endpoint_heading_jump_objective_term_s(
    seed: &CarMintimeNlpSeed,
    weights: CarMintimeObjectiveWeights,
    x: &[f64],
    left_interval: usize,
) -> f64 {
    if weights.penalty_endpoint_heading_jump <= 0.0 {
        return 0.0;
    }
    let residuals = car_endpoint_continuity_residuals(seed, x, left_interval);
    let normalizer = seed.dimensions.interval_count.max(1) as f64;
    let scale = weights.endpoint_heading_jump_scale_rad.max(1e-9);
    let jump = residuals.heading_jump_rad / scale;
    weights.penalty_endpoint_heading_jump * jump * jump / normalizer
}

fn car_endpoint_d2n_jump_objective_s(
    seed: &CarMintimeNlpSeed,
    weights: CarMintimeObjectiveWeights,
    x: &[f64],
) -> f64 {
    if weights.penalty_endpoint_d2n_jump <= 0.0 || seed.dimensions.interval_count == 0 {
        return 0.0;
    }

    (0..seed.dimensions.interval_count)
        .filter(|interval| seed_is_closed(seed) || *interval + 1 < seed.dimensions.station_count)
        .map(|interval| car_endpoint_d2n_jump_objective_term_s(seed, weights, x, interval))
        .sum()
}

fn car_endpoint_d2n_jump_objective_term_s(
    seed: &CarMintimeNlpSeed,
    weights: CarMintimeObjectiveWeights,
    x: &[f64],
    left_interval: usize,
) -> f64 {
    if weights.penalty_endpoint_d2n_jump <= 0.0 {
        return 0.0;
    }
    let right_interval = next_station_index(seed, left_interval)
        .min(seed.dimensions.interval_count.saturating_sub(1));
    let left_d2s = car_collocation_state_second_derivatives_at_tau(seed, x, left_interval, 1.0);
    let right_d2s = car_collocation_state_second_derivatives_at_tau(seed, x, right_interval, 0.0);
    let normalizer = seed.dimensions.interval_count.max(1) as f64;
    let scale = weights.endpoint_d2n_jump_scale.max(1e-9);
    let jump = (right_d2s.n_m - left_d2s.n_m) / scale;
    weights.penalty_endpoint_d2n_jump * jump * jump / normalizer
}

fn sigma_dt_ds_from(
    seed: &CarMintimeNlpSeed,
    params: CarDoubleTrackParams,
    x: &[f64],
    interval: usize,
) -> f64 {
    car_mintime_dynamics_from(seed, params, x, interval).sigma_dt_ds
}

fn car_mintime_dynamics_from(
    seed: &CarMintimeNlpSeed,
    params: CarDoubleTrackParams,
    x: &[f64],
    interval: usize,
) -> crate::vehicle_dynamics::CarDoubleTrackDynamics {
    car_mintime_dynamics_for_state_control(seed, params, x, interval, interval, interval)
}

fn car_mintime_path_dynamics_from(
    seed: &CarMintimeNlpSeed,
    params: CarDoubleTrackParams,
    x: &[f64],
    interval: usize,
) -> crate::vehicle_dynamics::CarDoubleTrackDynamics {
    let state_interval = next_station_index(seed, interval);
    car_mintime_dynamics_for_state_control(
        seed,
        params,
        x,
        state_interval,
        interval,
        state_interval,
    )
}

fn car_mintime_station_dynamics_from(
    seed: &CarMintimeNlpSeed,
    params: CarDoubleTrackParams,
    x: &[f64],
    station: usize,
) -> CarDoubleTrackDynamics {
    let control_interval = car_station_publication_interval(seed, station);
    car_mintime_dynamics_for_state_control(seed, params, x, station, control_interval, station)
}

fn car_station_publication_interval(seed: &CarMintimeNlpSeed, station: usize) -> usize {
    let interval_count = seed.dimensions.interval_count;
    if interval_count <= 1 || station == 0 && !seed_is_closed(seed) {
        return 0;
    }
    if station == 0 {
        interval_count - 1
    } else {
        (station - 1).min(interval_count - 1)
    }
}

fn car_mintime_dynamics_for_state_control(
    seed: &CarMintimeNlpSeed,
    params: CarDoubleTrackParams,
    x: &[f64],
    state_interval: usize,
    control_interval: usize,
    geometry_interval: usize,
) -> crate::vehicle_dynamics::CarDoubleTrackDynamics {
    let state = car_state_from(seed, x, state_interval);
    let geometry = station_sections_geometry(seed, geometry_interval);
    car_mintime_dynamics_with_sections_geometry(
        params,
        state,
        car_control_from(seed, x, control_interval),
        geometry,
    )
}

fn car_mintime_collocation_dynamics_from(
    seed: &CarMintimeNlpSeed,
    params: CarDoubleTrackParams,
    x: &[f64],
    interval: usize,
    point: usize,
) -> crate::vehicle_dynamics::CarDoubleTrackDynamics {
    let coefficients = car_legendre_collocation_coefficients_degree3();
    let geometry = interpolated_sections_geometry(seed, interval, coefficients.tau[point]);
    car_mintime_dynamics_with_sections_geometry(
        params,
        collocation_state_from(seed, x, interval, point - 1),
        car_control_from(seed, x, interval),
        geometry,
    )
}

fn car_mintime_dynamics_with_sections_geometry(
    params: CarDoubleTrackParams,
    state: CarDoubleTrackState,
    control: CarDoubleTrackControl,
    geometry: InterpolatedSectionsGeometry,
) -> crate::vehicle_dynamics::CarDoubleTrackDynamics {
    let mut dynamics = car_double_track_dynamics(params, state, control, geometry.kappa_1pm);
    let base_sigma = dynamics.sigma_dt_ds.max(1e-9);
    let section_progress = section_frame_progress_from_derivatives(
        state.n_m,
        state.v_mps,
        state.beta_rad,
        state.xi_rad,
        geometry.ref_tangent_xy,
        geometry.ref_left_normal_xy,
        geometry.centerline_derivative_xy,
        geometry.section_dir_xy,
        geometry.section_dir_derivative_xy,
    );
    let sections_sigma = section_progress.sigma_dt_ds;
    let scale = sections_sigma / base_sigma;

    dynamics.dv_ds *= scale;
    dynamics.dbeta_ds *= scale;
    dynamics.domega_z_ds *= scale;
    dynamics.dn_ds = section_progress.dn_ds;
    dynamics.dxi_ds = sections_sigma * state.omega_z_radps - geometry.heading_rate_per_s;
    dynamics.sigma_dt_ds = sections_sigma;
    dynamics
}

fn car_normalized_derivative(
    dynamics: crate::vehicle_dynamics::CarDoubleTrackDynamics,
) -> [f64; CAR_STATE_LEN] {
    [
        dynamics.dv_ds / CAR_STATE_SCALE[STATE_V_MPS],
        dynamics.dbeta_ds / CAR_STATE_SCALE[STATE_BETA_RAD],
        dynamics.domega_z_ds / CAR_STATE_SCALE[STATE_OMEGA_Z_RADPS],
        dynamics.dn_ds / CAR_STATE_SCALE[STATE_N_M],
        dynamics.dxi_ds / CAR_STATE_SCALE[STATE_XI_RAD],
    ]
}

fn lateral_load_transfer_path_residual_from(
    seed: &CarMintimeNlpSeed,
    params: CarDoubleTrackParams,
    x: &[f64],
    interval: usize,
) -> f64 {
    let control = car_control_from(seed, x, interval);
    let dynamics = car_mintime_path_dynamics_from(seed, params, x, interval);
    lateral_load_transfer_residual_for(params, control, dynamics.tire_forces)
}

fn lateral_load_transfer_vehicle_part_from(
    seed: &CarMintimeNlpSeed,
    params: CarDoubleTrackParams,
    x: &[f64],
    interval: usize,
) -> f64 {
    let control = car_control_from(seed, x, interval);
    lateral_load_transfer_path_residual_from(seed, params, x, interval) + control.gamma_y_n
}

fn lateral_load_transfer_residual_for(
    params: CarDoubleTrackParams,
    control: CarDoubleTrackControl,
    tire: CarDoubleTrackTireForces,
) -> f64 {
    let front_lateral = (tire.fy_fl_n + tire.fy_fr_n) * control.delta_rad.cos()
        + (tire.fx_fl_n + tire.fx_fr_n) * control.delta_rad.sin();
    let rear_lateral = tire.fy_rl_n + tire.fy_rr_n;
    let average_track_width_m =
        ((params.track_width_front_m + params.track_width_rear_m) / 2.0).max(1e-9);

    (front_lateral + rear_lateral) * params.cg_height_m / average_track_width_m - control.gamma_y_n
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct CarCollocationCoefficients {
    tau: [f64; CAR_COLLOCATION_DEGREE + 1],
    c: [[f64; CAR_COLLOCATION_DEGREE + 1]; CAR_COLLOCATION_DEGREE + 1],
    d: [f64; CAR_COLLOCATION_DEGREE + 1],
    b: [f64; CAR_COLLOCATION_DEGREE + 1],
}

fn car_legendre_collocation_coefficients_degree3() -> CarCollocationCoefficients {
    CarCollocationCoefficients {
        tau: [0.0, 0.112_701_665_379_258_3, 0.5, 0.887_298_334_620_741_7],
        c: [
            [
                -12.000_000_000_000_004,
                -6.000_000_000_000_004,
                2.999_999_999_999_996_4,
                -6.000_000_000_000_012,
            ],
            [
                13.121_638_910_345_695,
                5.000_000_000_000_002,
                -5.727_486_121_839_512,
                10.163_977_794_943_227,
            ],
            [
                -1.333_333_333_333_333_3,
                1.163_977_794_943_223,
                2.000_000_000_000_002_7,
                -9.163_977_794_943_216,
            ],
            [
                0.211_694_422_987_638_52,
                -0.163_977_794_943_222_5,
                0.727_486_121_839_514_1,
                5.0,
            ],
        ],
        d: [
            -1.000_000_000_000_003_6,
            1.666_666_666_666_671_4,
            -1.333_333_333_333_329_7,
            1.666_666_666_666_666_7,
        ],
        b: [
            0.0,
            0.277_777_777_777_778_57,
            0.444_444_444_444_444_2,
            0.277_777_777_777_777_85,
        ],
    }
}

type InterpolatedSectionsGeometry = SectionFrameGeometry;

fn interpolated_sections_geometry(
    seed: &CarMintimeNlpSeed,
    interval: usize,
    tau: f64,
) -> InterpolatedSectionsGeometry {
    SectionFrameMapView {
        station_s_m: &seed.station_s_m,
        centerline_xy_m: &seed.centerline_xy_m,
        tangent_xy: &seed.ref_tangent_xy,
        section_dir_xy: &seed.section_dir_xy,
        section_dir_derivative_xy: &seed.section_dir_derivative_xy,
        closed: seed_is_closed(seed),
    }
    .sample_at_interval_tau(interval, tau)
    .expect("valid car section-frame geometry")
}

fn station_sections_geometry(
    seed: &CarMintimeNlpSeed,
    station: usize,
) -> InterpolatedSectionsGeometry {
    let interval_count = seed.dimensions.interval_count;
    debug_assert!(interval_count > 0);
    if station < interval_count {
        interpolated_sections_geometry(seed, station, 0.0)
    } else {
        interpolated_sections_geometry(seed, interval_count - 1, 1.0)
    }
}

fn lerp(from: f64, to: f64, t: f64) -> f64 {
    from + (to - from) * t
}

fn lerp_point(from: Point2, to: Point2, t: f64) -> Point2 {
    [lerp(from[0], to[0], t), lerp(from[1], to[1], t)]
}

fn interval_ds_m(seed: &CarMintimeNlpSeed, interval: usize) -> f64 {
    let current = seed.station_s_m[interval];
    if let Some(next) = seed.station_s_m.get(interval + 1).copied() {
        if next > current {
            return next - current;
        }
    }

    let next = seed.station_s_m[next_station_index(seed, interval)];
    if next > current {
        return next - current;
    }

    let last = seed.station_s_m.last().copied().unwrap_or(current);
    let median_ds = median_positive_station_step_m(&seed.station_s_m).unwrap_or(0.0);
    (last - current + median_ds).max(median_ds)
}

fn median_positive_station_step_m(station_s_m: &[f64]) -> Option<f64> {
    let mut deltas = station_s_m
        .windows(2)
        .map(|window| window[1] - window[0])
        .filter(|value| value.is_finite() && *value > 0.0)
        .collect::<Vec<_>>();
    if deltas.is_empty() {
        return None;
    }
    deltas.sort_by(|left, right| left.partial_cmp(right).unwrap_or(std::cmp::Ordering::Equal));
    let midpoint = deltas.len() / 2;
    Some(if deltas.len() % 2 == 0 {
        (deltas[midpoint - 1] + deltas[midpoint]) / 2.0
    } else {
        deltas[midpoint]
    })
}

fn next_station_index(seed: &CarMintimeNlpSeed, interval: usize) -> usize {
    if seed_is_closed(seed) {
        (interval + 1) % seed.dimensions.station_count.max(1)
    } else {
        (interval + 1).min(seed.dimensions.station_count.saturating_sub(1))
    }
}

fn seed_is_closed(seed: &CarMintimeNlpSeed) -> bool {
    seed.dimensions.interval_count == seed.dimensions.station_count
}

fn worst_constraint_residual(
    constraints: &[CarMintimeConstraintRow],
    residuals: &[f64],
) -> (f64, String) {
    constraints
        .iter()
        .zip(residuals.iter())
        .map(|(row, residual)| (residual.abs(), row.label()))
        .max_by(|left, right| {
            left.0
                .partial_cmp(&right.0)
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .unwrap_or((0.0, "none".to_owned()))
}

fn max_constraint_bound_violation_for_values(
    constraints: &[CarMintimeConstraintRow],
    values: &[f64],
    lower_bounds: &[f64],
    upper_bounds: &[f64],
) -> (f64, String) {
    constraints
        .iter()
        .zip(values)
        .zip(lower_bounds.iter().zip(upper_bounds))
        .map(|((row, value), (lower, upper))| {
            let lower_violation = if lower.is_finite() && *value < *lower {
                *lower - *value
            } else {
                0.0
            };
            let upper_violation = if upper.is_finite() && *value > *upper {
                *value - *upper
            } else {
                0.0
            };
            (
                lower_violation.max(upper_violation),
                format!(
                    "{}: value={value:.9}, lower={lower:.9}, upper={upper:.9}",
                    row.label()
                ),
            )
        })
        .max_by(|left, right| {
            left.0
                .partial_cmp(&right.0)
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .unwrap_or((0.0, "none".to_owned()))
}

#[derive(Clone, Debug, PartialEq)]
struct ConstraintBoundViolationReportRow {
    violation: f64,
    abs_value: f64,
    family: &'static str,
    label: String,
    value: f64,
    lower: f64,
    upper: f64,
}

impl ConstraintBoundViolationReportRow {
    fn format_compact(&self) -> String {
        format!(
            "{}({}): violation={:.9}, abs_value={:.9}, value={:.9}, lower={:.9}, upper={:.9}",
            self.label,
            self.family,
            self.violation,
            self.abs_value,
            self.value,
            self.lower,
            self.upper
        )
    }
}

#[derive(Clone, Debug, PartialEq)]
struct ConstraintBoundViolationContextRow {
    violation: ConstraintBoundViolationReportRow,
    row: CarMintimeConstraintRow,
}

fn constraint_bound_violation_report_rows_for_values(
    constraints: &[CarMintimeConstraintRow],
    values: &[f64],
    lower_bounds: &[f64],
    upper_bounds: &[f64],
) -> Vec<ConstraintBoundViolationContextRow> {
    let mut rows = constraints
        .iter()
        .zip(values)
        .zip(lower_bounds.iter().zip(upper_bounds))
        .map(|((row, value), (lower, upper))| {
            let lower_violation = if lower.is_finite() && *value < *lower {
                *lower - *value
            } else {
                0.0
            };
            let upper_violation = if upper.is_finite() && *value > *upper {
                *value - *upper
            } else {
                0.0
            };

            ConstraintBoundViolationContextRow {
                violation: ConstraintBoundViolationReportRow {
                    violation: lower_violation.max(upper_violation),
                    abs_value: value.abs(),
                    family: row.family(),
                    label: row.label(),
                    value: *value,
                    lower: *lower,
                    upper: *upper,
                },
                row: row.clone(),
            }
        })
        .filter(|row| row.violation.violation > 0.0)
        .collect::<Vec<_>>();

    rows.sort_by(|left, right| {
        right
            .violation
            .violation
            .partial_cmp(&left.violation.violation)
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    rows
}

fn top_constraint_bound_violation_report(
    problem: &CarMintimeNlpProblem,
    x: &[f64],
    top_count: usize,
) -> String {
    let values = problem.constraint_values(x);
    let rows = constraint_bound_violation_report_rows_for_values(
        &problem.constraints,
        &values,
        &problem.constraint_lower_bounds,
        &problem.constraint_upper_bounds,
    );

    if rows.is_empty() {
        return "global_top=[]; family_worst=[]".to_owned();
    }

    let global_top = rows
        .iter()
        .take(top_count)
        .map(|row| {
            format!(
                "{}; context={}",
                row.violation.format_compact(),
                constraint_violation_context(problem, x, &row.row)
            )
        })
        .collect::<Vec<_>>()
        .join(" | ");
    let mut family_worst = Vec::<ConstraintBoundViolationContextRow>::new();
    for row in &rows {
        if !family_worst
            .iter()
            .any(|existing| existing.violation.family == row.violation.family)
        {
            family_worst.push(row.clone());
        }
    }
    let family_worst = family_worst
        .iter()
        .map(|row| {
            format!(
                "{}; context={}",
                row.violation.format_compact(),
                constraint_violation_context(problem, x, &row.row)
            )
        })
        .collect::<Vec<_>>()
        .join(" | ");

    format!("global_top=[{global_top}]; family_worst=[{family_worst}]")
}

fn car_mintime_diagnostics_json(
    problem: &CarMintimeNlpProblem,
    status: &str,
    status_code: i32,
    final_objective_s: f64,
    final_lap_time_s: f64,
    max_final_bound_violation: f64,
    worst_final_bound_violation_label: &str,
    final_bound_violation_report: &str,
    final_x: &[f64],
    trajectory: &TrajectoryResultSeriesV1,
    closed: bool,
) -> JsonValue {
    JsonValue::Object(vec![
        (
            "schema_version".to_owned(),
            "car_mintime_diagnostics.v1".into(),
        ),
        (
            "tire_envelope_contract".to_owned(),
            tire_envelope_contract_json(problem.params.tire_load_sensitivity_mode),
        ),
        (
            "formulation_contract".to_owned(),
            car_formulation_contract_json(problem),
        ),
        (
            "decision_variable_count".to_owned(),
            JsonValue::Integer(problem.decision_variable_count() as i64),
        ),
        (
            "constraint_count".to_owned(),
            JsonValue::Integer(problem.constraint_count() as i64),
        ),
        (
            "jacobian_entry_count".to_owned(),
            JsonValue::Integer(problem.jacobian_pattern.len() as i64),
        ),
        (
            "final_objective_s".to_owned(),
            json_number(final_objective_s),
        ),
        ("final_lap_time_s".to_owned(), json_number(final_lap_time_s)),
        (
            "final_residuals".to_owned(),
            JsonValue::Object(vec![
                (
                    "max_final_bound_violation".to_owned(),
                    json_number(max_final_bound_violation),
                ),
                (
                    "worst_final_bound_violation_label".to_owned(),
                    worst_final_bound_violation_label.to_owned().into(),
                ),
                (
                    "top_bound_violation_report".to_owned(),
                    final_bound_violation_report.to_owned().into(),
                ),
            ]),
        ),
        (
            "tire_envelope_summary".to_owned(),
            car_tire_envelope_summary_json(problem, final_x),
        ),
        (
            "feasibility_audit".to_owned(),
            car_feasibility_audit_json(problem, final_x),
        ),
        (
            "geometry_diagnostics".to_owned(),
            car_section_frame_geometry_diagnostics_json(problem, final_x),
        ),
        (
            "physics_bundle_v1".to_owned(),
            car_mintime_physics_bundle_json(problem, final_x, closed),
        ),
        (
            "ay_xy_consistency".to_owned(),
            ay_xy_consistency_json(trajectory, closed),
        ),
        (
            "station_trajectory_consistency_audit".to_owned(),
            car_station_trajectory_consistency_audit_json(problem, final_x, closed),
        ),
        (
            "local_station_interval_consistency_audit".to_owned(),
            car_local_station_interval_consistency_audit_json(problem, final_x, closed),
        ),
        (
            "collocation_node_reconstruction_audit".to_owned(),
            car_collocation_node_reconstruction_audit_json(&problem.seed, final_x),
        ),
        (
            "collocation_geometry_boundary_continuity_audit".to_owned(),
            car_collocation_geometry_boundary_continuity_audit_json(&problem.seed, final_x),
        ),
        (
            "ipopt".to_owned(),
            JsonValue::Object(vec![
                ("status".to_owned(), status.to_owned().into()),
                (
                    "status_code".to_owned(),
                    JsonValue::Integer(status_code as i64),
                ),
            ]),
        ),
    ])
}

fn car_mintime_physics_bundle_json(
    problem: &CarMintimeNlpProblem,
    x: &[f64],
    closed: bool,
) -> JsonValue {
    let station_columns = car_mintime_physics_bundle_columns();
    let collocation_columns = station_columns.clone();
    let final_points = (0..problem.seed.dimensions.station_count)
        .map(|station| station_xy_from(&problem.seed, x, station))
        .collect::<Vec<_>>();
    let station_rows = (0..problem.seed.dimensions.station_count)
        .map(|station| car_mintime_station_physics_row(problem, x, &final_points, station, closed))
        .collect::<Vec<_>>();
    let collocation_rows = (0..problem.seed.dimensions.interval_count)
        .flat_map(|interval| {
            (1..=CAR_COLLOCATION_DEGREE)
                .map(move |point| car_mintime_collocation_physics_row(problem, x, interval, point))
        })
        .collect::<Vec<_>>();

    JsonValue::Object(vec![
        ("schema_version".to_owned(), "car_physics_bundle_v1".into()),
        (
            "tire_envelope_contract".to_owned(),
            tire_envelope_contract_json(problem.params.tire_load_sensitivity_mode),
        ),
        (
            "formulation_contract".to_owned(),
            car_formulation_contract_json(problem),
        ),
        (
            "sample_frame".to_owned(),
            "stations and degree-3 collocation points in final physical solution".into(),
        ),
        (
            "geometry_frame".to_owned(),
            "station_hermite_coherent_frame for sample x/y; solver section geometry for dynamics"
                .into(),
        ),
        (
            "objective_split".to_owned(),
            car_mintime_objective_split_json(problem, x),
        ),
        (
            "station_columns".to_owned(),
            json_string_array(&station_columns),
        ),
        ("station_rows".to_owned(), JsonValue::Array(station_rows)),
        (
            "collocation_columns".to_owned(),
            json_string_array(&collocation_columns),
        ),
        (
            "collocation_rows".to_owned(),
            JsonValue::Array(collocation_rows),
        ),
    ])
}

fn car_mintime_objective_split_json(problem: &CarMintimeNlpProblem, x: &[f64]) -> JsonValue {
    let components = car_mintime_regularization_components_s(
        &problem.seed,
        problem.params,
        problem.objective_weights,
        x,
    );
    JsonValue::Object(vec![
        (
            "lap_time_s".to_owned(),
            json_number(car_mintime_collocation_objective_s(
                &problem.seed,
                problem.params,
                x,
            )),
        ),
        (
            "regularization_total_s".to_owned(),
            json_number(car_mintime_regularization_objective_s(
                &problem.seed,
                problem.params,
                problem.objective_weights,
                x,
            )),
        ),
        (
            "regularization_delta_s".to_owned(),
            json_number(components.delta_s),
        ),
        (
            "regularization_force_s".to_owned(),
            json_number(components.force_s),
        ),
        (
            "regularization_delta_dd_s".to_owned(),
            json_number(components.delta_dd_s),
        ),
        (
            "regularization_force_dd_s".to_owned(),
            json_number(components.force_dd_s),
        ),
        (
            "regularization_n_dd_s".to_owned(),
            json_number(components.n_dd_s),
        ),
        (
            "regularization_xi_dd_s".to_owned(),
            json_number(components.xi_dd_s),
        ),
        (
            "endpoint_c1_dn_s".to_owned(),
            json_number(components.endpoint_c1_dn_s),
        ),
        (
            "endpoint_c1_heading_s".to_owned(),
            json_number(components.endpoint_c1_heading_s),
        ),
        (
            "endpoint_heading_jump_s".to_owned(),
            json_number(components.endpoint_heading_jump_s),
        ),
        (
            "endpoint_d2n_jump_s".to_owned(),
            json_number(components.endpoint_d2n_jump_s),
        ),
    ])
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct CarRegularizationComponents {
    delta_s: f64,
    force_s: f64,
    delta_dd_s: f64,
    force_dd_s: f64,
    n_dd_s: f64,
    xi_dd_s: f64,
    endpoint_c1_dn_s: f64,
    endpoint_c1_heading_s: f64,
    endpoint_heading_jump_s: f64,
    endpoint_d2n_jump_s: f64,
}

fn car_mintime_regularization_components_s(
    seed: &CarMintimeNlpSeed,
    params: CarDoubleTrackParams,
    weights: CarMintimeObjectiveWeights,
    x: &[f64],
) -> CarRegularizationComponents {
    let count = seed.dimensions.interval_count;
    if count == 0 {
        return CarRegularizationComponents::default();
    }

    let delta = control_series(seed, x, CONTROL_DELTA_RAD, 1.0);
    let steering_curvature = steering_curvature_regularization_series(params, &delta);
    let force = drive_brake_regularization_series(seed, x);
    let n = state_series(x, count, STATE_N_M, 1.0);
    let xi = state_series(x, count, STATE_XI_RAD, 1.0);
    let closed = seed_is_closed(seed);

    CarRegularizationComponents {
        delta_s: weights.penalty_delta * first_difference_squared(&steering_curvature, closed),
        force_s: weights.penalty_f * first_difference_squared(&force, closed),
        delta_dd_s: weights.penalty_delta_dd
            * second_difference_squared(&steering_curvature, closed),
        force_dd_s: weights.penalty_f_dd * second_difference_squared(&force, closed),
        n_dd_s: weights.penalty_n_dd * second_difference_squared(&n, closed),
        xi_dd_s: weights.penalty_xi_dd * second_difference_squared(&xi, closed),
        endpoint_c1_dn_s: car_endpoint_c1_dn_objective_s(seed, weights, x),
        endpoint_c1_heading_s: car_endpoint_c1_heading_objective_s(seed, weights, x),
        endpoint_heading_jump_s: car_endpoint_heading_jump_objective_s(seed, weights, x),
        endpoint_d2n_jump_s: car_endpoint_d2n_jump_objective_s(seed, weights, x),
    }
}

fn car_mintime_physics_bundle_columns() -> Vec<&'static str> {
    vec![
        "sample_kind",
        "station_index",
        "interval_index",
        "collocation_point",
        "tau",
        "s_m",
        "x_m",
        "y_m",
        "n_lower_m",
        "n_upper_m",
        "n_normalized",
        "clearance_left_m",
        "clearance_right_m",
        "v_mps",
        "beta_rad",
        "omega_z_radps",
        "n_m",
        "xi_rad",
        "delta_rad",
        "f_drive_n",
        "f_brake_n",
        "gamma_y_n",
        "sigma_dt_ds",
        "ax_mps2",
        "ay_mps2",
        "dv_ds",
        "dbeta_ds",
        "domega_z_ds",
        "dn_ds",
        "dxi_ds",
        "fx_fl_n",
        "fx_fr_n",
        "fx_rl_n",
        "fx_rr_n",
        "fy_fl_n",
        "fy_fr_n",
        "fy_rl_n",
        "fy_rr_n",
        "fz_fl_n",
        "fz_fr_n",
        "fz_rl_n",
        "fz_rr_n",
        "alpha_fl_rad",
        "alpha_fr_rad",
        "alpha_rl_rad",
        "alpha_rr_rad",
        "fl_load_factor",
        "fr_load_factor",
        "rl_load_factor",
        "rr_load_factor",
        "fl_effective_mu_y",
        "fr_effective_mu_y",
        "rl_effective_mu_y",
        "rr_effective_mu_y",
        "fl_capacity_factor_margin",
        "fr_capacity_factor_margin",
        "rl_capacity_factor_margin",
        "rr_capacity_factor_margin",
        "fl_kamm",
        "fr_kamm",
        "rl_kamm",
        "rr_kamm",
        "fl_nominal_kamm",
        "fr_nominal_kamm",
        "rl_nominal_kamm",
        "rr_nominal_kamm",
        "fl_kamm_margin",
        "fr_kamm_margin",
        "rl_kamm_margin",
        "rr_kamm_margin",
        "fl_normal_load_margin_n",
        "fr_normal_load_margin_n",
        "rl_normal_load_margin_n",
        "rr_normal_load_margin_n",
        "front_kamm",
        "rear_kamm",
        "front_kamm_margin",
        "rear_kamm_margin",
        "load_transfer_residual_n",
        "power_w",
        "power_margin_w",
        "drive_brake_product_n2",
        "drive_brake_mutex_margin_n2",
        "delta_rate_radps",
        "delta_rate_margin_radps",
        "drive_rate_nps",
        "drive_rate_margin_nps",
        "brake_rate_nps",
        "brake_rate_margin_nps",
        "gamma_y_rate_nps",
        "gamma_y_rate_margin_nps",
        "delta_margin_rad",
        "drive_force_margin_n",
        "brake_force_margin_n",
        "kappa_ref_1pm",
        "kappa_xy_final_1pm",
        "ay_xy_mps2",
        "ay_model_minus_xy_mps2",
        "heading_path_rad",
        "heading_vehicle_rad",
        "heading_path_minus_vehicle_rad",
        "tire_utilization_combined",
        "tire_utilization_longitudinal",
        "tire_utilization_cornering",
        "drive_utilization",
        "brake_utilization",
        "power_utilization",
        "path_bound_margin_m",
    ]
}

fn car_mintime_station_physics_row(
    problem: &CarMintimeNlpProblem,
    x: &[f64],
    final_points: &[Point2],
    station: usize,
    closed: bool,
) -> JsonValue {
    let interval = car_station_publication_interval(&problem.seed, station);
    let state = car_state_from(&problem.seed, x, station);
    let control = car_control_from(&problem.seed, x, interval);
    let dynamics = car_mintime_station_dynamics_from(&problem.seed, problem.params, x, station);
    let point = final_points
        .get(station)
        .copied()
        .unwrap_or_else(|| station_xy_from(&problem.seed, x, station));
    let (n_lower, n_upper) = car_n_bounds_at_station(&problem.seed, station);
    let kappa_ref_1pm = problem.seed.kappa_1pm.get(station).copied().unwrap_or(0.0);
    let kappa_xy_final_1pm = station_xy_curvature_1pm(final_points, station, closed);
    let ay_xy_mps2 = state.v_mps * state.v_mps * kappa_xy_final_1pm;
    let ay_model_minus_xy_mps2 = dynamics.ay_mps2 - ay_xy_mps2;
    let heading_path_rad = path_heading_rad(final_points, station, closed);
    let ref_heading_rad =
        problem.seed.ref_tangent_xy[station][1].atan2(problem.seed.ref_tangent_xy[station][0]);
    let heading_vehicle_rad = normalize_angle_rad(ref_heading_rad + state.xi_rad + state.beta_rad);

    car_mintime_physics_row_json(
        0.0,
        station as f64,
        interval as f64,
        0.0,
        0.0,
        problem.seed.station_s_m[station],
        point,
        state,
        control,
        dynamics,
        problem,
        x,
        interval,
        n_lower,
        n_upper,
        kappa_ref_1pm,
        kappa_xy_final_1pm,
        ay_xy_mps2,
        ay_model_minus_xy_mps2,
        heading_path_rad,
        heading_vehicle_rad,
    )
}

fn car_mintime_collocation_physics_row(
    problem: &CarMintimeNlpProblem,
    x: &[f64],
    interval: usize,
    point: usize,
) -> JsonValue {
    let coeffs = car_legendre_collocation_coefficients_degree3();
    let tau = coeffs.tau[point];
    let state = collocation_state_from(&problem.seed, x, interval, point - 1);
    let control = car_control_from(&problem.seed, x, interval);
    let dynamics =
        car_mintime_collocation_dynamics_from(&problem.seed, problem.params, x, interval, point);
    let frame_sampler = DenseSectionFrameHermiteSampler {
        station_s_m: &problem.seed.station_s_m,
        centerline_xy_m: &problem.seed.centerline_xy_m,
        tangent_xy: &problem.seed.ref_tangent_xy,
        section_dir_xy: &problem.seed.section_dir_xy,
        section_dir_derivative_xy: &problem.seed.section_dir_derivative_xy,
        closed: seed_is_closed(&problem.seed),
    };
    let geometry = frame_sampler
        .sample_at_interval_tau(interval, tau)
        .expect("valid car physics bundle frame sample");
    let normal = [-geometry.section_dir[0], -geometry.section_dir[1]];
    let point_xy = [
        geometry.centerline_xy_m[0] + normal[0] * state.n_m,
        geometry.centerline_xy_m[1] + normal[1] * state.n_m,
    ];
    let (n_lower, n_upper) = car_n_bounds_at_interval_tau(&problem.seed, interval, tau);
    let next = next_station_index(&problem.seed, interval);
    let kappa_ref_1pm = lerp(
        problem.seed.kappa_1pm[interval],
        problem.seed.kappa_1pm[next],
        tau,
    );
    let ref_heading_rad = geometry.centerline_ds[1].atan2(geometry.centerline_ds[0]);
    let heading_vehicle_rad = normalize_angle_rad(ref_heading_rad + state.xi_rad + state.beta_rad);

    car_mintime_physics_row_json(
        1.0,
        interval as f64 + tau,
        interval as f64,
        point as f64,
        tau,
        geometry.s_m,
        point_xy,
        state,
        control,
        dynamics,
        problem,
        x,
        interval,
        n_lower,
        n_upper,
        kappa_ref_1pm,
        f64::NAN,
        f64::NAN,
        f64::NAN,
        f64::NAN,
        heading_vehicle_rad,
    )
}

#[allow(clippy::too_many_arguments)]
fn car_mintime_physics_row_json(
    sample_kind: f64,
    station_index: f64,
    interval_index: f64,
    collocation_point: f64,
    tau: f64,
    s_m: f64,
    point: Point2,
    state: CarDoubleTrackState,
    control: CarDoubleTrackControl,
    dynamics: CarDoubleTrackDynamics,
    problem: &CarMintimeNlpProblem,
    x: &[f64],
    interval: usize,
    n_lower: f64,
    n_upper: f64,
    kappa_ref_1pm: f64,
    kappa_xy_final_1pm: f64,
    ay_xy_mps2: f64,
    ay_model_minus_xy_mps2: f64,
    heading_path_rad: f64,
    heading_vehicle_rad: f64,
) -> JsonValue {
    let params = problem.params;
    let tire = dynamics.tire_forces;
    let fl = car_wheel_physics_metrics(params, tire, "fl");
    let fr = car_wheel_physics_metrics(params, tire, "fr");
    let rl = car_wheel_physics_metrics(params, tire, "rl");
    let rr = car_wheel_physics_metrics(params, tire, "rr");
    let front_kamm = fl.kamm.max(fr.kamm);
    let rear_kamm = rl.kamm.max(rr.kamm);
    let n_span = n_upper - n_lower;
    let n_normalized = if n_span.abs() > 1e-9 {
        (state.n_m - n_lower) / n_span
    } else {
        0.5
    };
    let power_w = state.v_mps * control.f_drive_n;
    let drive_brake_product = control.f_drive_n * control.f_brake_n;
    let delta_rate = car_control_rate_value(problem, x, interval, CONTROL_DELTA_RAD);
    let drive_rate = car_control_rate_value(problem, x, interval, CONTROL_F_DRIVE_N);
    let brake_rate = car_control_rate_value(problem, x, interval, CONTROL_F_BRAKE_N);
    let gamma_y_rate = car_control_rate_value(problem, x, interval, CONTROL_GAMMA_Y_N);
    let (delta_rate_lower, delta_rate_upper) = car_control_rate_bounds(params, "delta_rad");
    let (drive_rate_lower, drive_rate_upper) = car_control_rate_bounds(params, "f_drive_N");
    let (brake_rate_lower, brake_rate_upper) = car_control_rate_bounds(params, "f_brake_N");
    let (gamma_rate_lower, gamma_rate_upper) = car_control_rate_bounds(params, "gamma_y_N");
    let load_transfer_residual = lateral_load_transfer_residual_for(params, control, tire);
    let tire_utilization_combined = fl.kamm.max(fr.kamm).max(rl.kamm).max(rr.kamm).sqrt();
    let tire_utilization_longitudinal = fl
        .longitudinal_utilization
        .max(fr.longitudinal_utilization)
        .max(rl.longitudinal_utilization)
        .max(rr.longitudinal_utilization);
    let tire_utilization_cornering = fl
        .cornering_utilization
        .max(fr.cornering_utilization)
        .max(rl.cornering_utilization)
        .max(rr.cornering_utilization);
    let heading_path_minus_vehicle_rad =
        normalize_angle_rad(heading_path_rad - heading_vehicle_rad);
    let path_bound_margin_m = (n_upper - state.n_m).min(state.n_m - n_lower);

    json_number_array(vec![
        sample_kind,
        station_index,
        interval_index,
        collocation_point,
        tau,
        s_m,
        point[0],
        point[1],
        n_lower,
        n_upper,
        n_normalized,
        n_upper - state.n_m,
        state.n_m - n_lower,
        state.v_mps,
        state.beta_rad,
        state.omega_z_radps,
        state.n_m,
        state.xi_rad,
        control.delta_rad,
        control.f_drive_n,
        control.f_brake_n,
        control.gamma_y_n,
        dynamics.sigma_dt_ds,
        dynamics.ax_mps2,
        dynamics.ay_mps2,
        dynamics.dv_ds,
        dynamics.dbeta_ds,
        dynamics.domega_z_ds,
        dynamics.dn_ds,
        dynamics.dxi_ds,
        tire.fx_fl_n,
        tire.fx_fr_n,
        tire.fx_rl_n,
        tire.fx_rr_n,
        tire.fy_fl_n,
        tire.fy_fr_n,
        tire.fy_rl_n,
        tire.fy_rr_n,
        tire.fz_fl_n,
        tire.fz_fr_n,
        tire.fz_rl_n,
        tire.fz_rr_n,
        tire.alpha_fl_rad,
        tire.alpha_fr_rad,
        tire.alpha_rl_rad,
        tire.alpha_rr_rad,
        fl.load_factor,
        fr.load_factor,
        rl.load_factor,
        rr.load_factor,
        fl.effective_mu_y,
        fr.effective_mu_y,
        rl.effective_mu_y,
        rr.effective_mu_y,
        fl.capacity_factor_margin,
        fr.capacity_factor_margin,
        rl.capacity_factor_margin,
        rr.capacity_factor_margin,
        fl.kamm,
        fr.kamm,
        rl.kamm,
        rr.kamm,
        fl.nominal_kamm,
        fr.nominal_kamm,
        rl.nominal_kamm,
        rr.nominal_kamm,
        1.0 - fl.kamm,
        1.0 - fr.kamm,
        1.0 - rl.kamm,
        1.0 - rr.kamm,
        fl.normal_load_margin_n,
        fr.normal_load_margin_n,
        rl.normal_load_margin_n,
        rr.normal_load_margin_n,
        front_kamm,
        rear_kamm,
        1.0 - front_kamm,
        1.0 - rear_kamm,
        load_transfer_residual,
        power_w,
        params.power_max_w - power_w,
        drive_brake_product,
        drive_brake_product - CAR_DRIVE_BRAKE_MUTEX_LOWER_N2,
        delta_rate,
        interval_bound_margin(delta_rate, delta_rate_lower, delta_rate_upper),
        drive_rate,
        interval_bound_margin(drive_rate, drive_rate_lower, drive_rate_upper),
        brake_rate,
        interval_bound_margin(brake_rate, brake_rate_lower, brake_rate_upper),
        gamma_y_rate,
        interval_bound_margin(gamma_y_rate, gamma_rate_lower, gamma_rate_upper),
        params.steering_angle_max_rad - control.delta_rad.abs(),
        params.drive_force_max_n - control.f_drive_n,
        params.brake_force_max_n - control.f_brake_n.abs(),
        kappa_ref_1pm,
        kappa_xy_final_1pm,
        ay_xy_mps2,
        ay_model_minus_xy_mps2,
        heading_path_rad,
        heading_vehicle_rad,
        heading_path_minus_vehicle_rad,
        tire_utilization_combined,
        tire_utilization_longitudinal,
        tire_utilization_cornering,
        control.f_drive_n / params.drive_force_max_n.max(1e-9),
        control.f_brake_n.abs() / params.brake_force_max_n.max(1e-9),
        power_w / params.power_max_w.max(1e-9),
        path_bound_margin_m,
    ])
}

fn append_json_object_field(value: &mut JsonValue, key: &str, field_value: JsonValue) {
    if let JsonValue::Object(entries) = value {
        entries.push((key.to_owned(), field_value));
    }
}

fn car_section_frame_path_d2s(
    n_m: f64,
    dn_ds: f64,
    d2n_ds2: f64,
    geometry: InterpolatedSectionsGeometry,
) -> Point2 {
    let centerline_d2s = [
        geometry.ref_left_normal_xy[0] * geometry.kappa_1pm,
        geometry.ref_left_normal_xy[1] * geometry.kappa_1pm,
    ];
    let path_normal = [-geometry.section_dir_xy[0], -geometry.section_dir_xy[1]];
    let path_normal_ds = [
        -geometry.section_dir_derivative_xy[0],
        -geometry.section_dir_derivative_xy[1],
    ];
    let path_normal_d2s = [
        -geometry.section_dir_second_derivative_xy[0],
        -geometry.section_dir_second_derivative_xy[1],
    ];
    [
        centerline_d2s[0]
            + path_normal[0] * d2n_ds2
            + 2.0 * path_normal_ds[0] * dn_ds
            + path_normal_d2s[0] * n_m,
        centerline_d2s[1]
            + path_normal[1] * d2n_ds2
            + 2.0 * path_normal_ds[1] * dn_ds
            + path_normal_d2s[1] * n_m,
    ]
}

fn car_section_frame_coherence_audit_json(seed: &CarMintimeNlpSeed) -> JsonValue {
    let mut xy_ds_errors = Vec::new();
    let mut ds_d2s_errors = Vec::new();
    let tau_values = [0.25_f64, 0.5_f64, 0.75_f64];
    let eps_tau = 1e-4_f64;

    for interval in 0..seed.dimensions.interval_count {
        let ds_m = interval_ds_m(seed, interval).max(1e-9);
        for tau in tau_values {
            let tau_left = (tau - eps_tau).max(0.0);
            let tau_right = (tau + eps_tau).min(1.0);
            if tau_right <= tau_left {
                continue;
            }
            let denom = (tau_right - tau_left) * ds_m;
            let left_geometry = interpolated_sections_geometry(seed, interval, tau_left);
            let right_geometry = interpolated_sections_geometry(seed, interval, tau_right);
            let mid_geometry = interpolated_sections_geometry(seed, interval, tau);
            let next = next_station_index(seed, interval);
            let left_xy = lerp_point(
                seed.centerline_xy_m[interval],
                seed.centerline_xy_m[next],
                tau_left,
            );
            let right_xy = lerp_point(
                seed.centerline_xy_m[interval],
                seed.centerline_xy_m[next],
                tau_right,
            );
            let fd_xy_ds = [
                (right_xy[0] - left_xy[0]) / denom,
                (right_xy[1] - left_xy[1]) / denom,
            ];
            let analytic_ds = car_section_frame_path_ds(0.0, 0.0, mid_geometry);
            xy_ds_errors.push((fd_xy_ds[0] - analytic_ds[0]).hypot(fd_xy_ds[1] - analytic_ds[1]));

            let left_ds = car_section_frame_path_ds(0.0, 0.0, left_geometry);
            let right_ds = car_section_frame_path_ds(0.0, 0.0, right_geometry);
            let fd_ds_d2s = [
                (right_ds[0] - left_ds[0]) / denom,
                (right_ds[1] - left_ds[1]) / denom,
            ];
            let analytic_d2s = car_section_frame_path_d2s(0.0, 0.0, 0.0, mid_geometry);
            ds_d2s_errors
                .push((fd_ds_d2s[0] - analytic_d2s[0]).hypot(fd_ds_d2s[1] - analytic_d2s[1]));
        }
    }

    let xy_error_max = summary_max_abs(&xy_ds_errors);
    let d2s_error_max = summary_max_abs(&ds_d2s_errors);
    let mut warnings = Vec::new();
    if xy_error_max > 1e-3 {
        warnings.push("section_frame_xy_derivative_incoherent".to_owned());
    }
    if d2s_error_max > 1e-2 {
        warnings.push("section_frame_second_derivative_incoherent".to_owned());
    }
    let validation_status = if warnings.is_empty() {
        "clean"
    } else {
        "warning"
    };

    JsonValue::Object(vec![
        (
            "schema_version".to_owned(),
            "section_frame_coherence_audit.v1".into(),
        ),
        ("validation_status".to_owned(), validation_status.into()),
        (
            "warnings".to_owned(),
            JsonValue::Array(warnings.into_iter().map(JsonValue::from).collect()),
        ),
        (
            "finite_difference_xy_vs_path_ds_norm".to_owned(),
            sorted_abs_summary_json(&sorted_abs_values(&xy_ds_errors)),
        ),
        (
            "finite_difference_path_ds_vs_path_d2s_norm".to_owned(),
            sorted_abs_summary_json(&sorted_abs_values(&ds_d2s_errors)),
        ),
    ])
}

#[derive(Clone, Debug, Default)]
struct ScalarStats {
    values: Vec<f64>,
}

impl ScalarStats {
    fn push(&mut self, value: f64) {
        if value.is_finite() {
            self.values.push(value);
        }
    }

    fn json(&self) -> JsonValue {
        let count = self.values.len();
        if count == 0 {
            return JsonValue::Object(vec![
                ("count".to_owned(), JsonValue::Integer(0)),
                ("min".to_owned(), JsonValue::Null),
                ("max".to_owned(), JsonValue::Null),
                ("p05".to_owned(), JsonValue::Null),
                ("p95".to_owned(), JsonValue::Null),
            ]);
        }
        let mut sorted = self.values.clone();
        sorted.sort_by(|left, right| left.partial_cmp(right).unwrap_or(std::cmp::Ordering::Equal));
        JsonValue::Object(vec![
            ("count".to_owned(), JsonValue::Integer(count as i64)),
            ("min".to_owned(), json_number(sorted[0])),
            ("max".to_owned(), json_number(sorted[count - 1])),
            (
                "p05".to_owned(),
                json_number(percentile_sorted(&sorted, 0.05)),
            ),
            (
                "p95".to_owned(),
                json_number(percentile_sorted(&sorted, 0.95)),
            ),
        ])
    }
}

fn percentile_sorted(sorted: &[f64], percentile: f64) -> f64 {
    if sorted.is_empty() {
        return f64::NAN;
    }
    let position = percentile.clamp(0.0, 1.0) * (sorted.len().saturating_sub(1) as f64);
    let lower = position.floor() as usize;
    let upper = position.ceil() as usize;
    if lower == upper {
        sorted[lower]
    } else {
        let t = position - lower as f64;
        sorted[lower] * (1.0 - t) + sorted[upper] * t
    }
}

fn json_number(value: f64) -> JsonValue {
    if value.is_finite() {
        value.into()
    } else if value.is_nan() {
        "nan".into()
    } else if value.is_sign_negative() {
        "-inf".into()
    } else {
        "inf".into()
    }
}

fn json_number_array(values: Vec<f64>) -> JsonValue {
    JsonValue::Array(values.into_iter().map(json_number).collect())
}

fn json_string_array(values: &[&str]) -> JsonValue {
    JsonValue::Array(
        values
            .iter()
            .copied()
            .map(JsonValue::from)
            .collect::<Vec<_>>(),
    )
}

fn json_integer_array(values: Vec<i64>) -> JsonValue {
    JsonValue::Array(values.into_iter().map(JsonValue::Integer).collect())
}

fn car_dense_trajectory_json(
    seed: &CarMintimeNlpSeed,
    params: CarDoubleTrackParams,
    x: &[f64],
    samples_per_interval: usize,
) -> JsonValue {
    let sample_count = seed.dimensions.interval_count * samples_per_interval;
    let mut interval_index = Vec::with_capacity(sample_count);
    let mut tau_values = Vec::with_capacity(sample_count);
    let mut s_m = Vec::with_capacity(sample_count);
    let mut x_m = Vec::with_capacity(sample_count);
    let mut y_m = Vec::with_capacity(sample_count);
    let mut centerline_x_m = Vec::with_capacity(sample_count);
    let mut centerline_y_m = Vec::with_capacity(sample_count);
    let mut section_dir_x = Vec::with_capacity(sample_count);
    let mut section_dir_y = Vec::with_capacity(sample_count);
    let mut n_m = Vec::with_capacity(sample_count);
    let mut dn_ds = Vec::with_capacity(sample_count);
    let mut d2n_ds2 = Vec::with_capacity(sample_count);
    let mut v_mps = Vec::with_capacity(sample_count);
    let mut ax_model_mps2 = Vec::with_capacity(sample_count);
    let mut ay_model_mps2 = Vec::with_capacity(sample_count);
    let mut heading_geo_rad = Vec::with_capacity(sample_count);
    let mut kappa_geo_1pm = Vec::with_capacity(sample_count);
    for interval in 0..seed.dimensions.interval_count {
        for sample in 0..samples_per_interval {
            let tau = sample as f64 / samples_per_interval as f64;
            let solver_geometry = interpolated_sections_geometry(seed, interval, tau);
            let geometry = solver_geometry.dense_geometry();
            let state = car_collocation_state_at_tau(seed, x, interval, tau);
            let state_ds = car_collocation_state_derivatives_at_tau(seed, x, interval, tau);
            let state_d2s = car_collocation_state_second_derivatives_at_tau(seed, x, interval, tau);
            let dynamics = car_mintime_dynamics_with_sections_geometry(
                params,
                state,
                car_control_from(seed, x, interval),
                solver_geometry,
            );
            let (ax_velocity_mps2, ay_velocity_mps2) =
                velocity_frame_acceleration(dynamics.ax_mps2, dynamics.ay_mps2, state.beta_rad);
            let dense = build_dense_section_frame_sample_from_geometry(
                geometry,
                DenseSectionFrameInput {
                    n_m: state.n_m,
                    dn_ds: state_ds.n_m,
                    d2n_ds2: state_d2s.n_m,
                    v_mps: state.v_mps,
                },
            );

            interval_index.push(interval as i64);
            tau_values.push(tau);
            s_m.push(dense.s_m);
            x_m.push(dense.x_m);
            y_m.push(dense.y_m);
            centerline_x_m.push(geometry.centerline_xy_m[0]);
            centerline_y_m.push(geometry.centerline_xy_m[1]);
            section_dir_x.push(geometry.section_dir[0]);
            section_dir_y.push(geometry.section_dir[1]);
            n_m.push(state.n_m);
            dn_ds.push(state_ds.n_m);
            d2n_ds2.push(state_d2s.n_m);
            v_mps.push(state.v_mps);
            ax_model_mps2.push(ax_velocity_mps2);
            ay_model_mps2.push(ay_velocity_mps2);
            heading_geo_rad.push(dense.heading_geo_rad);
            kappa_geo_1pm.push(dense.kappa_geo_1pm);
        }
    }

    JsonValue::Object(vec![
        ("schema_version".to_owned(), "trajectory_dense.v1".into()),
        (
            "source_frame".to_owned(),
            "dense_section_frame_collocation_state_coherent_frame".into(),
        ),
        (
            "state_source".to_owned(),
            "collocation_lagrange_state".into(),
        ),
        (
            "geometry_source".to_owned(),
            "station_hermite_coherent_frame".into(),
        ),
        (
            "acceleration_frame".to_owned(),
            "velocity_tangent_normal".into(),
        ),
        (
            "sample_kind".to_owned(),
            "collocation_continuation_dense".into(),
        ),
        (
            "samples_per_interval".to_owned(),
            JsonValue::Integer(samples_per_interval as i64),
        ),
        (
            "interval_index".to_owned(),
            json_integer_array(interval_index),
        ),
        ("tau".to_owned(), json_number_array(tau_values)),
        ("s_m".to_owned(), json_number_array(s_m)),
        ("x_m".to_owned(), json_number_array(x_m)),
        ("y_m".to_owned(), json_number_array(y_m)),
        (
            "centerline_x_m".to_owned(),
            json_number_array(centerline_x_m),
        ),
        (
            "centerline_y_m".to_owned(),
            json_number_array(centerline_y_m),
        ),
        ("section_dir_x".to_owned(), json_number_array(section_dir_x)),
        ("section_dir_y".to_owned(), json_number_array(section_dir_y)),
        ("n_m".to_owned(), json_number_array(n_m)),
        ("dn_ds".to_owned(), json_number_array(dn_ds)),
        ("d2n_ds2".to_owned(), json_number_array(d2n_ds2)),
        ("v_mps".to_owned(), json_number_array(v_mps)),
        (
            "ax_mps2".to_owned(),
            json_number_array(ax_model_mps2.clone()),
        ),
        ("ax_model_mps2".to_owned(), json_number_array(ax_model_mps2)),
        ("ay_model_mps2".to_owned(), json_number_array(ay_model_mps2)),
        (
            "heading_geo_rad".to_owned(),
            json_number_array(heading_geo_rad),
        ),
        ("kappa_geo_1pm".to_owned(), json_number_array(kappa_geo_1pm)),
    ])
}

fn car_section_frame_geometry_diagnostics_json(
    problem: &CarMintimeNlpProblem,
    x: &[f64],
) -> JsonValue {
    let seed = &problem.seed;
    let mut min_section_det_station = f64::INFINITY;
    let mut min_section_det_collocation = f64::INFINITY;
    let mut min_section_det_dense = f64::INFINITY;
    let mut min_abs_section_det_station = f64::INFINITY;
    let mut min_abs_section_det_collocation = f64::INFINITY;
    let mut min_abs_section_det_dense = f64::INFINITY;
    let mut section_det_reference_sign = 0.0_f64;
    let mut section_det_sign_flip_count = 0_i64;
    let mut min_forward_progress_station = f64::INFINITY;
    let mut min_forward_progress_collocation = f64::INFINITY;
    let mut min_forward_progress_dense = f64::INFINITY;
    let mut pure_frenet_factor_min_debug = f64::INFINITY;
    let mut sigma_clamp_count = 0_i64;
    let mut worst_section_regularity_row = "none".to_owned();

    for station in 0..seed.dimensions.station_count {
        let state = car_state_from(seed, x, station);
        let geometry = station_sections_geometry(seed, station);
        let progress = section_frame_progress_from_derivatives(
            state.n_m,
            state.v_mps,
            state.beta_rad,
            state.xi_rad,
            geometry.ref_tangent_xy,
            geometry.ref_left_normal_xy,
            geometry.centerline_derivative_xy,
            geometry.section_dir_xy,
            geometry.section_dir_derivative_xy,
        );
        let label = format!("station:{station}");
        update_section_geometry_minima(
            &label,
            progress.det_geom,
            progress.forward_progress_per_speed,
            pure_frenet_path_factor(state.n_m, seed.kappa_1pm[station]),
            &mut min_section_det_station,
            &mut min_abs_section_det_station,
            &mut min_forward_progress_station,
            &mut pure_frenet_factor_min_debug,
            &mut sigma_clamp_count,
            &mut section_det_reference_sign,
            &mut section_det_sign_flip_count,
            &mut worst_section_regularity_row,
        );
    }

    let coefficients = car_legendre_collocation_coefficients_degree3();
    for interval in 0..seed.dimensions.interval_count {
        for point in 1..=CAR_COLLOCATION_DEGREE {
            let tau = coefficients.tau[point];
            let geometry = interpolated_sections_geometry(seed, interval, tau);
            let state = collocation_state_from(seed, x, interval, point - 1);
            let progress = section_frame_progress_from_derivatives(
                state.n_m,
                state.v_mps,
                state.beta_rad,
                state.xi_rad,
                geometry.ref_tangent_xy,
                geometry.ref_left_normal_xy,
                geometry.centerline_derivative_xy,
                geometry.section_dir_xy,
                geometry.section_dir_derivative_xy,
            );
            let label = format!("collocation:{interval}:{point}");
            update_section_geometry_minima(
                &label,
                progress.det_geom,
                progress.forward_progress_per_speed,
                pure_frenet_path_factor(state.n_m, geometry.kappa_1pm),
                &mut min_section_det_collocation,
                &mut min_abs_section_det_collocation,
                &mut min_forward_progress_collocation,
                &mut pure_frenet_factor_min_debug,
                &mut sigma_clamp_count,
                &mut section_det_reference_sign,
                &mut section_det_sign_flip_count,
                &mut worst_section_regularity_row,
            );
        }
    }

    for interval in 0..seed.dimensions.interval_count {
        for sample in 0..CAR_DENSE_FRENET_SAMPLES_PER_INTERVAL {
            let tau = sample as f64 / CAR_DENSE_FRENET_SAMPLES_PER_INTERVAL as f64;
            let geometry = interpolated_sections_geometry(seed, interval, tau);
            let state = car_collocation_state_at_tau(seed, x, interval, tau);
            let progress = section_frame_progress_from_derivatives(
                state.n_m,
                state.v_mps,
                state.beta_rad,
                state.xi_rad,
                geometry.ref_tangent_xy,
                geometry.ref_left_normal_xy,
                geometry.centerline_derivative_xy,
                geometry.section_dir_xy,
                geometry.section_dir_derivative_xy,
            );
            let label = format!("dense:{interval}:{sample}");
            update_section_geometry_minima(
                &label,
                progress.det_geom,
                progress.forward_progress_per_speed,
                pure_frenet_path_factor(state.n_m, geometry.kappa_1pm),
                &mut min_section_det_dense,
                &mut min_abs_section_det_dense,
                &mut min_forward_progress_dense,
                &mut pure_frenet_factor_min_debug,
                &mut sigma_clamp_count,
                &mut section_det_reference_sign,
                &mut section_det_sign_flip_count,
                &mut worst_section_regularity_row,
            );
        }
    }

    JsonValue::Object(vec![
        (
            "schema_version".to_owned(),
            "section_frame_geometry_diagnostics.v1".into(),
        ),
        ("geometry_source".to_owned(), "section_frame".into()),
        (
            "min_section_det_station".to_owned(),
            json_number(min_section_det_station),
        ),
        (
            "min_section_det_collocation".to_owned(),
            json_number(min_section_det_collocation),
        ),
        (
            "min_section_det_dense".to_owned(),
            json_number(min_section_det_dense),
        ),
        (
            "min_abs_section_det_station".to_owned(),
            json_number(min_abs_section_det_station),
        ),
        (
            "min_abs_section_det_collocation".to_owned(),
            json_number(min_abs_section_det_collocation),
        ),
        (
            "min_abs_section_det_dense".to_owned(),
            json_number(min_abs_section_det_dense),
        ),
        (
            "section_det_reference_sign".to_owned(),
            json_number(section_det_reference_sign),
        ),
        (
            "section_det_sign_flip_count".to_owned(),
            JsonValue::Integer(section_det_sign_flip_count),
        ),
        (
            "min_forward_progress_station".to_owned(),
            json_number(min_forward_progress_station),
        ),
        (
            "min_forward_progress_collocation".to_owned(),
            json_number(min_forward_progress_collocation),
        ),
        (
            "min_forward_progress_dense".to_owned(),
            json_number(min_forward_progress_dense),
        ),
        (
            "sigma_clamp_count".to_owned(),
            JsonValue::Integer(sigma_clamp_count),
        ),
        (
            "pure_frenet_factor_min_debug".to_owned(),
            json_number(pure_frenet_factor_min_debug),
        ),
        (
            "worst_section_regularity_row".to_owned(),
            worst_section_regularity_row.clone().into(),
        ),
        (
            "worst_section_regulariy_row".to_owned(),
            worst_section_regularity_row.into(),
        ),
    ])
}

fn update_section_geometry_minima(
    label: &str,
    section_det: f64,
    forward_progress: f64,
    pure_frenet_factor_debug: f64,
    min_section_det: &mut f64,
    min_abs_section_det: &mut f64,
    min_forward_progress: &mut f64,
    pure_frenet_factor_min_debug: &mut f64,
    sigma_clamp_count: &mut i64,
    section_det_reference_sign: &mut f64,
    section_det_sign_flip_count: &mut i64,
    worst_section_regularity_row: &mut String,
) {
    if section_det < *min_section_det {
        *min_section_det = section_det;
        *worst_section_regularity_row = label.to_owned();
    }
    if section_det.abs() < *min_abs_section_det {
        *min_abs_section_det = section_det.abs();
    }
    if section_det.abs() > 1.0e-9 {
        let sign = section_det.signum();
        if *section_det_reference_sign == 0.0 {
            *section_det_reference_sign = sign;
        } else if sign != *section_det_reference_sign {
            *section_det_sign_flip_count += 1;
        }
    }
    if forward_progress < *min_forward_progress {
        *min_forward_progress = forward_progress;
    }
    if pure_frenet_factor_debug < *pure_frenet_factor_min_debug {
        *pure_frenet_factor_min_debug = pure_frenet_factor_debug;
    }
    if section_det.abs() <= 1.0e-9 || forward_progress <= 0.0 {
        *sigma_clamp_count += 1;
    }
}

fn sorted_abs_values(values: &[f64]) -> Vec<f64> {
    let mut sorted = values
        .iter()
        .filter_map(|value| value.is_finite().then_some(value.abs()))
        .collect::<Vec<_>>();
    sorted.sort_by(|left, right| left.partial_cmp(right).unwrap_or(std::cmp::Ordering::Equal));
    sorted
}

fn summary_max_abs(values: &[f64]) -> f64 {
    values
        .iter()
        .filter_map(|value| value.is_finite().then_some(value.abs()))
        .fold(0.0, f64::max)
}

fn car_trajectory_contract_json() -> JsonValue {
    JsonValue::Object(vec![
        ("schema_version".to_owned(), "trajectory_contract.v1".into()),
        ("product_geometry".to_owned(), "trajectory_dense".into()),
        ("station_geometry".to_owned(), "trajectory_result".into()),
        (
            "acceleration_frame".to_owned(),
            "velocity_tangent_normal".into(),
        ),
        (
            "curvature_source".to_owned(),
            "trajectory_dense.kappa_geo_1pm".into(),
        ),
        (
            "lateral_accel_source".to_owned(),
            "trajectory_dense.ay_model_mps2".into(),
        ),
        (
            "longitudinal_accel_source".to_owned(),
            "trajectory_dense.ax_model_mps2".into(),
        ),
        (
            "lap_time_source".to_owned(),
            "station_collocation_solve".into(),
        ),
    ])
}

fn car_dense_section_frame_sample_at_tau(
    seed: &CarMintimeNlpSeed,
    x: &[f64],
    interval: usize,
    tau: f64,
) -> DenseSectionFrameSample {
    let frame_sampler = DenseSectionFrameHermiteSampler {
        station_s_m: &seed.station_s_m,
        centerline_xy_m: &seed.centerline_xy_m,
        tangent_xy: &seed.ref_tangent_xy,
        section_dir_xy: &seed.section_dir_xy,
        section_dir_derivative_xy: &seed.section_dir_derivative_xy,
        closed: true,
    };
    let geometry = frame_sampler
        .sample_at_interval_tau(interval, tau)
        .expect("valid coherent section-frame boundary sample");
    let state = car_collocation_state_at_tau(seed, x, interval, tau);
    let state_ds = car_collocation_state_derivatives_at_tau(seed, x, interval, tau);
    let state_d2s = car_collocation_state_second_derivatives_at_tau(seed, x, interval, tau);
    build_dense_section_frame_sample_from_geometry(
        geometry,
        DenseSectionFrameInput {
            n_m: state.n_m,
            dn_ds: state_ds.n_m,
            d2n_ds2: state_d2s.n_m,
            v_mps: state.v_mps,
        },
    )
}

#[derive(Clone, Debug)]
struct CarGeometryBoundaryContinuitySample {
    boundary_station: usize,
    left_interval: usize,
    right_interval: usize,
    xy_gap_m: f64,
    n_gap_m: f64,
    left_n_m: f64,
    right_n_m: f64,
    left_dn_ds: f64,
    right_dn_ds: f64,
    left_d2n_ds2: f64,
    right_d2n_ds2: f64,
    left_xi_rad: f64,
    right_xi_rad: f64,
    left_beta_rad: f64,
    right_beta_rad: f64,
    left_section_dir_xy: Point2,
    right_section_dir_xy: Point2,
    left_section_dir_derivative_xy: Point2,
    right_section_dir_derivative_xy: Point2,
    left_section_dir_second_derivative_xy: Point2,
    right_section_dir_second_derivative_xy: Point2,
    path_ds_jump_norm: f64,
    dn_ds_jump: f64,
    d2n_ds2_jump: f64,
    dn_ds_kin_left: f64,
    dn_ds_kin_right: f64,
    c1_kin_left: f64,
    c1_kin_right: f64,
    c1_heading_left_rad: f64,
    c1_heading_right_rad: f64,
    c1_heading_max_abs_rad: f64,
    heading_jump_rad_direct: f64,
    heading_left_rad: f64,
    heading_right_rad: f64,
    heading_jump_rad: f64,
    kappa_left_1pm: f64,
    kappa_right_1pm: f64,
    kappa_jump_1pm: f64,
}

#[derive(Clone, Copy, Debug)]
struct CarEndpointContinuityResiduals {
    path_ds_jump_norm: f64,
    dn_ds_kin_left: f64,
    dn_ds_kin_right: f64,
    c1_kin_left: f64,
    c1_kin_right: f64,
    c1_heading_left_rad: f64,
    c1_heading_right_rad: f64,
    heading_jump_rad: f64,
}

fn car_endpoint_continuity_residuals(
    seed: &CarMintimeNlpSeed,
    x: &[f64],
    left_interval: usize,
) -> CarEndpointContinuityResiduals {
    let boundary_station = next_station_index(seed, left_interval);
    let right_interval = boundary_station.min(seed.dimensions.interval_count.saturating_sub(1));
    let left_state = car_collocation_state_at_tau(seed, x, left_interval, 1.0);
    let right_state = car_collocation_state_at_tau(seed, x, right_interval, 0.0);
    let left_state_ds = car_collocation_state_derivatives_at_tau(seed, x, left_interval, 1.0);
    let right_state_ds = car_collocation_state_derivatives_at_tau(seed, x, right_interval, 0.0);
    let left_geometry = interpolated_sections_geometry(seed, left_interval, 1.0);
    let right_geometry = interpolated_sections_geometry(seed, right_interval, 0.0);

    let dn_ds_kin_left = section_frame_progress_from_derivatives(
        left_state.n_m,
        left_state.v_mps,
        left_state.beta_rad,
        left_state.xi_rad,
        left_geometry.ref_tangent_xy,
        left_geometry.ref_left_normal_xy,
        left_geometry.centerline_derivative_xy,
        left_geometry.section_dir_xy,
        left_geometry.section_dir_derivative_xy,
    )
    .dn_ds;
    let dn_ds_kin_right = section_frame_progress_from_derivatives(
        right_state.n_m,
        right_state.v_mps,
        right_state.beta_rad,
        right_state.xi_rad,
        right_geometry.ref_tangent_xy,
        right_geometry.ref_left_normal_xy,
        right_geometry.centerline_derivative_xy,
        right_geometry.section_dir_xy,
        right_geometry.section_dir_derivative_xy,
    )
    .dn_ds;

    let left_path_ds = car_section_frame_path_ds(left_state.n_m, left_state_ds.n_m, left_geometry);
    let right_path_ds =
        car_section_frame_path_ds(right_state.n_m, right_state_ds.n_m, right_geometry);
    let left_heading_geo_rad = left_path_ds[1].atan2(left_path_ds[0]);
    let right_heading_geo_rad = right_path_ds[1].atan2(right_path_ds[0]);
    let left_heading_velocity_rad =
        car_endpoint_velocity_heading_rad(left_state, left_geometry.ref_tangent_xy);
    let right_heading_velocity_rad =
        car_endpoint_velocity_heading_rad(right_state, right_geometry.ref_tangent_xy);

    CarEndpointContinuityResiduals {
        path_ds_jump_norm: (right_path_ds[0] - left_path_ds[0])
            .hypot(right_path_ds[1] - left_path_ds[1]),
        dn_ds_kin_left,
        dn_ds_kin_right,
        c1_kin_left: left_state_ds.n_m - dn_ds_kin_left,
        c1_kin_right: right_state_ds.n_m - dn_ds_kin_right,
        c1_heading_left_rad: normalize_angle_rad(left_heading_geo_rad - left_heading_velocity_rad),
        c1_heading_right_rad: normalize_angle_rad(
            right_heading_geo_rad - right_heading_velocity_rad,
        ),
        heading_jump_rad: normalize_angle_rad(right_heading_geo_rad - left_heading_geo_rad),
    }
}

fn car_endpoint_velocity_heading_rad(state: CarDoubleTrackState, ref_tangent_xy: Point2) -> f64 {
    normalize_angle_rad(ref_tangent_xy[1].atan2(ref_tangent_xy[0]) + state.xi_rad + state.beta_rad)
}

fn car_section_frame_path_ds(
    n_m: f64,
    dn_ds: f64,
    geometry: InterpolatedSectionsGeometry,
) -> Point2 {
    let path_normal = [-geometry.section_dir_xy[0], -geometry.section_dir_xy[1]];
    let path_normal_ds = [
        -geometry.section_dir_derivative_xy[0],
        -geometry.section_dir_derivative_xy[1],
    ];
    [
        geometry.ref_tangent_xy[0] + path_normal[0] * dn_ds + path_normal_ds[0] * n_m,
        geometry.ref_tangent_xy[1] + path_normal[1] * dn_ds + path_normal_ds[1] * n_m,
    ]
}

fn car_collocation_geometry_boundary_continuity_audit_json(
    seed: &CarMintimeNlpSeed,
    x: &[f64],
) -> JsonValue {
    let mut samples = Vec::with_capacity(seed.dimensions.interval_count);
    let mut heading_jumps = Vec::with_capacity(seed.dimensions.interval_count);
    let mut kappa_jumps = Vec::with_capacity(seed.dimensions.interval_count);
    let mut dn_jumps = Vec::with_capacity(seed.dimensions.interval_count);
    let mut d2n_jumps = Vec::with_capacity(seed.dimensions.interval_count);
    let mut xy_gaps = Vec::with_capacity(seed.dimensions.interval_count);
    let mut path_ds_jumps = Vec::with_capacity(seed.dimensions.interval_count);
    let mut c1_kin_left_values = Vec::with_capacity(seed.dimensions.interval_count);
    let mut c1_kin_right_values = Vec::with_capacity(seed.dimensions.interval_count);
    let mut c1_kin_max_values = Vec::with_capacity(seed.dimensions.interval_count);
    let mut c1_heading_left_values = Vec::with_capacity(seed.dimensions.interval_count);
    let mut c1_heading_right_values = Vec::with_capacity(seed.dimensions.interval_count);
    let mut c1_heading_max_values = Vec::with_capacity(seed.dimensions.interval_count);

    for left_interval in 0..seed.dimensions.interval_count {
        let boundary_station = next_station_index(seed, left_interval);
        if !seed_is_closed(seed) && boundary_station >= seed.dimensions.interval_count {
            continue;
        }
        let right_interval = boundary_station.min(seed.dimensions.interval_count.saturating_sub(1));
        let left = car_dense_section_frame_sample_at_tau(seed, x, left_interval, 1.0);
        let right = car_dense_section_frame_sample_at_tau(seed, x, right_interval, 0.0);
        let left_state = car_collocation_state_at_tau(seed, x, left_interval, 1.0);
        let right_state = car_collocation_state_at_tau(seed, x, right_interval, 0.0);
        let left_geometry = interpolated_sections_geometry(seed, left_interval, 1.0);
        let right_geometry = interpolated_sections_geometry(seed, right_interval, 0.0);
        let residuals = car_endpoint_continuity_residuals(seed, x, left_interval);
        let heading_jump = normalize_angle_rad(right.heading_geo_rad - left.heading_geo_rad);
        let sample = CarGeometryBoundaryContinuitySample {
            boundary_station,
            left_interval,
            right_interval,
            xy_gap_m: (right.x_m - left.x_m).hypot(right.y_m - left.y_m),
            n_gap_m: right.n_m - left.n_m,
            left_n_m: left.n_m,
            right_n_m: right.n_m,
            left_dn_ds: left.dn_ds,
            right_dn_ds: right.dn_ds,
            left_d2n_ds2: left.d2n_ds2,
            right_d2n_ds2: right.d2n_ds2,
            left_xi_rad: left_state.xi_rad,
            right_xi_rad: right_state.xi_rad,
            left_beta_rad: left_state.beta_rad,
            right_beta_rad: right_state.beta_rad,
            left_section_dir_xy: left_geometry.section_dir_xy,
            right_section_dir_xy: right_geometry.section_dir_xy,
            left_section_dir_derivative_xy: left_geometry.section_dir_derivative_xy,
            right_section_dir_derivative_xy: right_geometry.section_dir_derivative_xy,
            left_section_dir_second_derivative_xy: left_geometry.section_dir_second_derivative_xy,
            right_section_dir_second_derivative_xy: right_geometry.section_dir_second_derivative_xy,
            path_ds_jump_norm: residuals.path_ds_jump_norm,
            dn_ds_jump: right.dn_ds - left.dn_ds,
            d2n_ds2_jump: right.d2n_ds2 - left.d2n_ds2,
            dn_ds_kin_left: residuals.dn_ds_kin_left,
            dn_ds_kin_right: residuals.dn_ds_kin_right,
            c1_kin_left: residuals.c1_kin_left,
            c1_kin_right: residuals.c1_kin_right,
            c1_heading_left_rad: residuals.c1_heading_left_rad,
            c1_heading_right_rad: residuals.c1_heading_right_rad,
            c1_heading_max_abs_rad: residuals
                .c1_heading_left_rad
                .abs()
                .max(residuals.c1_heading_right_rad.abs()),
            heading_jump_rad_direct: residuals.heading_jump_rad,
            heading_left_rad: left.heading_geo_rad,
            heading_right_rad: right.heading_geo_rad,
            heading_jump_rad: heading_jump,
            kappa_left_1pm: left.kappa_geo_1pm,
            kappa_right_1pm: right.kappa_geo_1pm,
            kappa_jump_1pm: right.kappa_geo_1pm - left.kappa_geo_1pm,
        };
        heading_jumps.push(sample.heading_jump_rad.abs());
        kappa_jumps.push(sample.kappa_jump_1pm.abs());
        dn_jumps.push(sample.dn_ds_jump.abs());
        d2n_jumps.push(sample.d2n_ds2_jump.abs());
        xy_gaps.push(sample.xy_gap_m.abs());
        path_ds_jumps.push(sample.path_ds_jump_norm.abs());
        c1_kin_left_values.push(sample.c1_kin_left.abs());
        c1_kin_right_values.push(sample.c1_kin_right.abs());
        c1_kin_max_values.push(sample.c1_kin_left.abs().max(sample.c1_kin_right.abs()));
        c1_heading_left_values.push(sample.c1_heading_left_rad.abs());
        c1_heading_right_values.push(sample.c1_heading_right_rad.abs());
        c1_heading_max_values.push(sample.c1_heading_max_abs_rad);
        samples.push(sample);
    }

    heading_jumps
        .sort_by(|left, right| left.partial_cmp(right).unwrap_or(std::cmp::Ordering::Equal));
    kappa_jumps.sort_by(|left, right| left.partial_cmp(right).unwrap_or(std::cmp::Ordering::Equal));
    dn_jumps.sort_by(|left, right| left.partial_cmp(right).unwrap_or(std::cmp::Ordering::Equal));
    d2n_jumps.sort_by(|left, right| left.partial_cmp(right).unwrap_or(std::cmp::Ordering::Equal));
    xy_gaps.sort_by(|left, right| left.partial_cmp(right).unwrap_or(std::cmp::Ordering::Equal));
    path_ds_jumps
        .sort_by(|left, right| left.partial_cmp(right).unwrap_or(std::cmp::Ordering::Equal));
    c1_kin_left_values
        .sort_by(|left, right| left.partial_cmp(right).unwrap_or(std::cmp::Ordering::Equal));
    c1_kin_right_values
        .sort_by(|left, right| left.partial_cmp(right).unwrap_or(std::cmp::Ordering::Equal));
    c1_kin_max_values
        .sort_by(|left, right| left.partial_cmp(right).unwrap_or(std::cmp::Ordering::Equal));
    c1_heading_left_values
        .sort_by(|left, right| left.partial_cmp(right).unwrap_or(std::cmp::Ordering::Equal));
    c1_heading_right_values
        .sort_by(|left, right| left.partial_cmp(right).unwrap_or(std::cmp::Ordering::Equal));
    c1_heading_max_values
        .sort_by(|left, right| left.partial_cmp(right).unwrap_or(std::cmp::Ordering::Equal));

    let mut top_heading = samples.clone();
    top_heading.sort_by(|left, right| {
        right
            .heading_jump_rad
            .abs()
            .partial_cmp(&left.heading_jump_rad.abs())
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    let mut top_kappa = samples.clone();
    top_kappa.sort_by(|left, right| {
        right
            .kappa_jump_1pm
            .abs()
            .partial_cmp(&left.kappa_jump_1pm.abs())
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    let mut top_c1_kin = samples.clone();
    top_c1_kin.sort_by(|left, right| {
        right
            .c1_kin_left
            .abs()
            .max(right.c1_kin_right.abs())
            .partial_cmp(&left.c1_kin_left.abs().max(left.c1_kin_right.abs()))
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    let mut top_c1_heading = samples.clone();
    top_c1_heading.sort_by(|left, right| {
        right
            .c1_heading_max_abs_rad
            .partial_cmp(&left.c1_heading_max_abs_rad)
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    JsonValue::Object(vec![
        (
            "schema_version".to_owned(),
            "car_collocation_geometry_boundary_continuity_audit.v2".into(),
        ),
        (
            "interpretation".to_owned(),
            "Compares dense section-frame geometry at finite-element boundaries: left interval tau=1 against right interval tau=0. C0 position/n continuity can be clean while dn/ds, heading and curvature jump at station boundaries. endpoint_c1 residuals compare each endpoint dn/ds against the current section-frame kinematic dn/ds used by car dynamics."
                .into(),
        ),
        (
            "sample_count".to_owned(),
            JsonValue::Integer(samples.len() as i64),
        ),
        (
            "xy_gap_m_abs".to_owned(),
            sorted_abs_summary_json(&xy_gaps),
        ),
        (
            "path_ds_jump_norm_abs".to_owned(),
            sorted_abs_summary_json(&path_ds_jumps),
        ),
        (
            "heading_jump_rad_abs".to_owned(),
            sorted_abs_summary_json(&heading_jumps),
        ),
        (
            "endpoint_heading_jump_rad_abs".to_owned(),
            sorted_abs_summary_json(&heading_jumps),
        ),
        (
            "heading_jump_deg_abs".to_owned(),
            sorted_abs_summary_json(
                &heading_jumps
                    .iter()
                    .map(|value| value.to_degrees())
                    .collect::<Vec<_>>(),
            ),
        ),
        (
            "dn_ds_jump_abs".to_owned(),
            sorted_abs_summary_json(&dn_jumps),
        ),
        (
            "endpoint_c1_dn_left_abs".to_owned(),
            sorted_abs_summary_json(&c1_kin_left_values),
        ),
        (
            "endpoint_c1_dn_right_abs".to_owned(),
            sorted_abs_summary_json(&c1_kin_right_values),
        ),
        (
            "endpoint_c1_dn_max_abs".to_owned(),
            sorted_abs_summary_json(&c1_kin_max_values),
        ),
        (
            "endpoint_c1_heading_left_rad_abs".to_owned(),
            sorted_abs_summary_json(&c1_heading_left_values),
        ),
        (
            "endpoint_c1_heading_right_rad_abs".to_owned(),
            sorted_abs_summary_json(&c1_heading_right_values),
        ),
        (
            "endpoint_c1_heading_max_rad_abs".to_owned(),
            sorted_abs_summary_json(&c1_heading_max_values),
        ),
        (
            "d2n_ds2_jump_abs".to_owned(),
            sorted_abs_summary_json(&d2n_jumps),
        ),
        (
            "kappa_jump_1pm_abs".to_owned(),
            sorted_abs_summary_json(&kappa_jumps),
        ),
        (
            "top_heading_jumps".to_owned(),
            JsonValue::Array(
                top_heading
                    .iter()
                    .take(10)
                    .map(car_geometry_boundary_continuity_sample_json)
                    .collect(),
            ),
        ),
        (
            "top_endpoint_heading_jump".to_owned(),
            JsonValue::Array(
                top_heading
                    .iter()
                    .take(10)
                    .map(car_geometry_boundary_continuity_sample_json)
                    .collect(),
            ),
        ),
        (
            "top_kappa_jumps".to_owned(),
            JsonValue::Array(
                top_kappa
                    .iter()
                    .take(10)
                    .map(car_geometry_boundary_continuity_sample_json)
                    .collect(),
            ),
        ),
        (
            "top_endpoint_c1_dn".to_owned(),
            JsonValue::Array(
                top_c1_kin
                    .iter()
                    .take(10)
                    .map(car_geometry_boundary_continuity_sample_json)
                    .collect(),
            ),
        ),
        (
            "top_endpoint_c1_heading".to_owned(),
            JsonValue::Array(
                top_c1_heading
                    .iter()
                    .take(10)
                    .map(car_geometry_boundary_continuity_sample_json)
                    .collect(),
            ),
        ),
    ])
}

fn car_geometry_boundary_continuity_sample_json(
    sample: &CarGeometryBoundaryContinuitySample,
) -> JsonValue {
    JsonValue::Object(vec![
        (
            "boundary_station".to_owned(),
            JsonValue::Integer(sample.boundary_station as i64),
        ),
        (
            "left_interval".to_owned(),
            JsonValue::Integer(sample.left_interval as i64),
        ),
        (
            "right_interval".to_owned(),
            JsonValue::Integer(sample.right_interval as i64),
        ),
        ("xy_gap_m".to_owned(), json_number(sample.xy_gap_m)),
        ("n_gap_m".to_owned(), json_number(sample.n_gap_m)),
        ("left_n_m".to_owned(), json_number(sample.left_n_m)),
        ("right_n_m".to_owned(), json_number(sample.right_n_m)),
        ("left_dn_ds".to_owned(), json_number(sample.left_dn_ds)),
        ("right_dn_ds".to_owned(), json_number(sample.right_dn_ds)),
        ("left_d2n_ds2".to_owned(), json_number(sample.left_d2n_ds2)),
        (
            "right_d2n_ds2".to_owned(),
            json_number(sample.right_d2n_ds2),
        ),
        ("left_xi_rad".to_owned(), json_number(sample.left_xi_rad)),
        ("right_xi_rad".to_owned(), json_number(sample.right_xi_rad)),
        (
            "left_beta_rad".to_owned(),
            json_number(sample.left_beta_rad),
        ),
        (
            "right_beta_rad".to_owned(),
            json_number(sample.right_beta_rad),
        ),
        (
            "left_section_dir_xy".to_owned(),
            json_number_array(sample.left_section_dir_xy.to_vec()),
        ),
        (
            "right_section_dir_xy".to_owned(),
            json_number_array(sample.right_section_dir_xy.to_vec()),
        ),
        (
            "left_section_dir_derivative_xy".to_owned(),
            json_number_array(sample.left_section_dir_derivative_xy.to_vec()),
        ),
        (
            "right_section_dir_derivative_xy".to_owned(),
            json_number_array(sample.right_section_dir_derivative_xy.to_vec()),
        ),
        (
            "left_section_dir_second_derivative_xy".to_owned(),
            json_number_array(sample.left_section_dir_second_derivative_xy.to_vec()),
        ),
        (
            "right_section_dir_second_derivative_xy".to_owned(),
            json_number_array(sample.right_section_dir_second_derivative_xy.to_vec()),
        ),
        (
            "path_ds_jump_norm".to_owned(),
            json_number(sample.path_ds_jump_norm),
        ),
        ("dn_ds_jump".to_owned(), json_number(sample.dn_ds_jump)),
        ("d2n_ds2_jump".to_owned(), json_number(sample.d2n_ds2_jump)),
        (
            "dn_ds_kin_left".to_owned(),
            json_number(sample.dn_ds_kin_left),
        ),
        (
            "dn_ds_kin_right".to_owned(),
            json_number(sample.dn_ds_kin_right),
        ),
        (
            "endpoint_c1_dn_left".to_owned(),
            json_number(sample.c1_kin_left),
        ),
        (
            "endpoint_c1_dn_right".to_owned(),
            json_number(sample.c1_kin_right),
        ),
        (
            "endpoint_c1_dn_max_abs".to_owned(),
            json_number(sample.c1_kin_left.abs().max(sample.c1_kin_right.abs())),
        ),
        (
            "endpoint_c1_heading_left_rad".to_owned(),
            json_number(sample.c1_heading_left_rad),
        ),
        (
            "endpoint_c1_heading_right_rad".to_owned(),
            json_number(sample.c1_heading_right_rad),
        ),
        (
            "endpoint_c1_heading_max_abs_rad".to_owned(),
            json_number(sample.c1_heading_max_abs_rad),
        ),
        (
            "endpoint_heading_jump_rad".to_owned(),
            json_number(sample.heading_jump_rad_direct),
        ),
        (
            "heading_left_rad".to_owned(),
            json_number(sample.heading_left_rad),
        ),
        (
            "heading_right_rad".to_owned(),
            json_number(sample.heading_right_rad),
        ),
        (
            "heading_jump_rad".to_owned(),
            json_number(sample.heading_jump_rad),
        ),
        (
            "heading_jump_deg".to_owned(),
            json_number(sample.heading_jump_rad.to_degrees()),
        ),
        (
            "kappa_left_1pm".to_owned(),
            json_number(sample.kappa_left_1pm),
        ),
        (
            "kappa_right_1pm".to_owned(),
            json_number(sample.kappa_right_1pm),
        ),
        (
            "kappa_jump_1pm".to_owned(),
            json_number(sample.kappa_jump_1pm),
        ),
    ])
}

#[derive(Clone, Debug)]
struct CarCollocationNodeReconstructionSample {
    interval: usize,
    point: usize,
    tau: f64,
    raw_n_m: f64,
    reconstructed_n_m: f64,
    raw_minus_reconstructed_n_m: f64,
    station_linear_n_m: f64,
    raw_minus_station_linear_n_m: f64,
    raw_v_mps: f64,
    reconstructed_v_mps: f64,
    raw_minus_reconstructed_v_mps: f64,
    raw_beta_rad: f64,
    reconstructed_beta_rad: f64,
    raw_minus_reconstructed_beta_rad: f64,
    raw_omega_z_radps: f64,
    reconstructed_omega_z_radps: f64,
    raw_minus_reconstructed_omega_z_radps: f64,
    raw_xi_rad: f64,
    reconstructed_xi_rad: f64,
    raw_minus_reconstructed_xi_rad: f64,
}

fn car_collocation_node_reconstruction_audit_json(
    seed: &CarMintimeNlpSeed,
    x: &[f64],
) -> JsonValue {
    let coefficients = car_legendre_collocation_coefficients_degree3();
    let sample_capacity = seed.dimensions.interval_count * CAR_COLLOCATION_DEGREE;
    let mut samples = Vec::with_capacity(sample_capacity);
    let mut raw_reconstruction_abs_n = Vec::with_capacity(sample_capacity);
    let mut raw_reconstruction_abs_v = Vec::with_capacity(sample_capacity);
    let mut raw_reconstruction_abs_beta = Vec::with_capacity(sample_capacity);
    let mut raw_reconstruction_abs_omega = Vec::with_capacity(sample_capacity);
    let mut raw_reconstruction_abs_xi = Vec::with_capacity(sample_capacity);
    let mut raw_station_linear_abs_n = Vec::with_capacity(sample_capacity);

    for interval in 0..seed.dimensions.interval_count {
        let current = car_state_from(seed, x, interval);
        let next = car_state_from(seed, x, next_station_index(seed, interval));
        for point in 0..CAR_COLLOCATION_DEGREE {
            let tau = coefficients.tau[point + 1];
            let raw = collocation_state_from(seed, x, interval, point);
            let reconstructed = car_collocation_state_at_tau(seed, x, interval, tau);
            let station_linear_n_m = lerp(current.n_m, next.n_m, tau);
            let sample = CarCollocationNodeReconstructionSample {
                interval,
                point,
                tau,
                raw_n_m: raw.n_m,
                reconstructed_n_m: reconstructed.n_m,
                raw_minus_reconstructed_n_m: raw.n_m - reconstructed.n_m,
                station_linear_n_m,
                raw_minus_station_linear_n_m: raw.n_m - station_linear_n_m,
                raw_v_mps: raw.v_mps,
                reconstructed_v_mps: reconstructed.v_mps,
                raw_minus_reconstructed_v_mps: raw.v_mps - reconstructed.v_mps,
                raw_beta_rad: raw.beta_rad,
                reconstructed_beta_rad: reconstructed.beta_rad,
                raw_minus_reconstructed_beta_rad: raw.beta_rad - reconstructed.beta_rad,
                raw_omega_z_radps: raw.omega_z_radps,
                reconstructed_omega_z_radps: reconstructed.omega_z_radps,
                raw_minus_reconstructed_omega_z_radps: raw.omega_z_radps
                    - reconstructed.omega_z_radps,
                raw_xi_rad: raw.xi_rad,
                reconstructed_xi_rad: reconstructed.xi_rad,
                raw_minus_reconstructed_xi_rad: raw.xi_rad - reconstructed.xi_rad,
            };
            raw_reconstruction_abs_n.push(sample.raw_minus_reconstructed_n_m.abs());
            raw_reconstruction_abs_v.push(sample.raw_minus_reconstructed_v_mps.abs());
            raw_reconstruction_abs_beta.push(sample.raw_minus_reconstructed_beta_rad.abs());
            raw_reconstruction_abs_omega.push(sample.raw_minus_reconstructed_omega_z_radps.abs());
            raw_reconstruction_abs_xi.push(sample.raw_minus_reconstructed_xi_rad.abs());
            raw_station_linear_abs_n.push(sample.raw_minus_station_linear_n_m.abs());
            samples.push(sample);
        }
    }

    raw_reconstruction_abs_n
        .sort_by(|left, right| left.partial_cmp(right).unwrap_or(std::cmp::Ordering::Equal));
    raw_reconstruction_abs_v
        .sort_by(|left, right| left.partial_cmp(right).unwrap_or(std::cmp::Ordering::Equal));
    raw_reconstruction_abs_beta
        .sort_by(|left, right| left.partial_cmp(right).unwrap_or(std::cmp::Ordering::Equal));
    raw_reconstruction_abs_omega
        .sort_by(|left, right| left.partial_cmp(right).unwrap_or(std::cmp::Ordering::Equal));
    raw_reconstruction_abs_xi
        .sort_by(|left, right| left.partial_cmp(right).unwrap_or(std::cmp::Ordering::Equal));
    raw_station_linear_abs_n
        .sort_by(|left, right| left.partial_cmp(right).unwrap_or(std::cmp::Ordering::Equal));

    let mut top_raw_reconstruction = samples.clone();
    top_raw_reconstruction.sort_by(|left, right| {
        right
            .raw_minus_reconstructed_n_m
            .abs()
            .partial_cmp(&left.raw_minus_reconstructed_n_m.abs())
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    let mut top_station_linear = samples.clone();
    top_station_linear.sort_by(|left, right| {
        right
            .raw_minus_station_linear_n_m
            .abs()
            .partial_cmp(&left.raw_minus_station_linear_n_m.abs())
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    JsonValue::Object(vec![
        (
            "schema_version".to_owned(),
            "car_collocation_node_reconstruction_audit.v1".into(),
        ),
        (
            "interpretation".to_owned(),
            "Compares raw NLP collocation node states against car_collocation_state_at_tau at the exact collocation nodes; also reports how far raw collocation n departs from station-linear n."
                .into(),
        ),
        (
            "sample_count".to_owned(),
            JsonValue::Integer(samples.len() as i64),
        ),
        (
            "node_tau".to_owned(),
            json_number_array(coefficients.tau[1..].to_vec()),
        ),
        (
            "raw_minus_reconstructed_abs".to_owned(),
            JsonValue::Object(vec![
                (
                    "n_m".to_owned(),
                    sorted_abs_summary_json(&raw_reconstruction_abs_n),
                ),
                (
                    "v_mps".to_owned(),
                    sorted_abs_summary_json(&raw_reconstruction_abs_v),
                ),
                (
                    "beta_rad".to_owned(),
                    sorted_abs_summary_json(&raw_reconstruction_abs_beta),
                ),
                (
                    "omega_z_radps".to_owned(),
                    sorted_abs_summary_json(&raw_reconstruction_abs_omega),
                ),
                (
                    "xi_rad".to_owned(),
                    sorted_abs_summary_json(&raw_reconstruction_abs_xi),
                ),
            ]),
        ),
        (
            "raw_collocation_n_minus_station_linear_n_abs".to_owned(),
            sorted_abs_summary_json(&raw_station_linear_abs_n),
        ),
        (
            "top_raw_minus_reconstructed_n".to_owned(),
            JsonValue::Array(
                top_raw_reconstruction
                    .iter()
                    .take(8)
                    .map(car_collocation_node_reconstruction_sample_json)
                    .collect(),
            ),
        ),
        (
            "top_raw_collocation_n_minus_station_linear_n".to_owned(),
            JsonValue::Array(
                top_station_linear
                    .iter()
                    .take(8)
                    .map(car_collocation_node_reconstruction_sample_json)
                    .collect(),
            ),
        ),
    ])
}

fn sorted_abs_summary_json(sorted_abs_values: &[f64]) -> JsonValue {
    let count = sorted_abs_values.len();
    if count == 0 {
        return JsonValue::Object(vec![
            ("count".to_owned(), JsonValue::Integer(0)),
            ("rms".to_owned(), JsonValue::Null),
            ("p95".to_owned(), JsonValue::Null),
            ("max".to_owned(), JsonValue::Null),
        ]);
    }
    let rms = (sorted_abs_values
        .iter()
        .map(|value| value * value)
        .sum::<f64>()
        / count as f64)
        .sqrt();
    JsonValue::Object(vec![
        ("count".to_owned(), JsonValue::Integer(count as i64)),
        ("rms".to_owned(), json_number(rms)),
        (
            "p95".to_owned(),
            json_number(percentile_sorted(sorted_abs_values, 0.95)),
        ),
        (
            "max".to_owned(),
            json_number(sorted_abs_values.last().copied().unwrap_or(f64::NAN)),
        ),
    ])
}

fn car_collocation_node_reconstruction_sample_json(
    sample: &CarCollocationNodeReconstructionSample,
) -> JsonValue {
    JsonValue::Object(vec![
        (
            "interval_index".to_owned(),
            JsonValue::Integer(sample.interval as i64),
        ),
        (
            "collocation_point".to_owned(),
            JsonValue::Integer(sample.point as i64),
        ),
        ("tau".to_owned(), json_number(sample.tau)),
        ("raw_n_m".to_owned(), json_number(sample.raw_n_m)),
        (
            "reconstructed_n_m".to_owned(),
            json_number(sample.reconstructed_n_m),
        ),
        (
            "raw_minus_reconstructed_n_m".to_owned(),
            json_number(sample.raw_minus_reconstructed_n_m),
        ),
        (
            "station_linear_n_m".to_owned(),
            json_number(sample.station_linear_n_m),
        ),
        (
            "raw_minus_station_linear_n_m".to_owned(),
            json_number(sample.raw_minus_station_linear_n_m),
        ),
        ("raw_v_mps".to_owned(), json_number(sample.raw_v_mps)),
        (
            "reconstructed_v_mps".to_owned(),
            json_number(sample.reconstructed_v_mps),
        ),
        (
            "raw_minus_reconstructed_v_mps".to_owned(),
            json_number(sample.raw_minus_reconstructed_v_mps),
        ),
        ("raw_beta_rad".to_owned(), json_number(sample.raw_beta_rad)),
        (
            "reconstructed_beta_rad".to_owned(),
            json_number(sample.reconstructed_beta_rad),
        ),
        (
            "raw_minus_reconstructed_beta_rad".to_owned(),
            json_number(sample.raw_minus_reconstructed_beta_rad),
        ),
        (
            "raw_omega_z_radps".to_owned(),
            json_number(sample.raw_omega_z_radps),
        ),
        (
            "reconstructed_omega_z_radps".to_owned(),
            json_number(sample.reconstructed_omega_z_radps),
        ),
        (
            "raw_minus_reconstructed_omega_z_radps".to_owned(),
            json_number(sample.raw_minus_reconstructed_omega_z_radps),
        ),
        ("raw_xi_rad".to_owned(), json_number(sample.raw_xi_rad)),
        (
            "reconstructed_xi_rad".to_owned(),
            json_number(sample.reconstructed_xi_rad),
        ),
        (
            "raw_minus_reconstructed_xi_rad".to_owned(),
            json_number(sample.raw_minus_reconstructed_xi_rad),
        ),
    ])
}

fn tire_envelope_contract_json(mode: TireLoadSensitivityMode) -> JsonValue {
    JsonValue::Object(vec![
        (
            "tire_envelope_formula".to_owned(),
            "anisotropic_lateral_load_sensitive".into(),
        ),
        ("lambda_mode".to_owned(), mode.as_str().into()),
        ("longitudinal_capacity".to_owned(), "mu_x_fz".into()),
        ("lateral_capacity".to_owned(), "mu_y_fz_lambda_y".into()),
        (
            "canonical_mode".to_owned(),
            "current_simplified_product_contract".into(),
        ),
        (
            "note".to_owned(),
            "This freezes the current simplified anisotropic Kamm contract; it is not a final high-fidelity tire model.".into(),
        ),
    ])
}

#[derive(Clone, Debug)]
struct CarTireEnvelopeSample {
    station: usize,
    wheel: &'static str,
    lambda_y: f64,
    effective_mu_y: f64,
    reference_load_n: f64,
    nominal_kamm: f64,
    active_kamm: f64,
    normal_load_n: f64,
    power_violation_w: f64,
}

fn car_tire_envelope_summary_json(problem: &CarMintimeNlpProblem, x: &[f64]) -> JsonValue {
    let mut lambda_all = ScalarStats::default();
    let mut lambda_fl = ScalarStats::default();
    let mut lambda_fr = ScalarStats::default();
    let mut lambda_rl = ScalarStats::default();
    let mut lambda_rr = ScalarStats::default();
    let mut effective_mu_all = ScalarStats::default();
    let mut effective_mu_fl = ScalarStats::default();
    let mut effective_mu_fr = ScalarStats::default();
    let mut effective_mu_rl = ScalarStats::default();
    let mut effective_mu_rr = ScalarStats::default();
    let mut active_kamm = ScalarStats::default();
    let mut nominal_kamm = ScalarStats::default();
    let mut samples = Vec::<CarTireEnvelopeSample>::new();

    for station in 0..problem.seed.dimensions.station_count {
        let interval = station.min(problem.seed.dimensions.interval_count.saturating_sub(1));
        let dynamics = car_mintime_path_dynamics_from(&problem.seed, problem.params, x, interval);
        let tire = dynamics.tire_forces;
        let state = car_state_from(&problem.seed, x, station);
        let control = car_control_from(&problem.seed, x, interval);
        let power_violation_w =
            (state.v_mps * control.f_drive_n - problem.params.power_max_w).max(0.0);
        for wheel in ["fl", "fr", "rl", "rr"] {
            let (_, _, fz, eps, fz0_n) = car_wheel_force_values(tire, problem.params, wheel);
            let lambda_y = car_tire_capacity_factor(problem.params, fz, eps, fz0_n);
            samples.push(CarTireEnvelopeSample {
                station,
                wheel,
                lambda_y,
                effective_mu_y: problem.params.lateral_grip_level * lambda_y,
                reference_load_n: fz0_n,
                nominal_kamm: car_wheel_nominal_kamm(problem.params, tire, wheel),
                active_kamm: tire.wheel_ellipse_utilization(problem.params, wheel),
                normal_load_n: fz,
                power_violation_w,
            });
        }
    }

    let mut min_lambda_samples = samples.clone();
    min_lambda_samples.sort_by(|left, right| {
        left.lambda_y
            .partial_cmp(&right.lambda_y)
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    let mut worst_active_kamm = samples.clone();
    worst_active_kamm.sort_by(|left, right| {
        right
            .active_kamm
            .partial_cmp(&left.active_kamm)
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    let mut max_kamm_violation = 0.0_f64;
    let mut min_normal_load = f64::INFINITY;
    let mut max_power_violation = 0.0_f64;
    for sample in &samples {
        lambda_all.push(sample.lambda_y);
        effective_mu_all.push(sample.effective_mu_y);
        active_kamm.push(sample.active_kamm);
        nominal_kamm.push(sample.nominal_kamm);
        match sample.wheel {
            "fl" => {
                lambda_fl.push(sample.lambda_y);
                effective_mu_fl.push(sample.effective_mu_y);
            }
            "fr" => {
                lambda_fr.push(sample.lambda_y);
                effective_mu_fr.push(sample.effective_mu_y);
            }
            "rl" => {
                lambda_rl.push(sample.lambda_y);
                effective_mu_rl.push(sample.effective_mu_y);
            }
            "rr" => {
                lambda_rr.push(sample.lambda_y);
                effective_mu_rr.push(sample.effective_mu_y);
            }
            _ => {}
        }
        max_kamm_violation = max_kamm_violation.max((sample.active_kamm - 1.0).max(0.0));
        min_normal_load = min_normal_load.min(sample.normal_load_n);
        max_power_violation = max_power_violation.max(sample.power_violation_w);
    }

    JsonValue::Object(vec![
        ("sample_frame".to_owned(), "station path points".into()),
        (
            "reference_loads_n".to_owned(),
            JsonValue::Object(vec![
                (
                    "shared_legacy".to_owned(),
                    json_number(problem.params.tire_fz0_n),
                ),
                (
                    "front".to_owned(),
                    json_number(problem.params.tire_fz0_front_n),
                ),
                (
                    "rear".to_owned(),
                    json_number(problem.params.tire_fz0_rear_n),
                ),
            ]),
        ),
        (
            "lambda_y".to_owned(),
            JsonValue::Object(vec![
                ("all".to_owned(), lambda_all.json()),
                ("fl".to_owned(), lambda_fl.json()),
                ("fr".to_owned(), lambda_fr.json()),
                ("rl".to_owned(), lambda_rl.json()),
                ("rr".to_owned(), lambda_rr.json()),
            ]),
        ),
        (
            "effective_mu_y".to_owned(),
            JsonValue::Object(vec![
                ("all".to_owned(), effective_mu_all.json()),
                ("fl".to_owned(), effective_mu_fl.json()),
                ("fr".to_owned(), effective_mu_fr.json()),
                ("rl".to_owned(), effective_mu_rl.json()),
                ("rr".to_owned(), effective_mu_rr.json()),
            ]),
        ),
        (
            "kamm_utilization".to_owned(),
            JsonValue::Object(vec![
                ("active_envelope".to_owned(), active_kamm.json()),
                ("nominal_mu_fz_circle".to_owned(), nominal_kamm.json()),
            ]),
        ),
        (
            "min_lambda_samples".to_owned(),
            JsonValue::Array(
                min_lambda_samples
                    .iter()
                    .take(8)
                    .map(car_tire_sample_json)
                    .collect(),
            ),
        ),
        (
            "worst_active_kamm_samples".to_owned(),
            JsonValue::Array(
                worst_active_kamm
                    .iter()
                    .take(8)
                    .map(car_tire_sample_json)
                    .collect(),
            ),
        ),
        (
            "post_solve_feasibility".to_owned(),
            JsonValue::Object(vec![
                (
                    "sample_frame".to_owned(),
                    "station path points; dense collocation audit is a separate follow-up".into(),
                ),
                (
                    "max_active_kamm_violation".to_owned(),
                    json_number(max_kamm_violation),
                ),
                ("min_normal_load_n".to_owned(), json_number(min_normal_load)),
                (
                    "max_power_violation_w".to_owned(),
                    json_number(max_power_violation),
                ),
            ]),
        ),
    ])
}

fn car_tire_sample_json(sample: &CarTireEnvelopeSample) -> JsonValue {
    JsonValue::Object(vec![
        (
            "station_index".to_owned(),
            JsonValue::Integer(sample.station as i64),
        ),
        ("wheel".to_owned(), sample.wheel.into()),
        ("lambda_y".to_owned(), json_number(sample.lambda_y)),
        (
            "effective_mu_y".to_owned(),
            json_number(sample.effective_mu_y),
        ),
        (
            "reference_load_n".to_owned(),
            json_number(sample.reference_load_n),
        ),
        ("active_kamm".to_owned(), json_number(sample.active_kamm)),
        ("nominal_kamm".to_owned(), json_number(sample.nominal_kamm)),
        (
            "normal_load_n".to_owned(),
            json_number(sample.normal_load_n),
        ),
        (
            "power_violation_w".to_owned(),
            json_number(sample.power_violation_w),
        ),
    ])
}

fn car_wheel_force_values(
    tire: CarDoubleTrackTireForces,
    params: CarDoubleTrackParams,
    wheel: &str,
) -> (f64, f64, f64, f64, f64) {
    match wheel {
        "fl" => (
            tire.fx_fl_n,
            tire.fy_fl_n,
            tire.fz_fl_n,
            params.tire_eps_front,
            params.tire_fz0_front_n,
        ),
        "fr" => (
            tire.fx_fr_n,
            tire.fy_fr_n,
            tire.fz_fr_n,
            params.tire_eps_front,
            params.tire_fz0_front_n,
        ),
        "rl" => (
            tire.fx_rl_n,
            tire.fy_rl_n,
            tire.fz_rl_n,
            params.tire_eps_rear,
            params.tire_fz0_rear_n,
        ),
        "rr" => (
            tire.fx_rr_n,
            tire.fy_rr_n,
            tire.fz_rr_n,
            params.tire_eps_rear,
            params.tire_fz0_rear_n,
        ),
        _ => (0.0, 0.0, 1.0, 0.0, params.tire_fz0_n),
    }
}

fn car_formulation_contract_json(problem: &CarMintimeNlpProblem) -> JsonValue {
    let mode = problem.options.formulation_mode;
    JsonValue::Object(vec![
        (
            "schema_version".to_owned(),
            "car_mintime_formulation_contract.v1".into(),
        ),
        ("formulation_mode".to_owned(), mode.as_str().into()),
        (
            "tire_force_mode".to_owned(),
            if mode.uses_prepeak_grip_domain() {
                "pacejka_prepeak_grip_domain"
            } else {
                "full_pacejka_with_descending_branch"
            }
            .into(),
        ),
        (
            "grip_constraint_scaling".to_owned(),
            if mode.uses_prepeak_grip_domain() {
                "alpha_over_alpha_peak"
            } else {
                "none"
            }
            .into(),
        ),
        (
            "prepeak_grip_margin".to_owned(),
            if mode.uses_prepeak_grip_domain() {
                json_number(problem.options.prepeak_grip_margin)
            } else {
                JsonValue::Null
            },
        ),
        (
            "front_alpha_peak_rad".to_owned(),
            json_number(car_pacejka_peak_slip_rad(problem.params, "fl")),
        ),
        (
            "rear_alpha_peak_rad".to_owned(),
            json_number(car_pacejka_peak_slip_rad(problem.params, "rl")),
        ),
    ])
}

fn car_wheel_slip_rad(tire: CarDoubleTrackTireForces, wheel: &str) -> f64 {
    match wheel {
        "fl" => tire.alpha_fl_rad,
        "fr" => tire.alpha_fr_rad,
        "rl" => tire.alpha_rl_rad,
        "rr" => tire.alpha_rr_rad,
        _ => unreachable!("unsupported car wheel: {wheel}"),
    }
}

fn car_pacejka_peak_slip_rad(params: CarDoubleTrackParams, wheel: &str) -> f64 {
    let (b, c, e) = match wheel {
        "fl" | "fr" => (
            params.tire_b_front,
            params.tire_c_front,
            params.tire_e_front,
        ),
        "rl" | "rr" => (params.tire_b_rear, params.tire_c_rear, params.tire_e_rear),
        _ => unreachable!("unsupported car wheel: {wheel}"),
    };
    let b = b.max(1.0e-9);
    let c = c.max(1.0e-9);
    let target = (std::f64::consts::FRAC_PI_2 / c).tan();
    let shape = |alpha: f64| {
        let b_alpha = b * alpha;
        b_alpha - e * (b_alpha - b_alpha.atan())
    };
    let mut lower = 0.0;
    let mut upper = 0.25;
    while shape(upper) < target && upper < 4.0 {
        upper *= 2.0;
    }
    for _ in 0..80 {
        let middle = 0.5 * (lower + upper);
        if shape(middle) < target {
            lower = middle;
        } else {
            upper = middle;
        }
    }
    0.5 * (lower + upper)
}

fn car_tire_capacity_factor(
    params: CarDoubleTrackParams,
    normal_load_n: f64,
    eps: f64,
    fz0_n: f64,
) -> f64 {
    let normal_load_n = normal_load_n.max(1e-6);
    let fz0_n = fz0_n.max(1e-6);
    match params.tire_load_sensitivity_mode {
        TireLoadSensitivityMode::UpstreamRaw => 1.0 + eps * normal_load_n / fz0_n,
        TireLoadSensitivityMode::ReferenceNormalizedDfz => {
            1.0 + eps * ((normal_load_n - fz0_n) / fz0_n)
        }
    }
}

fn car_wheel_nominal_kamm(
    params: CarDoubleTrackParams,
    tire: CarDoubleTrackTireForces,
    wheel: &str,
) -> f64 {
    let (fx, fy, fz, _, _) = car_wheel_force_values(tire, params, wheel);
    let longitudinal_capacity = directional_longitudinal_tire_capacity_n(
        params.drive_grip_level,
        params.brake_grip_level,
        fx,
        fz,
    );
    let lateral_capacity = (params.lateral_grip_level * fz.max(1e-6)).max(1e-6);
    (fx / longitudinal_capacity).powi(2) + (fy / lateral_capacity).powi(2)
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct CarWheelPhysicsMetrics {
    load_factor: f64,
    effective_mu_y: f64,
    capacity_factor_margin: f64,
    kamm: f64,
    nominal_kamm: f64,
    normal_load_margin_n: f64,
    longitudinal_utilization: f64,
    cornering_utilization: f64,
}

fn car_wheel_physics_metrics(
    params: CarDoubleTrackParams,
    tire: CarDoubleTrackTireForces,
    wheel: &str,
) -> CarWheelPhysicsMetrics {
    let (fx, fy, fz, eps, fz0_n) = car_wheel_force_values(tire, params, wheel);
    let load_factor = car_tire_capacity_factor(params, fz, eps, fz0_n);
    let longitudinal_capacity = directional_longitudinal_tire_capacity_n(
        params.drive_grip_level,
        params.brake_grip_level,
        fx,
        fz,
    );
    let lateral_capacity = (params.lateral_grip_level * fz.max(1e-6) * load_factor).max(1e-6);

    CarWheelPhysicsMetrics {
        load_factor,
        effective_mu_y: params.lateral_grip_level * load_factor,
        capacity_factor_margin: load_factor,
        kamm: tire.wheel_ellipse_utilization(params, wheel),
        nominal_kamm: car_wheel_nominal_kamm(params, tire, wheel),
        normal_load_margin_n: fz,
        longitudinal_utilization: fx.abs() / longitudinal_capacity,
        cornering_utilization: fy.abs() / lateral_capacity,
    }
}

fn car_n_bounds_at_station(seed: &CarMintimeNlpSeed, station: usize) -> (f64, f64) {
    let offset = state_offset(station) + STATE_N_M;
    (seed.lower_bounds[offset], seed.upper_bounds[offset])
}

fn car_n_bounds_at_interval_tau(seed: &CarMintimeNlpSeed, interval: usize, tau: f64) -> (f64, f64) {
    let next = next_station_index(seed, interval);
    let (lower_current, upper_current) = car_n_bounds_at_station(seed, interval);
    let (lower_next, upper_next) = car_n_bounds_at_station(seed, next);
    (
        lerp(lower_current, lower_next, tau),
        lerp(upper_current, upper_next, tau),
    )
}

fn car_control_rate_value(
    problem: &CarMintimeNlpProblem,
    x: &[f64],
    interval: usize,
    control_index: usize,
) -> f64 {
    let current = control_value_from(&problem.seed, x, interval, control_index);
    let previous_interval = interval.saturating_sub(1);
    let previous = control_value_from(&problem.seed, x, previous_interval, control_index);
    let ds = interval_ds_m(&problem.seed, previous_interval).max(1e-6);
    let sigma = sigma_dt_ds_from(&problem.seed, problem.params, x, previous_interval);
    (current - previous) / (ds * sigma).max(1e-6)
}

fn car_control_rate_bounds(params: CarDoubleTrackParams, control_name: &str) -> (f64, f64) {
    match control_name {
        "delta_rad" => (
            -params.steering_angle_max_rad / params.steering_response_s,
            params.steering_angle_max_rad / params.steering_response_s,
        ),
        "f_drive_N" => (
            f64::NEG_INFINITY,
            params.drive_force_max_n / params.throttle_response_s,
        ),
        "f_brake_N" => (
            -params.brake_force_max_n / params.brake_response_s,
            f64::INFINITY,
        ),
        "gamma_y_N" => (f64::NEG_INFINITY, f64::INFINITY),
        _ => unreachable!("unknown car control {control_name}"),
    }
}

fn interval_bound_margin(value: f64, lower: f64, upper: f64) -> f64 {
    let lower_margin = if lower.is_finite() {
        value - lower
    } else {
        f64::INFINITY
    };
    let upper_margin = if upper.is_finite() {
        upper - value
    } else {
        f64::INFINITY
    };
    lower_margin.min(upper_margin)
}

#[derive(Clone, Debug)]
struct FeasibilityViolationSample {
    family: &'static str,
    sample_kind: &'static str,
    interval: usize,
    point: Option<usize>,
    tau: f64,
    subject: String,
    value: f64,
    limit: f64,
    violation: f64,
}

#[derive(Clone, Debug, Default)]
struct FeasibilityFamilyAudit {
    max_violation: f64,
    worst: Option<FeasibilityViolationSample>,
}

impl FeasibilityFamilyAudit {
    fn push(&mut self, sample: FeasibilityViolationSample) {
        if sample.violation > self.max_violation {
            self.max_violation = sample.violation;
            self.worst = Some(sample);
        }
    }
}

fn car_feasibility_audit_json(problem: &CarMintimeNlpProblem, x: &[f64]) -> JsonValue {
    let mut kamm = FeasibilityFamilyAudit::default();
    let mut normal_load = FeasibilityFamilyAudit::default();
    let mut power = FeasibilityFamilyAudit::default();
    let mut kamm_station_path = FeasibilityFamilyAudit::default();
    let mut kamm_collocation = FeasibilityFamilyAudit::default();
    let mut normal_load_station_path = FeasibilityFamilyAudit::default();
    let mut normal_load_collocation = FeasibilityFamilyAudit::default();
    let mut power_station_path = FeasibilityFamilyAudit::default();
    let mut power_collocation = FeasibilityFamilyAudit::default();
    let mut sample_count = 0_i64;

    for interval in 0..problem.seed.dimensions.interval_count {
        let station = next_station_index(&problem.seed, interval);
        let state = car_state_from(&problem.seed, x, station);
        let control = car_control_from(&problem.seed, x, interval);
        let dynamics = car_mintime_path_dynamics_from(&problem.seed, problem.params, x, interval);
        car_push_feasibility_samples(
            problem.params,
            dynamics.tire_forces,
            state.v_mps * control.f_drive_n,
            "station_path",
            interval,
            None,
            1.0,
            &mut kamm_station_path,
            &mut normal_load_station_path,
            &mut power_station_path,
        );
        sample_count += car_push_feasibility_samples(
            problem.params,
            dynamics.tire_forces,
            state.v_mps * control.f_drive_n,
            "station_path",
            interval,
            None,
            1.0,
            &mut kamm,
            &mut normal_load,
            &mut power,
        );

        for point in 1..=CAR_COLLOCATION_DEGREE {
            let coeffs = car_legendre_collocation_coefficients_degree3();
            let state = collocation_state_from(&problem.seed, x, interval, point - 1);
            let control = car_control_from(&problem.seed, x, interval);
            let dynamics = car_mintime_collocation_dynamics_from(
                &problem.seed,
                problem.params,
                x,
                interval,
                point,
            );
            car_push_feasibility_samples(
                problem.params,
                dynamics.tire_forces,
                state.v_mps * control.f_drive_n,
                "collocation",
                interval,
                Some(point),
                coeffs.tau[point],
                &mut kamm_collocation,
                &mut normal_load_collocation,
                &mut power_collocation,
            );
            sample_count += car_push_feasibility_samples(
                problem.params,
                dynamics.tire_forces,
                state.v_mps * control.f_drive_n,
                "collocation",
                interval,
                Some(point),
                coeffs.tau[point],
                &mut kamm,
                &mut normal_load,
                &mut power,
            );
        }

        for (dense_index, tau) in [0.25_f64, 0.75_f64].into_iter().enumerate() {
            let state = car_linear_state_at_tau(&problem.seed, x, interval, tau);
            let control = car_control_from(&problem.seed, x, interval);
            let geometry = interpolated_sections_geometry(&problem.seed, interval, tau);
            let dynamics = car_mintime_dynamics_with_sections_geometry(
                problem.params,
                state,
                control,
                geometry,
            );
            sample_count += car_push_feasibility_samples(
                problem.params,
                dynamics.tire_forces,
                state.v_mps * control.f_drive_n,
                "linear_dense",
                interval,
                Some(dense_index + 1),
                tau,
                &mut kamm,
                &mut normal_load,
                &mut power,
            );
        }
    }

    JsonValue::Object(vec![
        (
            "schema_version".to_owned(),
            "post_solve_feasibility_audit.v1".into(),
        ),
        (
            "sample_frame".to_owned(),
            "station path points, degree-3 collocation points, and linear interpolated dense samples at tau=0.25/0.75".into(),
        ),
        ("sample_count".to_owned(), JsonValue::Integer(sample_count)),
        (
            "max_violation_by_family".to_owned(),
            JsonValue::Object(vec![
                ("kamm".to_owned(), json_number(kamm.max_violation)),
                (
                    "normal_load".to_owned(),
                    json_number(normal_load.max_violation),
                ),
                ("power".to_owned(), json_number(power.max_violation)),
            ]),
        ),
        (
            "max_kamm_violation_station".to_owned(),
            json_number(kamm_station_path.max_violation),
        ),
        (
            "max_kamm_violation_collocation".to_owned(),
            json_number(kamm_collocation.max_violation),
        ),
        (
            "max_normal_load_violation_station_n".to_owned(),
            json_number(normal_load_station_path.max_violation),
        ),
        (
            "max_normal_load_violation_collocation_n".to_owned(),
            json_number(normal_load_collocation.max_violation),
        ),
        (
            "max_power_violation_station_w".to_owned(),
            json_number(power_station_path.max_violation),
        ),
        (
            "max_power_violation_collocation_w".to_owned(),
            json_number(power_collocation.max_violation),
        ),
        (
            "worst_points_by_family".to_owned(),
            JsonValue::Object(vec![
                (
                    "kamm".to_owned(),
                    feasibility_worst_json(kamm.worst.as_ref()),
                ),
                (
                    "normal_load".to_owned(),
                    feasibility_worst_json(normal_load.worst.as_ref()),
                ),
                (
                    "power".to_owned(),
                    feasibility_worst_json(power.worst.as_ref()),
                ),
                (
                    "kamm_station_path".to_owned(),
                    feasibility_worst_json(kamm_station_path.worst.as_ref()),
                ),
                (
                    "kamm_collocation".to_owned(),
                    feasibility_worst_json(kamm_collocation.worst.as_ref()),
                ),
                (
                    "normal_load_station_path".to_owned(),
                    feasibility_worst_json(normal_load_station_path.worst.as_ref()),
                ),
                (
                    "normal_load_collocation".to_owned(),
                    feasibility_worst_json(normal_load_collocation.worst.as_ref()),
                ),
                (
                    "power_station_path".to_owned(),
                    feasibility_worst_json(power_station_path.worst.as_ref()),
                ),
                (
                    "power_collocation".to_owned(),
                    feasibility_worst_json(power_collocation.worst.as_ref()),
                ),
            ]),
        ),
    ])
}

fn car_push_feasibility_samples(
    params: CarDoubleTrackParams,
    tire: CarDoubleTrackTireForces,
    power_w: f64,
    sample_kind: &'static str,
    interval: usize,
    point: Option<usize>,
    tau: f64,
    kamm: &mut FeasibilityFamilyAudit,
    normal_load: &mut FeasibilityFamilyAudit,
    power: &mut FeasibilityFamilyAudit,
) -> i64 {
    for wheel in ["fl", "fr", "rl", "rr"] {
        let (_, _, fz, _, _) = car_wheel_force_values(tire, params, wheel);
        let active_kamm = tire.wheel_ellipse_utilization(params, wheel);
        kamm.push(FeasibilityViolationSample {
            family: "kamm",
            sample_kind,
            interval,
            point,
            tau,
            subject: wheel.to_owned(),
            value: active_kamm,
            limit: 1.0,
            violation: (active_kamm - 1.0).max(0.0),
        });
        normal_load.push(FeasibilityViolationSample {
            family: "normal_load",
            sample_kind,
            interval,
            point,
            tau,
            subject: wheel.to_owned(),
            value: fz,
            limit: 0.0,
            violation: (-fz).max(0.0),
        });
    }
    power.push(FeasibilityViolationSample {
        family: "power",
        sample_kind,
        interval,
        point,
        tau,
        subject: "drive_power".to_owned(),
        value: power_w,
        limit: params.power_max_w,
        violation: (power_w - params.power_max_w).max(0.0),
    });
    1
}

fn feasibility_worst_json(sample: Option<&FeasibilityViolationSample>) -> JsonValue {
    sample
        .map(|sample| {
            JsonValue::Object(vec![
                ("family".to_owned(), sample.family.into()),
                ("sample_kind".to_owned(), sample.sample_kind.into()),
                (
                    "interval".to_owned(),
                    JsonValue::Integer(sample.interval as i64),
                ),
                (
                    "point".to_owned(),
                    sample
                        .point
                        .map(|point| JsonValue::Integer(point as i64))
                        .unwrap_or(JsonValue::Null),
                ),
                ("tau".to_owned(), json_number(sample.tau)),
                ("subject".to_owned(), sample.subject.clone().into()),
                ("value".to_owned(), json_number(sample.value)),
                ("limit".to_owned(), json_number(sample.limit)),
                ("violation".to_owned(), json_number(sample.violation)),
            ])
        })
        .unwrap_or(JsonValue::Null)
}

fn car_linear_state_at_tau(
    seed: &CarMintimeNlpSeed,
    x: &[f64],
    interval: usize,
    tau: f64,
) -> CarDoubleTrackState {
    let current = car_state_from(seed, x, interval);
    let next = car_state_from(seed, x, next_station_index(seed, interval));
    CarDoubleTrackState {
        v_mps: lerp(current.v_mps, next.v_mps, tau),
        beta_rad: lerp(current.beta_rad, next.beta_rad, tau),
        omega_z_radps: lerp(current.omega_z_radps, next.omega_z_radps, tau),
        n_m: lerp(current.n_m, next.n_m, tau),
        xi_rad: lerp(current.xi_rad, next.xi_rad, tau),
    }
}

// Phase 3 provider: wired into result JSON in Phase 4.
#[allow(dead_code)]
pub(crate) fn car_collocation_state_at_tau(
    seed: &CarMintimeNlpSeed,
    x: &[f64],
    interval: usize,
    tau: f64,
) -> CarDoubleTrackState {
    let basis = car_collocation_lagrange_basis_at_tau(tau);
    let states = car_collocation_interval_states(seed, x, interval);
    car_collocation_state_from_basis(&states, &basis, 1.0)
}

#[allow(dead_code)]
pub(crate) fn car_collocation_state_derivatives_at_tau(
    seed: &CarMintimeNlpSeed,
    x: &[f64],
    interval: usize,
    tau: f64,
) -> CarDoubleTrackState {
    let basis = car_collocation_lagrange_basis_derivative_at_tau(tau);
    let states = car_collocation_interval_states(seed, x, interval);
    car_collocation_state_from_basis(&states, &basis, 1.0 / interval_ds_m(seed, interval))
}

#[allow(dead_code)]
pub(crate) fn car_collocation_state_second_derivatives_at_tau(
    seed: &CarMintimeNlpSeed,
    x: &[f64],
    interval: usize,
    tau: f64,
) -> CarDoubleTrackState {
    let ds_m = interval_ds_m(seed, interval);
    let basis = car_collocation_lagrange_basis_second_derivative_at_tau(tau);
    let states = car_collocation_interval_states(seed, x, interval);
    car_collocation_state_from_basis(&states, &basis, 1.0 / (ds_m * ds_m))
}

#[allow(dead_code)]
fn car_collocation_interval_states(
    seed: &CarMintimeNlpSeed,
    x: &[f64],
    interval: usize,
) -> [CarDoubleTrackState; CAR_COLLOCATION_DEGREE + 1] {
    [
        car_state_from(seed, x, interval),
        collocation_state_from(seed, x, interval, 0),
        collocation_state_from(seed, x, interval, 1),
        collocation_state_from(seed, x, interval, 2),
    ]
}

#[allow(dead_code)]
fn car_collocation_lagrange_basis_at_tau(tau: f64) -> [f64; CAR_COLLOCATION_DEGREE + 1] {
    let nodes = car_legendre_collocation_coefficients_degree3().tau;
    let mut basis = [0.0; CAR_COLLOCATION_DEGREE + 1];
    for node_index in 0..=CAR_COLLOCATION_DEGREE {
        let mut value = 1.0;
        for other_index in 0..=CAR_COLLOCATION_DEGREE {
            if node_index == other_index {
                continue;
            }
            value *= (tau - nodes[other_index]) / (nodes[node_index] - nodes[other_index]);
        }
        basis[node_index] = value;
    }
    basis
}

#[allow(dead_code)]
fn car_collocation_lagrange_basis_derivative_at_tau(tau: f64) -> [f64; CAR_COLLOCATION_DEGREE + 1] {
    let nodes = car_legendre_collocation_coefficients_degree3().tau;
    let mut basis = [0.0; CAR_COLLOCATION_DEGREE + 1];
    for node_index in 0..=CAR_COLLOCATION_DEGREE {
        let mut value = 0.0;
        for derivative_index in 0..=CAR_COLLOCATION_DEGREE {
            if derivative_index == node_index {
                continue;
            }
            let mut term = 1.0 / (nodes[node_index] - nodes[derivative_index]);
            for product_index in 0..=CAR_COLLOCATION_DEGREE {
                if product_index == node_index || product_index == derivative_index {
                    continue;
                }
                term *= (tau - nodes[product_index]) / (nodes[node_index] - nodes[product_index]);
            }
            value += term;
        }
        basis[node_index] = value;
    }
    basis
}

#[allow(dead_code)]
fn car_collocation_lagrange_basis_second_derivative_at_tau(
    tau: f64,
) -> [f64; CAR_COLLOCATION_DEGREE + 1] {
    let nodes = car_legendre_collocation_coefficients_degree3().tau;
    let mut basis = [0.0; CAR_COLLOCATION_DEGREE + 1];
    for node_index in 0..=CAR_COLLOCATION_DEGREE {
        let mut value = 0.0;
        for first_derivative_index in 0..=CAR_COLLOCATION_DEGREE {
            if first_derivative_index == node_index {
                continue;
            }
            for second_derivative_index in 0..=CAR_COLLOCATION_DEGREE {
                if second_derivative_index == node_index
                    || second_derivative_index == first_derivative_index
                {
                    continue;
                }
                let mut term = 1.0
                    / ((nodes[node_index] - nodes[first_derivative_index])
                        * (nodes[node_index] - nodes[second_derivative_index]));
                for product_index in 0..=CAR_COLLOCATION_DEGREE {
                    if product_index == node_index
                        || product_index == first_derivative_index
                        || product_index == second_derivative_index
                    {
                        continue;
                    }
                    term *=
                        (tau - nodes[product_index]) / (nodes[node_index] - nodes[product_index]);
                }
                value += term;
            }
        }
        basis[node_index] = value;
    }
    basis
}

#[allow(dead_code)]
fn car_collocation_state_from_basis(
    states: &[CarDoubleTrackState; CAR_COLLOCATION_DEGREE + 1],
    basis: &[f64; CAR_COLLOCATION_DEGREE + 1],
    scale: f64,
) -> CarDoubleTrackState {
    CarDoubleTrackState {
        v_mps: scale * car_collocation_lerp_state_component(states, basis, |state| state.v_mps),
        beta_rad: scale
            * car_collocation_lerp_state_component(states, basis, |state| state.beta_rad),
        omega_z_radps: scale
            * car_collocation_lerp_state_component(states, basis, |state| state.omega_z_radps),
        n_m: scale * car_collocation_lerp_state_component(states, basis, |state| state.n_m),
        xi_rad: scale * car_collocation_lerp_state_component(states, basis, |state| state.xi_rad),
    }
}

#[allow(dead_code)]
fn car_collocation_lerp_state_component(
    states: &[CarDoubleTrackState; CAR_COLLOCATION_DEGREE + 1],
    basis: &[f64; CAR_COLLOCATION_DEGREE + 1],
    component: impl Fn(CarDoubleTrackState) -> f64,
) -> f64 {
    states
        .iter()
        .zip(basis.iter())
        .map(|(state, weight)| weight * component(*state))
        .sum()
}

#[derive(Clone, Debug)]
struct AyConsistencySample {
    station: usize,
    ay_model_mps2: f64,
    ay_xy_mps2: f64,
    diff_mps2: f64,
    kappa_xy_1pm: f64,
    kappa_published_1pm: f64,
    speed_mps: f64,
}

fn ay_xy_consistency_json(trajectory: &TrajectoryResultSeriesV1, closed: bool) -> JsonValue {
    let count = trajectory
        .x_m
        .len()
        .min(trajectory.y_m.len())
        .min(trajectory.v_mps.len())
        .min(trajectory.ay_mps2.len())
        .min(trajectory.kappa_1pm.len());
    let mut samples = Vec::<AyConsistencySample>::new();
    let mut abs_diffs = Vec::<f64>::new();
    let mut sum_sq = 0.0;
    let mut max_model_ay = 0.0_f64;

    for station in 0..count {
        if !closed && (station == 0 || station + 1 == count) {
            continue;
        }
        let previous = if station == 0 {
            count.saturating_sub(1)
        } else {
            station - 1
        };
        let next = if station + 1 == count { 0 } else { station + 1 };
        let prev = [trajectory.x_m[previous], trajectory.y_m[previous]];
        let current = [trajectory.x_m[station], trajectory.y_m[station]];
        let following = [trajectory.x_m[next], trajectory.y_m[next]];
        let kappa_xy = signed_three_point_curvature_1pm(prev, current, following);
        if !kappa_xy.is_finite() {
            continue;
        }
        let ay_xy = trajectory.v_mps[station] * trajectory.v_mps[station] * kappa_xy;
        let ay_model = trajectory.ay_mps2[station];
        let diff = ay_model - ay_xy;
        max_model_ay = max_model_ay.max(ay_model.abs());
        sum_sq += diff * diff;
        abs_diffs.push(diff.abs());
        samples.push(AyConsistencySample {
            station,
            ay_model_mps2: ay_model,
            ay_xy_mps2: ay_xy,
            diff_mps2: diff,
            kappa_xy_1pm: kappa_xy,
            kappa_published_1pm: trajectory.kappa_1pm[station],
            speed_mps: trajectory.v_mps[station],
        });
    }

    samples.sort_by(|left, right| {
        right
            .diff_mps2
            .abs()
            .partial_cmp(&left.diff_mps2.abs())
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    abs_diffs.sort_by(|left, right| left.partial_cmp(right).unwrap_or(std::cmp::Ordering::Equal));
    let sample_count = abs_diffs.len();
    let rms = if sample_count > 0 {
        (sum_sq / sample_count as f64).sqrt()
    } else {
        f64::NAN
    };
    let p95 = percentile_sorted(&abs_diffs, 0.95);
    let max_abs = abs_diffs.last().copied().unwrap_or(f64::NAN);
    let threshold = 0.5_f64.max(0.10 * max_model_ay);
    let reliable = rms.is_finite() && rms <= threshold;

    JsonValue::Object(vec![
        (
            "sample_count".to_owned(),
            JsonValue::Integer(sample_count as i64),
        ),
        (
            "ay_model_source".to_owned(),
            "trajectory_result.ay_mps2".into(),
        ),
        (
            "ay_xy_formula".to_owned(),
            "v_mps^2 * signed_three_point_curvature(final_xy)".into(),
        ),
        ("rms_diff_mps2".to_owned(), json_number(rms)),
        ("p95_abs_diff_mps2".to_owned(), json_number(p95)),
        ("max_abs_diff_mps2".to_owned(), json_number(max_abs)),
        (
            "reliability_threshold_mps2".to_owned(),
            json_number(threshold),
        ),
        (
            "overlay_conclusions_reliable".to_owned(),
            JsonValue::Bool(reliable),
        ),
        (
            "warning".to_owned(),
            if reliable {
                JsonValue::Null
            } else {
                "large ay_model vs v^2*kappa_xy mismatch; geometry-only overlay conclusions need caution".into()
            },
        ),
        (
            "worst_samples".to_owned(),
            JsonValue::Array(
                samples
                    .iter()
                    .take(8)
                    .map(ay_consistency_sample_json)
                    .collect(),
            ),
        ),
    ])
}

fn ay_consistency_sample_json(sample: &AyConsistencySample) -> JsonValue {
    JsonValue::Object(vec![
        (
            "station_index".to_owned(),
            JsonValue::Integer(sample.station as i64),
        ),
        (
            "ay_model_mps2".to_owned(),
            json_number(sample.ay_model_mps2),
        ),
        ("ay_xy_mps2".to_owned(), json_number(sample.ay_xy_mps2)),
        (
            "ay_model_minus_xy_mps2".to_owned(),
            json_number(sample.diff_mps2),
        ),
        ("kappa_xy_1pm".to_owned(), json_number(sample.kappa_xy_1pm)),
        (
            "kappa_published_1pm".to_owned(),
            json_number(sample.kappa_published_1pm),
        ),
        ("speed_mps".to_owned(), json_number(sample.speed_mps)),
    ])
}

#[derive(Clone, Debug)]
struct StationTrajectoryConsistencySample {
    station: usize,
    ay_model_mps2: f64,
    ay_xy_mps2: f64,
    ay_model_minus_xy_mps2: f64,
    kappa_dyn_1pm: f64,
    kappa_yaw_1pm: f64,
    kappa_xy_1pm: f64,
    kappa_ref_1pm: f64,
    kappa_dyn_minus_xy_1pm: f64,
    kappa_yaw_minus_xy_1pm: f64,
    v_mps: f64,
    n_m: f64,
    dn_ds_model: f64,
    dn_ds_fd: f64,
    dn_ds_model_minus_fd: f64,
    xi_rad: f64,
    beta_rad: f64,
    omega_z_radps: f64,
    heading_path_rad: f64,
    heading_state_rad: f64,
    heading_vehicle_rad: f64,
    heading_path_minus_state_rad: f64,
    heading_path_minus_vehicle_rad: f64,
    kappa_from_heading_path_1pm: f64,
    kappa_from_heading_state_1pm: f64,
    kappa_from_heading_vehicle_1pm: f64,
    kappa_heading_path_minus_xy_1pm: f64,
    kappa_heading_state_minus_dyn_1pm: f64,
    kappa_heading_vehicle_minus_dyn_1pm: f64,
    ds_prev_m: f64,
    ds_next_m: f64,
}

fn car_station_trajectory_consistency_audit_json(
    problem: &CarMintimeNlpProblem,
    x: &[f64],
    closed: bool,
) -> JsonValue {
    let count = problem.seed.dimensions.station_count;
    let final_points = (0..count)
        .map(|station| station_xy_from(&problem.seed, x, station))
        .collect::<Vec<_>>();
    let n_values = (0..count)
        .map(|station| car_state_from(&problem.seed, x, station).n_m)
        .collect::<Vec<_>>();
    let dn_values = (0..count)
        .map(|station| car_finite_station_derivative(&problem.seed, &n_values, station, closed))
        .collect::<Vec<_>>();
    let heading_path_values = (0..count)
        .map(|station| path_heading_rad(&final_points, station, closed))
        .collect::<Vec<_>>();
    let heading_state_values = (0..count)
        .map(|station| {
            let state = car_state_from(&problem.seed, x, station);
            let ref_heading = problem.seed.ref_tangent_xy[station][1]
                .atan2(problem.seed.ref_tangent_xy[station][0]);
            normalize_angle_rad(ref_heading + state.xi_rad)
        })
        .collect::<Vec<_>>();
    let heading_vehicle_values = (0..count)
        .map(|station| {
            let state = car_state_from(&problem.seed, x, station);
            let ref_heading = problem.seed.ref_tangent_xy[station][1]
                .atan2(problem.seed.ref_tangent_xy[station][0]);
            normalize_angle_rad(ref_heading + state.xi_rad + state.beta_rad)
        })
        .collect::<Vec<_>>();
    let rows = (0..count)
        .filter(|station| closed || (*station > 0 && *station + 1 < count))
        .map(|station| {
            car_station_trajectory_consistency_sample(
                problem,
                x,
                &final_points,
                &dn_values,
                &heading_path_values,
                &heading_state_values,
                &heading_vehicle_values,
                station,
                closed,
            )
        })
        .collect::<Vec<_>>();

    station_trajectory_consistency_audit_json("car", rows, count)
}

fn station_trajectory_consistency_audit_json(
    model_family: &str,
    rows: Vec<StationTrajectoryConsistencySample>,
    count: usize,
) -> JsonValue {
    let ay_diffs = rows
        .iter()
        .map(|sample| sample.ay_model_minus_xy_mps2)
        .collect::<Vec<_>>();
    let heading_state_diffs = rows
        .iter()
        .map(|sample| sample.heading_path_minus_state_rad)
        .collect::<Vec<_>>();
    let heading_vehicle_diffs = rows
        .iter()
        .map(|sample| sample.heading_path_minus_vehicle_rad)
        .collect::<Vec<_>>();
    let dn_diffs = rows
        .iter()
        .map(|sample| sample.dn_ds_model_minus_fd)
        .collect::<Vec<_>>();
    let kappa_dyn_diffs = rows
        .iter()
        .map(|sample| sample.kappa_dyn_minus_xy_1pm)
        .collect::<Vec<_>>();
    let kappa_yaw_diffs = rows
        .iter()
        .map(|sample| sample.kappa_yaw_minus_xy_1pm)
        .collect::<Vec<_>>();
    let kappa_heading_path_diffs = rows
        .iter()
        .map(|sample| sample.kappa_heading_path_minus_xy_1pm)
        .collect::<Vec<_>>();
    let kappa_heading_state_diffs = rows
        .iter()
        .map(|sample| sample.kappa_heading_state_minus_dyn_1pm)
        .collect::<Vec<_>>();
    let kappa_heading_vehicle_diffs = rows
        .iter()
        .map(|sample| sample.kappa_heading_vehicle_minus_dyn_1pm)
        .collect::<Vec<_>>();

    JsonValue::Object(vec![
        (
            "schema_version".to_owned(),
            "station_trajectory_consistency_audit.v2".into(),
        ),
        ("model_family".to_owned(), model_family.into()),
        (
            "interpretation".to_owned(),
            "Compares station-offset published XY with the station dynamics frame. heading_state=ref_heading+xi; heading_vehicle=ref_heading+xi+beta; kappa_dyn=ay_model/v^2; kappa_yaw=omega_z/v; dn/ds_model is the NLP path derivative.".into(),
        ),
        ("station_count".to_owned(), JsonValue::Integer(count as i64)),
        (
            "sample_count".to_owned(),
            JsonValue::Integer(rows.len() as i64),
        ),
        (
            "summary".to_owned(),
            JsonValue::Object(vec![
                (
                    "ay_model_minus_xy_mps2".to_owned(),
                    signed_delta_stats_json(&ay_diffs),
                ),
                (
                    "heading_path_minus_state_rad".to_owned(),
                    signed_delta_stats_json(&heading_state_diffs),
                ),
                (
                    "heading_path_minus_vehicle_rad".to_owned(),
                    signed_delta_stats_json(&heading_vehicle_diffs),
                ),
                (
                    "dn_ds_model_minus_fd".to_owned(),
                    signed_delta_stats_json(&dn_diffs),
                ),
                (
                    "kappa_dyn_minus_xy_1pm".to_owned(),
                    signed_delta_stats_json(&kappa_dyn_diffs),
                ),
                (
                    "kappa_yaw_minus_xy_1pm".to_owned(),
                    signed_delta_stats_json(&kappa_yaw_diffs),
                ),
                (
                    "kappa_heading_path_minus_xy_1pm".to_owned(),
                    signed_delta_stats_json(&kappa_heading_path_diffs),
                ),
                (
                    "kappa_heading_state_minus_dyn_1pm".to_owned(),
                    signed_delta_stats_json(&kappa_heading_state_diffs),
                ),
                (
                    "kappa_heading_vehicle_minus_dyn_1pm".to_owned(),
                    signed_delta_stats_json(&kappa_heading_vehicle_diffs),
                ),
            ]),
        ),
        (
            "top_ay_mismatch_stations".to_owned(),
            JsonValue::Array(top_station_consistency_rows(&rows, |sample| {
                sample.ay_model_minus_xy_mps2.abs()
            })),
        ),
        (
            "top_heading_state_mismatch_stations".to_owned(),
            JsonValue::Array(top_station_consistency_rows(&rows, |sample| {
                sample.heading_path_minus_state_rad.abs()
            })),
        ),
        (
            "top_heading_vehicle_mismatch_stations".to_owned(),
            JsonValue::Array(top_station_consistency_rows(&rows, |sample| {
                sample.heading_path_minus_vehicle_rad.abs()
            })),
        ),
        (
            "top_dn_ds_mismatch_stations".to_owned(),
            JsonValue::Array(top_station_consistency_rows(&rows, |sample| {
                sample.dn_ds_model_minus_fd.abs()
            })),
        ),
        (
            "top_kappa_dyn_mismatch_stations".to_owned(),
            JsonValue::Array(top_station_consistency_rows(&rows, |sample| {
                sample.kappa_dyn_minus_xy_1pm.abs()
            })),
        ),
        (
            "top_heading_curvature_state_mismatch_stations".to_owned(),
            JsonValue::Array(top_station_consistency_rows(&rows, |sample| {
                sample.kappa_heading_state_minus_dyn_1pm.abs()
            })),
        ),
        (
            "top_heading_curvature_vehicle_mismatch_stations".to_owned(),
            JsonValue::Array(top_station_consistency_rows(&rows, |sample| {
                sample.kappa_heading_vehicle_minus_dyn_1pm.abs()
            })),
        ),
        (
            "focus_stations".to_owned(),
            JsonValue::Array(
                [12_usize, 16, 22, 0, count / 4, count / 2, (3 * count) / 4]
                    .into_iter()
                    .filter_map(|station| rows.iter().find(|row| row.station == station))
                    .map(station_trajectory_consistency_sample_json)
                    .collect(),
            ),
        ),
    ])
}

fn top_station_consistency_rows(
    rows: &[StationTrajectoryConsistencySample],
    score: impl Fn(&StationTrajectoryConsistencySample) -> f64,
) -> Vec<JsonValue> {
    let mut sorted = rows.iter().collect::<Vec<_>>();
    sorted.sort_by(|left, right| {
        score(right)
            .partial_cmp(&score(left))
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    sorted
        .into_iter()
        .take(12)
        .map(station_trajectory_consistency_sample_json)
        .collect()
}

fn signed_delta_stats_json(values: &[f64]) -> JsonValue {
    let finite = values
        .iter()
        .copied()
        .filter(|value| value.is_finite())
        .collect::<Vec<_>>();
    if finite.is_empty() {
        return JsonValue::Object(vec![("count".to_owned(), JsonValue::Integer(0))]);
    }

    let count = finite.len();
    let mean = finite.iter().sum::<f64>() / count as f64;
    let rms = (finite.iter().map(|value| value * value).sum::<f64>() / count as f64).sqrt();
    let mut abs_values = finite.iter().map(|value| value.abs()).collect::<Vec<_>>();
    abs_values.sort_by(|left, right| left.partial_cmp(right).unwrap_or(std::cmp::Ordering::Equal));
    JsonValue::Object(vec![
        ("count".to_owned(), JsonValue::Integer(count as i64)),
        ("mean".to_owned(), json_number(mean)),
        ("rms".to_owned(), json_number(rms)),
        (
            "p95_abs".to_owned(),
            json_number(percentile_sorted(&abs_values, 0.95)),
        ),
        (
            "max_abs".to_owned(),
            json_number(abs_values.last().copied().unwrap_or(f64::NAN)),
        ),
    ])
}

fn car_station_trajectory_consistency_sample(
    problem: &CarMintimeNlpProblem,
    x: &[f64],
    final_points: &[Point2],
    dn_values: &[f64],
    heading_path_values: &[f64],
    heading_state_values: &[f64],
    heading_vehicle_values: &[f64],
    station: usize,
    closed: bool,
) -> StationTrajectoryConsistencySample {
    let seed = &problem.seed;
    let state = car_state_from(seed, x, station);
    let interval = station.min(seed.dimensions.interval_count.saturating_sub(1));
    let dynamics = car_mintime_dynamics_from(seed, problem.params, x, interval);
    let kappa_xy_1pm = station_xy_curvature_1pm(final_points, station, closed);
    let ay_xy_mps2 = state.v_mps * state.v_mps * kappa_xy_1pm;
    let v_safe = state.v_mps.abs().max(1e-6);
    let ref_heading_rad = seed.ref_tangent_xy[station][1].atan2(seed.ref_tangent_xy[station][0]);
    let heading_state_rad = normalize_angle_rad(ref_heading_rad + state.xi_rad);
    let heading_vehicle_rad = normalize_angle_rad(ref_heading_rad + state.xi_rad + state.beta_rad);
    let heading_path_rad = path_heading_rad(final_points, station, closed);
    let kappa_from_heading_path_1pm =
        car_finite_angle_station_derivative(seed, heading_path_values, station, closed);
    let kappa_from_heading_state_1pm =
        car_finite_angle_station_derivative(seed, heading_state_values, station, closed);
    let kappa_from_heading_vehicle_1pm =
        car_finite_angle_station_derivative(seed, heading_vehicle_values, station, closed);
    let kappa_dyn_1pm = velocity_heading_curvature_1pm(
        state.v_mps,
        state.omega_z_radps,
        dynamics.dbeta_ds,
        dynamics.sigma_dt_ds,
    );
    let prev_interval = car_previous_interval_index(seed, station);
    let next_interval = station.min(seed.dimensions.interval_count.saturating_sub(1));

    StationTrajectoryConsistencySample {
        station,
        ay_model_mps2: dynamics.ay_mps2,
        ay_xy_mps2,
        ay_model_minus_xy_mps2: dynamics.ay_mps2 - ay_xy_mps2,
        kappa_dyn_1pm,
        kappa_yaw_1pm: state.omega_z_radps / v_safe,
        kappa_xy_1pm,
        kappa_ref_1pm: seed.kappa_1pm[station],
        kappa_dyn_minus_xy_1pm: kappa_dyn_1pm - kappa_xy_1pm,
        kappa_yaw_minus_xy_1pm: state.omega_z_radps / v_safe - kappa_xy_1pm,
        v_mps: state.v_mps,
        n_m: state.n_m,
        dn_ds_model: dynamics.dn_ds,
        dn_ds_fd: dn_values[station],
        dn_ds_model_minus_fd: dynamics.dn_ds - dn_values[station],
        xi_rad: state.xi_rad,
        beta_rad: state.beta_rad,
        omega_z_radps: state.omega_z_radps,
        heading_path_rad,
        heading_state_rad,
        heading_vehicle_rad,
        heading_path_minus_state_rad: normalize_angle_rad(heading_path_rad - heading_state_rad),
        heading_path_minus_vehicle_rad: normalize_angle_rad(heading_path_rad - heading_vehicle_rad),
        kappa_from_heading_path_1pm,
        kappa_from_heading_state_1pm,
        kappa_from_heading_vehicle_1pm,
        kappa_heading_path_minus_xy_1pm: kappa_from_heading_path_1pm - kappa_xy_1pm,
        kappa_heading_state_minus_dyn_1pm: kappa_from_heading_state_1pm - kappa_dyn_1pm,
        kappa_heading_vehicle_minus_dyn_1pm: kappa_from_heading_vehicle_1pm - kappa_dyn_1pm,
        ds_prev_m: interval_ds_m(seed, prev_interval),
        ds_next_m: interval_ds_m(seed, next_interval),
    }
}

fn station_xy_curvature_1pm(points: &[Point2], station: usize, closed: bool) -> f64 {
    if points.len() < 3 || station >= points.len() {
        return f64::NAN;
    }
    if !closed && (station == 0 || station + 1 == points.len()) {
        return f64::NAN;
    }
    let previous = if station == 0 {
        points.len().saturating_sub(1)
    } else {
        station - 1
    };
    let next = if station + 1 == points.len() {
        0
    } else {
        station + 1
    };
    signed_three_point_curvature_1pm(points[previous], points[station], points[next])
}

fn car_previous_interval_index(seed: &CarMintimeNlpSeed, station: usize) -> usize {
    if station == 0 && seed_is_closed(seed) {
        seed.dimensions.interval_count.saturating_sub(1)
    } else {
        station
            .saturating_sub(1)
            .min(seed.dimensions.interval_count.saturating_sub(1))
    }
}

fn car_finite_station_derivative(
    seed: &CarMintimeNlpSeed,
    values: &[f64],
    station: usize,
    closed: bool,
) -> f64 {
    if values.is_empty() || station >= values.len() {
        return f64::NAN;
    }
    let previous = if station == 0 && closed {
        values.len().saturating_sub(1)
    } else {
        station.saturating_sub(1)
    };
    let next = if closed {
        (station + 1) % values.len()
    } else {
        (station + 1).min(values.len().saturating_sub(1))
    };
    let prev_interval = car_previous_interval_index(seed, station);
    let next_interval = station.min(seed.dimensions.interval_count.saturating_sub(1));
    let ds = (interval_ds_m(seed, prev_interval) + interval_ds_m(seed, next_interval)).max(1e-9);
    (values[next] - values[previous]) / ds
}

fn car_finite_angle_station_derivative(
    seed: &CarMintimeNlpSeed,
    values: &[f64],
    station: usize,
    closed: bool,
) -> f64 {
    if values.is_empty() || station >= values.len() {
        return f64::NAN;
    }
    let previous = if station == 0 && closed {
        values.len().saturating_sub(1)
    } else {
        station.saturating_sub(1)
    };
    let next = if closed {
        (station + 1) % values.len()
    } else {
        (station + 1).min(values.len().saturating_sub(1))
    };
    let prev_interval = car_previous_interval_index(seed, station);
    let next_interval = station.min(seed.dimensions.interval_count.saturating_sub(1));
    let ds = (interval_ds_m(seed, prev_interval) + interval_ds_m(seed, next_interval)).max(1e-9);
    normalize_angle_rad(values[next] - values[previous]) / ds
}

fn station_trajectory_consistency_sample_json(
    sample: &StationTrajectoryConsistencySample,
) -> JsonValue {
    JsonValue::Object(vec![
        (
            "station_index".to_owned(),
            JsonValue::Integer(sample.station as i64),
        ),
        (
            "ay_model_mps2".to_owned(),
            json_number(sample.ay_model_mps2),
        ),
        ("ay_xy_mps2".to_owned(), json_number(sample.ay_xy_mps2)),
        (
            "ay_model_minus_xy_mps2".to_owned(),
            json_number(sample.ay_model_minus_xy_mps2),
        ),
        (
            "kappa_dyn_1pm".to_owned(),
            json_number(sample.kappa_dyn_1pm),
        ),
        (
            "kappa_yaw_1pm".to_owned(),
            json_number(sample.kappa_yaw_1pm),
        ),
        ("kappa_xy_1pm".to_owned(), json_number(sample.kappa_xy_1pm)),
        (
            "kappa_ref_1pm".to_owned(),
            json_number(sample.kappa_ref_1pm),
        ),
        (
            "kappa_dyn_minus_xy_1pm".to_owned(),
            json_number(sample.kappa_dyn_minus_xy_1pm),
        ),
        (
            "kappa_yaw_minus_xy_1pm".to_owned(),
            json_number(sample.kappa_yaw_minus_xy_1pm),
        ),
        ("v_mps".to_owned(), json_number(sample.v_mps)),
        ("n_m".to_owned(), json_number(sample.n_m)),
        ("dn_ds_model".to_owned(), json_number(sample.dn_ds_model)),
        ("dn_ds_fd".to_owned(), json_number(sample.dn_ds_fd)),
        (
            "dn_ds_model_minus_fd".to_owned(),
            json_number(sample.dn_ds_model_minus_fd),
        ),
        ("xi_rad".to_owned(), json_number(sample.xi_rad)),
        ("beta_rad".to_owned(), json_number(sample.beta_rad)),
        (
            "omega_z_radps".to_owned(),
            json_number(sample.omega_z_radps),
        ),
        (
            "heading_path_rad".to_owned(),
            json_number(sample.heading_path_rad),
        ),
        (
            "heading_state_rad".to_owned(),
            json_number(sample.heading_state_rad),
        ),
        (
            "heading_vehicle_rad".to_owned(),
            json_number(sample.heading_vehicle_rad),
        ),
        (
            "heading_path_minus_state_rad".to_owned(),
            json_number(sample.heading_path_minus_state_rad),
        ),
        (
            "heading_path_minus_vehicle_rad".to_owned(),
            json_number(sample.heading_path_minus_vehicle_rad),
        ),
        (
            "kappa_from_heading_path_1pm".to_owned(),
            json_number(sample.kappa_from_heading_path_1pm),
        ),
        (
            "kappa_from_heading_state_1pm".to_owned(),
            json_number(sample.kappa_from_heading_state_1pm),
        ),
        (
            "kappa_from_heading_vehicle_1pm".to_owned(),
            json_number(sample.kappa_from_heading_vehicle_1pm),
        ),
        (
            "kappa_heading_path_minus_xy_1pm".to_owned(),
            json_number(sample.kappa_heading_path_minus_xy_1pm),
        ),
        (
            "kappa_heading_state_minus_dyn_1pm".to_owned(),
            json_number(sample.kappa_heading_state_minus_dyn_1pm),
        ),
        (
            "kappa_heading_vehicle_minus_dyn_1pm".to_owned(),
            json_number(sample.kappa_heading_vehicle_minus_dyn_1pm),
        ),
        ("ds_prev_m".to_owned(), json_number(sample.ds_prev_m)),
        ("ds_next_m".to_owned(), json_number(sample.ds_next_m)),
    ])
}

#[derive(Clone, Debug)]
struct LocalStationIntervalConsistencySample {
    interval: usize,
    station: usize,
    next_station: usize,
    published_step_length_m: f64,
    euler_state_step_length_m: f64,
    euler_vehicle_step_length_m: f64,
    euler_state_step_length_error_m: f64,
    euler_vehicle_step_length_error_m: f64,
    published_step_heading_rad: f64,
    euler_state_step_heading_rad: f64,
    euler_vehicle_step_heading_rad: f64,
    euler_state_step_heading_error_rad: f64,
    euler_vehicle_step_heading_error_rad: f64,
    euler_state_step_vector_error_m: f64,
    euler_vehicle_step_vector_error_m: f64,
    kappa_xy_1pm: f64,
    kappa_dyn_1pm: f64,
    kappa_yaw_1pm: f64,
    kappa_from_heading_state_1pm: f64,
    kappa_from_heading_vehicle_1pm: f64,
    dn_ds_model: f64,
    dn_ds_fd: f64,
    dn_ds_model_minus_fd: f64,
    v_mps: f64,
    n_m: f64,
    xi_rad: f64,
    beta_rad: f64,
    omega_z_radps: f64,
    sigma_dt_ds: f64,
    ds_m: f64,
}

fn car_local_station_interval_consistency_audit_json(
    problem: &CarMintimeNlpProblem,
    x: &[f64],
    closed: bool,
) -> JsonValue {
    // Accumulating a diagnostic trajectory around the full lap was tried and removed:
    // small per-interval drift turns into misleading loop-scale offsets.
    let seed = &problem.seed;
    let count = seed.dimensions.station_count;
    let final_points = (0..count)
        .map(|station| station_xy_from(seed, x, station))
        .collect::<Vec<_>>();
    let n_values = (0..count)
        .map(|station| car_state_from(seed, x, station).n_m)
        .collect::<Vec<_>>();
    let dn_values = (0..count)
        .map(|station| car_finite_station_derivative(seed, &n_values, station, closed))
        .collect::<Vec<_>>();
    let heading_state_values = (0..count)
        .map(|station| {
            let state = car_state_from(seed, x, station);
            let ref_heading =
                seed.ref_tangent_xy[station][1].atan2(seed.ref_tangent_xy[station][0]);
            normalize_angle_rad(ref_heading + state.xi_rad)
        })
        .collect::<Vec<_>>();
    let heading_vehicle_values = (0..count)
        .map(|station| {
            let state = car_state_from(seed, x, station);
            let ref_heading =
                seed.ref_tangent_xy[station][1].atan2(seed.ref_tangent_xy[station][0]);
            normalize_angle_rad(ref_heading + state.xi_rad + state.beta_rad)
        })
        .collect::<Vec<_>>();
    let rows = (0..seed.dimensions.interval_count)
        .map(|interval| {
            car_local_station_interval_consistency_sample(
                problem,
                x,
                &final_points,
                &dn_values,
                &heading_state_values,
                &heading_vehicle_values,
                interval,
                closed,
            )
        })
        .collect::<Vec<_>>();
    local_station_interval_consistency_audit_json("car", rows, count)
}

fn local_station_interval_consistency_audit_json(
    model_family: &str,
    rows: Vec<LocalStationIntervalConsistencySample>,
    station_count: usize,
) -> JsonValue {
    let state_length = rows
        .iter()
        .map(|sample| sample.euler_state_step_length_error_m)
        .collect::<Vec<_>>();
    let vehicle_length = rows
        .iter()
        .map(|sample| sample.euler_vehicle_step_length_error_m)
        .collect::<Vec<_>>();
    let state_heading = rows
        .iter()
        .map(|sample| sample.euler_state_step_heading_error_rad)
        .collect::<Vec<_>>();
    let vehicle_heading = rows
        .iter()
        .map(|sample| sample.euler_vehicle_step_heading_error_rad)
        .collect::<Vec<_>>();
    let state_vector = rows
        .iter()
        .map(|sample| sample.euler_state_step_vector_error_m)
        .collect::<Vec<_>>();
    let vehicle_vector = rows
        .iter()
        .map(|sample| sample.euler_vehicle_step_vector_error_m)
        .collect::<Vec<_>>();
    let dn_diffs = rows
        .iter()
        .map(|sample| sample.dn_ds_model_minus_fd)
        .collect::<Vec<_>>();

    JsonValue::Object(vec![
        (
            "schema_version".to_owned(),
            "local_station_interval_debug_audit.v2".into(),
        ),
        ("model_family".to_owned(), model_family.into()),
        (
            "interpretation".to_owned(),
            "Weak debug-only audit. The step fields are single-interval forward-Euler predictions from the interval start state; they are useful for local inspection but are not trajectory-contract evidence and are not acceptance metrics.".into(),
        ),
        (
            "acceptance_use".to_owned(),
            JsonValue::Bool(false),
        ),
        (
            "evidence_level".to_owned(),
            "weak_debug_only".into(),
        ),
        (
            "station_count".to_owned(),
            JsonValue::Integer(station_count as i64),
        ),
        (
            "interval_count".to_owned(),
            JsonValue::Integer(rows.len() as i64),
        ),
        (
            "summary".to_owned(),
            JsonValue::Object(vec![
                (
                    "euler_state_step_length_error_m".to_owned(),
                    signed_delta_stats_json(&state_length),
                ),
                (
                    "euler_vehicle_step_length_error_m".to_owned(),
                    signed_delta_stats_json(&vehicle_length),
                ),
                (
                    "euler_state_step_heading_error_rad".to_owned(),
                    signed_delta_stats_json(&state_heading),
                ),
                (
                    "euler_vehicle_step_heading_error_rad".to_owned(),
                    signed_delta_stats_json(&vehicle_heading),
                ),
                (
                    "euler_state_step_vector_debug_error_m".to_owned(),
                    signed_delta_stats_json(&state_vector),
                ),
                (
                    "euler_vehicle_step_vector_debug_error_m".to_owned(),
                    signed_delta_stats_json(&vehicle_vector),
                ),
                (
                    "dn_ds_model_minus_fd".to_owned(),
                    signed_delta_stats_json(&dn_diffs),
                ),
            ]),
        ),
        (
            "focus_stations".to_owned(),
            JsonValue::Array(
                [
                    12_usize,
                    16,
                    22,
                    0,
                    station_count / 4,
                    station_count / 2,
                    (3 * station_count) / 4,
                ]
                .into_iter()
                .filter_map(|station| rows.iter().find(|row| row.station == station))
                .map(local_station_interval_consistency_sample_json)
                .collect(),
            ),
        ),
        (
            "top_euler_state_step_vector_debug_intervals".to_owned(),
            JsonValue::Array(top_local_station_interval_rows(&rows, |sample| {
                sample.euler_state_step_vector_error_m
            })),
        ),
        (
            "top_euler_vehicle_step_vector_debug_intervals".to_owned(),
            JsonValue::Array(top_local_station_interval_rows(&rows, |sample| {
                sample.euler_vehicle_step_vector_error_m
            })),
        ),
    ])
}

fn car_local_station_interval_consistency_sample(
    problem: &CarMintimeNlpProblem,
    x: &[f64],
    final_points: &[Point2],
    dn_values: &[f64],
    heading_state_values: &[f64],
    heading_vehicle_values: &[f64],
    interval: usize,
    closed: bool,
) -> LocalStationIntervalConsistencySample {
    let seed = &problem.seed;
    let station = interval.min(seed.dimensions.station_count.saturating_sub(1));
    let next_station = next_station_index(seed, interval);
    let state = car_state_from(seed, x, station);
    let dynamics = car_mintime_dynamics_from(seed, problem.params, x, interval);
    let ds_m = interval_ds_m(seed, interval);
    let published_step = [
        final_points[next_station][0] - final_points[station][0],
        final_points[next_station][1] - final_points[station][1],
    ];
    let published_step_length_m = published_step[0].hypot(published_step[1]);
    let published_step_heading_rad = published_step[1].atan2(published_step[0]);
    let ref_heading_rad = seed.ref_tangent_xy[station][1].atan2(seed.ref_tangent_xy[station][0]);
    let euler_state_step_heading_rad = normalize_angle_rad(ref_heading_rad + state.xi_rad);
    let euler_vehicle_step_heading_rad =
        normalize_angle_rad(ref_heading_rad + state.xi_rad + state.beta_rad);
    let euler_state_step_length_m = ds_m * (1.0 - state.n_m * seed.kappa_1pm[station])
        / signed_max_abs(state.xi_rad.cos(), 1e-6);
    let euler_vehicle_step_length_m = ds_m * state.v_mps * dynamics.sigma_dt_ds;
    let euler_state_step = [
        euler_state_step_length_m * euler_state_step_heading_rad.cos(),
        euler_state_step_length_m * euler_state_step_heading_rad.sin(),
    ];
    let euler_vehicle_step = [
        euler_vehicle_step_length_m * euler_vehicle_step_heading_rad.cos(),
        euler_vehicle_step_length_m * euler_vehicle_step_heading_rad.sin(),
    ];
    let kappa_dyn_1pm = velocity_heading_curvature_1pm(
        state.v_mps,
        state.omega_z_radps,
        dynamics.dbeta_ds,
        dynamics.sigma_dt_ds,
    );
    let kappa_yaw_1pm = state.omega_z_radps / state.v_mps.abs().max(1e-6);

    LocalStationIntervalConsistencySample {
        interval,
        station,
        next_station,
        published_step_length_m,
        euler_state_step_length_m,
        euler_vehicle_step_length_m,
        euler_state_step_length_error_m: euler_state_step_length_m - published_step_length_m,
        euler_vehicle_step_length_error_m: euler_vehicle_step_length_m - published_step_length_m,
        published_step_heading_rad,
        euler_state_step_heading_rad,
        euler_vehicle_step_heading_rad,
        euler_state_step_heading_error_rad: normalize_angle_rad(
            euler_state_step_heading_rad - published_step_heading_rad,
        ),
        euler_vehicle_step_heading_error_rad: normalize_angle_rad(
            euler_vehicle_step_heading_rad - published_step_heading_rad,
        ),
        euler_state_step_vector_error_m: (euler_state_step[0] - published_step[0])
            .hypot(euler_state_step[1] - published_step[1]),
        euler_vehicle_step_vector_error_m: (euler_vehicle_step[0] - published_step[0])
            .hypot(euler_vehicle_step[1] - published_step[1]),
        kappa_xy_1pm: station_xy_curvature_1pm(final_points, station, closed),
        kappa_dyn_1pm,
        kappa_yaw_1pm,
        kappa_from_heading_state_1pm: car_finite_angle_station_derivative(
            seed,
            heading_state_values,
            station,
            closed,
        ),
        kappa_from_heading_vehicle_1pm: car_finite_angle_station_derivative(
            seed,
            heading_vehicle_values,
            station,
            closed,
        ),
        dn_ds_model: dynamics.dn_ds,
        dn_ds_fd: dn_values[station],
        dn_ds_model_minus_fd: dynamics.dn_ds - dn_values[station],
        v_mps: state.v_mps,
        n_m: state.n_m,
        xi_rad: state.xi_rad,
        beta_rad: state.beta_rad,
        omega_z_radps: state.omega_z_radps,
        sigma_dt_ds: dynamics.sigma_dt_ds,
        ds_m,
    }
}

fn top_local_station_interval_rows(
    rows: &[LocalStationIntervalConsistencySample],
    score: impl Fn(&LocalStationIntervalConsistencySample) -> f64,
) -> Vec<JsonValue> {
    let mut sorted = rows.iter().collect::<Vec<_>>();
    sorted.sort_by(|left, right| {
        score(right)
            .partial_cmp(&score(left))
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    sorted
        .into_iter()
        .take(12)
        .map(local_station_interval_consistency_sample_json)
        .collect()
}

fn local_station_interval_consistency_sample_json(
    sample: &LocalStationIntervalConsistencySample,
) -> JsonValue {
    JsonValue::Object(vec![
        (
            "interval_index".to_owned(),
            JsonValue::Integer(sample.interval as i64),
        ),
        (
            "station_index".to_owned(),
            JsonValue::Integer(sample.station as i64),
        ),
        (
            "next_station_index".to_owned(),
            JsonValue::Integer(sample.next_station as i64),
        ),
        (
            "published_step_length_m".to_owned(),
            json_number(sample.published_step_length_m),
        ),
        (
            "euler_state_step_length_m".to_owned(),
            json_number(sample.euler_state_step_length_m),
        ),
        (
            "euler_vehicle_step_length_m".to_owned(),
            json_number(sample.euler_vehicle_step_length_m),
        ),
        (
            "euler_state_step_length_error_m".to_owned(),
            json_number(sample.euler_state_step_length_error_m),
        ),
        (
            "euler_vehicle_step_length_error_m".to_owned(),
            json_number(sample.euler_vehicle_step_length_error_m),
        ),
        (
            "published_step_heading_rad".to_owned(),
            json_number(sample.published_step_heading_rad),
        ),
        (
            "euler_state_step_heading_rad".to_owned(),
            json_number(sample.euler_state_step_heading_rad),
        ),
        (
            "euler_vehicle_step_heading_rad".to_owned(),
            json_number(sample.euler_vehicle_step_heading_rad),
        ),
        (
            "euler_state_step_heading_error_rad".to_owned(),
            json_number(sample.euler_state_step_heading_error_rad),
        ),
        (
            "euler_vehicle_step_heading_error_rad".to_owned(),
            json_number(sample.euler_vehicle_step_heading_error_rad),
        ),
        (
            "euler_state_step_vector_debug_error_m".to_owned(),
            json_number(sample.euler_state_step_vector_error_m),
        ),
        (
            "euler_vehicle_step_vector_debug_error_m".to_owned(),
            json_number(sample.euler_vehicle_step_vector_error_m),
        ),
        ("kappa_xy_1pm".to_owned(), json_number(sample.kappa_xy_1pm)),
        (
            "kappa_dyn_1pm".to_owned(),
            json_number(sample.kappa_dyn_1pm),
        ),
        (
            "kappa_yaw_1pm".to_owned(),
            json_number(sample.kappa_yaw_1pm),
        ),
        (
            "kappa_from_heading_state_1pm".to_owned(),
            json_number(sample.kappa_from_heading_state_1pm),
        ),
        (
            "kappa_from_heading_vehicle_1pm".to_owned(),
            json_number(sample.kappa_from_heading_vehicle_1pm),
        ),
        ("dn_ds_model".to_owned(), json_number(sample.dn_ds_model)),
        ("dn_ds_fd".to_owned(), json_number(sample.dn_ds_fd)),
        (
            "dn_ds_model_minus_fd".to_owned(),
            json_number(sample.dn_ds_model_minus_fd),
        ),
        ("v_mps".to_owned(), json_number(sample.v_mps)),
        ("n_m".to_owned(), json_number(sample.n_m)),
        ("xi_rad".to_owned(), json_number(sample.xi_rad)),
        ("beta_rad".to_owned(), json_number(sample.beta_rad)),
        (
            "omega_z_radps".to_owned(),
            json_number(sample.omega_z_radps),
        ),
        ("sigma_dt_ds".to_owned(), json_number(sample.sigma_dt_ds)),
        ("ds_m".to_owned(), json_number(sample.ds_m)),
    ])
}

fn signed_three_point_curvature_1pm(previous: Point2, current: Point2, next: Point2) -> f64 {
    let ax = current[0] - previous[0];
    let ay = current[1] - previous[1];
    let bx = next[0] - current[0];
    let by = next[1] - current[1];
    let cx = next[0] - previous[0];
    let cy = next[1] - previous[1];
    let ab = ax.hypot(ay);
    let bc = bx.hypot(by);
    let ac = cx.hypot(cy);
    let denom = ab * bc * ac;
    if denom <= 1e-9 {
        return f64::NAN;
    }
    let cross = ax * by - ay * bx;
    2.0 * cross / denom
}

fn constraint_violation_context(
    problem: &CarMintimeNlpProblem,
    x: &[f64],
    row: &CarMintimeConstraintRow,
) -> String {
    match row {
        CarMintimeConstraintRow::LateralLoadTransfer { interval } => {
            let state_interval = next_station_index(&problem.seed, *interval);
            let state = car_state_from(&problem.seed, x, state_interval);
            let control = car_control_from(&problem.seed, x, *interval);
            let dynamics =
                car_mintime_path_dynamics_from(&problem.seed, problem.params, x, *interval);
            let tire = dynamics.tire_forces;
            let vehicle_part = lateral_load_transfer_vehicle_part_from(
                &problem.seed,
                problem.params,
                x,
                *interval,
            );

            format!(
                "interval={interval}, path_state_index={state_interval}, v={:.6}, beta={:.6}, omega_z={:.6}, n={:.6}, xi={:.6}, delta={:.6}, f_drive={:.6}, f_brake={:.6}, gamma_y={:.6}, vehicle_load_transfer={:.6}, ax={:.6}, ay={:.6}, sigma={:.9}, tire_util={:.6}, tire_fz=[{:.3},{:.3},{:.3},{:.3}], tire_fy=[{:.3},{:.3},{:.3},{:.3}]",
                state.v_mps,
                state.beta_rad,
                state.omega_z_radps,
                state.n_m,
                state.xi_rad,
                control.delta_rad,
                control.f_drive_n,
                control.f_brake_n,
                control.gamma_y_n,
                vehicle_part,
                dynamics.ax_mps2,
                dynamics.ay_mps2,
                dynamics.sigma_dt_ds,
                car_tire_max_utilization(tire),
                tire.fz_fl_n,
                tire.fz_fr_n,
                tire.fz_rl_n,
                tire.fz_rr_n,
                tire.fy_fl_n,
                tire.fy_fr_n,
                tire.fy_rl_n,
                tire.fy_rr_n
            )
        }
        CarMintimeConstraintRow::TireEllipse { interval, wheel } => {
            let dynamics =
                car_mintime_path_dynamics_from(&problem.seed, problem.params, x, *interval);
            let tire = dynamics.tire_forces;

            format!(
                "interval={interval}, wheel={wheel}, wheel_util={:.6}, max_tire_util={:.6}, ax={:.6}, ay={:.6}, tire_fx=[{:.3},{:.3},{:.3},{:.3}], tire_fy=[{:.3},{:.3},{:.3},{:.3}], tire_fz=[{:.3},{:.3},{:.3},{:.3}]",
                tire.wheel_ellipse_utilization(problem.params, wheel),
                car_tire_max_utilization(tire),
                dynamics.ax_mps2,
                dynamics.ay_mps2,
                tire.fx_fl_n,
                tire.fx_fr_n,
                tire.fx_rl_n,
                tire.fx_rr_n,
                tire.fy_fl_n,
                tire.fy_fr_n,
                tire.fy_rl_n,
                tire.fy_rr_n,
                tire.fz_fl_n,
                tire.fz_fr_n,
                tire.fz_rl_n,
                tire.fz_rr_n
            )
        }
        CarMintimeConstraintRow::CollocationTireEllipse {
            interval,
            point,
            wheel,
        } => {
            let dynamics = car_mintime_collocation_dynamics_from(
                &problem.seed,
                problem.params,
                x,
                *interval,
                *point,
            );
            let tire = dynamics.tire_forces;

            format!(
                "interval={interval}, point={point}, wheel={wheel}, wheel_util={:.6}, max_tire_util={:.6}, ax={:.6}, ay={:.6}, tire_fx=[{:.3},{:.3},{:.3},{:.3}], tire_fy=[{:.3},{:.3},{:.3},{:.3}], tire_fz=[{:.3},{:.3},{:.3},{:.3}]",
                tire.wheel_ellipse_utilization(problem.params, wheel),
                car_tire_max_utilization(tire),
                dynamics.ax_mps2,
                dynamics.ay_mps2,
                tire.fx_fl_n,
                tire.fx_fr_n,
                tire.fx_rl_n,
                tire.fx_rr_n,
                tire.fy_fl_n,
                tire.fy_fr_n,
                tire.fy_rl_n,
                tire.fy_rr_n,
                tire.fz_fl_n,
                tire.fz_fr_n,
                tire.fz_rl_n,
                tire.fz_rr_n
            )
        }
        CarMintimeConstraintRow::NormalLoad { interval, wheel } => {
            let dynamics =
                car_mintime_path_dynamics_from(&problem.seed, problem.params, x, *interval);
            let (_, _, fz_n, _, _) =
                car_wheel_force_values(dynamics.tire_forces, problem.params, wheel);
            format!("interval={interval}, wheel={wheel}, fz={fz_n:.6}")
        }
        CarMintimeConstraintRow::CollocationNormalLoad {
            interval,
            point,
            wheel,
        } => {
            let dynamics = car_mintime_collocation_dynamics_from(
                &problem.seed,
                problem.params,
                x,
                *interval,
                *point,
            );
            let (_, _, fz_n, _, _) =
                car_wheel_force_values(dynamics.tire_forces, problem.params, wheel);
            format!("interval={interval}, point={point}, wheel={wheel}, fz={fz_n:.6}")
        }
        CarMintimeConstraintRow::DriveBrakeMutex { interval } => {
            let control = car_control_from(&problem.seed, x, *interval);

            format!(
                "interval={interval}, f_drive={:.6}, f_brake={:.6}, product={:.6}",
                control.f_drive_n,
                control.f_brake_n,
                control.f_drive_n * control.f_brake_n
            )
        }
        CarMintimeConstraintRow::CollocationDynamics {
            interval,
            point,
            state_name,
        } => {
            let dynamics = car_mintime_collocation_dynamics_from(
                &problem.seed,
                problem.params,
                x,
                *interval,
                *point,
            );
            let state = collocation_state_from(&problem.seed, x, *interval, point - 1);
            let control = car_control_from(&problem.seed, x, *interval);

            format!(
                "interval={interval}, point={point}, state={state_name}, v={:.6}, beta={:.6}, omega_z={:.6}, n={:.6}, xi={:.6}, delta={:.6}, f_drive={:.6}, f_brake={:.6}, gamma_y={:.6}, ax={:.6}, ay={:.6}, sigma={:.9}, domega_z_ds={:.9}",
                state.v_mps,
                state.beta_rad,
                state.omega_z_radps,
                state.n_m,
                state.xi_rad,
                control.delta_rad,
                control.f_drive_n,
                control.f_brake_n,
                control.gamma_y_n,
                dynamics.ax_mps2,
                dynamics.ay_mps2,
                dynamics.sigma_dt_ds,
                dynamics.domega_z_ds
            )
        }
        CarMintimeConstraintRow::Continuity {
            interval,
            state_name,
        } => format!("interval={interval}, state={state_name}"),
        CarMintimeConstraintRow::Dynamics {
            interval,
            state_name,
        } => format!("interval={interval}, state={state_name}"),
        CarMintimeConstraintRow::PowerLimit { interval } => {
            let control = car_control_from(&problem.seed, x, *interval);
            let state = car_state_from(
                &problem.seed,
                x,
                next_station_index(&problem.seed, *interval),
            );

            format!(
                "interval={interval}, v={:.6}, f_drive={:.6}, power={:.6}",
                state.v_mps,
                control.f_drive_n,
                state.v_mps * control.f_drive_n
            )
        }
        CarMintimeConstraintRow::CollocationPowerLimit { interval, point } => {
            let control = car_control_from(&problem.seed, x, *interval);
            let state = collocation_state_from(&problem.seed, x, *interval, point - 1);

            format!(
                "interval={interval}, point={point}, v={:.6}, f_drive={:.6}, power={:.6}",
                state.v_mps,
                control.f_drive_n,
                state.v_mps * control.f_drive_n
            )
        }
        CarMintimeConstraintRow::SlipPrepeak { interval, wheel } => {
            let tire = car_mintime_path_dynamics_from(&problem.seed, problem.params, x, *interval)
                .tire_forces;
            format!(
                "interval={interval}, wheel={wheel}, alpha_rad={:.9}, alpha_peak_rad={:.9}",
                car_wheel_slip_rad(tire, wheel),
                car_pacejka_peak_slip_rad(problem.params, wheel)
            )
        }
        CarMintimeConstraintRow::CollocationSlipPrepeak {
            interval,
            point,
            wheel,
        } => {
            let tire = car_mintime_collocation_dynamics_from(
                &problem.seed,
                problem.params,
                x,
                *interval,
                *point,
            )
            .tire_forces;
            format!(
                "interval={interval}, point={point}, wheel={wheel}, alpha_rad={:.9}, alpha_peak_rad={:.9}",
                car_wheel_slip_rad(tire, wheel),
                car_pacejka_peak_slip_rad(problem.params, wheel)
            )
        }
        CarMintimeConstraintRow::ControlRate {
            interval,
            control_name,
        } => format!("interval={interval}, control={control_name}"),
    }
}

fn car_tire_max_utilization(tire: CarDoubleTrackTireForces) -> f64 {
    [
        tire.fx_fl_n.hypot(tire.fy_fl_n) / tire.fz_fl_n.abs().max(1e-9),
        tire.fx_fr_n.hypot(tire.fy_fr_n) / tire.fz_fr_n.abs().max(1e-9),
        tire.fx_rl_n.hypot(tire.fy_rl_n) / tire.fz_rl_n.abs().max(1e-9),
        tire.fx_rr_n.hypot(tire.fy_rr_n) / tire.fz_rr_n.abs().max(1e-9),
    ]
    .into_iter()
    .fold(0.0, f64::max)
}

fn car_state_from(seed: &CarMintimeNlpSeed, x: &[f64], station: usize) -> CarDoubleTrackState {
    CarDoubleTrackState {
        v_mps: state_value_from(seed, x, station, STATE_V_MPS),
        beta_rad: state_value_from(seed, x, station, STATE_BETA_RAD),
        omega_z_radps: state_value_from(seed, x, station, STATE_OMEGA_Z_RADPS),
        n_m: state_value_from(seed, x, station, STATE_N_M),
        xi_rad: state_value_from(seed, x, station, STATE_XI_RAD),
    }
}

fn car_control_from(seed: &CarMintimeNlpSeed, x: &[f64], interval: usize) -> CarDoubleTrackControl {
    CarDoubleTrackControl {
        delta_rad: control_value_from(seed, x, interval, CONTROL_DELTA_RAD),
        f_drive_n: control_value_from(seed, x, interval, CONTROL_F_DRIVE_N),
        f_brake_n: control_value_from(seed, x, interval, CONTROL_F_BRAKE_N),
        gamma_y_n: control_value_from(seed, x, interval, CONTROL_GAMMA_Y_N),
    }
}

fn collocation_state_from(
    seed: &CarMintimeNlpSeed,
    x: &[f64],
    interval: usize,
    point: usize,
) -> CarDoubleTrackState {
    CarDoubleTrackState {
        v_mps: collocation_state_value_from(seed, x, interval, point, STATE_V_MPS),
        beta_rad: collocation_state_value_from(seed, x, interval, point, STATE_BETA_RAD),
        omega_z_radps: collocation_state_value_from(seed, x, interval, point, STATE_OMEGA_Z_RADPS),
        n_m: collocation_state_value_from(seed, x, interval, point, STATE_N_M),
        xi_rad: collocation_state_value_from(seed, x, interval, point, STATE_XI_RAD),
    }
}

fn station_xy_from(seed: &CarMintimeNlpSeed, x: &[f64], station: usize) -> Point2 {
    let center = seed.centerline_xy_m[station];
    let normal = station_path_normal(seed, station);
    let n_m = state_value_from(seed, x, station, STATE_N_M);

    [center[0] + normal[0] * n_m, center[1] + normal[1] * n_m]
}

fn station_path_normal(seed: &CarMintimeNlpSeed, station: usize) -> Point2 {
    seed.section_dir_xy
        .get(station)
        .copied()
        .filter(|direction| direction[0].hypot(direction[1]) > 1e-9)
        .map(|direction| {
            let normalized = normalize_point(direction);
            [-normalized[0], -normalized[1]]
        })
        .unwrap_or_else(|| station_normal(seed, station, seed_is_closed(seed)))
}

fn station_normal(seed: &CarMintimeNlpSeed, station: usize, closed: bool) -> Point2 {
    let previous = if station == 0 && closed {
        seed.centerline_xy_m.len().saturating_sub(1)
    } else if station == 0 {
        0
    } else {
        station - 1
    };
    let next = if closed {
        (station + 1) % seed.centerline_xy_m.len().max(1)
    } else {
        (station + 1).min(seed.centerline_xy_m.len().saturating_sub(1))
    };
    let tangent = [
        seed.centerline_xy_m[next][0] - seed.centerline_xy_m[previous][0],
        seed.centerline_xy_m[next][1] - seed.centerline_xy_m[previous][1],
    ];
    let length = tangent[0].hypot(tangent[1]).max(1e-9);

    [-tangent[1] / length, tangent[0] / length]
}

fn path_heading_rad(points: &[Point2], station: usize, closed: bool) -> f64 {
    if points.len() < 2 {
        return 0.0;
    }

    let previous = if station == 0 && closed {
        points.len() - 1
    } else if station == 0 {
        0
    } else {
        station - 1
    };
    let next = if closed {
        (station + 1) % points.len().max(1)
    } else {
        (station + 1).min(points.len() - 1)
    };

    (points[next][1] - points[previous][1]).atan2(points[next][0] - points[previous][0])
}

fn kappa_1pm(seed: &CarMintimeNlpSeed, station: usize) -> f64 {
    seed.kappa_1pm.get(station).copied().unwrap_or(0.0)
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ClosedSectionGeometry {
    pub(crate) ref_tangent_xy: Vec<Point2>,
    pub(crate) ref_left_normal_xy: Vec<Point2>,
    pub(crate) section_dir_xy: Vec<Point2>,
    pub(crate) section_dir_derivative_xy: Vec<Point2>,
}

pub(crate) fn closed_section_geometry(
    points: &[Point2],
    station_s_m: &[f64],
    section_dirs_xy: Option<&[Point2]>,
) -> ClosedSectionGeometry {
    let ref_tangent_xy = closed_ref_tangents(points);
    let ref_left_normal_xy = ref_tangent_xy
        .iter()
        .map(|tangent| [-tangent[1], tangent[0]])
        .collect::<Vec<_>>();
    let section_dir_xy = section_dirs_xy
        .map(|dirs| {
            dirs.iter()
                .copied()
                .map(normalize_point)
                .collect::<Vec<_>>()
        })
        .unwrap_or_else(|| {
            ref_left_normal_xy
                .iter()
                .map(|normal| [-normal[0], -normal[1]])
                .collect()
        });
    let section_dir_derivative_xy =
        periodic_central_derivative(&section_dir_xy, points, station_s_m);

    ClosedSectionGeometry {
        ref_tangent_xy,
        ref_left_normal_xy,
        section_dir_xy,
        section_dir_derivative_xy,
    }
}

fn open_section_geometry(
    points: &[Point2],
    station_s_m: &[f64],
    section_dirs_xy: Option<&[Point2]>,
) -> ClosedSectionGeometry {
    let ref_tangent_xy = open_ref_tangents(points);
    let ref_left_normal_xy = ref_tangent_xy
        .iter()
        .map(|tangent| [-tangent[1], tangent[0]])
        .collect::<Vec<_>>();
    let section_dir_xy = section_dirs_xy
        .map(|dirs| {
            dirs.iter()
                .copied()
                .map(normalize_point)
                .collect::<Vec<_>>()
        })
        .unwrap_or_else(|| {
            ref_left_normal_xy
                .iter()
                .map(|normal| [-normal[0], -normal[1]])
                .collect()
        });
    let section_dir_derivative_xy = open_central_derivative(&section_dir_xy, station_s_m);

    ClosedSectionGeometry {
        ref_tangent_xy,
        ref_left_normal_xy,
        section_dir_xy,
        section_dir_derivative_xy,
    }
}

pub(crate) fn closed_chord_station_m(points: &[Point2]) -> Vec<f64> {
    if points.is_empty() {
        return Vec::new();
    }

    let mut station = Vec::with_capacity(points.len() + 1);
    station.push(0.0);
    for index in 0..points.len() {
        let next = (index + 1) % points.len();
        let ds = (points[next][0] - points[index][0]).hypot(points[next][1] - points[index][1]);
        station.push(station.last().copied().unwrap_or(0.0) + ds);
    }
    station
}

fn open_chord_station_m(points: &[Point2]) -> Vec<f64> {
    if points.is_empty() {
        return Vec::new();
    }

    let mut station = Vec::with_capacity(points.len());
    station.push(0.0);
    for pair in points.windows(2) {
        let ds = (pair[1][0] - pair[0][0]).hypot(pair[1][1] - pair[0][1]);
        station.push(station.last().copied().unwrap_or(0.0) + ds);
    }
    station
}

fn closed_ref_tangents(points: &[Point2]) -> Vec<Point2> {
    if points.is_empty() {
        return Vec::new();
    }

    (0..points.len())
        .map(|index| {
            let previous = wrap_index(index as isize - 1, points.len());
            let next = wrap_index(index as isize + 1, points.len());
            normalize_point([
                points[next][0] - points[previous][0],
                points[next][1] - points[previous][1],
            ])
        })
        .collect()
}

fn open_ref_tangents(points: &[Point2]) -> Vec<Point2> {
    match points.len() {
        0 => Vec::new(),
        1 => vec![[1.0, 0.0]],
        count => (0..count)
            .map(|index| {
                let previous = if index == 0 { 0 } else { index - 1 };
                let next = if index + 1 >= count {
                    count - 1
                } else {
                    index + 1
                };
                normalize_point([
                    points[next][0] - points[previous][0],
                    points[next][1] - points[previous][1],
                ])
            })
            .collect(),
    }
}

fn periodic_central_derivative(
    values: &[Point2],
    points: &[Point2],
    station_s_m: &[f64],
) -> Vec<Point2> {
    if values.is_empty() {
        return Vec::new();
    }

    let element_lengths = closed_element_lengths(points, station_s_m);
    let station = closed_station_from_elements(&element_lengths);
    let total_length = station.last().copied().unwrap_or(0.0).max(1e-9);

    (0..values.len())
        .map(|index| {
            let previous = wrap_index(index as isize - 1, values.len());
            let next = wrap_index(index as isize + 1, values.len());
            let ds = forward_closed_station_delta(
                station[previous],
                station[next],
                total_length,
                next < previous,
            )
            .max(1e-9);

            [
                (values[next][0] - values[previous][0]) / ds,
                (values[next][1] - values[previous][1]) / ds,
            ]
        })
        .collect()
}

fn open_central_derivative(values: &[Point2], station_s_m: &[f64]) -> Vec<Point2> {
    match values.len() {
        0 => Vec::new(),
        1 => vec![[0.0, 0.0]],
        count => (0..count)
            .map(|index| {
                let previous = if index == 0 { 0 } else { index - 1 };
                let next = if index + 1 >= count {
                    count - 1
                } else {
                    index + 1
                };
                let ds = (station_s_m.get(next).copied().unwrap_or(next as f64)
                    - station_s_m
                        .get(previous)
                        .copied()
                        .unwrap_or(previous as f64))
                .abs()
                .max(1e-9);

                [
                    (values[next][0] - values[previous][0]) / ds,
                    (values[next][1] - values[previous][1]) / ds,
                ]
            })
            .collect(),
    }
}

fn normalize_point(point: Point2) -> Point2 {
    let length = point[0].hypot(point[1]).max(1e-9);
    [point[0] / length, point[1] / length]
}

pub(crate) fn python_compatible_closed_kappa_1pm(
    points: &[Point2],
    station_s_m: &[f64],
) -> Vec<f64> {
    if points.len() < 3 {
        return vec![0.0; points.len()];
    }

    let element_lengths = python_like_closed_spline_lengths(points)
        .unwrap_or_else(|| closed_element_lengths(points, station_s_m));
    let average_ds = average_positive(&element_lengths).unwrap_or(1.0);
    let heading_preview = preview_review_index_step(1.0, average_ds);
    let heading_review = preview_review_index_step(1.0, average_ds);
    let curvature_preview = preview_review_index_step(2.0, average_ds);
    let curvature_review = preview_review_index_step(2.0, average_ds);
    let curvature_total = curvature_preview + curvature_review;

    let heading_rad = (0..points.len())
        .map(|index| {
            let previous = wrap_index(index as isize - heading_review as isize, points.len());
            let next = wrap_index(index as isize + heading_preview as isize, points.len());
            let tangent = [
                points[next][0] - points[previous][0],
                points[next][1] - points[previous][1],
            ];

            normalize_angle_rad(tangent[1].atan2(tangent[0]) - std::f64::consts::FRAC_PI_2)
        })
        .collect::<Vec<_>>();
    let station = closed_station_from_elements(&element_lengths);
    let total_length = station.last().copied().unwrap_or(0.0).max(1e-9);

    (0..points.len())
        .map(|index| {
            let previous = wrap_index(index as isize - curvature_review as isize, points.len());
            let next = wrap_index(index as isize + curvature_preview as isize, points.len());
            let delta_heading = normalize_angle_rad(heading_rad[next] - heading_rad[previous]);
            let ds = forward_closed_station_delta(
                station[previous],
                station[next],
                total_length,
                next < previous || curvature_total >= points.len(),
            );

            delta_heading / ds.max(1e-9)
        })
        .collect()
}

fn open_kappa_1pm(points: &[Point2], station_s_m: &[f64]) -> Vec<f64> {
    if points.len() < 3 {
        return vec![0.0; points.len()];
    }

    let tangents = open_ref_tangents(points);
    let mut heading_rad = tangents
        .iter()
        .map(|tangent| {
            normalize_angle_rad(tangent[1].atan2(tangent[0]) - std::f64::consts::FRAC_PI_2)
        })
        .collect::<Vec<_>>();
    for index in 1..heading_rad.len() {
        let delta = normalize_angle_rad(heading_rad[index] - heading_rad[index - 1]);
        heading_rad[index] = heading_rad[index - 1] + delta;
    }

    (0..points.len())
        .map(|index| {
            let previous = if index == 0 { 0 } else { index - 1 };
            let next = if index + 1 >= points.len() {
                points.len() - 1
            } else {
                index + 1
            };
            let ds = (station_s_m.get(next).copied().unwrap_or(next as f64)
                - station_s_m
                    .get(previous)
                    .copied()
                    .unwrap_or(previous as f64))
            .abs()
            .max(1e-9);
            (heading_rad[next] - heading_rad[previous]) / ds
        })
        .collect()
}

fn python_like_closed_spline_lengths(points: &[Point2]) -> Option<Vec<f64>> {
    if points.len() < 3 {
        return None;
    }

    let coefficients = python_like_closed_spline_coefficients(points)?;
    let t_steps = (0..15).map(|index| index as f64 / 14.0).collect::<Vec<_>>();

    Some(
        coefficients
            .iter()
            .map(|segment| {
                let mut previous = spline_point(segment, t_steps[0]);
                let mut length = 0.0;
                for t in t_steps.iter().copied().skip(1) {
                    let current = spline_point(segment, t);
                    length += point_distance(previous, current);
                    previous = current;
                }
                length.max(1e-9)
            })
            .collect(),
    )
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct CubicSplineSegment {
    x: [f64; 4],
    y: [f64; 4],
}

fn python_like_closed_spline_coefficients(points: &[Point2]) -> Option<Vec<CubicSplineSegment>> {
    let count = points.len();
    if count < 3 {
        return None;
    }

    let element_lengths = (0..count)
        .map(|index| point_distance(points[index], points[(index + 1) % count]).max(1e-9))
        .collect::<Vec<_>>();
    let mut scaled_lengths = element_lengths.clone();
    scaled_lengths.push(element_lengths[0]);
    let scaling = (0..count)
        .map(|index| scaled_lengths[index] / scaled_lengths[index + 1])
        .collect::<Vec<_>>();
    let matrix_size = count * 4;
    let mut matrix = vec![vec![0.0; matrix_size]; matrix_size];
    let mut bx = vec![0.0; matrix_size];
    let mut by = vec![0.0; matrix_size];

    for index in 0..count {
        let row = index * 4;
        let col = index * 4;

        if index < count - 1 {
            matrix[row][col] = 1.0;
            matrix[row + 1][col] = 1.0;
            matrix[row + 1][col + 1] = 1.0;
            matrix[row + 1][col + 2] = 1.0;
            matrix[row + 1][col + 3] = 1.0;
            matrix[row + 2][col + 1] = 1.0;
            matrix[row + 2][col + 2] = 2.0;
            matrix[row + 2][col + 3] = 3.0;
            matrix[row + 2][col + 5] = -scaling[index];
            matrix[row + 3][col + 2] = 2.0;
            matrix[row + 3][col + 3] = 6.0;
            matrix[row + 3][col + 6] = -2.0 * scaling[index] * scaling[index];
        } else {
            matrix[row][col] = 1.0;
            matrix[row + 1][col] = 1.0;
            matrix[row + 1][col + 1] = 1.0;
            matrix[row + 1][col + 2] = 1.0;
            matrix[row + 1][col + 3] = 1.0;
        }

        let next = (index + 1) % count;
        bx[row] = points[index][0];
        by[row] = points[index][1];
        bx[row + 1] = points[next][0];
        by[row + 1] = points[next][1];
    }

    let last_row = matrix_size - 2;
    matrix[last_row][1] = scaling[count - 1];
    matrix[last_row][matrix_size - 3] = -1.0;
    matrix[last_row][matrix_size - 2] = -2.0;
    matrix[last_row][matrix_size - 1] = -3.0;

    let curvature_row = matrix_size - 1;
    matrix[curvature_row][2] = 2.0 * scaling[count - 1] * scaling[count - 1];
    matrix[curvature_row][matrix_size - 2] = -2.0;
    matrix[curvature_row][matrix_size - 1] = -6.0;

    let x_solution = solve_linear_system(matrix.clone(), bx)?;
    let y_solution = solve_linear_system(matrix, by)?;
    Some(
        (0..count)
            .map(|index| {
                let offset = index * 4;
                CubicSplineSegment {
                    x: [
                        x_solution[offset],
                        x_solution[offset + 1],
                        x_solution[offset + 2],
                        x_solution[offset + 3],
                    ],
                    y: [
                        y_solution[offset],
                        y_solution[offset + 1],
                        y_solution[offset + 2],
                        y_solution[offset + 3],
                    ],
                }
            })
            .collect(),
    )
}

fn solve_linear_system(mut matrix: Vec<Vec<f64>>, mut rhs: Vec<f64>) -> Option<Vec<f64>> {
    let size = rhs.len();
    if matrix.len() != size || matrix.iter().any(|row| row.len() != size) {
        return None;
    }

    for pivot_index in 0..size {
        let pivot_row = (pivot_index..size).max_by(|left, right| {
            matrix[*left][pivot_index]
                .abs()
                .partial_cmp(&matrix[*right][pivot_index].abs())
                .unwrap_or(std::cmp::Ordering::Equal)
        })?;
        if matrix[pivot_row][pivot_index].abs() < 1e-12 {
            return None;
        }
        matrix.swap(pivot_index, pivot_row);
        rhs.swap(pivot_index, pivot_row);

        let pivot = matrix[pivot_index][pivot_index];
        for col in pivot_index..size {
            matrix[pivot_index][col] /= pivot;
        }
        rhs[pivot_index] /= pivot;

        for row in 0..size {
            if row == pivot_index {
                continue;
            }
            let factor = matrix[row][pivot_index];
            if factor.abs() <= 1e-15 {
                continue;
            }
            for col in pivot_index..size {
                matrix[row][col] -= factor * matrix[pivot_index][col];
            }
            rhs[row] -= factor * rhs[pivot_index];
        }
    }

    Some(rhs)
}

fn spline_point(segment: &CubicSplineSegment, t: f64) -> Point2 {
    let t2 = t * t;
    let t3 = t2 * t;
    [
        segment.x[0] + segment.x[1] * t + segment.x[2] * t2 + segment.x[3] * t3,
        segment.y[0] + segment.y[1] * t + segment.y[2] * t2 + segment.y[3] * t3,
    ]
}

fn closed_element_lengths(points: &[Point2], station_s_m: &[f64]) -> Vec<f64> {
    if station_s_m.len() == points.len() && station_s_m.len() >= 2 {
        let mut lengths = station_s_m
            .windows(2)
            .map(|pair| (pair[1] - pair[0]).abs().max(1e-9))
            .collect::<Vec<_>>();
        let geometric_close = point_distance(points[points.len() - 1], points[0]).max(1e-9);
        lengths.push(geometric_close);
        return lengths;
    }

    (0..points.len())
        .map(|index| point_distance(points[index], points[(index + 1) % points.len()]).max(1e-9))
        .collect()
}

fn point_distance(left: Point2, right: Point2) -> f64 {
    (right[0] - left[0]).hypot(right[1] - left[1])
}

fn closed_station_from_elements(element_lengths: &[f64]) -> Vec<f64> {
    let mut station = Vec::with_capacity(element_lengths.len() + 1);
    station.push(0.0);

    for length in element_lengths {
        station.push(station.last().copied().unwrap_or(0.0) + length);
    }

    station
}

fn average_positive(values: &[f64]) -> Option<f64> {
    let finite = values
        .iter()
        .copied()
        .filter(|value| value.is_finite() && *value > 0.0)
        .collect::<Vec<_>>();

    if finite.is_empty() {
        None
    } else {
        Some(finite.iter().sum::<f64>() / finite.len() as f64)
    }
}

fn preview_review_index_step(distance_m: f64, average_ds_m: f64) -> usize {
    ((distance_m / average_ds_m.max(1e-9)).round() as usize).max(1)
}

fn wrap_index(index: isize, len: usize) -> usize {
    index.rem_euclid(len as isize) as usize
}

fn normalize_angle_rad(angle: f64) -> f64 {
    let two_pi = std::f64::consts::TAU;
    (angle + std::f64::consts::PI).rem_euclid(two_pi) - std::f64::consts::PI
}

fn forward_closed_station_delta(
    previous_station: f64,
    next_station: f64,
    total_length: f64,
    wrapped: bool,
) -> f64 {
    if wrapped {
        total_length - previous_station + next_station
    } else {
        next_station - previous_station
    }
}

fn state_value_from(
    _seed: &CarMintimeNlpSeed,
    x: &[f64],
    station: usize,
    state_index: usize,
) -> f64 {
    x[state_offset(station) + state_index]
}

fn state_norm_value_from(
    seed: &CarMintimeNlpSeed,
    x: &[f64],
    station: usize,
    state_index: usize,
) -> f64 {
    state_value_from(seed, x, station, state_index) / CAR_STATE_SCALE[state_index]
}

fn control_value_from(
    seed: &CarMintimeNlpSeed,
    x: &[f64],
    interval: usize,
    control_index: usize,
) -> f64 {
    x[control_offset(seed, interval) + control_index]
}

fn collocation_state_value_from(
    seed: &CarMintimeNlpSeed,
    x: &[f64],
    interval: usize,
    point: usize,
    state_index: usize,
) -> f64 {
    x[collocation_state_offset(seed, interval, point) + state_index]
}

fn collocation_state_norm_value_from(
    seed: &CarMintimeNlpSeed,
    x: &[f64],
    interval: usize,
    point: usize,
    state_index: usize,
) -> f64 {
    collocation_state_value_from(seed, x, interval, point, state_index)
        / CAR_STATE_SCALE[state_index]
}

fn state_offset(station: usize) -> usize {
    station * CAR_STATE_LEN
}

fn control_offset(seed: &CarMintimeNlpSeed, interval: usize) -> usize {
    seed.dimensions.state_variable_count + interval * CAR_CONTROL_LEN
}

fn collocation_state_offset(seed: &CarMintimeNlpSeed, interval: usize, point: usize) -> usize {
    seed.dimensions.state_variable_count
        + seed.dimensions.control_variable_count
        + (interval * CAR_COLLOCATION_DEGREE + point) * CAR_STATE_LEN
}

fn state_series(x: &[f64], count: usize, state_index: usize, scale: f64) -> Vec<f64> {
    (0..count)
        .map(|index| x[state_offset(index) + state_index] * scale)
        .collect()
}

fn control_series(
    seed: &CarMintimeNlpSeed,
    x: &[f64],
    control_index: usize,
    scale: f64,
) -> Vec<f64> {
    (0..seed.dimensions.interval_count)
        .map(|index| x[control_offset(seed, index) + control_index] * scale)
        .collect()
}

fn drive_brake_regularization_series(seed: &CarMintimeNlpSeed, x: &[f64]) -> Vec<f64> {
    (0..seed.dimensions.interval_count)
        .map(|index| {
            let offset = control_offset(seed, index);
            (x[offset + CONTROL_F_DRIVE_N] + x[offset + CONTROL_F_BRAKE_N]) / 10_000.0
        })
        .collect()
}

fn steering_curvature_regularization_series(
    params: CarDoubleTrackParams,
    delta_rad: &[f64],
) -> Vec<f64> {
    let wheelbase = params.wheelbase_m.max(1e-9);
    delta_rad
        .iter()
        .map(|delta| delta.tan() / wheelbase)
        .collect()
}

fn steering_curvature_regularization_derivative(
    params: CarDoubleTrackParams,
    delta_rad: f64,
) -> f64 {
    let wheelbase = params.wheelbase_m.max(1e-9);
    let cos_delta = delta_rad.cos();
    1.0 / (wheelbase * cos_delta * cos_delta)
}

fn first_difference_squared(values: &[f64], closed: bool) -> f64 {
    let count = values.len();
    if count < 2 {
        return 0.0;
    }

    let segment_count = if closed { count } else { count - 1 };
    (0..segment_count)
        .map(|index| {
            let next = if index + 1 == count { 0 } else { index + 1 };
            let diff = values[index] - values[next];
            diff * diff
        })
        .sum()
}

fn second_difference_squared(values: &[f64], closed: bool) -> f64 {
    let count = values.len();
    if count < 3 {
        return 0.0;
    }

    if closed {
        (0..count)
            .map(|index| {
                let diff = values[(index + count - 1) % count] - 2.0 * values[index]
                    + values[(index + 1) % count];
                diff * diff
            })
            .sum()
    } else {
        (1..count - 1)
            .map(|index| {
                let diff = values[index - 1] - 2.0 * values[index] + values[index + 1];
                diff * diff
            })
            .sum()
    }
}

fn add_car_mintime_regularization_gradient(
    seed: &CarMintimeNlpSeed,
    params: CarDoubleTrackParams,
    weights: CarMintimeObjectiveWeights,
    x: &[f64],
    grad: &mut [f64],
) {
    let count = seed.dimensions.interval_count;
    if count == 0 {
        return;
    }

    let delta = control_series(seed, x, CONTROL_DELTA_RAD, 1.0);
    let steering_curvature = steering_curvature_regularization_series(params, &delta);
    let force = drive_brake_regularization_series(seed, x);
    let n = state_series(x, count, STATE_N_M, 1.0);
    let xi = state_series(x, count, STATE_XI_RAD, 1.0);
    let closed = seed_is_closed(seed);

    let steering_curvature_grad = regularization_series_gradient(
        &steering_curvature,
        weights.penalty_delta,
        weights.penalty_delta_dd,
        closed,
    );
    let force_grad =
        regularization_series_gradient(&force, weights.penalty_f, weights.penalty_f_dd, closed);
    let n_grad = regularization_series_gradient(&n, 0.0, weights.penalty_n_dd, closed);
    let xi_grad = regularization_series_gradient(&xi, 0.0, weights.penalty_xi_dd, closed);

    for index in 0..count {
        grad[state_offset(index) + STATE_N_M] += n_grad[index];
        grad[state_offset(index) + STATE_XI_RAD] += xi_grad[index];

        let control_offset = control_offset(seed, index);
        grad[control_offset + CONTROL_DELTA_RAD] += steering_curvature_grad[index]
            * steering_curvature_regularization_derivative(params, delta[index]);
        grad[control_offset + CONTROL_F_DRIVE_N] += force_grad[index] / 10_000.0;
        grad[control_offset + CONTROL_F_BRAKE_N] += force_grad[index] / 10_000.0;
    }

    add_car_endpoint_c1_dn_gradient_numeric(seed, weights, x, grad);
    add_car_endpoint_c1_heading_gradient_numeric(seed, weights, x, grad);
    add_car_endpoint_heading_jump_gradient_numeric(seed, weights, x, grad);
    add_car_endpoint_d2n_jump_gradient_numeric(seed, weights, x, grad);
}

fn add_car_endpoint_c1_dn_gradient_numeric(
    seed: &CarMintimeNlpSeed,
    weights: CarMintimeObjectiveWeights,
    x: &[f64],
    grad: &mut [f64],
) {
    if weights.penalty_endpoint_c1_dn <= 0.0 || seed.dimensions.interval_count == 0 {
        return;
    }

    let mut plus = x.to_vec();
    let mut minus = x.to_vec();
    for left_interval in 0..seed.dimensions.interval_count {
        if !seed_is_closed(seed) && left_interval + 1 >= seed.dimensions.station_count {
            continue;
        }

        let mut variable_indices = car_endpoint_c1_dn_variable_indices(seed, left_interval);
        variable_indices.sort_unstable();
        variable_indices.dedup();

        for variable_index in variable_indices {
            let h = 1e-6 * x[variable_index].abs().max(1.0);
            plus[variable_index] = x[variable_index] + h;
            minus[variable_index] = x[variable_index] - h;
            grad[variable_index] +=
                (car_endpoint_c1_dn_objective_term_s(seed, weights, &plus, left_interval)
                    - car_endpoint_c1_dn_objective_term_s(seed, weights, &minus, left_interval))
                    / (2.0 * h);
            plus[variable_index] = x[variable_index];
            minus[variable_index] = x[variable_index];
        }
    }
}

fn add_car_endpoint_c1_heading_gradient_numeric(
    seed: &CarMintimeNlpSeed,
    weights: CarMintimeObjectiveWeights,
    x: &[f64],
    grad: &mut [f64],
) {
    if weights.penalty_endpoint_c1_heading <= 0.0 || seed.dimensions.interval_count == 0 {
        return;
    }

    let mut plus = x.to_vec();
    let mut minus = x.to_vec();
    for left_interval in 0..seed.dimensions.interval_count {
        if !seed_is_closed(seed) && left_interval + 1 >= seed.dimensions.station_count {
            continue;
        }

        let mut variable_indices = car_endpoint_c1_variable_indices(seed, left_interval);
        variable_indices.sort_unstable();
        variable_indices.dedup();

        for variable_index in variable_indices {
            let h = 1e-6 * x[variable_index].abs().max(1.0);
            plus[variable_index] = x[variable_index] + h;
            minus[variable_index] = x[variable_index] - h;
            grad[variable_index] +=
                (car_endpoint_c1_heading_objective_term_s(seed, weights, &plus, left_interval)
                    - car_endpoint_c1_heading_objective_term_s(
                        seed,
                        weights,
                        &minus,
                        left_interval,
                    ))
                    / (2.0 * h);
            plus[variable_index] = x[variable_index];
            minus[variable_index] = x[variable_index];
        }
    }
}

fn add_car_endpoint_heading_jump_gradient_numeric(
    seed: &CarMintimeNlpSeed,
    weights: CarMintimeObjectiveWeights,
    x: &[f64],
    grad: &mut [f64],
) {
    if weights.penalty_endpoint_heading_jump <= 0.0 || seed.dimensions.interval_count == 0 {
        return;
    }

    let mut plus = x.to_vec();
    let mut minus = x.to_vec();
    for left_interval in 0..seed.dimensions.interval_count {
        if !seed_is_closed(seed) && left_interval + 1 >= seed.dimensions.station_count {
            continue;
        }

        let mut variable_indices = car_endpoint_c1_variable_indices(seed, left_interval);
        variable_indices.sort_unstable();
        variable_indices.dedup();

        for variable_index in variable_indices {
            let h = 1e-6 * x[variable_index].abs().max(1.0);
            plus[variable_index] = x[variable_index] + h;
            minus[variable_index] = x[variable_index] - h;
            grad[variable_index] +=
                (car_endpoint_heading_jump_objective_term_s(seed, weights, &plus, left_interval)
                    - car_endpoint_heading_jump_objective_term_s(
                        seed,
                        weights,
                        &minus,
                        left_interval,
                    ))
                    / (2.0 * h);
            plus[variable_index] = x[variable_index];
            minus[variable_index] = x[variable_index];
        }
    }
}

fn add_car_endpoint_d2n_jump_gradient_numeric(
    seed: &CarMintimeNlpSeed,
    weights: CarMintimeObjectiveWeights,
    x: &[f64],
    grad: &mut [f64],
) {
    if weights.penalty_endpoint_d2n_jump <= 0.0 || seed.dimensions.interval_count == 0 {
        return;
    }

    let mut plus = x.to_vec();
    let mut minus = x.to_vec();
    for left_interval in 0..seed.dimensions.interval_count {
        if !seed_is_closed(seed) && left_interval + 1 >= seed.dimensions.station_count {
            continue;
        }

        let mut variable_indices = car_endpoint_c1_variable_indices(seed, left_interval);
        variable_indices.sort_unstable();
        variable_indices.dedup();

        for variable_index in variable_indices {
            let h = 1e-6 * x[variable_index].abs().max(1.0);
            plus[variable_index] = x[variable_index] + h;
            minus[variable_index] = x[variable_index] - h;
            grad[variable_index] +=
                (car_endpoint_d2n_jump_objective_term_s(seed, weights, &plus, left_interval)
                    - car_endpoint_d2n_jump_objective_term_s(seed, weights, &minus, left_interval))
                    / (2.0 * h);
            plus[variable_index] = x[variable_index];
            minus[variable_index] = x[variable_index];
        }
    }
}

fn car_endpoint_c1_dn_variable_indices(
    seed: &CarMintimeNlpSeed,
    left_interval: usize,
) -> Vec<usize> {
    car_endpoint_c1_variable_indices(seed, left_interval)
}

fn car_endpoint_c1_variable_indices(seed: &CarMintimeNlpSeed, left_interval: usize) -> Vec<usize> {
    let right_interval = next_station_index(seed, left_interval)
        .min(seed.dimensions.interval_count.saturating_sub(1));
    let mut indices = Vec::with_capacity(2 * (CAR_COLLOCATION_DEGREE + 1) * CAR_STATE_LEN);
    push_car_collocation_interval_state_indices(seed, left_interval, &mut indices);
    push_car_collocation_interval_state_indices(seed, right_interval, &mut indices);
    indices
}

fn push_car_collocation_interval_state_indices(
    seed: &CarMintimeNlpSeed,
    interval: usize,
    indices: &mut Vec<usize>,
) {
    let station_offset = state_offset(interval);
    indices.extend(station_offset..station_offset + CAR_STATE_LEN);
    for point in 0..CAR_COLLOCATION_DEGREE {
        let offset = collocation_state_offset(seed, interval, point);
        indices.extend(offset..offset + CAR_STATE_LEN);
    }
}

fn regularization_series_gradient(
    values: &[f64],
    first_weight: f64,
    second_weight: f64,
    closed: bool,
) -> Vec<f64> {
    let count = values.len();
    let mut grad = vec![0.0; count];
    if count == 0 {
        return grad;
    }

    if first_weight != 0.0 {
        if closed {
            for index in 0..count {
                let previous = values[(index + count - 1) % count];
                let current = values[index];
                let next = values[(index + 1) % count];
                grad[index] += 2.0 * first_weight * (2.0 * current - previous - next);
            }
        } else {
            for index in 0..count.saturating_sub(1) {
                let diff = values[index] - values[index + 1];
                grad[index] += 2.0 * first_weight * diff;
                grad[index + 1] -= 2.0 * first_weight * diff;
            }
        }
    }

    if second_weight != 0.0 {
        if closed {
            let second_diff: Vec<f64> = (0..count)
                .map(|index| {
                    values[(index + count - 1) % count] - 2.0 * values[index]
                        + values[(index + 1) % count]
                })
                .collect();
            for index in 0..count {
                grad[index] += 2.0
                    * second_weight
                    * (second_diff[(index + count - 1) % count] - 2.0 * second_diff[index]
                        + second_diff[(index + 1) % count]);
            }
        } else if count >= 3 {
            for index in 1..count - 1 {
                let diff = values[index - 1] - 2.0 * values[index] + values[index + 1];
                grad[index - 1] += 2.0 * second_weight * diff;
                grad[index] -= 4.0 * second_weight * diff;
                grad[index + 1] += 2.0 * second_weight * diff;
            }
        }
    }

    grad
}

fn state_index(state_name: &str) -> usize {
    match state_name {
        "v_mps" => STATE_V_MPS,
        "beta_rad" => STATE_BETA_RAD,
        "omega_z_radps" => STATE_OMEGA_Z_RADPS,
        "n_m" => STATE_N_M,
        "xi_rad" => STATE_XI_RAD,
        _ => unreachable!("unknown car state {state_name}"),
    }
}

fn control_index(control_name: &str) -> usize {
    match control_name {
        "delta_rad" => CONTROL_DELTA_RAD,
        "f_drive_N" => CONTROL_F_DRIVE_N,
        "f_brake_N" => CONTROL_F_BRAKE_N,
        "gamma_y_N" => CONTROL_GAMMA_Y_N,
        _ => unreachable!("unknown car control {control_name}"),
    }
}

fn insert_state_columns(columns: &mut BTreeSet<usize>, station: usize) {
    for state_index in 0..CAR_STATE_LEN {
        columns.insert(state_offset(station) + state_index);
    }
}

fn insert_control_columns(
    seed: &CarMintimeNlpSeed,
    columns: &mut BTreeSet<usize>,
    interval: usize,
) {
    for control_index in 0..CAR_CONTROL_LEN {
        columns.insert(control_offset(seed, interval) + control_index);
    }
}

fn insert_collocation_point_state_columns(
    seed: &CarMintimeNlpSeed,
    columns: &mut BTreeSet<usize>,
    interval: usize,
    point: usize,
) {
    for state_index in 0..CAR_STATE_LEN {
        columns.insert(collocation_state_offset(seed, interval, point) + state_index);
    }
}

fn insert_collocation_state_component_columns(
    seed: &CarMintimeNlpSeed,
    columns: &mut BTreeSet<usize>,
    interval: usize,
    state_index: usize,
) {
    for point in 0..CAR_COLLOCATION_DEGREE {
        columns.insert(collocation_state_offset(seed, interval, point) + state_index);
    }
}

fn validate_sections_for_car_mintime(
    sections: &SectionsTrackViewV1,
    expected_station_count: usize,
) -> Result<(), SolverApiError> {
    let count = sections.station_s_m.len();
    if count != expected_station_count {
        return Err(SolverApiError::new(
            "solve.invalidRequest",
            format!("station builder returned {count} stations, expected {expected_station_count}"),
        ));
    }

    if sections.centerline_xy_m.len() != count
        || sections.width_left_m.len() != count
        || sections.width_right_m.len() != count
    {
        return Err(SolverApiError::new(
            "solve.invalidRequest",
            "station builder returned inconsistent car mintime station arrays",
        ));
    }

    validate_station_topology(sections).map_err(|issue| {
        SolverApiError::new(issue.code, issue.message)
            .with_details(JsonValue::Object(issue.diagnostics))
    })?;

    Ok(())
}

fn car_mintime_width_opt_m(request: &MintimeSolveRequestV1) -> f64 {
    mintime_option_f64(request, "width_opt_m")
        .or_else(|| mintime_option_f64(request, "width_opt"))
        .or_else(|| {
            request
                .vehicle_dynamics_profile
                .numeric_param("width_opt_m")
        })
        .or_else(|| request.vehicle_dynamics_profile.numeric_param("width_opt"))
        .or_else(|| {
            let preset = request
                .vehicle_dynamics_profile
                .preset_id
                .as_deref()
                .unwrap_or("");
            let profile = request.vehicle_dynamics_profile.profile_id.as_str();
            if preset.contains("kart") || profile.contains("kart") {
                Some(CAR_MINTIME_KART_WIDTH_OPT_M)
            } else if preset.contains("gt3") || profile.contains("gt3") {
                Some(CAR_MINTIME_DEFAULT_WIDTH_OPT_M)
            } else {
                request.vehicle_dynamics_profile.numeric_param("width")
            }
        })
        .unwrap_or(CAR_MINTIME_DEFAULT_WIDTH_OPT_M)
        .max(0.0)
}

fn car_mintime_n_bounds_m(
    width_left_m: f64,
    width_right_m: f64,
    half_width_opt_m: f64,
) -> (f64, f64) {
    let lower_n_m = -width_right_m + half_width_opt_m;
    let upper_n_m = width_left_m - half_width_opt_m;

    if lower_n_m <= upper_n_m {
        (lower_n_m, upper_n_m)
    } else {
        let midpoint = 0.5 * (lower_n_m + upper_n_m);
        (midpoint, midpoint)
    }
}

fn push_state_row(
    initial_guess: &mut Vec<f64>,
    lower_bounds: &mut Vec<f64>,
    upper_bounds: &mut Vec<f64>,
    initial_state: [f64; CAR_STATE_LEN],
    lower_n_m: f64,
    upper_n_m: f64,
    max_speed_mps: f64,
) {
    initial_guess.extend_from_slice(&initial_state);
    lower_bounds.extend_from_slice(&[0.1, -0.5, -20.0, lower_n_m, -std::f64::consts::PI]);
    upper_bounds.extend_from_slice(&[max_speed_mps, 0.5, 20.0, upper_n_m, std::f64::consts::PI]);
}

fn fix_decision_variable(
    initial_guess: &mut [f64],
    lower_bounds: &mut [f64],
    upper_bounds: &mut [f64],
    index: usize,
    value: f64,
) {
    initial_guess[index] = value;
    lower_bounds[index] = value;
    upper_bounds[index] = value;
}

fn push_control_row(
    initial_guess: &mut Vec<f64>,
    lower_bounds: &mut Vec<f64>,
    upper_bounds: &mut Vec<f64>,
    initial_control: [f64; CAR_CONTROL_LEN],
    params: CarDoubleTrackParams,
) {
    initial_guess.extend_from_slice(&initial_control);
    lower_bounds.extend_from_slice(&[
        -params.steering_angle_max_rad,
        0.0,
        -params.brake_force_max_n,
        -params.mass_kg * params.lateral_grip_level * params.gravity_mps2,
    ]);
    upper_bounds.extend_from_slice(&[
        params.steering_angle_max_rad,
        params.drive_force_max_n,
        0.0,
        params.mass_kg * params.lateral_grip_level * params.gravity_mps2,
    ]);
}

fn initial_state_guesses_from_kappa(
    params: CarDoubleTrackParams,
    kappa_1pm: &[f64],
) -> Vec<[f64; CAR_STATE_LEN]> {
    kappa_1pm
        .iter()
        .map(|_| initial_state_guess_for_kappa(params))
        .collect()
}

fn initial_state_guess_for_kappa(params: CarDoubleTrackParams) -> [f64; CAR_STATE_LEN] {
    let speed = 20.0_f64.clamp(1.0, params.max_speed_mps);

    [speed, 0.0, 0.0, 0.0, 0.0]
}

fn initial_control_guesses_from_kappa(
    initial_states: &[[f64; CAR_STATE_LEN]],
) -> Vec<[f64; CAR_CONTROL_LEN]> {
    vec![[0.0; CAR_CONTROL_LEN]; initial_states.len()]
}

fn lerp_state_guess(
    from: [f64; CAR_STATE_LEN],
    to: [f64; CAR_STATE_LEN],
    tau: f64,
) -> [f64; CAR_STATE_LEN] {
    let mut output = [0.0; CAR_STATE_LEN];
    for index in 0..CAR_STATE_LEN {
        output[index] = lerp(from[index], to[index], tau);
    }
    output
}

fn emit_progress(progress: &mut Option<MintimeProgressCallback<'_>>, event: MintimeProgressEvent) {
    if let Some(callback) = progress.as_mut() {
        callback(event);
    }
}

pub fn solve_car_mintime_json(input_json: &str) -> Result<String, SolverApiError> {
    solve_car_mintime_json_with_progress(input_json, None, None)
}

pub fn car_mintime_initial_x_json(input_json: &str) -> Result<String, SolverApiError> {
    let request =
        MintimeSolveRequestV1::parse_product(input_json, VehicleDynamicsModelFamily::CarDynamics)?;
    let params = CarDoubleTrackParams::from_profile(&request.vehicle_dynamics_profile)
        .map_err(|message| SolverApiError::new("solve.invalidRequest", message))?;
    let seed = build_car_mintime_nlp_seed(&request, params)?;
    Ok(f64_array_json(&seed.initial_guess))
}

pub fn solve_car_mintime_json_with_initial_x(
    input_json: &str,
    initial_x: Vec<f64>,
) -> Result<String, SolverApiError> {
    let request =
        MintimeSolveRequestV1::parse_product(input_json, VehicleDynamicsModelFamily::CarDynamics)?;
    let params = CarDoubleTrackParams::from_profile(&request.vehicle_dynamics_profile)
        .map_err(|message| SolverApiError::new("solve.invalidRequest", message))?;
    let options = CarMintimeSolveOptions::try_from_request(&request)?;
    let seed = build_car_mintime_nlp_seed(&request, params)?;
    let problem = build_car_mintime_nlp_problem_with_options(seed, params, options.clone())?;
    let result =
        solve_car_mintime_with_ipopt_initial(problem, options, None, None, Some(initial_x))?;

    Ok(mintime_result_to_json(&result).to_pretty_string())
}

fn f64_array_json(values: &[f64]) -> String {
    let mut output = String::from("[");
    for (index, value) in values.iter().enumerate() {
        if index > 0 {
            output.push(',');
        }
        output.push_str(&value.to_string());
    }
    output.push(']');
    output
}

pub fn solve_car_mintime_json_with_progress<'a>(
    input_json: &str,
    progress: Option<MintimeProgressCallback<'a>>,
    cancel_token: Option<&'a dyn SolverCancelToken>,
) -> Result<String, SolverApiError> {
    let request =
        MintimeSolveRequestV1::parse_product(input_json, VehicleDynamicsModelFamily::CarDynamics)?;
    let backend = CarDoubleTrackMintimeBackend;
    let result = backend.solve_with_cancel(request, progress, cancel_token)?;

    Ok(mintime_result_to_json(&result).to_pretty_string())
}

#[cfg(test)]
mod tests {
    use super::build_car_mintime_nlp_problem;
    use super::{build_car_mintime_nlp_seed, solve_car_mintime_json};
    use crate::contracts::{
        station_geometry_content_hash_v1, station_geometry_content_hash_v2,
        station_options_hash_v2, Point2, StationSourceRefV1, TrackAreaContractV1,
        SECTIONS_TRACK_VIEW_HASH_V1,
    };
    use crate::json::{parse_json_str, JsonValue};
    use crate::mintime::{
        MintimeGeometryInput, MintimeNlpLayout, MintimeSolveRequestV1, PreparedStationGeometryV3,
    };
    use crate::station_generation::{
        generate_station_geometry, station_generation_response_json, StationCountMode,
        StationGenerationRequestV1,
    };
    use crate::vehicle_dynamics::{
        CarDoubleTrackParams, VehicleDynamicsModelFamily, VehicleDynamicsProfileV1,
    };
    use crate::{JsonObject, ToJsonValue};
    use std::fs;
    use std::path::Path;

    fn metadata_str<'a>(metadata: &'a JsonObject, key: &str) -> &'a str {
        metadata
            .iter()
            .find(|(entry_key, _)| entry_key == key)
            .and_then(|(_, value)| value.as_str())
            .unwrap_or_else(|| panic!("missing metadata string key {key}"))
    }

    fn metadata_str_optional<'a>(metadata: &'a JsonObject, key: &str) -> Option<&'a str> {
        metadata
            .iter()
            .find(|(entry_key, _)| entry_key == key)
            .and_then(|(_, value)| value.as_str())
    }

    fn metadata_f64_optional(metadata: &JsonObject, key: &str) -> Option<f64> {
        metadata
            .iter()
            .find(|(entry_key, _)| entry_key == key)
            .and_then(|(_, value)| value.as_f64())
    }

    fn metadata_bool_optional(metadata: &JsonObject, key: &str) -> Option<bool> {
        metadata
            .iter()
            .find(|(entry_key, _)| entry_key == key)
            .and_then(|(_, value)| match value {
                JsonValue::Bool(value) => Some(*value),
                _ => None,
            })
    }

    fn assert_close(left: f64, right: f64) {
        assert!(
            (left - right).abs() < 1e-9,
            "expected {left} ~= {right}, diff={}",
            left - right
        );
    }

    fn car_mintime_closed_test_request_json(
        station_count: usize,
        solve_options_json: &str,
    ) -> String {
        let track_area_json = fs::read_to_string(crate_path(
            "tests/public-fixtures/compact-oval-track-area-v1.json",
        ))
        .unwrap();

        format!(
            r#"{{
              "schema_version": "rust_solver_http_request.v1",
              "request_id": "req-1",
              "project_id": "project-1",
              "station_count": {station_count},
              "solve_options": {solve_options_json},
              "track_area": {track_area_json},
              "vehicle_dynamics_profile": {{
                "schema_version": "vehicle_dynamics_profile.v1",
                "profile_id": "car_dynamics:kart_125cc",
                "model_family": "car_dynamics",
                "preset_id": "kart_125cc",
                "solver_id": "old_car_mintime",
                "parameters": {{
                  "mass_kg": 165,
                  "max_speed_mps": 34,
                  "delta_max_rad": 0.6,
                  "f_drive_max_n": 1800,
                  "f_brake_max_n": 2600
                }}
              }}
            }}"#
        )
    }

    fn car_mintime_closed_test_request(station_count: usize) -> MintimeSolveRequestV1 {
        MintimeSolveRequestV1::parse(
            &car_mintime_closed_test_request_json(station_count, "{}"),
            VehicleDynamicsModelFamily::CarDynamics,
        )
        .unwrap()
    }

    fn car_mintime_closed_test_request_for_direction(
        station_count: usize,
        direction: &str,
    ) -> MintimeSolveRequestV1 {
        let json = car_mintime_closed_test_request_json(station_count, "{}").replace(
            r#""direction": "clockwise""#,
            &format!(r#""direction": "{direction}""#),
        );
        MintimeSolveRequestV1::parse(&json, VehicleDynamicsModelFamily::CarDynamics).unwrap()
    }

    fn car_mintime_product_test_request_json(solve_options: JsonValue) -> String {
        let track = read_track_area_contract(&crate_path(
            "tests/public-fixtures/compact-oval-track-area-v1.json",
        ));
        let source_ref = StationSourceRefV1 {
            project_id: "10000000-0000-4000-8000-000000000001".to_owned(),
            geometry_id: "10000000-0000-4000-8000-000000000002".to_owned(),
            geometry_content_hash: station_geometry_content_hash_v2(&track),
            route_id: track.track_id.clone(),
        };
        let station_options = JsonValue::Object(Vec::new());
        let station_result = generate_station_geometry(
            &StationGenerationRequestV1 {
                request_id: "synthetic-car-stations".to_owned(),
                request_key: "synthetic-car-stations-exact-24".to_owned(),
                project_id: source_ref.project_id.clone(),
                station_count: 24,
                count_mode: StationCountMode::Exact,
                track_area: track,
                station_options: crate::station::FixedCenterlineStationOptions {
                    sample_count: 24,
                    dense_count: 320,
                    ..Default::default()
                },
                station_options_hash: station_options_hash_v2(&station_options),
                source_ref: source_ref.clone(),
            },
            None,
        );
        let station_response = station_generation_response_json(&station_result);
        let prepared = JsonValue::Object(vec![
            (
                "schema_version".to_owned(),
                "prepared_station_geometry.v4".into(),
            ),
            (
                "requested_count_mode".to_owned(),
                station_response
                    .get("requested_count_mode")
                    .expect("station response must report count mode")
                    .clone(),
            ),
            (
                "resolved_station_count".to_owned(),
                station_response
                    .get("resolved_station_count")
                    .expect("station response must report resolved count")
                    .clone(),
            ),
            (
                "complexity_report".to_owned(),
                station_response
                    .get("complexity_report")
                    .expect("station response must report complexity")
                    .clone(),
            ),
            (
                "bundle".to_owned(),
                station_response
                    .get("bundle")
                    .expect("station response must contain prepared bundle")
                    .clone(),
            ),
            (
                "diagnostics".to_owned(),
                station_response
                    .get("diagnostics")
                    .expect("station response must contain diagnostics")
                    .clone(),
            ),
        ]);

        JsonValue::Object(vec![
            (
                "schema_version".to_owned(),
                "rust_solver_http_request.v5".into(),
            ),
            ("request_id".to_owned(), "synthetic-car-solve".into()),
            (
                "project_id".to_owned(),
                source_ref.project_id.clone().into(),
            ),
            (
                "source_ref".to_owned(),
                JsonValue::Object(vec![
                    ("schema_version".to_owned(), "station_source_ref.v1".into()),
                    ("project_id".to_owned(), source_ref.project_id.into()),
                    ("geometry_id".to_owned(), source_ref.geometry_id.into()),
                    (
                        "geometry_content_hash".to_owned(),
                        source_ref.geometry_content_hash.into(),
                    ),
                    ("route_id".to_owned(), source_ref.route_id.into()),
                ]),
            ),
            ("station_count".to_owned(), JsonValue::Integer(24)),
            ("solve_options".to_owned(), solve_options),
            ("prepared_station_geometry".to_owned(), prepared),
            (
                "vehicle_dynamics_profile".to_owned(),
                test_car_profile().to_json_value(),
            ),
        ])
        .to_pretty_string()
    }

    fn crate_path(relative_path: &str) -> std::path::PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR")).join(relative_path)
    }

    fn read_track_area_contract(path: &Path) -> TrackAreaContractV1 {
        TrackAreaContractV1::from_json(&crate::read_json_value(path).unwrap()).unwrap()
    }

    fn test_car_profile() -> VehicleDynamicsProfileV1 {
        VehicleDynamicsProfileV1 {
            schema_version: VehicleDynamicsProfileV1::SCHEMA_VERSION.to_owned(),
            profile_id: "car_dynamics:kart_125cc".to_owned(),
            model_family: VehicleDynamicsModelFamily::CarDynamics,
            preset_id: Some("kart_125cc".to_owned()),
            solver_id: Some(super::OLD_CAR_MINTIME_SOLVER_ID.to_owned()),
            parameters: vec![
                ("mass_kg".to_owned(), 165.0.into()),
                ("max_speed_mps".to_owned(), 34.0.into()),
                ("delta_max_rad".to_owned(), 0.6.into()),
                ("f_drive_max_n".to_owned(), 1800.0.into()),
                ("f_brake_max_n".to_owned(), 2600.0.into()),
            ],
            native_parameters: Vec::new(),
            metadata: Vec::new(),
        }
    }

    fn open_lab_track_boundaries_json(center: &[Point2], half_width_m: f64) -> (String, String) {
        let mut left = Vec::with_capacity(center.len());
        let mut right = Vec::with_capacity(center.len());

        for index in 0..center.len() {
            let tangent = if index == 0 {
                normalize_test_vector(point_delta(center[0], center[1]))
            } else if index + 1 == center.len() {
                normalize_test_vector(point_delta(center[index - 1], center[index]))
            } else {
                normalize_test_vector(point_delta(center[index - 1], center[index + 1]))
            };
            let right_normal = [tangent[1], -tangent[0]];
            left.push([
                center[index][0] - right_normal[0] * half_width_m,
                center[index][1] - right_normal[1] * half_width_m,
            ]);
            right.push([
                center[index][0] + right_normal[0] * half_width_m,
                center[index][1] + right_normal[1] * half_width_m,
            ]);
        }

        (points_json(&left), points_json(&right))
    }

    fn point_delta(from: Point2, to: Point2) -> Point2 {
        [to[0] - from[0], to[1] - from[1]]
    }

    fn normalize_test_vector(vector: Point2) -> Point2 {
        let length = vector[0].hypot(vector[1]).max(1e-12);
        [vector[0] / length, vector[1] / length]
    }

    fn points_json(points: &[Point2]) -> String {
        points
            .iter()
            .map(|point| format!("[{:.12},{:.12}]", point[0], point[1]))
            .collect::<Vec<_>>()
            .join(",")
    }

    fn open_car_mintime_smoke_request_json(
        track_id: &str,
        station_count: usize,
        center: Vec<Point2>,
        half_width_m: f64,
        max_iter: usize,
    ) -> String {
        let (left_boundary_xy_m, right_boundary_xy_m) =
            open_lab_track_boundaries_json(&center, half_width_m);
        format!(
            r#"{{
              "schema_version": "rust_solver_http_request.v1",
              "request_id": "{track_id}-car-open-smoke",
              "project_id": "open-car-smoke",
              "station_count": {station_count},
              "solve_options": {{
                "open_start_speed_mps": 0,
                "max_iter": {max_iter},
                "tol": 0.00001,
                "acceptable_tol": 0.00001,
                "acceptable_iter": 5,
                "ipopt_print_level": 0,
                "production_station_builder": "open_area_station_generator",
                "dense_count": 384,
                "target_spacing_max_adjacent_ratio": 1.35,
                "target_spacing_metric": "hybrid_area_centerline"
              }},
              "track_area": {{
                "schema_version": "TrackAreaContractV1",
                "track_id": "{track_id}",
                "units": "m",
                "trajectory_mode": "open",
                "left_boundary_xy_m": [{left_boundary_xy_m}],
                "right_boundary_xy_m": [{right_boundary_xy_m}],
                "metadata": {{}}
              }},
              "vehicle_dynamics_profile": {{
                "schema_version": "vehicle_dynamics_profile.v1",
                "profile_id": "car_dynamics:kart_125cc_open_smoke",
                "model_family": "car_dynamics",
                "preset_id": "kart_125cc",
                "solver_id": "old_car_mintime",
                "parameters": {{
                  "v_max_mps": 34,
                  "mass_kg": 165,
                  "wheelbase_front": 0.60,
                  "wheelbase_rear": 0.44,
                  "track_width_front_m": 1.05,
                  "track_width_rear_m": 1.20,
                  "cog_z": 0.32,
                  "mue": 1.35,
                  "I_z": 55.0,
                  "liftcoeff_front": 0.0,
                  "liftcoeff_rear": 0.0,
                  "k_brake_front": 0.05,
                  "k_drive_front": 0.0,
                  "k_roll": 0.48,
                  "t_delta": 0.08,
                  "t_drive": 0.08,
                  "t_brake": 0.06,
                  "power_max": 18000.0,
                  "f_drive_max": 2200.0,
                  "f_brake_max": 3200.0,
                  "delta_max": 1.20,
                  "c_roll": 0.015,
                  "f_z0": 405.0,
                  "B_front": 10.0,
                  "C_front": 2.2,
                  "eps_front": -0.10,
                  "E_front": 1.0,
                  "B_rear": 10.0,
                  "C_rear": 2.2,
                  "eps_rear": -0.10,
                  "E_rear": 1.0
                }}
              }}
            }}"#
        )
    }

    fn open_car_mintime_smoke_case(track_id: &str) -> Option<(usize, f64, usize, Vec<Point2>)> {
        match track_id {
            "open_straight_lab_v1" => Some((
                20,
                4.0,
                1200,
                (0..=30).map(|index| [index as f64 * 4.0, 0.0]).collect(),
            )),
            "open_s_bend_lab_v1" => Some((
                32,
                4.5,
                2200,
                (0..34)
                    .map(|index| {
                        let t = index as f64 / 33.0;
                        [116.0 * t, 18.0 * (2.0 * std::f64::consts::PI * t).sin()]
                    })
                    .collect(),
            )),
            "open_chicane_lab_v1" => Some((
                40,
                4.2,
                2600,
                vec![
                    [0.0, 0.0],
                    [25.0, 0.0],
                    [45.0, 18.0],
                    [70.0, -18.0],
                    [95.0, -18.0],
                    [120.0, 0.0],
                    [150.0, 0.0],
                ],
            )),
            _ => None,
        }
    }

    fn assert_open_car_mintime_smoke_result(response: &str, expected_station_count: usize) {
        let value = parse_json_str(response).unwrap();
        assert_eq!(
            value.get("status").and_then(JsonValue::as_str),
            Some("Solve_Succeeded")
        );
        assert!(value
            .get("open_run_time_s")
            .and_then(JsonValue::as_f64)
            .is_some_and(|value| value.is_finite() && value > 0.0));
        let trajectory = value.get("trajectory_result").unwrap();
        let s_m = trajectory.get("s_m").and_then(JsonValue::as_array).unwrap();
        let ay_mps2 = trajectory
            .get("ay_mps2")
            .and_then(JsonValue::as_array)
            .unwrap();
        assert_eq!(s_m.len(), expected_station_count);
        assert_eq!(ay_mps2[0].as_f64(), Some(0.0));
        assert_eq!(
            value
                .get("visualization")
                .and_then(|value| value.get("display_trajectory"))
                .and_then(|value| value.get("closed")),
            Some(&JsonValue::Bool(false))
        );
    }

    #[test]
    fn car_mintime_steering_regularization_scales_with_wheelbase() {
        let short_params = CarDoubleTrackParams {
            wheelbase_m: 1.0,
            ..CarDoubleTrackParams::from_profile(&test_car_profile()).unwrap()
        };
        let long_params = CarDoubleTrackParams {
            wheelbase_m: 4.0,
            ..short_params
        };
        let delta = [0.0, 0.2];

        let short_curvature = super::steering_curvature_regularization_series(short_params, &delta);
        let long_curvature = super::steering_curvature_regularization_series(long_params, &delta);
        let short_step = (short_curvature[1] - short_curvature[0]).abs();
        let long_step = (long_curvature[1] - long_curvature[0]).abs();

        assert!(
            (long_step / short_step - 0.25).abs() < 1e-12,
            "steering regularization must use tan(delta) / wheelbase, not raw delta"
        );
    }

    #[test]
    fn car_mintime_validates_request_then_reports_missing_ipopt_backend() {
        let request = car_mintime_product_test_request_json(JsonValue::Object(vec![
            ("direction".to_owned(), "clockwise".into()),
            ("station_count".to_owned(), JsonValue::Integer(24)),
            ("station_options".to_owned(), JsonValue::Object(Vec::new())),
            (
                "ipopt_dll_path".to_owned(),
                "missing/libipopt-does-not-exist.dll".into(),
            ),
            ("max_iter".to_owned(), JsonValue::Integer(1)),
        ]));
        let error = solve_car_mintime_json(&request).unwrap_err();

        assert_eq!(
            error.code, "solve.nativeBackendUnavailable",
            "unexpected solver error: {error:?}"
        );
        assert!(error.message.contains("libipopt-does-not-exist"));
    }

    #[test]
    fn car_mintime_builds_nlp_seed_from_track_area_and_profile() {
        let request = MintimeSolveRequestV1::parse(
            r#"{
              "schema_version": "rust_solver_http_request.v1",
              "request_id": "req-1",
              "project_id": "project-1",
              "station_count": 20,
              "track_area": {
                "schema_version": "TrackAreaContractV1",
                "track_id": "track-1",
                "units": "m",
                "left_boundary_xy_m": [[0,0], [0,20], [20,20], [20,0]],
                "right_boundary_xy_m": [[4,4], [4,16], [16,16], [16,4]],
                "trajectory_mode": "closed",
                "metadata": {}
              },
              "vehicle_dynamics_profile": {
                "schema_version": "vehicle_dynamics_profile.v1",
                "profile_id": "car_dynamics:kart_125cc",
                "model_family": "car_dynamics",
                "preset_id": "kart_125cc",
                "solver_id": "old_car_mintime",
                "parameters": {
                  "mass_kg": 165,
                  "max_speed_mps": 34,
                  "delta_max_rad": 0.6,
                  "f_drive_max_n": 1800,
                  "f_brake_max_n": 2600
                }
              }
            }"#,
            VehicleDynamicsModelFamily::CarDynamics,
        )
        .unwrap();
        let params = CarDoubleTrackParams::from_profile(&request.vehicle_dynamics_profile).unwrap();
        let seed = build_car_mintime_nlp_seed(&request, params).unwrap();

        assert_eq!(seed.dimensions.station_count, 20);
        assert_eq!(seed.dimensions.interval_count, 20);
        assert_eq!(seed.station_s_m.len(), seed.dimensions.interval_count + 1);
        assert_eq!(
            seed.initial_guess.len(),
            seed.dimensions.decision_variable_count()
        );
        assert_eq!(seed.lower_bounds.len(), seed.initial_guess.len());
        assert_eq!(seed.upper_bounds.len(), seed.initial_guess.len());
        let half_width_opt_m = super::CAR_MINTIME_KART_WIDTH_OPT_M * 0.5;
        assert_eq!(
            seed.lower_bounds[3],
            -seed.width_right_m[0].max(1e-3) + half_width_opt_m
        );
        assert_eq!(
            seed.upper_bounds[3],
            seed.width_left_m[0].max(1e-3) - half_width_opt_m
        );
        assert_eq!(seed.upper_bounds[0], 34.0);
        let first_control = seed.dimensions.state_variable_count;
        assert_eq!(
            seed.lower_bounds[first_control + super::CONTROL_F_BRAKE_N],
            -2600.0
        );
        assert_eq!(
            seed.upper_bounds[first_control + super::CONTROL_F_BRAKE_N],
            0.0
        );
        assert_eq!(
            seed.model_track_area.left_boundary_xy_m.len(),
            seed.dimensions.station_count
        );
    }

    #[test]
    fn car_mintime_uses_prepared_sections_without_regenerating_them() {
        let mut request = MintimeSolveRequestV1::parse(
            r#"{
              "schema_version": "rust_solver_http_request.v1",
              "request_id": "req-prepared",
              "project_id": "project-1",
              "station_count": 20,
              "track_area": {
                "schema_version": "TrackAreaContractV1",
                "track_id": "track-prepared",
                "units": "m",
                "left_boundary_xy_m": [[0,0], [0,20], [20,20], [20,0]],
                "right_boundary_xy_m": [[4,4], [4,16], [16,16], [16,4]],
                "trajectory_mode": "closed",
                "metadata": {}
              },
              "vehicle_dynamics_profile": {
                "schema_version": "vehicle_dynamics_profile.v1",
                "profile_id": "car_dynamics:kart_125cc",
                "model_family": "car_dynamics",
                "preset_id": "kart_125cc",
                "solver_id": "old_car_mintime",
                "parameters": {
                  "mass_kg": 165,
                  "max_speed_mps": 34,
                  "delta_max_rad": 0.6,
                  "f_drive_max_n": 1800,
                  "f_brake_max_n": 2600
                }
              }
            }"#,
            VehicleDynamicsModelFamily::CarDynamics,
        )
        .unwrap();
        let station_options = crate::station::FixedCenterlineStationOptions {
            sample_count: request.station_count,
            ..Default::default()
        };
        let track_area = request.track_area();
        let source_ref = StationSourceRefV1 {
            project_id: request.project_id.clone(),
            geometry_id: "geometry-test".to_owned(),
            geometry_content_hash: station_geometry_content_hash_v1(&track_area),
            route_id: track_area.track_id.clone(),
        };
        let station_result = generate_station_geometry(
            &StationGenerationRequestV1 {
                request_key: "test_station_request".to_owned(),
                request_id: request.request_id.clone(),
                project_id: request.project_id.clone(),
                station_count: request.station_count,
                count_mode: StationCountMode::Exact,
                track_area,
                station_options,
                station_options_hash: "fnv1a_optionstest".to_owned(),
                source_ref,
            },
            None,
        );
        let mut prepared_sections = station_result.sections_track_view;
        prepared_sections.centerline_xy_m[0][0] += 0.01;
        prepared_sections.left_boundary_xy_m[0][0] += 0.01;
        prepared_sections.right_boundary_xy_m[0][0] += 0.01;
        request.geometry_input =
            MintimeGeometryInput::PreparedStationGeometry(PreparedStationGeometryV3 {
                source_ref: station_result.source_ref.clone(),
                prepared_bundle_hash: "test-bundle-hash".to_owned(),
                prepared_bundle_hash_algorithm: crate::contracts::PREPARED_STATION_BUNDLE_HASH_V2
                    .to_owned(),
                sections_track_view_hash: "test-hash".to_owned(),
                sections_hash_algorithm: SECTIONS_TRACK_VIEW_HASH_V1.to_owned(),
                station_options_hash: "options-test".to_owned(),
                direction: request
                    .track_area()
                    .direction
                    .unwrap_or_else(|| "clockwise".to_owned()),
                generator_contract: "station-generation-test".to_owned(),
                generator_version: "test".to_owned(),
                validation_contract: "station-validation-test".to_owned(),
                validation_version: "test".to_owned(),
                resolved_station_count: prepared_sections.station_s_m.len(),
                route_identity: crate::mintime::PreparedRouteIdentityV1 {
                    track_id: station_result.model_track_area.track_id.clone(),
                    units: station_result.model_track_area.units.clone(),
                    trajectory_mode: station_result.model_track_area.trajectory_mode.clone(),
                    direction: station_result.model_track_area.direction.clone(),
                    start_finish_xy_m: station_result.model_track_area.start_finish_xy_m.clone(),
                    finish_line_xy_m: station_result.model_track_area.finish_line_xy_m.clone(),
                },
                sections_track_view: prepared_sections.clone(),
            });

        let params = CarDoubleTrackParams::from_profile(&request.vehicle_dynamics_profile).unwrap();
        let seed = build_car_mintime_nlp_seed(&request, params).unwrap();

        assert_eq!(seed.centerline_xy_m, prepared_sections.centerline_xy_m);
        for (actual, expected) in seed
            .section_dir_xy
            .iter()
            .zip(&prepared_sections.section_dirs_xy)
        {
            assert!((actual[0] - expected[0]).abs() < 1e-12);
            assert!((actual[1] - expected[1]).abs() < 1e-12);
        }
        assert_eq!(seed.width_left_m, prepared_sections.width_left_m);
        assert_eq!(seed.width_right_m, prepared_sections.width_right_m);
        assert_eq!(
            metadata_str(&seed.model_track_area.metadata, "station_geometry_source"),
            "prepared_station_geometry"
        );
        assert_eq!(
            metadata_str(
                &seed.model_track_area.metadata,
                "station_geometry_artifact_key"
            ),
            "test-bundle-hash"
        );
    }

    #[test]
    fn car_mintime_open_seed_uses_open_topology_and_start_speed_floor() {
        let request = MintimeSolveRequestV1::parse(
            r#"{
              "schema_version": "rust_solver_http_request.v1",
              "request_id": "req-open-1",
              "project_id": "project-1",
              "station_count": 20,
              "solve_options": {
                "open_start_speed_mps": 0,
                "open_finish_speed_mps": 0
              },
              "track_area": {
                "schema_version": "TrackAreaContractV1",
                "track_id": "open-s-bend-lab-v1",
                "units": "m",
                "trajectory_mode": "open",
                "start_finish_xy_m": {"p1_m": [0,-3], "p2_m": [0,3]},
                "finish_line_xy_m": {"p1_m": [116,5], "p2_m": [116,11]},
                "left_boundary_xy_m": [[0,-3], [30,-1], [60,4], [90,2], [116,5]],
                "right_boundary_xy_m": [[0,3], [30,5], [60,10], [90,8], [116,11]],
                "metadata": {}
              },
              "vehicle_dynamics_profile": {
                "schema_version": "vehicle_dynamics_profile.v1",
                "profile_id": "car_dynamics:kart_125cc",
                "model_family": "car_dynamics",
                "preset_id": "kart_125cc",
                "solver_id": "old_car_mintime",
                "parameters": {
                  "mass_kg": 165,
                  "max_speed_mps": 34,
                  "delta_max_rad": 0.6,
                  "f_drive_max_n": 1800,
                  "f_brake_max_n": 2600
                }
              }
            }"#,
            VehicleDynamicsModelFamily::CarDynamics,
        )
        .unwrap();
        let params = CarDoubleTrackParams::from_profile(&request.vehicle_dynamics_profile).unwrap();
        let seed = build_car_mintime_nlp_seed(&request, params).unwrap();

        assert_eq!(seed.model_track_area.trajectory_mode, "open");
        assert_eq!(seed.dimensions.station_count, 20);
        assert_eq!(seed.dimensions.interval_count, 19);
        assert_eq!(seed.station_s_m.len(), seed.dimensions.station_count);
        assert_eq!(seed.station_s_m[0], 0.0);
        assert!(
            seed.station_s_m.last().copied().unwrap_or_default() > 100.0,
            "open station path should preserve finish distance"
        );
        assert_eq!(seed.initial_guess[super::STATE_V_MPS], 0.1);
        assert_eq!(seed.lower_bounds[super::STATE_V_MPS], 0.1);
        assert_eq!(seed.upper_bounds[super::STATE_V_MPS], 0.1);
        let final_speed_index =
            super::state_offset(seed.dimensions.station_count - 1) + super::STATE_V_MPS;
        assert_eq!(seed.initial_guess[final_speed_index], 0.1);
        assert_eq!(seed.lower_bounds[final_speed_index], 0.1);
        assert_eq!(seed.upper_bounds[final_speed_index], 0.1);
        for state_index in [
            super::STATE_BETA_RAD,
            super::STATE_OMEGA_Z_RADPS,
            super::STATE_N_M,
            super::STATE_XI_RAD,
        ] {
            assert_eq!(
                seed.initial_guess[super::state_offset(0) + state_index],
                0.0
            );
            assert_eq!(seed.lower_bounds[super::state_offset(0) + state_index], 0.0);
            assert_eq!(seed.upper_bounds[super::state_offset(0) + state_index], 0.0);
        }
        for control_index in [super::CONTROL_DELTA_RAD, super::CONTROL_GAMMA_Y_N] {
            assert_eq!(
                seed.initial_guess[super::control_offset(&seed, 0) + control_index],
                0.0
            );
            assert_eq!(
                seed.lower_bounds[super::control_offset(&seed, 0) + control_index],
                0.0
            );
            assert_eq!(
                seed.upper_bounds[super::control_offset(&seed, 0) + control_index],
                0.0
            );
        }
        assert_eq!(
            metadata_str_optional(&seed.model_track_area.metadata, "station_geometry_source"),
            Some("universal_area_route_pair")
        );
        assert_eq!(
            metadata_f64_optional(
                &seed.model_track_area.metadata,
                "open_start_speed_requested_mps"
            ),
            Some(0.0)
        );
        assert_eq!(
            metadata_f64_optional(
                &seed.model_track_area.metadata,
                "open_start_speed_effective_mps"
            ),
            Some(0.1)
        );
        assert_eq!(
            metadata_bool_optional(&seed.model_track_area.metadata, "open_start_pose_locked"),
            Some(true)
        );
        assert_eq!(
            metadata_bool_optional(
                &seed.model_track_area.metadata,
                "open_start_first_lateral_control_locked"
            ),
            Some(true)
        );
        assert_eq!(
            metadata_f64_optional(
                &seed.model_track_area.metadata,
                "open_finish_speed_requested_mps"
            ),
            Some(0.0)
        );
        assert_eq!(
            metadata_f64_optional(
                &seed.model_track_area.metadata,
                "open_finish_speed_effective_mps"
            ),
            Some(0.1)
        );

        let rows = super::car_mintime_constraint_rows(
            seed.dimensions,
            &super::CarMintimeSolveOptions::default(),
        );
        assert!(
            rows.iter().any(|row| row.label() == "continuity_v_mps_18"),
            "open seed should keep final interval continuity into finish node"
        );
        assert!(
            !rows.iter().any(|row| row.label() == "continuity_v_mps_19"),
            "open seed must not add final->first continuity"
        );
        assert!(
            !rows
                .iter()
                .any(|row| row.label() == "control_rate_delta_rad_19"),
            "open seed must not add periodic control-rate continuity"
        );
    }

    #[test]
    fn car_mintime_published_station_xy_uses_section_basis() {
        let layout = MintimeNlpLayout::for_family(VehicleDynamicsModelFamily::CarDynamics);
        let dimensions = layout.dimensions_for_station_count(4, true);
        let decision_count = dimensions.decision_variable_count();
        let seed = super::CarMintimeNlpSeed {
            layout,
            dimensions,
            model_track_area: TrackAreaContractV1::new("synthetic", Vec::new(), Vec::new()),
            station_s_m: vec![0.0, 1.0, 2.0, 3.0],
            centerline_xy_m: vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]],
            kappa_1pm: vec![0.0; 4],
            ref_tangent_xy: vec![[1.0, 0.0]; 4],
            ref_left_normal_xy: vec![[0.0, 1.0]; 4],
            section_dir_xy: vec![[0.0, -1.0]; 4],
            section_dir_derivative_xy: vec![[0.0, 0.0]; 4],
            width_left_m: vec![3.0; 4],
            width_right_m: vec![3.0; 4],
            initial_guess: vec![0.0; decision_count],
            lower_bounds: vec![0.0; decision_count],
            upper_bounds: vec![0.0; decision_count],
        };
        let mut x = vec![0.0; decision_count];
        x[super::state_offset(1) + super::STATE_N_M] = 2.0;

        let point = super::station_xy_from(&seed, &x, 1);

        assert!((point[0] - 1.0).abs() < 1e-12, "{point:?}");
        assert!((point[1] - 2.0).abs() < 1e-12, "{point:?}");
    }

    #[test]
    fn car_mintime_synthetic_curvature_fixture_matches_straight_and_circle() {
        let straight =
            super::signed_three_point_curvature_1pm([0.0, 0.0], [10.0, 0.0], [20.0, 0.0]);
        assert!(straight.abs() < 1e-12, "{straight}");

        let radius_m = 50.0;
        let angle_rad = 0.1_f64;
        let previous = [radius_m * (-angle_rad).cos(), radius_m * (-angle_rad).sin()];
        let current = [radius_m, 0.0];
        let next = [radius_m * angle_rad.cos(), radius_m * angle_rad.sin()];
        let circle = super::signed_three_point_curvature_1pm(previous, current, next);
        assert!(
            (circle - 1.0 / radius_m).abs() < 1e-12,
            "circle curvature={circle}"
        );

        let speed_mps = 20.0;
        let ay_xy_mps2 = speed_mps * speed_mps * circle;
        assert!((ay_xy_mps2 - 8.0).abs() < 1e-10, "{ay_xy_mps2}");
    }

    #[test]
    fn car_mintime_seed_uses_production_station_selector_for_asymmetric_loop() {
        let track_area = read_track_area_contract(&crate_path(
            "tests/public-fixtures/asymmetric-loop-track-area-v1.json",
        ));
        let request = MintimeSolveRequestV1 {
            request_id: "req-asymmetric-loop".to_owned(),
            project_id: "project-1".to_owned(),
            station_count: 160,
            geometry_input: MintimeGeometryInput::LegacyRawGeometry(track_area),
            vehicle_dynamics_profile: test_car_profile(),
            solve_options: Vec::new(),
        };
        let params = CarDoubleTrackParams::from_profile(&request.vehicle_dynamics_profile).unwrap();

        let seed = build_car_mintime_nlp_seed(&request, params).unwrap();

        assert_eq!(seed.dimensions.station_count, 160);
        assert_eq!(
            metadata_str(&seed.model_track_area.metadata, "station_geometry_source"),
            "universal_area_route_pair"
        );
        assert_eq!(
            metadata_str(&seed.model_track_area.metadata, "station_builder"),
            "universal_area_route_pair"
        );
    }

    #[test]
    fn car_mintime_seed_uses_production_station_selector_for_compact_oval() {
        let track_area = read_track_area_contract(&crate_path(
            "tests/public-fixtures/compact-oval-track-area-v1.json",
        ));
        let request = MintimeSolveRequestV1 {
            request_id: "req-compact-oval".to_owned(),
            project_id: "project-1".to_owned(),
            station_count: 160,
            geometry_input: MintimeGeometryInput::LegacyRawGeometry(track_area),
            vehicle_dynamics_profile: test_car_profile(),
            solve_options: Vec::new(),
        };
        let params = CarDoubleTrackParams::from_profile(&request.vehicle_dynamics_profile).unwrap();

        let seed = build_car_mintime_nlp_seed(&request, params).unwrap();

        assert_eq!(seed.dimensions.station_count, 160);
        assert_eq!(
            metadata_str(&seed.model_track_area.metadata, "station_geometry_source"),
            "universal_area_route_pair"
        );
        assert_eq!(
            metadata_str(&seed.model_track_area.metadata, "station_builder"),
            "universal_area_route_pair"
        );
    }

    #[test]
    fn car_mintime_seed_rejects_explicit_station_builder_option() {
        let request = MintimeSolveRequestV1::parse(
            r#"{
              "schema_version": "rust_solver_http_request.v1",
              "request_id": "req-explicit-station-builder",
              "project_id": "project-1",
              "station_count": 32,
              "solve_options": {
                "production_station_builder": "generated_boundary_pair"
              },
              "track_area": {
                "schema_version": "TrackAreaContractV1",
                "track_id": "rice_manual",
                "units": "m",
                "left_boundary_xy_m": [[0,0], [0,20], [20,20], [20,0]],
                "right_boundary_xy_m": [[4,4], [4,16], [16,16], [16,4]],
                "trajectory_mode": "closed",
                "metadata": {}
              },
              "vehicle_dynamics_profile": {
                "schema_version": "vehicle_dynamics_profile.v1",
                "profile_id": "car_dynamics:kart_125cc",
                "model_family": "car_dynamics",
                "preset_id": "kart_125cc",
                "solver_id": "old_car_mintime",
                "parameters": {
                  "mass_kg": 165,
                  "max_speed_mps": 34,
                  "delta_max_rad": 0.6,
                  "f_drive_max_n": 1800,
                  "f_brake_max_n": 2600
                }
              }
            }"#,
            VehicleDynamicsModelFamily::CarDynamics,
        )
        .unwrap();
        let params = CarDoubleTrackParams::from_profile(&request.vehicle_dynamics_profile).unwrap();
        let error = build_car_mintime_nlp_seed(&request, params).unwrap_err();

        assert_eq!(error.code, "solve.invalidRequest");
        assert!(error.message.contains("legacy station builder selection"));
    }

    #[test]
    fn car_mintime_problem_builds_python_style_constraint_diagnostics() {
        let request = car_mintime_closed_test_request(32);
        let params = CarDoubleTrackParams::from_profile(&request.vehicle_dynamics_profile).unwrap();
        let seed = build_car_mintime_nlp_seed(&request, params).unwrap();
        let problem = build_car_mintime_nlp_problem(seed, params).unwrap();
        let expected_constraint_count = 32 * (super::CAR_COLLOCATION_DEGREE * 18 + 20) + 31 * 3;
        let expected_collocation_dynamics_jacobian_entries = 32
            * super::CAR_COLLOCATION_DEGREE
            * super::CAR_STATE_LEN
            * (super::CAR_STATE_LEN + super::CAR_COLLOCATION_DEGREE + super::CAR_CONTROL_LEN);
        let expected_continuity_jacobian_entries =
            32 * super::CAR_STATE_LEN * (2 + super::CAR_COLLOCATION_DEGREE);
        let expected_path_jacobian_entries = 32
            * (2 + 4 * (super::CAR_STATE_LEN + super::CAR_CONTROL_LEN)
                + 4 * (super::CAR_STATE_LEN + super::CAR_CONTROL_LEN)
                + 4 * (super::CAR_STATE_LEN + super::CAR_CONTROL_LEN)
                + super::CAR_STATE_LEN
                + super::CAR_CONTROL_LEN
                + 2);
        let expected_collocation_feasibility_jacobian_entries = 32
            * super::CAR_COLLOCATION_DEGREE
            * (12 * (super::CAR_STATE_LEN + super::CAR_CONTROL_LEN) + 2);
        let expected_control_rate_jacobian_entries =
            31 * 3 * (1 + super::CAR_STATE_LEN + super::CAR_CONTROL_LEN);
        let expected_jacobian_entry_count = expected_collocation_dynamics_jacobian_entries
            + expected_continuity_jacobian_entries
            + expected_path_jacobian_entries
            + expected_collocation_feasibility_jacobian_entries
            + expected_control_rate_jacobian_entries;

        assert_eq!(problem.constraints.len(), expected_constraint_count);
        assert_eq!(problem.constraint_count(), expected_constraint_count);
        assert_eq!(
            problem.decision_variable_count(),
            problem.seed.initial_guess.len()
        );
        assert_eq!(
            problem.constraint_lower_bounds.len(),
            expected_constraint_count
        );
        assert_eq!(
            problem.constraint_upper_bounds.len(),
            expected_constraint_count
        );
        assert_eq!(
            problem.initial_diagnostics.constraint_count,
            expected_constraint_count
        );
        assert!(problem.initial_diagnostics.objective_initial_s > 0.0);
        assert_eq!(
            problem.objective(&problem.seed.initial_guess),
            problem.initial_diagnostics.objective_initial_s
        );
        let mut full_numeric_gradient = vec![f64::NAN; problem.decision_variable_count()];
        let mut structured_gradient = vec![f64::NAN; problem.decision_variable_count()];
        problem.objective_gradient_numeric(&problem.seed.initial_guess, &mut full_numeric_gradient);
        problem.objective_gradient_structured_numeric(
            &problem.seed.initial_guess,
            &mut structured_gradient,
        );
        let max_gradient_delta = full_numeric_gradient
            .iter()
            .zip(&structured_gradient)
            .map(|(left, right)| (left - right).abs())
            .fold(0.0, f64::max);
        assert!(
            max_gradient_delta <= 1e-8,
            "structured objective gradient drifted from full numeric gradient: {max_gradient_delta}"
        );
        assert_eq!(
            problem.constraint_values(&problem.seed.initial_guess).len(),
            expected_constraint_count
        );
        assert_eq!(
            problem.jacobian_pattern.len(),
            expected_jacobian_entry_count
        );
        let full_numeric_jacobian = problem.jacobian_values_numeric(&problem.seed.initial_guess);
        let structured_numeric_jacobian =
            problem.jacobian_values_structured_numeric(&problem.seed.initial_guess);
        let max_jacobian_delta = full_numeric_jacobian
            .iter()
            .zip(&structured_numeric_jacobian)
            .map(|(left, right)| (left - right).abs())
            .fold(0.0, f64::max);
        assert!(
            max_jacobian_delta <= 1e-8,
            "structured Jacobian drifted from full numeric Jacobian: {max_jacobian_delta}"
        );
        assert_eq!(
            structured_numeric_jacobian.len(),
            problem.jacobian_pattern.len()
        );
        assert!(problem
            .initial_diagnostics
            .max_initial_abs_residual
            .is_finite());
        assert_eq!(problem.constraints[0].label(), "colloc_v_mps_0_1");
        assert!(problem
            .constraints
            .iter()
            .any(|row| row.label() == "continuity_v_mps_0"));
        assert!(problem
            .constraints
            .iter()
            .any(|row| row.label() == "tire_fl_0"));
        assert!(problem
            .constraints
            .iter()
            .any(|row| row.label() == "control_rate_delta_rad_1"));
        let power_index = problem
            .constraints
            .iter()
            .position(|row| row.label() == "power_limit_0")
            .unwrap();
        assert!(problem.constraint_lower_bounds[power_index].is_infinite());
        assert_eq!(
            problem.constraint_upper_bounds[power_index],
            problem.params.power_max_w
        );
        let tire_index = problem
            .constraints
            .iter()
            .position(|row| row.label() == "tire_fl_0")
            .unwrap();
        assert_eq!(problem.constraint_lower_bounds[tire_index], 0.0);
        assert_eq!(problem.constraint_upper_bounds[tire_index], 1.0);
        let brake_rate_index = problem
            .constraints
            .iter()
            .position(|row| row.label() == "control_rate_f_brake_N_1")
            .unwrap();
        assert_eq!(
            problem.constraint_lower_bounds[brake_rate_index],
            -problem.params.brake_force_max_n / problem.params.brake_response_s
        );
        assert!(problem.constraint_upper_bounds[brake_rate_index].is_infinite());
    }

    #[test]
    fn car_physics_bundle_v1_exports_comparable_station_and_collocation_rows() {
        let request = car_mintime_closed_test_request(20);
        let params = CarDoubleTrackParams::from_profile(&request.vehicle_dynamics_profile).unwrap();
        let seed = build_car_mintime_nlp_seed(&request, params).unwrap();
        let problem = build_car_mintime_nlp_problem(seed, params).unwrap();
        let bundle =
            super::car_mintime_physics_bundle_json(&problem, &problem.seed.initial_guess, true);

        assert_eq!(
            bundle.get("schema_version").and_then(JsonValue::as_str),
            Some("car_physics_bundle_v1")
        );
        let station_columns = bundle
            .get("station_columns")
            .and_then(JsonValue::as_array)
            .expect("station columns");
        let station_rows = bundle
            .get("station_rows")
            .and_then(JsonValue::as_array)
            .expect("station rows");
        let collocation_rows = bundle
            .get("collocation_rows")
            .and_then(JsonValue::as_array)
            .expect("collocation rows");

        assert_eq!(station_rows.len(), problem.seed.dimensions.station_count);
        assert_eq!(
            collocation_rows.len(),
            problem.seed.dimensions.interval_count * super::CAR_COLLOCATION_DEGREE
        );
        assert!(station_columns
            .iter()
            .any(|column| column.as_str() == Some("front_kamm")));
        assert!(station_columns
            .iter()
            .any(|column| column.as_str() == Some("rear_kamm")));
        assert!(station_columns
            .iter()
            .any(|column| column.as_str() == Some("power_margin_w")));
        assert!(station_columns
            .iter()
            .any(|column| column.as_str() == Some("path_bound_margin_m")));
        assert_eq!(
            station_rows[0].as_array().expect("station row").len(),
            station_columns.len()
        );
        assert_eq!(
            collocation_rows[0]
                .as_array()
                .expect("collocation row")
                .len(),
            station_columns.len()
        );
        assert!(bundle
            .get("objective_split")
            .and_then(|value| value.get("lap_time_s"))
            .and_then(JsonValue::as_f64)
            .is_some_and(f64::is_finite));
        let formulation = bundle
            .get("formulation_contract")
            .expect("formulation contract");
        assert_eq!(
            formulation
                .get("formulation_mode")
                .and_then(JsonValue::as_str),
            Some("prepeak_grip_v1")
        );
        assert_eq!(
            formulation
                .get("grip_constraint_scaling")
                .and_then(JsonValue::as_str),
            Some("alpha_over_alpha_peak")
        );
        assert_close(
            formulation
                .get("prepeak_grip_margin")
                .and_then(JsonValue::as_f64)
                .expect("prepeak margin"),
            0.98,
        );
        assert!(formulation
            .get("front_alpha_peak_rad")
            .and_then(JsonValue::as_f64)
            .is_some_and(|value| value > 0.0));
    }

    #[test]
    fn car_prepeak_grip_formulation_is_the_default_and_uses_normalized_constraints() {
        let mut request = car_mintime_closed_test_request(20);
        request.solve_options.push((
            "car_prepeak_grip_margin".to_owned(),
            JsonValue::Number(0.98),
        ));
        let params = CarDoubleTrackParams::from_profile(&request.vehicle_dynamics_profile).unwrap();
        let options = super::CarMintimeSolveOptions::try_from_request(&request).unwrap();
        let seed = build_car_mintime_nlp_seed(&request, params).unwrap();
        let problem =
            super::build_car_mintime_nlp_problem_with_options(seed, params, options).unwrap();

        assert_eq!(
            problem.options.formulation_mode,
            super::CarMintimeFormulationMode::PrepeakGripV1
        );
        assert_eq!(problem.options.prepeak_grip_margin, 0.98);
        let front_peak = super::car_pacejka_peak_slip_rad(params, "fl");
        assert!(front_peak.is_finite() && front_peak > 0.0);
        for label in [
            "slip_prepeak_fl_0",
            "slip_prepeak_rr_0",
            "colloc_slip_prepeak_fl_0_1",
            "colloc_slip_prepeak_rr_0_3",
        ] {
            let index = problem
                .constraints
                .iter()
                .position(|row| row.label() == label)
                .unwrap_or_else(|| panic!("missing pre-peak row {label}"));
            assert_close(problem.constraint_lower_bounds[index], -0.98);
            assert_close(problem.constraint_upper_bounds[index], 0.98);
            assert!(problem.constraint_values(&problem.seed.initial_guess)[index].is_finite());
        }

        let mut x = problem.seed.initial_guess.clone();
        x[super::control_offset(&problem.seed, 0) + super::CONTROL_DELTA_RAD] = 0.5 * front_peak;
        let front_index = problem
            .constraints
            .iter()
            .position(|row| row.label() == "slip_prepeak_fl_0")
            .unwrap();
        assert_close(problem.constraint_values(&x)[front_index], 0.5);
    }

    #[test]
    fn car_legacy_full_pacejka_formulation_explicitly_disables_prepeak_constraints() {
        let mut request = car_mintime_closed_test_request(20);
        request.solve_options.push((
            "car_mintime_formulation_mode".to_owned(),
            JsonValue::String("legacy_full_pacejka".to_owned()),
        ));
        let params = CarDoubleTrackParams::from_profile(&request.vehicle_dynamics_profile).unwrap();
        let options = super::CarMintimeSolveOptions::try_from_request(&request).unwrap();
        let seed = build_car_mintime_nlp_seed(&request, params).unwrap();
        let problem =
            super::build_car_mintime_nlp_problem_with_options(seed, params, options).unwrap();

        assert_eq!(
            problem.options.formulation_mode,
            super::CarMintimeFormulationMode::LegacyFullPacejka
        );
        assert!(!problem
            .constraints
            .iter()
            .any(|row| row.family() == "slip_prepeak"));
    }

    #[test]
    fn car_formulation_mode_rejects_unknown_values() {
        let mut request = car_mintime_closed_test_request(20);
        request.solve_options.push((
            "car_mintime_formulation_mode".to_owned(),
            JsonValue::String("magic_apex_mode".to_owned()),
        ));

        let error = super::CarMintimeSolveOptions::try_from_request(&request).unwrap_err();
        assert_eq!(error.code, "solve.invalidRequest");
        assert!(error.message.contains("car_mintime_formulation_mode"));
    }

    #[test]
    fn car_collocation_polynomial_dense_state_matches_collocation_nodes() {
        let request = car_mintime_closed_test_request(32);
        let params = CarDoubleTrackParams::from_profile(&request.vehicle_dynamics_profile).unwrap();
        let seed = build_car_mintime_nlp_seed(&request, params).unwrap();
        let mut x = seed.initial_guess.clone();
        let interval = 3;

        x[super::state_offset(interval) + super::STATE_V_MPS] = 11.0;
        x[super::state_offset(interval) + super::STATE_N_M] = -0.4;
        for point in 0..super::CAR_COLLOCATION_DEGREE {
            let offset = super::collocation_state_offset(&seed, interval, point);
            x[offset + super::STATE_V_MPS] = 20.0 + point as f64;
            x[offset + super::STATE_N_M] = 0.5 + 0.25 * point as f64;
        }

        let coeffs = super::car_legendre_collocation_coefficients_degree3();
        let start = super::car_collocation_state_at_tau(&seed, &x, interval, 0.0);
        assert_close(start.v_mps, 11.0);
        assert_close(start.n_m, -0.4);

        for point in 1..=super::CAR_COLLOCATION_DEGREE {
            let reconstructed =
                super::car_collocation_state_at_tau(&seed, &x, interval, coeffs.tau[point]);
            let actual = super::collocation_state_from(&seed, &x, interval, point - 1);
            assert_close(reconstructed.v_mps, actual.v_mps);
            assert_close(reconstructed.n_m, actual.n_m);
        }
    }

    #[test]
    fn car_collocation_state_derivatives_match_synthetic_polynomial() {
        let request = car_mintime_closed_test_request(32);
        let params = CarDoubleTrackParams::from_profile(&request.vehicle_dynamics_profile).unwrap();
        let seed = build_car_mintime_nlp_seed(&request, params).unwrap();
        let mut x = seed.initial_guess.clone();
        let interval = 4;
        let coeffs = super::car_legendre_collocation_coefficients_degree3();
        let poly = |tau: f64| 2.0 + 3.0 * tau + 5.0 * tau * tau - 7.0 * tau * tau * tau;
        let dpoly = |tau: f64| 3.0 + 10.0 * tau - 21.0 * tau * tau;
        let d2poly = |tau: f64| 10.0 - 42.0 * tau;

        x[super::state_offset(interval) + super::STATE_N_M] = poly(coeffs.tau[0]);
        for point in 0..super::CAR_COLLOCATION_DEGREE {
            let offset = super::collocation_state_offset(&seed, interval, point);
            x[offset + super::STATE_N_M] = poly(coeffs.tau[point + 1]);
        }

        let tau = 0.37;
        let ds_m = super::interval_ds_m(&seed, interval);
        let derivative = super::car_collocation_state_derivatives_at_tau(&seed, &x, interval, tau);
        let second_derivative =
            super::car_collocation_state_second_derivatives_at_tau(&seed, &x, interval, tau);

        assert_close(derivative.n_m, dpoly(tau) / ds_m);
        assert_close(second_derivative.n_m, d2poly(tau) / (ds_m * ds_m));
    }

    #[test]
    fn car_collocation_polynomial_handles_closed_final_interval() {
        let request = car_mintime_closed_test_request(32);
        let params = CarDoubleTrackParams::from_profile(&request.vehicle_dynamics_profile).unwrap();
        let seed = build_car_mintime_nlp_seed(&request, params).unwrap();
        let x = seed.initial_guess.clone();
        let final_interval = seed.dimensions.interval_count - 1;

        let start = super::car_collocation_state_at_tau(&seed, &x, final_interval, 0.0);
        let station = super::car_state_from(&seed, &x, final_interval);
        assert_close(start.v_mps, station.v_mps);
        assert_close(start.n_m, station.n_m);
    }

    #[test]
    fn car_section_geometry_diagnostics_accept_consistent_negative_orientation() {
        let mut min_section_det = f64::INFINITY;
        let mut min_abs_section_det = f64::INFINITY;
        let mut min_forward_progress = f64::INFINITY;
        let mut pure_frenet_factor_min_debug = f64::INFINITY;
        let mut sigma_clamp_count = 0_i64;
        let mut section_det_reference_sign = 0.0_f64;
        let mut section_det_sign_flip_count = 0_i64;
        let mut worst_row = "none".to_owned();

        for (label, determinant) in [("left", -0.8), ("right", -1.2)] {
            super::update_section_geometry_minima(
                label,
                determinant,
                0.9,
                1.0,
                &mut min_section_det,
                &mut min_abs_section_det,
                &mut min_forward_progress,
                &mut pure_frenet_factor_min_debug,
                &mut sigma_clamp_count,
                &mut section_det_reference_sign,
                &mut section_det_sign_flip_count,
                &mut worst_row,
            );
        }

        assert_eq!(min_section_det, -1.2);
        assert_eq!(min_abs_section_det, 0.8);
        assert_eq!(section_det_reference_sign, -1.0);
        assert_eq!(section_det_sign_flip_count, 0);
        assert_eq!(sigma_clamp_count, 0);
    }

    #[test]
    fn car_dense_trajectory_json_is_product_contract() {
        let request = car_mintime_closed_test_request(20);
        let params = CarDoubleTrackParams::from_profile(&request.vehicle_dynamics_profile).unwrap();
        let seed = build_car_mintime_nlp_seed(&request, params).unwrap();
        let x = seed.initial_guess.clone();
        let dense = super::car_dense_trajectory_json(&seed, params, &x, 4);

        assert_eq!(
            dense.get("schema_version").and_then(JsonValue::as_str),
            Some("trajectory_dense.v1")
        );
        assert_eq!(
            dense.get("source_frame").and_then(JsonValue::as_str),
            Some("dense_section_frame_collocation_state_coherent_frame")
        );
        assert_eq!(
            dense.get("state_source").and_then(JsonValue::as_str),
            Some("collocation_lagrange_state")
        );
        assert_eq!(
            dense.get("geometry_source").and_then(JsonValue::as_str),
            Some("station_hermite_coherent_frame")
        );
        assert_eq!(
            dense.get("acceleration_frame").and_then(JsonValue::as_str),
            Some("velocity_tangent_normal")
        );
        assert!(dense.get("provisional").is_none());
        assert!(dense.get("debug_only").is_none());
        assert!(dense.get("use_for_product_overlay").is_none());
        assert!(dense.get("use_for_product_curvature").is_none());
        assert_eq!(
            dense.get("sample_kind").and_then(JsonValue::as_str),
            Some("collocation_continuation_dense")
        );
        assert!(dense.get("ay_geo_mps2").is_none());
        assert!(dense.get("ay_model_mps2").is_some());
        let s_values = dense
            .get("s_m")
            .and_then(JsonValue::as_array)
            .expect("dense trajectory should have s array");
        assert_eq!(s_values.len(), seed.dimensions.interval_count * 4);
        let x_values = dense
            .get("x_m")
            .and_then(JsonValue::as_array)
            .expect("dense trajectory should have x array");
        let y_values = dense
            .get("y_m")
            .and_then(JsonValue::as_array)
            .expect("dense trajectory should have y array");
        let ax_values = dense
            .get("ax_mps2")
            .and_then(JsonValue::as_array)
            .expect("dense trajectory should have ax array");
        assert_eq!(ax_values.len(), seed.dimensions.interval_count * 4);
        let station_n = super::car_state_from(&seed, &x, 0).n_m;
        let expected_x = seed.centerline_xy_m[0][0] - seed.section_dir_xy[0][0] * station_n;
        let expected_y = seed.centerline_xy_m[0][1] - seed.section_dir_xy[0][1] * station_n;
        assert_close(x_values[0].as_f64().unwrap(), expected_x);
        assert_close(y_values[0].as_f64().unwrap(), expected_y);
    }

    #[test]
    fn published_car_acceleration_uses_velocity_tangent_normal_frame() {
        let (longitudinal, lateral) =
            super::velocity_frame_acceleration(2.0, 3.0, std::f64::consts::FRAC_PI_2);

        assert_close(longitudinal, 3.0);
        assert_close(lateral, -2.0);
    }

    #[test]
    fn car_section_frame_coherence_audit_reports_generated_frame_status() {
        let request = car_mintime_closed_test_request(20);
        let params = CarDoubleTrackParams::from_profile(&request.vehicle_dynamics_profile).unwrap();
        let seed = build_car_mintime_nlp_seed(&request, params).unwrap();
        let audit = super::car_section_frame_coherence_audit_json(&seed);

        assert_eq!(
            audit.get("schema_version").and_then(JsonValue::as_str),
            Some("section_frame_coherence_audit.v1")
        );
        assert!(audit.get("finite_difference_xy_vs_path_ds_norm").is_some());
        assert!(audit
            .get("finite_difference_path_ds_vs_path_d2s_norm")
            .is_some());
        assert!(audit
            .get("validation_status")
            .and_then(JsonValue::as_str)
            .is_some());
    }

    #[test]
    fn car_solver_geometry_matches_published_geometry_at_collocation_nodes() {
        for direction in ["clockwise", "counterclockwise"] {
            let request = car_mintime_closed_test_request_for_direction(20, direction);
            let params =
                CarDoubleTrackParams::from_profile(&request.vehicle_dynamics_profile).unwrap();
            let seed = build_car_mintime_nlp_seed(&request, params).unwrap();
            let sampler = crate::dense_frenet::DenseSectionFrameHermiteSampler {
                station_s_m: &seed.station_s_m,
                centerline_xy_m: &seed.centerline_xy_m,
                tangent_xy: &seed.ref_tangent_xy,
                section_dir_xy: &seed.section_dir_xy,
                section_dir_derivative_xy: &seed.section_dir_derivative_xy,
                closed: true,
            };
            let coefficients = super::car_legendre_collocation_coefficients_degree3();

            for interval in 0..seed.dimensions.interval_count {
                for point in 1..=super::CAR_COLLOCATION_DEGREE {
                    let tau = coefficients.tau[point];
                    let solver = super::interpolated_sections_geometry(&seed, interval, tau);
                    let published = sampler
                        .sample_at_interval_tau(interval, tau)
                        .expect("valid published section-frame geometry");
                    let centerline_speed_sq = published.centerline_ds[0]
                        * published.centerline_ds[0]
                        + published.centerline_ds[1] * published.centerline_ds[1];
                    let centerline_speed = centerline_speed_sq.sqrt();
                    let published_tangent = [
                        published.centerline_ds[0] / centerline_speed,
                        published.centerline_ds[1] / centerline_speed,
                    ];
                    let published_kappa = (published.centerline_ds[0]
                        * published.centerline_d2s[1]
                        - published.centerline_ds[1] * published.centerline_d2s[0])
                        / centerline_speed_sq.powf(1.5);

                    assert_close(solver.kappa_1pm, published_kappa);
                    assert_close(solver.ref_tangent_xy[0], published_tangent[0]);
                    assert_close(solver.ref_tangent_xy[1], published_tangent[1]);
                    assert_close(solver.section_dir_xy[0], published.section_dir[0]);
                    assert_close(solver.section_dir_xy[1], published.section_dir[1]);
                    assert_close(
                        solver.section_dir_derivative_xy[0],
                        published.section_dir_ds[0],
                    );
                    assert_close(
                        solver.section_dir_derivative_xy[1],
                        published.section_dir_ds[1],
                    );
                    assert_close(
                        solver.section_dir_second_derivative_xy[0],
                        published.section_dir_d2s[0],
                    );
                    assert_close(
                        solver.section_dir_second_derivative_xy[1],
                        published.section_dir_d2s[1],
                    );
                }
            }
        }
    }

    #[test]
    fn car_section_progress_matches_published_path_velocity_direction() {
        for direction in ["clockwise", "counterclockwise"] {
            let request = car_mintime_closed_test_request_for_direction(20, direction);
            let params =
                CarDoubleTrackParams::from_profile(&request.vehicle_dynamics_profile).unwrap();
            let seed = build_car_mintime_nlp_seed(&request, params).unwrap();
            let mut x = seed.initial_guess.clone();
            let interval = 2;
            let point = 1;
            let collocation_offset = super::collocation_state_offset(&seed, interval, point - 1);
            x[collocation_offset + super::STATE_N_M] = 0.7;
            x[collocation_offset + super::STATE_BETA_RAD] = 0.04;
            x[collocation_offset + super::STATE_XI_RAD] = -0.015;

            let coefficients = super::car_legendre_collocation_coefficients_degree3();
            let tau = coefficients.tau[point];
            let sampler = crate::dense_frenet::DenseSectionFrameHermiteSampler {
                station_s_m: &seed.station_s_m,
                centerline_xy_m: &seed.centerline_xy_m,
                tangent_xy: &seed.ref_tangent_xy,
                section_dir_xy: &seed.section_dir_xy,
                section_dir_derivative_xy: &seed.section_dir_derivative_xy,
                closed: true,
            };
            let geometry = sampler
                .sample_at_interval_tau(interval, tau)
                .expect("valid published section-frame geometry");
            let state = super::collocation_state_from(&seed, &x, interval, point - 1);
            let dynamics =
                super::car_mintime_collocation_dynamics_from(&seed, params, &x, interval, point);
            let tangent_norm = geometry.centerline_ds[0].hypot(geometry.centerline_ds[1]);
            let tangent = [
                geometry.centerline_ds[0] / tangent_norm,
                geometry.centerline_ds[1] / tangent_norm,
            ];
            let left_normal = [-tangent[1], tangent[0]];
            let theta = state.xi_rad + state.beta_rad;
            let velocity_direction = [
                theta.cos() * tangent[0] + theta.sin() * left_normal[0],
                theta.cos() * tangent[1] + theta.sin() * left_normal[1],
            ];
            let path_ds = [
                geometry.centerline_ds[0]
                    - state.n_m * geometry.section_dir_ds[0]
                    - dynamics.dn_ds * geometry.section_dir[0],
                geometry.centerline_ds[1]
                    - state.n_m * geometry.section_dir_ds[1]
                    - dynamics.dn_ds * geometry.section_dir[1],
            ];
            let direction_cross =
                path_ds[0] * velocity_direction[1] - path_ds[1] * velocity_direction[0];

            assert!(
                direction_cross.abs() <= 1e-10,
                "published path derivative must align with velocity direction for {direction}, cross={direction_cross}"
            );
        }
    }

    #[test]
    fn car_boundary_continuity_audit_reports_endpoint_c1_residuals() {
        let request = car_mintime_closed_test_request(20);
        let params = CarDoubleTrackParams::from_profile(&request.vehicle_dynamics_profile).unwrap();
        let seed = build_car_mintime_nlp_seed(&request, params).unwrap();
        let x = seed.initial_guess.clone();
        let audit = super::car_collocation_geometry_boundary_continuity_audit_json(&seed, &x);

        assert_eq!(
            audit.get("schema_version").and_then(JsonValue::as_str),
            Some("car_collocation_geometry_boundary_continuity_audit.v2")
        );
        assert!(audit.get("path_ds_jump_norm_abs").is_some());
        assert!(audit.get("endpoint_heading_jump_rad_abs").is_some());
        assert!(audit.get("endpoint_c1_dn_left_abs").is_some());
        assert!(audit.get("endpoint_c1_dn_right_abs").is_some());
        assert!(audit.get("endpoint_c1_dn_max_abs").is_some());
        assert!(audit.get("endpoint_c1_heading_left_rad_abs").is_some());
        assert!(audit.get("endpoint_c1_heading_right_rad_abs").is_some());
        assert!(audit.get("endpoint_c1_heading_max_rad_abs").is_some());
        let top = audit
            .get("top_endpoint_c1_dn")
            .and_then(JsonValue::as_array)
            .expect("audit should include top C1 rows");
        assert!(!top.is_empty());
        let top_heading = audit
            .get("top_endpoint_c1_heading")
            .and_then(JsonValue::as_array)
            .expect("audit should include top heading C1 rows");
        assert!(!top_heading.is_empty());
        let top_jump = audit
            .get("top_endpoint_heading_jump")
            .and_then(JsonValue::as_array)
            .expect("audit should include top endpoint heading jump rows");
        assert!(!top_jump.is_empty());
        let row = &top[0];
        assert!(row.get("left_n_m").is_some());
        assert!(row.get("right_n_m").is_some());
        assert!(row.get("left_dn_ds").is_some());
        assert!(row.get("right_dn_ds").is_some());
        assert!(row.get("left_d2n_ds2").is_some());
        assert!(row.get("right_d2n_ds2").is_some());
        assert!(row.get("left_xi_rad").is_some());
        assert!(row.get("right_xi_rad").is_some());
        assert!(row.get("left_beta_rad").is_some());
        assert!(row.get("right_beta_rad").is_some());
        assert!(row.get("left_section_dir_xy").is_some());
        assert!(row.get("right_section_dir_xy").is_some());
        assert!(row.get("left_section_dir_derivative_xy").is_some());
        assert!(row.get("right_section_dir_derivative_xy").is_some());
        assert!(row.get("left_section_dir_second_derivative_xy").is_some());
        assert!(row.get("right_section_dir_second_derivative_xy").is_some());
        assert!(row.get("dn_ds_kin_left").is_some());
        assert!(row.get("dn_ds_kin_right").is_some());
        assert!(row.get("endpoint_c1_dn_left").is_some());
        assert!(row.get("endpoint_c1_dn_right").is_some());
        assert!(row.get("endpoint_c1_heading_left_rad").is_some());
        assert!(row.get("endpoint_c1_heading_right_rad").is_some());
        assert!(row.get("endpoint_heading_jump_rad").is_some());
    }

    #[test]
    fn car_endpoint_c1_residuals_use_endpoint_state_kinematics() {
        let request = car_mintime_closed_test_request(20);
        let params = CarDoubleTrackParams::from_profile(&request.vehicle_dynamics_profile).unwrap();
        let seed = build_car_mintime_nlp_seed(&request, params).unwrap();
        let mut x = seed.initial_guess.clone();
        let interval = 3;
        let boundary_station = super::next_station_index(&seed, interval);

        x[super::state_offset(boundary_station) + super::STATE_N_M] = -0.25;
        x[super::state_offset(boundary_station) + super::STATE_BETA_RAD] = -0.02;
        x[super::state_offset(boundary_station) + super::STATE_XI_RAD] = 0.015;
        for point in 0..super::CAR_COLLOCATION_DEGREE {
            let offset = super::collocation_state_offset(&seed, interval, point);
            x[offset + super::STATE_N_M] = 0.4 + 0.05 * point as f64;
            x[offset + super::STATE_BETA_RAD] = 0.03 + 0.01 * point as f64;
            x[offset + super::STATE_XI_RAD] = -0.04 + 0.005 * point as f64;
        }

        let residuals = super::car_endpoint_continuity_residuals(&seed, &x, interval);
        let left_state = super::car_collocation_state_at_tau(&seed, &x, interval, 1.0);
        let station_state = super::car_state_from(&seed, &x, boundary_station);
        let left_state_ds =
            super::car_collocation_state_derivatives_at_tau(&seed, &x, interval, 1.0);
        let left_geometry = super::interpolated_sections_geometry(&seed, interval, 1.0);
        let expected_dn_ds_kin = crate::section_frame::section_frame_progress_from_derivatives(
            left_state.n_m,
            left_state.v_mps,
            left_state.beta_rad,
            left_state.xi_rad,
            left_geometry.ref_tangent_xy,
            left_geometry.ref_left_normal_xy,
            left_geometry.centerline_derivative_xy,
            left_geometry.section_dir_xy,
            left_geometry.section_dir_derivative_xy,
        )
        .dn_ds;

        assert!((left_state.n_m - station_state.n_m).abs() > 1e-3);
        assert_close(residuals.dn_ds_kin_left, expected_dn_ds_kin);
        assert_close(
            residuals.c1_kin_left,
            left_state_ds.n_m - expected_dn_ds_kin,
        );
    }

    #[test]
    fn car_endpoint_c1_penalty_is_default_off_and_increases_bad_objective() {
        let request = car_mintime_closed_test_request(20);
        let params = CarDoubleTrackParams::from_profile(&request.vehicle_dynamics_profile).unwrap();
        let seed = build_car_mintime_nlp_seed(&request, params).unwrap();
        let mut x = seed.initial_guess.clone();
        let weighted_options = super::CarMintimeSolveOptions {
            penalty_endpoint_c1_dn: 0.10,
            endpoint_c1_dn_scale: 0.005,
            penalty_endpoint_c1_heading: 0.10,
            endpoint_c1_heading_scale_rad: 0.005,
            penalty_endpoint_heading_jump: 0.10,
            endpoint_heading_jump_scale_rad: 0.005,
            penalty_endpoint_d2n_jump: 0.10,
            endpoint_d2n_jump_scale: 0.02,
            ..Default::default()
        };
        let default_problem = build_car_mintime_nlp_problem(seed.clone(), params).unwrap();
        let weighted_problem = super::build_car_mintime_nlp_problem_with_options(
            seed.clone(),
            params,
            weighted_options,
        )
        .unwrap();
        let interval = 3;

        for point in 0..super::CAR_COLLOCATION_DEGREE {
            let offset = super::collocation_state_offset(&seed, interval, point);
            x[offset + super::STATE_N_M] = 0.45 + 0.2 * point as f64;
            x[offset + super::STATE_BETA_RAD] = 0.03 + 0.01 * point as f64;
            x[offset + super::STATE_XI_RAD] = -0.02 + 0.008 * point as f64;
        }
        let right_interval = super::next_station_index(&seed, interval);
        for point in 0..super::CAR_COLLOCATION_DEGREE {
            let offset = super::collocation_state_offset(&seed, right_interval, point);
            let p = point as f64 + 1.0;
            x[offset + super::STATE_N_M] = -0.15 + 0.05 * p * p;
        }

        let default_weights = super::CarMintimeObjectiveWeights::default();
        let default_endpoint_term =
            super::car_endpoint_c1_dn_objective_s(&seed, default_weights, &x);
        let weighted_endpoint_term =
            super::car_endpoint_c1_dn_objective_s(&seed, weighted_problem.objective_weights, &x);
        let default_heading_term =
            super::car_endpoint_c1_heading_objective_s(&seed, default_weights, &x);
        let weighted_heading_term = super::car_endpoint_c1_heading_objective_s(
            &seed,
            weighted_problem.objective_weights,
            &x,
        );
        let default_heading_jump_term =
            super::car_endpoint_heading_jump_objective_s(&seed, default_weights, &x);
        let weighted_heading_jump_term = super::car_endpoint_heading_jump_objective_s(
            &seed,
            weighted_problem.objective_weights,
            &x,
        );
        let default_d2n_jump_term =
            super::car_endpoint_d2n_jump_objective_s(&seed, default_weights, &x);
        let weighted_d2n_jump_term =
            super::car_endpoint_d2n_jump_objective_s(&seed, weighted_problem.objective_weights, &x);

        assert_close(default_endpoint_term, 0.0);
        assert!(weighted_endpoint_term > 0.0);
        assert_close(default_heading_term, 0.0);
        assert!(weighted_heading_term > 0.0);
        assert_close(default_heading_jump_term, 0.0);
        assert!(weighted_heading_jump_term > 0.0);
        assert_close(default_d2n_jump_term, 0.0);
        assert!(weighted_d2n_jump_term > 0.0);
        assert_close(
            default_problem.objective(&x),
            super::car_mintime_collocation_objective_s(&seed, params, &x)
                + super::car_mintime_regularization_objective_s(
                    &seed,
                    params,
                    default_problem.objective_weights,
                    &x,
                ),
        );
        assert!(weighted_problem.objective(&x) > default_problem.objective(&x));
    }

    #[test]
    fn car_endpoint_c1_options_parse_weight_and_scale() {
        let mut request = car_mintime_closed_test_request(20);
        request
            .solve_options
            .push(("penalty_delta".to_owned(), JsonValue::Number(11.0)));
        request
            .solve_options
            .push(("penalty_f".to_owned(), JsonValue::Number(0.02)));
        request
            .solve_options
            .push(("penalty_delta_dd".to_owned(), JsonValue::Number(0.06)));
        request
            .solve_options
            .push(("penalty_f_dd".to_owned(), JsonValue::Number(0.03)));
        request
            .solve_options
            .push(("penalty_n_dd".to_owned(), JsonValue::Number(0.04)));
        request
            .solve_options
            .push(("penalty_xi_dd".to_owned(), JsonValue::Number(0.07)));
        request
            .solve_options
            .push(("penalty_endpoint_c1_dn".to_owned(), JsonValue::Number(0.25)));
        request
            .solve_options
            .push(("endpoint_c1_dn_scale".to_owned(), JsonValue::Number(0.005)));
        request.solve_options.push((
            "penalty_endpoint_c1_heading".to_owned(),
            JsonValue::Number(0.5),
        ));
        request.solve_options.push((
            "endpoint_c1_heading_scale_rad".to_owned(),
            JsonValue::Number(0.003),
        ));
        request.solve_options.push((
            "penalty_endpoint_heading_jump".to_owned(),
            JsonValue::Number(0.75),
        ));
        request.solve_options.push((
            "endpoint_heading_jump_scale_rad".to_owned(),
            JsonValue::Number(0.004),
        ));
        request.solve_options.push((
            "penalty_endpoint_d2n_jump".to_owned(),
            JsonValue::Number(0.6),
        ));
        request.solve_options.push((
            "endpoint_d2n_jump_scale".to_owned(),
            JsonValue::Number(0.02),
        ));

        let options = super::CarMintimeSolveOptions::try_from_request(&request).unwrap();
        let weights = super::CarMintimeObjectiveWeights::from_options(&options);

        assert_close(options.penalty_delta, 11.0);
        assert_close(options.penalty_f, 0.02);
        assert_close(options.penalty_delta_dd, 0.06);
        assert_close(options.penalty_f_dd, 0.03);
        assert_close(options.penalty_n_dd, 0.04);
        assert_close(options.penalty_xi_dd, 0.07);
        assert_close(options.penalty_endpoint_c1_dn, 0.25);
        assert_close(options.endpoint_c1_dn_scale, 0.005);
        assert_close(options.penalty_endpoint_c1_heading, 0.5);
        assert_close(options.endpoint_c1_heading_scale_rad, 0.003);
        assert_close(options.penalty_endpoint_heading_jump, 0.75);
        assert_close(options.endpoint_heading_jump_scale_rad, 0.004);
        assert_close(options.penalty_endpoint_d2n_jump, 0.6);
        assert_close(options.endpoint_d2n_jump_scale, 0.02);
        assert_close(weights.penalty_delta, 11.0);
        assert_close(weights.penalty_f, 0.02);
        assert_close(weights.penalty_delta_dd, 0.06);
        assert_close(weights.penalty_f_dd, 0.03);
        assert_close(weights.penalty_n_dd, 0.04);
        assert_close(weights.penalty_xi_dd, 0.07);
        assert_close(weights.penalty_endpoint_c1_dn, 0.25);
        assert_close(weights.endpoint_c1_dn_scale, 0.005);
        assert_close(weights.penalty_endpoint_c1_heading, 0.5);
        assert_close(weights.endpoint_c1_heading_scale_rad, 0.003);
        assert_close(weights.penalty_endpoint_heading_jump, 0.75);
        assert_close(weights.endpoint_heading_jump_scale_rad, 0.004);
        assert_close(weights.penalty_endpoint_d2n_jump, 0.6);
        assert_close(weights.endpoint_d2n_jump_scale, 0.02);
    }

    #[test]
    fn car_endpoint_c1_penalty_gradient_matches_numeric_objective_gradient() {
        let request = car_mintime_closed_test_request(20);
        let params = CarDoubleTrackParams::from_profile(&request.vehicle_dynamics_profile).unwrap();
        let seed = build_car_mintime_nlp_seed(&request, params).unwrap();
        let options = super::CarMintimeSolveOptions {
            penalty_endpoint_c1_dn: 0.10,
            endpoint_c1_dn_scale: 0.005,
            penalty_endpoint_c1_heading: 0.10,
            endpoint_c1_heading_scale_rad: 0.005,
            penalty_endpoint_heading_jump: 0.10,
            endpoint_heading_jump_scale_rad: 0.005,
            penalty_endpoint_d2n_jump: 0.10,
            endpoint_d2n_jump_scale: 0.02,
            ..Default::default()
        };
        let problem =
            super::build_car_mintime_nlp_problem_with_options(seed, params, options).unwrap();
        let mut x = problem.seed.initial_guess.clone();

        for interval in [2_usize, 3, 4] {
            for point in 0..super::CAR_COLLOCATION_DEGREE {
                let offset = super::collocation_state_offset(&problem.seed, interval, point);
                x[offset + super::STATE_N_M] += 0.05 * (point as f64 + 1.0);
                x[offset + super::STATE_BETA_RAD] += 0.005 * (point as f64 + 1.0);
                x[offset + super::STATE_XI_RAD] -= 0.004 * (point as f64 + 1.0);
            }
        }

        let mut full_numeric_gradient = vec![f64::NAN; problem.decision_variable_count()];
        let mut structured_gradient = vec![f64::NAN; problem.decision_variable_count()];
        problem.objective_gradient_numeric(&x, &mut full_numeric_gradient);
        problem.objective_gradient_structured_numeric(&x, &mut structured_gradient);
        let max_gradient_delta = full_numeric_gradient
            .iter()
            .zip(&structured_gradient)
            .map(|(left, right)| (left - right).abs())
            .fold(0.0, f64::max);

        assert!(
            max_gradient_delta <= 1e-6,
            "endpoint C1 structured gradient drifted from full numeric gradient: {max_gradient_delta}"
        );
    }

    #[test]
    fn car_mintime_params_accept_app_power_kw_and_negative_python_brake() {
        let profile = crate::vehicle_dynamics::VehicleDynamicsProfileV1::from_json(
            &crate::json::parse_json_str(
                r#"{
                  "schema_version": "vehicle_dynamics_profile.v1",
                  "profile_id": "car_dynamics:gt3_track_car",
                  "model_family": "car_dynamics",
                  "preset_id": "gt3_track_car",
                  "solver_id": "old_car_mintime",
                  "parameters": {
                    "mass_kg": 1340,
                    "power_kw": 416,
                    "max_speed_mps": 82,
                    "f_brake_max_n": 12000
                  }
                }"#,
            )
            .unwrap(),
        )
        .unwrap();
        let params = CarDoubleTrackParams::from_profile(&profile).unwrap();

        assert_eq!(params.power_max_w, 416000.0);
        assert_eq!(params.brake_force_max_n, 12000.0);
    }

    #[test]
    #[ignore = "requires local Ipopt DLL and is a slow optimizer smoke test"]
    fn car_mintime_reaches_ipopt_for_kart_small_station_smoke() {
        let result = solve_car_mintime_json(
            r#"{
              "schema_version": "rust_solver_http_request.v1",
              "request_id": "kart-smoke",
              "project_id": "rice-manual",
              "station_count": 20,
              "solve_options": {
                "max_iter": 160,
                "tol": 0.001,
                "acceptable_tol": 0.01,
                "acceptable_iter": 2,
                "ipopt_print_level": 0
              },
              "track_area": {
                "schema_version": "TrackAreaContractV1",
                "track_id": "track-1",
                "units": "m",
                "left_boundary_xy_m": [[-2,-2], [-2,42], [42,42], [42,-2]],
                "right_boundary_xy_m": [[4,4], [4,36], [36,36], [36,4]],
                "trajectory_mode": "closed",
                "metadata": {}
              },
              "vehicle_dynamics_profile": {
                "schema_version": "vehicle_dynamics_profile.v1",
                "profile_id": "car_dynamics:kart_125cc",
                "model_family": "car_dynamics",
                "preset_id": "kart_125cc",
                "solver_id": "old_car_mintime",
                "parameters": {
                  "v_max_mps": 34,
                  "mass_kg": 165,
                  "wheelbase_front": 0.60,
                  "wheelbase_rear": 0.44,
                  "track_width_front_m": 1.05,
                  "track_width_rear_m": 1.20,
                  "cog_z": 0.32,
                  "mue": 1.35,
                  "I_z": 55.0,
                  "liftcoeff_front": 0.0,
                  "liftcoeff_rear": 0.0,
                  "k_brake_front": 0.05,
                  "k_drive_front": 0.0,
                  "k_roll": 0.48,
                  "t_delta": 0.08,
                  "t_drive": 0.08,
                  "t_brake": 0.06,
                  "power_max": 18000.0,
                  "f_drive_max": 2200.0,
                  "f_brake_max": 3200.0,
                  "delta_max": 1.20,
                  "c_roll": 0.015,
                  "f_z0": 405.0,
                  "B_front": 10.0,
                  "C_front": 2.2,
                  "eps_front": -0.10,
                  "E_front": 1.0,
                  "B_rear": 10.0,
                  "C_rear": 2.2,
                  "eps_rear": -0.10,
                  "E_rear": 1.0
                }
              }
            }"#,
        );

        match result {
            Ok(response) => {
                let value = parse_json_str(&response).unwrap();
                let lap_time = value
                    .get("lap_time_estimate_s")
                    .and_then(JsonValue::as_f64)
                    .unwrap();
                let trajectory = value.get("trajectory_result").unwrap();
                let s_m = trajectory.get("s_m").unwrap();

                assert!(lap_time.is_finite() && lap_time > 0.0);
                assert!(matches!(s_m, JsonValue::Array(values) if values.len() == 20));
            }
            Err(error) => {
                assert_eq!(error.code, "solve.nativeBackendUnavailable");
                assert!(
                    error.message.contains("Maximum_Iterations_Exceeded"),
                    "expected Ipopt to run and report iteration exhaustion, got {error:?}"
                );
            }
        }
    }

    #[test]
    #[ignore = "requires local Ipopt DLL and runs multiple open car mintime solves"]
    fn car_mintime_solves_open_car_fixture_smokes() {
        for track_id in [
            "open_straight_lab_v1",
            "open_s_bend_lab_v1",
            "open_chicane_lab_v1",
        ] {
            let (station_count, half_width_m, max_iter, center) =
                open_car_mintime_smoke_case(track_id)
                    .unwrap_or_else(|| panic!("unknown open car smoke fixture {track_id}"));
            let request = open_car_mintime_smoke_request_json(
                track_id,
                station_count,
                center,
                half_width_m,
                max_iter,
            );
            let response = solve_car_mintime_json(&request)
                .unwrap_or_else(|error| panic!("{track_id} car smoke failed: {error}"));
            assert_open_car_mintime_smoke_result(&response, station_count);
            println!("{track_id} open car mintime smoke passed");
        }
    }
}
