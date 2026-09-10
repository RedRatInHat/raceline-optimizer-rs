use crate::bike_dynamics_v1::{
    bike_countersteer_lean_dynamics_v1, BikeCountersteerLeanControlV1,
    BikeCountersteerLeanParamsV1, BikeCountersteerLeanStateV1,
};
use crate::mintime::{
    BIKE_COUNTERSTEER_LEAN_V1_CONTROL_COLUMNS, BIKE_COUNTERSTEER_LEAN_V1_STATE_COLUMNS,
};
use crate::mintime_common::{
    build_mintime_seed_bounds, collocation_defects, generic_mintime_constraint_rows,
    CollocationDegree3, CollocationIntervalInput, DecisionLayout, GenericConstraintRowOptions,
    GenericMintimeConstraintRow, MintimeDimensions, MintimeLayout, MintimeModelSpec,
};
use crate::vehicle_dynamics::VehicleDynamicsModelFamily;

pub const BIKE_V1_STATE_LEN: usize = 9;
pub const BIKE_V1_CONTROL_LEN: usize = 3;
pub const BIKE_V1_COLLOCATION_DEGREE: usize = 3;

pub const V1_STATE_V_MPS: usize = 0;
pub const V1_STATE_BETA_RAD: usize = 1;
pub const V1_STATE_OMEGA_Z_RADPS: usize = 2;
pub const V1_STATE_N_M: usize = 3;
pub const V1_STATE_XI_RAD: usize = 4;
pub const V1_STATE_PHI_RAD: usize = 5;
pub const V1_STATE_PHI_DOT_RADPS: usize = 6;
pub const V1_STATE_DELTA_RAD: usize = 7;
pub const V1_STATE_DELTA_DOT_RADPS: usize = 8;

pub const V1_CONTROL_STEERING_TORQUE_NM: usize = 0;
pub const V1_CONTROL_F_DRIVE_N: usize = 1;
pub const V1_CONTROL_F_BRAKE_N: usize = 2;

pub const BIKE_V1_COLLOCATION_TAU: [f64; BIKE_V1_COLLOCATION_DEGREE + 1] =
    CollocationDegree3::legendre().tau;

pub struct BikeV1Spec;

impl MintimeModelSpec for BikeV1Spec {
    const STATE_LEN: usize = BIKE_V1_STATE_LEN;
    const CONTROL_LEN: usize = BIKE_V1_CONTROL_LEN;

    type Params = BikeCountersteerLeanParamsV1;
    type State = BikeCountersteerLeanStateV1;
    type Control = BikeCountersteerLeanControlV1;

    fn model_id() -> &'static str {
        "bike_countersteer_lean_v1"
    }

    fn state_columns() -> &'static [&'static str] {
        BIKE_COUNTERSTEER_LEAN_V1_STATE_COLUMNS
    }

    fn control_columns() -> &'static [&'static str] {
        BIKE_COUNTERSTEER_LEAN_V1_CONTROL_COLUMNS
    }

    fn state_from_slice(values: &[f64]) -> Self::State {
        bike_v1_state_from_slice(values)
    }

    fn control_from_slice(values: &[f64]) -> Self::Control {
        bike_v1_control_from_slice(values)
    }

    fn initial_state(params: Self::Params) -> Vec<f64> {
        bike_v1_initial_state_guess(params).to_vec()
    }

    fn initial_control(_params: Self::Params) -> Vec<f64> {
        bike_v1_initial_control_guess().to_vec()
    }

    fn state_bounds(params: Self::Params, lower_n_m: f64, upper_n_m: f64) -> (Vec<f64>, Vec<f64>) {
        let (lower, upper) = bike_v1_state_bounds(params, lower_n_m, upper_n_m);
        (lower.to_vec(), upper.to_vec())
    }

    fn control_bounds(params: Self::Params) -> (Vec<f64>, Vec<f64>) {
        let (lower, upper) = bike_v1_control_bounds(params);
        (lower.to_vec(), upper.to_vec())
    }

    fn dynamics_s(
        params: Self::Params,
        state: Self::State,
        control: Self::Control,
        kappa_1pm: f64,
    ) -> Vec<f64> {
        bike_v1_dynamics_rhs_s(params, state, control, kappa_1pm).to_vec()
    }

    fn sigma_dt_ds(
        params: Self::Params,
        state: Self::State,
        control: Self::Control,
        kappa_1pm: f64,
    ) -> f64 {
        bike_countersteer_lean_dynamics_v1(params, state, control, kappa_1pm).sigma_dt_ds
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct BikeV1MintimeScaffold {
    pub layout: MintimeLayout,
    pub decision_layout: DecisionLayout,
    pub dimensions: MintimeDimensions,
    pub collocation_state_variable_count: usize,
    pub initial_state: [f64; BIKE_V1_STATE_LEN],
    pub state_lower_bounds: [f64; BIKE_V1_STATE_LEN],
    pub state_upper_bounds: [f64; BIKE_V1_STATE_LEN],
    pub initial_control: [f64; BIKE_V1_CONTROL_LEN],
    pub control_lower_bounds: [f64; BIKE_V1_CONTROL_LEN],
    pub control_upper_bounds: [f64; BIKE_V1_CONTROL_LEN],
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BikeV1IntervalCollocationInput {
    pub start_state: [f64; BIKE_V1_STATE_LEN],
    pub collocation_states: [[f64; BIKE_V1_STATE_LEN]; BIKE_V1_COLLOCATION_DEGREE],
    pub end_state: [f64; BIKE_V1_STATE_LEN],
    pub control: [f64; BIKE_V1_CONTROL_LEN],
    pub ds_m: f64,
    pub kappa_1pm: [f64; BIKE_V1_COLLOCATION_DEGREE],
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BikeV1IntervalCollocationDefects {
    pub dynamics: [[f64; BIKE_V1_STATE_LEN]; BIKE_V1_COLLOCATION_DEGREE],
    pub continuity: [f64; BIKE_V1_STATE_LEN],
}

#[must_use]
pub fn bike_v1_mintime_layout() -> MintimeLayout {
    BikeV1Spec::layout(VehicleDynamicsModelFamily::BikeDynamics)
}

#[must_use]
pub fn bike_v1_mintime_scaffold(
    station_count: usize,
    closed: bool,
    params: BikeCountersteerLeanParamsV1,
    lower_n_m: f64,
    upper_n_m: f64,
) -> BikeV1MintimeScaffold {
    let common = build_mintime_seed_bounds::<BikeV1Spec>(
        VehicleDynamicsModelFamily::BikeDynamics,
        station_count,
        closed,
        params,
        lower_n_m,
        upper_n_m,
    )
    .expect("Bike V1 scaffold spec must return dimensionally valid initial values and bounds");

    BikeV1MintimeScaffold {
        layout: common.layout,
        decision_layout: common.decision_layout,
        dimensions: common.dimensions,
        collocation_state_variable_count: common.dimensions.collocation_state_variable_count,
        initial_state: bike_v1_array_from_vec(common.station_initial_state),
        state_lower_bounds: bike_v1_array_from_vec(common.state_lower_bounds),
        state_upper_bounds: bike_v1_array_from_vec(common.state_upper_bounds),
        initial_control: bike_v1_control_array_from_vec(common.initial_control),
        control_lower_bounds: bike_v1_control_array_from_vec(common.control_lower_bounds),
        control_upper_bounds: bike_v1_control_array_from_vec(common.control_upper_bounds),
    }
}

#[must_use]
pub fn bike_v1_constraint_rows(
    scaffold: &BikeV1MintimeScaffold,
) -> Vec<GenericMintimeConstraintRow> {
    generic_mintime_constraint_rows(
        scaffold.decision_layout,
        GenericConstraintRowOptions::with_control_rate(),
    )
}

#[must_use]
pub fn bike_v1_initial_state_guess(
    params: BikeCountersteerLeanParamsV1,
) -> [f64; BIKE_V1_STATE_LEN] {
    let speed = 20.0_f64.clamp(1.0, params.base.max_speed_mps);
    [speed, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
}

#[must_use]
pub fn bike_v1_initial_control_guess() -> [f64; BIKE_V1_CONTROL_LEN] {
    [0.0; BIKE_V1_CONTROL_LEN]
}

#[must_use]
pub fn bike_v1_state_bounds(
    params: BikeCountersteerLeanParamsV1,
    lower_n_m: f64,
    upper_n_m: f64,
) -> ([f64; BIKE_V1_STATE_LEN], [f64; BIKE_V1_STATE_LEN]) {
    let delta_dot_max =
        (params.base.steering_angle_max_rad / params.base.steering_response_s.max(0.02)).max(1.0);
    (
        [
            1.0,
            -params.base.beta_max_rad,
            -std::f64::consts::FRAC_PI_2,
            lower_n_m,
            -params.base.xi_max_rad,
            -params.base.lean_angle_max_rad,
            -params.base.lean_rate_max_radps,
            -params.base.steering_angle_max_rad,
            -delta_dot_max,
        ],
        [
            params.base.max_speed_mps,
            params.base.beta_max_rad,
            std::f64::consts::FRAC_PI_2,
            upper_n_m,
            params.base.xi_max_rad,
            params.base.lean_angle_max_rad,
            params.base.lean_rate_max_radps,
            params.base.steering_angle_max_rad,
            delta_dot_max,
        ],
    )
}

#[must_use]
pub fn bike_v1_control_bounds(
    params: BikeCountersteerLeanParamsV1,
) -> ([f64; BIKE_V1_CONTROL_LEN], [f64; BIKE_V1_CONTROL_LEN]) {
    (
        [
            -params.steering_torque_max_nm,
            0.0,
            -params.base.brake_force_max_n,
        ],
        [
            params.steering_torque_max_nm,
            params.base.drive_force_max_n,
            0.0,
        ],
    )
}

#[must_use]
pub fn bike_v1_state_from_slice(x: &[f64]) -> BikeCountersteerLeanStateV1 {
    BikeCountersteerLeanStateV1 {
        v_mps: x[V1_STATE_V_MPS],
        beta_rad: x[V1_STATE_BETA_RAD],
        omega_z_radps: x[V1_STATE_OMEGA_Z_RADPS],
        n_m: x[V1_STATE_N_M],
        xi_rad: x[V1_STATE_XI_RAD],
        phi_rad: x[V1_STATE_PHI_RAD],
        phi_dot_radps: x[V1_STATE_PHI_DOT_RADPS],
        delta_rad: x[V1_STATE_DELTA_RAD],
        delta_dot_radps: x[V1_STATE_DELTA_DOT_RADPS],
    }
}

#[must_use]
pub fn bike_v1_control_from_slice(x: &[f64]) -> BikeCountersteerLeanControlV1 {
    BikeCountersteerLeanControlV1 {
        steering_torque_nm: x[V1_CONTROL_STEERING_TORQUE_NM],
        f_drive_n: x[V1_CONTROL_F_DRIVE_N],
        f_brake_n: x[V1_CONTROL_F_BRAKE_N],
    }
}

#[must_use]
pub fn bike_v1_dynamics_rhs_s(
    params: BikeCountersteerLeanParamsV1,
    state: BikeCountersteerLeanStateV1,
    control: BikeCountersteerLeanControlV1,
    kappa_1pm: f64,
) -> [f64; BIKE_V1_STATE_LEN] {
    let dynamics = bike_countersteer_lean_dynamics_v1(params, state, control, kappa_1pm);
    [
        dynamics.dv_ds,
        dynamics.dbeta_ds,
        dynamics.domega_z_ds,
        dynamics.dn_ds,
        dynamics.dxi_ds,
        dynamics.dphi_ds,
        dynamics.dphi_dot_ds,
        dynamics.ddelta_ds,
        dynamics.ddelta_dot_ds,
    ]
}

#[must_use]
pub fn bike_v1_interval_collocation_defects(
    params: BikeCountersteerLeanParamsV1,
    input: BikeV1IntervalCollocationInput,
) -> BikeV1IntervalCollocationDefects {
    let generic = collocation_defects::<BikeV1Spec>(
        params,
        &CollocationIntervalInput {
            start_state: input.start_state.to_vec(),
            collocation_states: input
                .collocation_states
                .iter()
                .map(|state| state.to_vec())
                .collect(),
            end_state: input.end_state.to_vec(),
            control: input.control.to_vec(),
            ds_m: input.ds_m,
            kappa_1pm: input.kappa_1pm.to_vec(),
        },
    )
    .expect("typed Bike V1 collocation input must satisfy generic layout");

    BikeV1IntervalCollocationDefects {
        dynamics: bike_v1_defect_rows_from_vec(generic.dynamics),
        continuity: bike_v1_defect_row_from_vec(generic.continuity),
    }
}

fn bike_v1_defect_rows_from_vec(
    rows: Vec<Vec<f64>>,
) -> [[f64; BIKE_V1_STATE_LEN]; BIKE_V1_COLLOCATION_DEGREE] {
    assert_eq!(rows.len(), BIKE_V1_COLLOCATION_DEGREE);
    let mut result = [[0.0; BIKE_V1_STATE_LEN]; BIKE_V1_COLLOCATION_DEGREE];
    for (row_index, row) in rows.into_iter().enumerate() {
        result[row_index] = bike_v1_defect_row_from_vec(row);
    }
    result
}

fn bike_v1_defect_row_from_vec(row: Vec<f64>) -> [f64; BIKE_V1_STATE_LEN] {
    assert_eq!(row.len(), BIKE_V1_STATE_LEN);
    let mut result = [0.0; BIKE_V1_STATE_LEN];
    result.copy_from_slice(&row);
    result
}

fn bike_v1_array_from_vec(values: Vec<f64>) -> [f64; BIKE_V1_STATE_LEN] {
    assert_eq!(values.len(), BIKE_V1_STATE_LEN);
    let mut result = [0.0; BIKE_V1_STATE_LEN];
    result.copy_from_slice(&values);
    result
}

fn bike_v1_control_array_from_vec(values: Vec<f64>) -> [f64; BIKE_V1_CONTROL_LEN] {
    assert_eq!(values.len(), BIKE_V1_CONTROL_LEN);
    let mut result = [0.0; BIKE_V1_CONTROL_LEN];
    result.copy_from_slice(&values);
    result
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bike_dynamics_v1::BikeCountersteerLeanParamsV1;
    use crate::mintime_common::{
        generic_mintime_constraint_values, generic_mintime_lap_time_objective_s,
        generic_mintime_objective_gradient_numeric, generic_mintime_sparse_jacobian_numeric,
        generic_mintime_sparse_pattern, NumericSparseJacobianEntry,
    };
    use crate::vehicle_dynamics::{
        BikeSingleTrackLeanParams, VehicleDynamicsModelFamily, VehicleDynamicsProfileV1, G_MPS2,
    };

    fn test_params() -> BikeCountersteerLeanParamsV1 {
        let profile = VehicleDynamicsProfileV1 {
            schema_version: VehicleDynamicsProfileV1::SCHEMA_VERSION.to_owned(),
            profile_id: "bike_dynamics:v1_scaffold_test".to_owned(),
            model_family: VehicleDynamicsModelFamily::BikeDynamics,
            preset_id: None,
            solver_id: None,
            parameters: vec![
                ("mass".to_owned(), 200.0.into()),
                ("g".to_owned(), G_MPS2.into()),
                ("wheelbase".to_owned(), 1.4.into()),
                ("front_weight_bias".to_owned(), 0.5.into()),
                ("cog_z".to_owned(), 0.6.into()),
                ("I_z".to_owned(), 80.0.into()),
                ("mue".to_owned(), 1.2.into()),
                ("dragcoeff".to_owned(), 0.0.into()),
                ("c_roll".to_owned(), 0.0.into()),
                ("delta_max".to_owned(), 0.8.into()),
                ("phi_max".to_owned(), 1.1.into()),
                ("phi_dot_max".to_owned(), 2.8.into()),
                ("t_delta".to_owned(), 0.1.into()),
                ("strict_product_gates".to_owned(), false.into()),
            ],
            native_parameters: Vec::new(),
            metadata: Vec::new(),
        };
        BikeCountersteerLeanParamsV1::from_v05(
            BikeSingleTrackLeanParams::from_profile(&profile).unwrap(),
        )
    }

    #[test]
    fn bike_v1_layout_uses_countersteer_state_and_control_columns() {
        let layout = bike_v1_mintime_layout();

        assert_eq!(BikeV1Spec::model_id(), "bike_countersteer_lean_v1");
        assert_eq!(layout.state_columns.len(), BIKE_V1_STATE_LEN);
        assert_eq!(layout.control_columns.len(), BIKE_V1_CONTROL_LEN);
        assert_eq!(layout.state_columns[V1_STATE_DELTA_RAD], "delta_rad");
        assert_eq!(
            layout.control_columns[V1_CONTROL_STEERING_TORQUE_NM],
            "steering_torque_Nm"
        );
    }

    #[test]
    fn bike_v1_scaffold_dimensions_bounds_and_initial_guess_are_consistent() {
        let params = test_params();
        let scaffold = bike_v1_mintime_scaffold(40, true, params, -1.2, 1.4);

        assert_eq!(scaffold.dimensions.station_count, 40);
        assert_eq!(scaffold.dimensions.interval_count, 40);
        assert_eq!(scaffold.decision_layout.state_len, BIKE_V1_STATE_LEN);
        assert_eq!(scaffold.decision_layout.control_len, BIKE_V1_CONTROL_LEN);
        assert_eq!(
            scaffold.decision_layout.collocation_degree,
            BIKE_V1_COLLOCATION_DEGREE
        );
        assert_eq!(
            scaffold.dimensions.state_variable_count,
            40 * BIKE_V1_STATE_LEN
        );
        assert_eq!(
            scaffold.dimensions.control_variable_count,
            40 * BIKE_V1_CONTROL_LEN
        );
        assert_eq!(
            scaffold.dimensions.collocation_state_variable_count,
            40 * BIKE_V1_COLLOCATION_DEGREE * BIKE_V1_STATE_LEN
        );
        assert_eq!(scaffold.initial_state[V1_STATE_V_MPS], 20.0);
        assert_eq!(scaffold.initial_state[V1_STATE_DELTA_RAD], 0.0);
        assert_eq!(scaffold.initial_control[V1_CONTROL_STEERING_TORQUE_NM], 0.0);
        assert_eq!(scaffold.state_lower_bounds[V1_STATE_N_M], -1.2);
        assert_eq!(scaffold.state_upper_bounds[V1_STATE_N_M], 1.4);
        assert!(scaffold.state_lower_bounds[V1_STATE_DELTA_DOT_RADPS] < 0.0);
        assert!(scaffold.state_upper_bounds[V1_STATE_DELTA_DOT_RADPS] > 0.0);
        assert!(scaffold.control_lower_bounds[V1_CONTROL_STEERING_TORQUE_NM] < 0.0);
        assert!(scaffold.control_upper_bounds[V1_CONTROL_STEERING_TORQUE_NM] > 0.0);
    }

    #[test]
    fn bike_v1_common_seed_bounds_match_scaffold_templates() {
        let params = test_params();
        let scaffold = bike_v1_mintime_scaffold(8, true, params, -1.5, 2.0);
        let common = build_mintime_seed_bounds::<BikeV1Spec>(
            VehicleDynamicsModelFamily::BikeDynamics,
            8,
            true,
            params,
            -1.5,
            2.0,
        )
        .expect("Bike V1 common seed should build");

        assert_eq!(common.layout, scaffold.layout);
        assert_eq!(common.decision_layout, scaffold.decision_layout);
        assert_eq!(common.dimensions, scaffold.dimensions);
        assert_eq!(
            common.station_initial_state,
            scaffold.initial_state.to_vec()
        );
        assert_eq!(common.initial_control, scaffold.initial_control.to_vec());
        assert_eq!(
            common.state_lower_bounds,
            scaffold.state_lower_bounds.to_vec()
        );
        assert_eq!(
            common.state_upper_bounds,
            scaffold.state_upper_bounds.to_vec()
        );
        assert_eq!(
            common.control_lower_bounds,
            scaffold.control_lower_bounds.to_vec()
        );
        assert_eq!(
            common.control_upper_bounds,
            scaffold.control_upper_bounds.to_vec()
        );
        assert_eq!(
            common.initial_decision.len(),
            scaffold.dimensions.decision_variable_count()
        );

        let control_offset = scaffold.decision_layout.control_offset(0);
        assert_eq!(
            &common.initial_decision[control_offset..control_offset + BIKE_V1_CONTROL_LEN],
            scaffold.initial_control.as_slice()
        );
        let colloc_offset = scaffold.decision_layout.collocation_state_offset(0, 0);
        assert_eq!(
            &common.initial_decision[colloc_offset..colloc_offset + BIKE_V1_STATE_LEN],
            scaffold.initial_state.as_slice()
        );
    }

    #[test]
    fn bike_v1_can_build_generic_constraint_rows() {
        let params = test_params();
        let scaffold = bike_v1_mintime_scaffold(5, true, params, -1.0, 1.0);
        let rows = bike_v1_constraint_rows(&scaffold);

        let expected_continuity = 5 * BIKE_V1_STATE_LEN;
        let expected_dynamics = 5 * BIKE_V1_COLLOCATION_DEGREE * BIKE_V1_STATE_LEN;
        let expected_control_rate = 5 * BIKE_V1_CONTROL_LEN;
        assert_eq!(
            rows.len(),
            expected_continuity + expected_dynamics + expected_control_rate
        );
        assert_eq!(
            rows.first().copied(),
            Some(GenericMintimeConstraintRow::Continuity {
                interval: 0,
                state_index: 0,
            })
        );
        assert!(
            rows.contains(&GenericMintimeConstraintRow::CollocationDynamics {
                interval: 4,
                point: BIKE_V1_COLLOCATION_DEGREE - 1,
                state_index: BIKE_V1_STATE_LEN - 1,
            })
        );
        assert_eq!(
            rows.last().copied(),
            Some(GenericMintimeConstraintRow::ControlRate {
                interval: 4,
                control_index: BIKE_V1_CONTROL_LEN - 1,
            })
        );
    }

    #[test]
    fn bike_v1_generic_constraint_values_are_zero_for_straight_seed() {
        let params = test_params();
        let common = build_mintime_seed_bounds::<BikeV1Spec>(
            VehicleDynamicsModelFamily::BikeDynamics,
            3,
            true,
            params,
            -1.0,
            1.0,
        )
        .expect("Bike V1 common seed should build");
        let rows = generic_mintime_constraint_rows(
            common.decision_layout,
            GenericConstraintRowOptions::with_control_rate(),
        );
        let values = generic_mintime_constraint_values::<BikeV1Spec>(
            params,
            common.decision_layout,
            &common.initial_decision,
            &rows,
            &[4.0; 3],
            &vec![vec![0.0; BIKE_V1_COLLOCATION_DEGREE]; 3],
        )
        .expect("straight seed constraints should evaluate");

        assert_eq!(values.len(), rows.len());
        for (index, value) in values.iter().enumerate() {
            assert!(value.abs() < 1.0e-9, "row={index} value={value}");
        }
    }

    #[test]
    fn bike_v1_generic_sparse_jacobian_contains_expected_continuity_entry() {
        let params = test_params();
        let common = build_mintime_seed_bounds::<BikeV1Spec>(
            VehicleDynamicsModelFamily::BikeDynamics,
            3,
            true,
            params,
            -1.0,
            1.0,
        )
        .expect("Bike V1 common seed should build");
        let rows = bike_v1_constraint_rows(&bike_v1_mintime_scaffold(3, true, params, -1.0, 1.0));
        let pattern = generic_mintime_sparse_pattern(common.decision_layout, &rows);
        let continuity_next_v = (0, common.decision_layout.state_offset(1) + V1_STATE_V_MPS);
        assert!(pattern.contains(&continuity_next_v));

        let entries = generic_mintime_sparse_jacobian_numeric::<BikeV1Spec>(
            params,
            common.decision_layout,
            &common.initial_decision,
            &rows,
            &[4.0; 3],
            &vec![vec![0.0; BIKE_V1_COLLOCATION_DEGREE]; 3],
            1.0e-6,
        )
        .expect("Bike V1 sparse numeric Jacobian should evaluate");

        assert_eq!(entries.len(), pattern.len());
        let entry = entries
            .iter()
            .find(|entry| entry.row == continuity_next_v.0 && entry.variable == continuity_next_v.1)
            .copied()
            .unwrap_or(NumericSparseJacobianEntry {
                row: usize::MAX,
                variable: usize::MAX,
                value: f64::NAN,
            });
        assert!((entry.value + 1.0).abs() < 1.0e-7, "{entry:?}");
        assert!(entries.iter().all(|entry| entry.value.is_finite()));
    }

    #[test]
    fn bike_v1_generic_lap_time_objective_matches_straight_sigma_sum() {
        let params = test_params();
        let common = build_mintime_seed_bounds::<BikeV1Spec>(
            VehicleDynamicsModelFamily::BikeDynamics,
            3,
            true,
            params,
            -1.0,
            1.0,
        )
        .expect("Bike V1 common seed should build");
        let objective = generic_mintime_lap_time_objective_s::<BikeV1Spec>(
            params,
            common.decision_layout,
            &common.initial_decision,
            &[4.0; 3],
            &[0.0; 3],
        )
        .expect("Bike V1 generic lap objective should evaluate");

        assert!((objective - 0.6).abs() < 1.0e-12, "{objective}");
    }

    #[test]
    fn bike_v1_generic_objective_gradient_matches_numeric_speed_sensitivity() {
        let params = test_params();
        let common = build_mintime_seed_bounds::<BikeV1Spec>(
            VehicleDynamicsModelFamily::BikeDynamics,
            2,
            true,
            params,
            -1.0,
            1.0,
        )
        .expect("Bike V1 common seed should build");
        let gradient = generic_mintime_objective_gradient_numeric::<BikeV1Spec>(
            params,
            common.decision_layout,
            &common.initial_decision,
            &[4.0; 2],
            &[0.0; 2],
            1.0e-6,
        )
        .expect("Bike V1 generic objective gradient should evaluate");

        let station_0_v = common.decision_layout.state_offset(0) + V1_STATE_V_MPS;
        let station_1_v = common.decision_layout.state_offset(1) + V1_STATE_V_MPS;
        assert!((gradient[station_0_v] + 0.01).abs() < 1.0e-7);
        assert!((gradient[station_1_v] + 0.01).abs() < 1.0e-7);
        assert!(gradient.iter().all(|value| value.is_finite()));
    }

    #[test]
    fn bike_v1_spec_reproduces_scaffold_initial_values_bounds_and_dynamics() {
        let params = test_params();
        let spec_initial = BikeV1Spec::initial_state(params);
        let direct_initial = bike_v1_initial_state_guess(params);
        assert_eq!(spec_initial, direct_initial.to_vec());

        let spec_control = BikeV1Spec::initial_control(params);
        let direct_control = bike_v1_initial_control_guess();
        assert_eq!(spec_control, direct_control.to_vec());

        let (spec_state_lower, spec_state_upper) = BikeV1Spec::state_bounds(params, -1.0, 1.2);
        let (direct_state_lower, direct_state_upper) = bike_v1_state_bounds(params, -1.0, 1.2);
        assert_eq!(spec_state_lower, direct_state_lower.to_vec());
        assert_eq!(spec_state_upper, direct_state_upper.to_vec());

        let (spec_control_lower, spec_control_upper) = BikeV1Spec::control_bounds(params);
        let (direct_control_lower, direct_control_upper) = bike_v1_control_bounds(params);
        assert_eq!(spec_control_lower, direct_control_lower.to_vec());
        assert_eq!(spec_control_upper, direct_control_upper.to_vec());

        let state = BikeV1Spec::state_from_slice(&direct_initial);
        let control = BikeV1Spec::control_from_slice(&direct_control);
        assert_eq!(
            BikeV1Spec::dynamics_s(params, state, control, 0.0),
            bike_v1_dynamics_rhs_s(params, state, control, 0.0).to_vec()
        );
    }

    #[test]
    fn bike_v1_slice_converters_match_column_order() {
        let state = [12.0, 0.1, 0.2, -0.3, 0.4, 0.5, -0.6, 0.7, -0.8];
        let control = [4.0, 120.0, -300.0];

        let state = bike_v1_state_from_slice(&state);
        let control = bike_v1_control_from_slice(&control);

        assert_eq!(state.v_mps, 12.0);
        assert_eq!(state.delta_rad, 0.7);
        assert_eq!(state.delta_dot_radps, -0.8);
        assert_eq!(control.steering_torque_nm, 4.0);
        assert_eq!(control.f_drive_n, 120.0);
        assert_eq!(control.f_brake_n, -300.0);
    }

    #[test]
    fn bike_v1_zero_straight_constant_interval_has_zero_collocation_defects() {
        let params = test_params();
        let state = bike_v1_initial_state_guess(params);
        let control = bike_v1_initial_control_guess();
        let defects = bike_v1_interval_collocation_defects(
            params,
            BikeV1IntervalCollocationInput {
                start_state: state,
                collocation_states: [state; BIKE_V1_COLLOCATION_DEGREE],
                end_state: state,
                control,
                ds_m: 4.0,
                kappa_1pm: [0.0; BIKE_V1_COLLOCATION_DEGREE],
            },
        );

        for point in 0..BIKE_V1_COLLOCATION_DEGREE {
            for state_index in 0..BIKE_V1_STATE_LEN {
                assert!(
                    defects.dynamics[point][state_index].abs() < 1e-9,
                    "point={point} state={state_index} defect={}",
                    defects.dynamics[point][state_index]
                );
            }
        }
        for state_index in 0..BIKE_V1_STATE_LEN {
            assert!(
                defects.continuity[state_index].abs() < 1e-9,
                "state={state_index} continuity={}",
                defects.continuity[state_index]
            );
        }
    }

    #[test]
    fn bike_v1_steering_torque_enters_delta_dot_collocation_defect() {
        let params = test_params();
        let state = bike_v1_initial_state_guess(params);
        let mut control = bike_v1_initial_control_guess();
        control[V1_CONTROL_STEERING_TORQUE_NM] = 5.0;
        let defects = bike_v1_interval_collocation_defects(
            params,
            BikeV1IntervalCollocationInput {
                start_state: state,
                collocation_states: [state; BIKE_V1_COLLOCATION_DEGREE],
                end_state: state,
                control,
                ds_m: 4.0,
                kappa_1pm: [0.0; BIKE_V1_COLLOCATION_DEGREE],
            },
        );

        assert!(defects.dynamics[0][V1_STATE_DELTA_DOT_RADPS] < 0.0);
        assert!(defects.dynamics[1][V1_STATE_DELTA_DOT_RADPS] < 0.0);
        assert!(defects.dynamics[2][V1_STATE_DELTA_DOT_RADPS] < 0.0);
    }

    #[test]
    fn bike_v1_generic_collocation_defects_match_typed_wrapper() {
        let params = test_params();
        let mut start_state = bike_v1_initial_state_guess(params);
        start_state[V1_STATE_DELTA_RAD] = 0.03;
        let mut collocation_states = [start_state; BIKE_V1_COLLOCATION_DEGREE];
        collocation_states[0][V1_STATE_DELTA_DOT_RADPS] = 0.01;
        collocation_states[1][V1_STATE_PHI_RAD] = 0.02;
        collocation_states[2][V1_STATE_BETA_RAD] = -0.01;
        let mut end_state = start_state;
        end_state[V1_STATE_DELTA_RAD] = 0.04;
        let control = [2.0, 10.0, -5.0];
        let kappa_1pm = [0.01, 0.015, 0.02];

        let typed = bike_v1_interval_collocation_defects(
            params,
            BikeV1IntervalCollocationInput {
                start_state,
                collocation_states,
                end_state,
                control,
                ds_m: 3.0,
                kappa_1pm,
            },
        );
        let generic = collocation_defects::<BikeV1Spec>(
            params,
            &CollocationIntervalInput {
                start_state: start_state.to_vec(),
                collocation_states: collocation_states
                    .iter()
                    .map(|state| state.to_vec())
                    .collect(),
                end_state: end_state.to_vec(),
                control: control.to_vec(),
                ds_m: 3.0,
                kappa_1pm: kappa_1pm.to_vec(),
            },
        )
        .expect("generic Bike V1 defects should evaluate");

        assert_eq!(generic.dynamics, typed.dynamics.map(|row| row.to_vec()));
        assert_eq!(generic.continuity, typed.continuity.to_vec());
    }

    #[test]
    fn bike_v1_generic_collocation_defects_reject_wrong_lengths() {
        let params = test_params();
        let state = bike_v1_initial_state_guess(params);
        let control = bike_v1_initial_control_guess();

        let err = collocation_defects::<BikeV1Spec>(
            params,
            &CollocationIntervalInput {
                start_state: state[..BIKE_V1_STATE_LEN - 1].to_vec(),
                collocation_states: vec![state.to_vec(); BIKE_V1_COLLOCATION_DEGREE],
                end_state: state.to_vec(),
                control: control.to_vec(),
                ds_m: 1.0,
                kappa_1pm: vec![0.0; BIKE_V1_COLLOCATION_DEGREE],
            },
        )
        .expect_err("generic defects should reject malformed state rows");

        assert!(err.contains("start_state length"));
        assert!(err.contains(BikeV1Spec::model_id()));
    }
}
