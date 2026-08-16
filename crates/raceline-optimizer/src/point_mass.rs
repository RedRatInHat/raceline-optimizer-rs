use std::ffi::c_void;
use std::path::{Path, PathBuf};
use std::ptr;

use crate::contracts::{
    AccelerationEnvelopeV1, Point2, PointMassProfileV1, SectionsTrackViewV1,
    TrajectoryResultSeriesV1,
};

const POINT_MASS_OUTPUT_SAMPLES_PER_STATION: usize = 5;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum TrackTopology {
    Closed,
    Open,
}

impl TrackTopology {
    fn from_sections(view: &SectionsTrackViewV1) -> Self {
        if metadata_str(&view.metadata, "trajectory_mode") == Some("open") {
            Self::Open
        } else {
            Self::Closed
        }
    }

    fn is_closed(self) -> bool {
        self == Self::Closed
    }
}

#[derive(Clone, Debug)]
pub struct PointMassSolveOptions {
    pub n_second_diff_weight: f64,
    pub velocity_second_diff_weight: f64,
    pub control_slew_weight: f64,
    pub g_mps2: f64,
    pub envelope_safety: f64,
    pub smooth_abs_eps: f64,
    pub accel_component_bound_mps2: Option<f64>,
    pub min_segment_time_s: f64,
    pub max_segment_time_s: f64,
    pub max_iter: i32,
    pub tol: f64,
    pub acceptable_tol: f64,
    pub acceptable_iter: i32,
    pub ipopt_print_level: i32,
    pub ipopt_linear_solver: Option<String>,
    pub ipopt_dll_path: Option<PathBuf>,
    pub envelope_check_points: EnvelopeCheckPoints,
    pub publish_geometry_mode: PublishGeometryMode,
    pub output_sample_count: Option<usize>,
    pub width_opt_m: f64,
}

impl Default for PointMassSolveOptions {
    fn default() -> Self {
        Self {
            n_second_diff_weight: 0.003,
            velocity_second_diff_weight: 0.0,
            control_slew_weight: 0.02,
            g_mps2: 9.81,
            envelope_safety: 0.999,
            smooth_abs_eps: 1e-6,
            accel_component_bound_mps2: None,
            min_segment_time_s: 1e-3,
            max_segment_time_s: 10.0,
            max_iter: 700,
            tol: 1e-5,
            acceptable_tol: 1e-5,
            acceptable_iter: 5,
            ipopt_print_level: 0,
            ipopt_linear_solver: default_ipopt_linear_solver(),
            ipopt_dll_path: None,
            envelope_check_points: EnvelopeCheckPoints::StartMidEnd,
            publish_geometry_mode: PublishGeometryMode::SectionInterpolated,
            output_sample_count: None,
            width_opt_m: 0.0,
        }
    }
}

fn default_ipopt_linear_solver() -> Option<String> {
    None
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EnvelopeCheckPoints {
    Midpoint,
    SegmentAverage,
    StartMidEnd,
}

impl EnvelopeCheckPoints {
    pub fn parse(value: &str) -> Result<Self, String> {
        match value {
            "midpoint" => Ok(Self::Midpoint),
            "segment_average" => Ok(Self::SegmentAverage),
            "start_mid_end" => Ok(Self::StartMidEnd),
            _ => Err(format!(
                "envelope_check_points must be one of midpoint, segment_average, start_mid_end, got {value:?}"
            )),
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PublishGeometryMode {
    SectionInterpolated,
    DenseDynamics,
    StationPolyline,
}

impl PublishGeometryMode {
    pub fn parse(value: &str) -> Result<Self, String> {
        match value {
            "section_interpolated" => Ok(Self::SectionInterpolated),
            "dense_dynamics" => Ok(Self::DenseDynamics),
            "station_polyline" => Ok(Self::StationPolyline),
            _ => Err(format!(
                "publish_geometry_mode must be one of section_interpolated, dense_dynamics, station_polyline, got {value:?}"
            )),
        }
    }
}

#[derive(Clone, Debug)]
pub struct PointMassSolveResult {
    pub series: TrajectoryResultSeriesV1,
    pub lap_time_s: f64,
    pub status: String,
    pub objective_value: f64,
}

#[derive(Clone, Debug)]
pub struct PointMassIterationPreview {
    pub iteration: Option<u32>,
    pub series: TrajectoryResultSeriesV1,
    pub lap_time_s: f64,
    pub objective_value: f64,
    pub max_envelope_utilization: f64,
}

#[derive(Clone, Debug)]
pub enum PointMassProgressUpdate {
    OptimizerIteration {
        iteration: u32,
        objective_value: f64,
    },
    Preview(Box<PointMassIterationPreview>),
}

pub type PointMassSolveProgressCallback<'a> = &'a mut dyn FnMut(PointMassProgressUpdate);

#[derive(Clone, Copy, Debug)]
enum ConstraintRow {
    DynPosX(usize),
    DynPosY(usize),
    DynVelX(usize),
    DynVelY(usize),
    SpeedSq(usize),
    Env {
        station: usize,
        sample: EnvelopeSample,
        side: EnvelopeSide,
    },
}

#[allow(dead_code)]
#[derive(Clone, Copy, Debug)]
enum EnvelopeSample {
    Start,
    Mid,
    End,
}

#[derive(Clone, Copy, Debug)]
enum EnvelopeSide {
    Drive,
    Brake,
}

struct PointMassNlp<'a> {
    count: usize,
    interval_count: usize,
    topology: TrackTopology,
    center: Vec<Point2>,
    normals: Vec<Point2>,
    lower_n: Vec<f64>,
    upper_n: Vec<f64>,
    vehicle_v_max_mps: f64,
    envelope: AccelerationEnvelopeV1,
    options: PointMassSolveOptions,
    rows: Vec<ConstraintRow>,
    jac_pattern: Vec<(i32, i32)>,
    progress_callback: Option<PointMassSolveProgressCallback<'a>>,
    objective_eval_count: u32,
    last_preview_eval_count: u32,
    last_ipopt_iteration: Option<u32>,
}

impl<'a> PointMassNlp<'a> {
    fn new(
        view: &SectionsTrackViewV1,
        profile: &PointMassProfileV1,
        envelope: &AccelerationEnvelopeV1,
        options: PointMassSolveOptions,
    ) -> Result<Self, String> {
        let count = view.centerline_xy_m.len();
        if count < 3 {
            return Err("point mass OCP requires at least three stations".to_owned());
        }
        if view.normals_xy.len() != count
            || view.width_left_m.len() != count
            || view.width_right_m.len() != count
        {
            return Err("sections track view arrays have mismatched lengths".to_owned());
        }
        let topology = TrackTopology::from_sections(view);
        let interval_count = match topology {
            TrackTopology::Closed => count,
            TrackTopology::Open => count.saturating_sub(1),
        };
        let vehicle_v_max_mps = profile_param(profile, "v_max_mps")
            .unwrap_or_else(|| envelope.speed_mps.iter().copied().fold(0.0_f64, f64::max));
        let mut rows = Vec::with_capacity(count * 11);
        for index in 0..interval_count {
            rows.push(ConstraintRow::DynPosX(index));
            rows.push(ConstraintRow::DynPosY(index));
            rows.push(ConstraintRow::DynVelX(index));
            rows.push(ConstraintRow::DynVelY(index));
            match options.envelope_check_points {
                EnvelopeCheckPoints::Midpoint | EnvelopeCheckPoints::SegmentAverage => {
                    rows.push(ConstraintRow::Env {
                        station: index,
                        sample: EnvelopeSample::Mid,
                        side: EnvelopeSide::Drive,
                    });
                    rows.push(ConstraintRow::Env {
                        station: index,
                        sample: EnvelopeSample::Mid,
                        side: EnvelopeSide::Brake,
                    });
                }
                EnvelopeCheckPoints::StartMidEnd => {
                    for sample in [
                        EnvelopeSample::Start,
                        EnvelopeSample::Mid,
                        EnvelopeSample::End,
                    ] {
                        rows.push(ConstraintRow::Env {
                            station: index,
                            sample,
                            side: EnvelopeSide::Drive,
                        });
                        rows.push(ConstraintRow::Env {
                            station: index,
                            sample,
                            side: EnvelopeSide::Brake,
                        });
                    }
                }
            }
        }
        for index in 0..count {
            rows.push(ConstraintRow::SpeedSq(index));
        }

        let mut problem = Self {
            count,
            interval_count,
            topology,
            center: view.centerline_xy_m.clone(),
            normals: view.normals_xy.clone(),
            lower_n: view
                .width_left_m
                .iter()
                .zip(&view.width_right_m)
                .map(|(left, right)| point_mass_n_bounds_m(*left, *right, options.width_opt_m).0)
                .collect(),
            upper_n: view
                .width_left_m
                .iter()
                .zip(&view.width_right_m)
                .map(|(left, right)| point_mass_n_bounds_m(*left, *right, options.width_opt_m).1)
                .collect(),
            vehicle_v_max_mps,
            envelope: envelope.clone(),
            options,
            rows,
            jac_pattern: Vec::new(),
            progress_callback: None,
            objective_eval_count: 0,
            last_preview_eval_count: 0,
            last_ipopt_iteration: None,
        };
        problem.jac_pattern = problem.build_jacobian_pattern()?;
        Ok(problem)
    }

    fn set_progress_callback(&mut self, callback: PointMassSolveProgressCallback<'a>) {
        self.progress_callback = Some(callback);
    }

    fn variable_count(&self) -> usize {
        self.count * 6
    }

    fn next_station(&self, index: usize) -> usize {
        match self.topology {
            TrackTopology::Closed => (index + 1) % self.count,
            TrackTopology::Open => index + 1,
        }
    }

    fn previous_station(&self, index: usize) -> Option<usize> {
        match self.topology {
            TrackTopology::Closed => Some((index + self.count - 1) % self.count),
            TrackTopology::Open => index.checked_sub(1),
        }
    }

    fn next_interval(&self, index: usize) -> Option<usize> {
        match self.topology {
            TrackTopology::Closed => Some((index + 1) % self.interval_count),
            TrackTopology::Open => (index + 1 < self.interval_count).then_some(index + 1),
        }
    }

    fn second_difference_station_indices(&self) -> Vec<usize> {
        match self.topology {
            TrackTopology::Closed => (0..self.count).collect(),
            TrackTopology::Open => (1..self.count.saturating_sub(1)).collect(),
        }
    }

    fn station_arclength(&self, points: &[Point2]) -> (Vec<f64>, Vec<f64>, f64) {
        match self.topology {
            TrackTopology::Closed => closed_arclength(points),
            TrackTopology::Open => open_arclength(points),
        }
    }

    fn station_headings(&self, points: &[Point2]) -> Vec<f64> {
        match self.topology {
            TrackTopology::Closed => closed_headings_from_xy(points),
            TrackTopology::Open => open_headings_from_xy(points),
        }
    }

    fn initial_solution(&self) -> Vec<f64> {
        let speed = initial_speed_profile(
            &self.center,
            self.vehicle_v_max_mps,
            &self.envelope,
            self.topology,
        );
        let n_offset = vec![0.0; self.count];
        let station_xy = self.station_xy(&n_offset);
        let (_, ds, _) = self.station_arclength(&station_xy);
        let heading = self.station_headings(&station_xy);
        let mut x = vec![0.0; self.variable_count()];
        for index in 0..self.count {
            let v = speed[index].max(1e-3);
            x[idx_n(self.count, index)] = 0.0;
            x[idx_vx(self.count, index)] = v * heading[index].cos();
            x[idx_vy(self.count, index)] = v * heading[index].sin();
        }
        for index in 0..self.interval_count {
            let next = self.next_station(index);
            let speed_avg = 0.5 * (speed[index] + speed[next]).max(1e-3);
            let dt = ds[index] / speed_avg.max(1e-3);
            x[idx_dt(self.count, index)] = dt.clamp(
                self.options.min_segment_time_s,
                self.options.max_segment_time_s,
            );
        }
        for index in self.interval_count..self.count {
            x[idx_dt(self.count, index)] = self.options.min_segment_time_s;
        }
        for index in 0..self.interval_count {
            let next = self.next_station(index);
            let dt = x[idx_dt(self.count, index)].max(1e-3);
            x[idx_ax(self.count, index)] =
                (x[idx_vx(self.count, next)] - x[idx_vx(self.count, index)]) / dt;
            x[idx_ay(self.count, index)] =
                (x[idx_vy(self.count, next)] - x[idx_vy(self.count, index)]) / dt;
        }
        self.clip_variables(&mut x);
        x
    }

    fn variable_bounds(&self) -> (Vec<f64>, Vec<f64>) {
        let mut lower = vec![0.0; self.variable_count()];
        let mut upper = vec![0.0; self.variable_count()];
        let accel_bound = self.accel_component_bound();
        for index in 0..self.count {
            lower[idx_n(self.count, index)] = self.lower_n[index].min(self.upper_n[index] - 1e-3);
            upper[idx_n(self.count, index)] = self.upper_n[index];
            lower[idx_vx(self.count, index)] = -self.vehicle_v_max_mps;
            upper[idx_vx(self.count, index)] = self.vehicle_v_max_mps;
            lower[idx_vy(self.count, index)] = -self.vehicle_v_max_mps;
            upper[idx_vy(self.count, index)] = self.vehicle_v_max_mps;
            lower[idx_ax(self.count, index)] = -accel_bound;
            upper[idx_ax(self.count, index)] = accel_bound;
            lower[idx_ay(self.count, index)] = -accel_bound;
            upper[idx_ay(self.count, index)] = accel_bound;
            lower[idx_dt(self.count, index)] = self.options.min_segment_time_s;
            upper[idx_dt(self.count, index)] = self.options.max_segment_time_s;
        }
        (lower, upper)
    }

    fn constraint_bounds(&self) -> (Vec<f64>, Vec<f64>) {
        let mut lower = Vec::with_capacity(self.rows.len());
        let mut upper = Vec::with_capacity(self.rows.len());
        let envelope_upper = self
            .options
            .envelope_safety
            .powf(self.envelope.coupling_exponent);
        for row in &self.rows {
            match row {
                ConstraintRow::DynPosX(_)
                | ConstraintRow::DynPosY(_)
                | ConstraintRow::DynVelX(_)
                | ConstraintRow::DynVelY(_) => {
                    lower.push(0.0);
                    upper.push(0.0);
                }
                ConstraintRow::SpeedSq(_) => {
                    lower.push(0.0);
                    upper.push(self.vehicle_v_max_mps * self.vehicle_v_max_mps);
                }
                ConstraintRow::Env { .. } => {
                    lower.push(0.0);
                    upper.push(envelope_upper);
                }
            }
        }
        (lower, upper)
    }

    fn objective(&self, x: &[f64]) -> f64 {
        let mut objective = 0.0;
        let accel_weight = self.options.control_slew_weight / self.options.g_mps2.max(1e-9).powi(2);
        let velocity_weight =
            self.options.velocity_second_diff_weight / self.vehicle_v_max_mps.max(1.0).powi(2);
        for index in 0..self.interval_count {
            objective += x[idx_dt(self.count, index)];
            if let Some(next_interval) = self.next_interval(index) {
                objective += accel_weight
                    * ((x[idx_ax(self.count, next_interval)] - x[idx_ax(self.count, index)])
                        .powi(2)
                        + (x[idx_ay(self.count, next_interval)] - x[idx_ay(self.count, index)])
                            .powi(2));
            }
        }
        for index in self.second_difference_station_indices() {
            let Some(prev) = self.previous_station(index) else {
                continue;
            };
            let next = self.next_station(index);
            let n_second = x[idx_n(self.count, prev)] - 2.0 * x[idx_n(self.count, index)]
                + x[idx_n(self.count, next)];
            objective += self.options.n_second_diff_weight * n_second * n_second;
            if self.options.velocity_second_diff_weight != 0.0 {
                let vx_second = x[idx_vx(self.count, prev)] - 2.0 * x[idx_vx(self.count, index)]
                    + x[idx_vx(self.count, next)];
                let vy_second = x[idx_vy(self.count, prev)] - 2.0 * x[idx_vy(self.count, index)]
                    + x[idx_vy(self.count, next)];
                objective += velocity_weight * (vx_second * vx_second + vy_second * vy_second);
            }
        }
        objective
    }

    fn objective_gradient(&self, x: &[f64], grad: &mut [f64]) {
        grad.fill(0.0);
        let accel_weight = self.options.control_slew_weight / self.options.g_mps2.max(1e-9).powi(2);
        let velocity_weight =
            self.options.velocity_second_diff_weight / self.vehicle_v_max_mps.max(1.0).powi(2);
        for index in 0..self.interval_count {
            grad[idx_dt(self.count, index)] += 1.0;
        }
        for index in 0..self.interval_count {
            if let Some(next_interval) = self.next_interval(index) {
                let ax_diff = x[idx_ax(self.count, next_interval)] - x[idx_ax(self.count, index)];
                let ay_diff = x[idx_ay(self.count, next_interval)] - x[idx_ay(self.count, index)];
                grad[idx_ax(self.count, index)] -= 2.0 * accel_weight * ax_diff;
                grad[idx_ax(self.count, next_interval)] += 2.0 * accel_weight * ax_diff;
                grad[idx_ay(self.count, index)] -= 2.0 * accel_weight * ay_diff;
                grad[idx_ay(self.count, next_interval)] += 2.0 * accel_weight * ay_diff;
            }
        }
        for index in self.second_difference_station_indices() {
            let Some(prev) = self.previous_station(index) else {
                continue;
            };
            let next = self.next_station(index);
            let n_second = x[idx_n(self.count, prev)] - 2.0 * x[idx_n(self.count, index)]
                + x[idx_n(self.count, next)];
            grad[idx_n(self.count, prev)] += 2.0 * self.options.n_second_diff_weight * n_second;
            grad[idx_n(self.count, index)] -= 4.0 * self.options.n_second_diff_weight * n_second;
            grad[idx_n(self.count, next)] += 2.0 * self.options.n_second_diff_weight * n_second;
            if self.options.velocity_second_diff_weight != 0.0 {
                let vx_second = x[idx_vx(self.count, prev)] - 2.0 * x[idx_vx(self.count, index)]
                    + x[idx_vx(self.count, next)];
                let vy_second = x[idx_vy(self.count, prev)] - 2.0 * x[idx_vy(self.count, index)]
                    + x[idx_vy(self.count, next)];
                grad[idx_vx(self.count, prev)] += 2.0 * velocity_weight * vx_second;
                grad[idx_vx(self.count, index)] -= 4.0 * velocity_weight * vx_second;
                grad[idx_vx(self.count, next)] += 2.0 * velocity_weight * vx_second;
                grad[idx_vy(self.count, prev)] += 2.0 * velocity_weight * vy_second;
                grad[idx_vy(self.count, index)] -= 4.0 * velocity_weight * vy_second;
                grad[idx_vy(self.count, next)] += 2.0 * velocity_weight * vy_second;
            }
        }
    }

    fn constraints(&self, x: &[f64], out: &mut [f64]) {
        for (row_index, row) in self.rows.iter().enumerate() {
            out[row_index] = self.constraint_value(*row, x);
        }
    }

    fn max_envelope_utilization(&self, x: &[f64]) -> Option<f64> {
        let mut maximum = 0.0_f64;
        let mut saw_envelope_row = false;
        for row in &self.rows {
            if let ConstraintRow::Env { .. } = row {
                let utilization = self.constraint_value(*row, x);
                if !utilization.is_finite() {
                    return None;
                }
                maximum = maximum.max(utilization);
                saw_envelope_row = true;
            }
        }
        saw_envelope_row.then_some(maximum)
    }

    fn constraint_value(&self, row: ConstraintRow, x: &[f64]) -> f64 {
        match row {
            ConstraintRow::DynPosX(index) => {
                let next = self.next_station(index);
                self.point_x(next, x)
                    - self.point_x(index, x)
                    - 0.5
                        * (x[idx_vx(self.count, index)] + x[idx_vx(self.count, next)])
                        * x[idx_dt(self.count, index)]
            }
            ConstraintRow::DynPosY(index) => {
                let next = self.next_station(index);
                self.point_y(next, x)
                    - self.point_y(index, x)
                    - 0.5
                        * (x[idx_vy(self.count, index)] + x[idx_vy(self.count, next)])
                        * x[idx_dt(self.count, index)]
            }
            ConstraintRow::DynVelX(index) => {
                let next = self.next_station(index);
                x[idx_vx(self.count, next)]
                    - x[idx_vx(self.count, index)]
                    - x[idx_ax(self.count, index)] * x[idx_dt(self.count, index)]
            }
            ConstraintRow::DynVelY(index) => {
                let next = self.next_station(index);
                x[idx_vy(self.count, next)]
                    - x[idx_vy(self.count, index)]
                    - x[idx_ay(self.count, index)] * x[idx_dt(self.count, index)]
            }
            ConstraintRow::SpeedSq(index) => {
                x[idx_vx(self.count, index)].powi(2) + x[idx_vy(self.count, index)].powi(2)
            }
            ConstraintRow::Env {
                station,
                sample,
                side,
            } => self.envelope_constraint(station, sample, side, x),
        }
    }

    fn jacobian_values(&self, x: &[f64], values: &mut [f64]) {
        for (entry_index, (row, col)) in self.jac_pattern.iter().copied().enumerate() {
            let row_spec = self.rows[row as usize];
            values[entry_index] = self.constraint_derivative(row_spec, col as usize, x);
        }
    }

    fn constraint_derivative(&self, row: ConstraintRow, col: usize, x: &[f64]) -> f64 {
        match row {
            ConstraintRow::DynPosX(index) => {
                let next = self.next_station(index);
                if col == idx_n(self.count, index) {
                    -self.normals[index][0]
                } else if col == idx_n(self.count, next) {
                    self.normals[next][0]
                } else if col == idx_vx(self.count, index) || col == idx_vx(self.count, next) {
                    -0.5 * x[idx_dt(self.count, index)]
                } else if col == idx_dt(self.count, index) {
                    -0.5 * (x[idx_vx(self.count, index)] + x[idx_vx(self.count, next)])
                } else {
                    0.0
                }
            }
            ConstraintRow::DynPosY(index) => {
                let next = self.next_station(index);
                if col == idx_n(self.count, index) {
                    -self.normals[index][1]
                } else if col == idx_n(self.count, next) {
                    self.normals[next][1]
                } else if col == idx_vy(self.count, index) || col == idx_vy(self.count, next) {
                    -0.5 * x[idx_dt(self.count, index)]
                } else if col == idx_dt(self.count, index) {
                    -0.5 * (x[idx_vy(self.count, index)] + x[idx_vy(self.count, next)])
                } else {
                    0.0
                }
            }
            ConstraintRow::DynVelX(index) => {
                let next = self.next_station(index);
                if col == idx_vx(self.count, next) {
                    1.0
                } else if col == idx_vx(self.count, index) {
                    -1.0
                } else if col == idx_ax(self.count, index) {
                    -x[idx_dt(self.count, index)]
                } else if col == idx_dt(self.count, index) {
                    -x[idx_ax(self.count, index)]
                } else {
                    0.0
                }
            }
            ConstraintRow::DynVelY(index) => {
                let next = self.next_station(index);
                if col == idx_vy(self.count, next) {
                    1.0
                } else if col == idx_vy(self.count, index) {
                    -1.0
                } else if col == idx_ay(self.count, index) {
                    -x[idx_dt(self.count, index)]
                } else if col == idx_dt(self.count, index) {
                    -x[idx_ay(self.count, index)]
                } else {
                    0.0
                }
            }
            ConstraintRow::SpeedSq(index) => {
                if col == idx_vx(self.count, index) {
                    2.0 * x[idx_vx(self.count, index)]
                } else if col == idx_vy(self.count, index) {
                    2.0 * x[idx_vy(self.count, index)]
                } else {
                    0.0
                }
            }
            ConstraintRow::Env { .. } => self.numeric_constraint_derivative(row, col, x),
        }
    }

    fn numeric_constraint_derivative(&self, row: ConstraintRow, col: usize, x: &[f64]) -> f64 {
        let h = 1e-6 * x[col].abs().max(1.0);
        let mut plus = x.to_vec();
        let mut minus = x.to_vec();
        plus[col] += h;
        minus[col] -= h;
        (self.constraint_value(row, &plus) - self.constraint_value(row, &minus)) / (2.0 * h)
    }

    fn build_jacobian_pattern(&self) -> Result<Vec<(i32, i32)>, String> {
        let mut pattern = Vec::new();
        for (row_index, row) in self.rows.iter().copied().enumerate() {
            let cols = self.constraint_columns(row);
            for col in cols {
                pattern.push((
                    i32::try_from(row_index).map_err(|_| "too many constraints".to_owned())?,
                    i32::try_from(col).map_err(|_| "too many variables".to_owned())?,
                ));
            }
        }
        Ok(pattern)
    }

    fn constraint_columns(&self, row: ConstraintRow) -> Vec<usize> {
        match row {
            ConstraintRow::DynPosX(index) => {
                let next = self.next_station(index);
                vec![
                    idx_n(self.count, index),
                    idx_n(self.count, next),
                    idx_vx(self.count, index),
                    idx_vx(self.count, next),
                    idx_dt(self.count, index),
                ]
            }
            ConstraintRow::DynPosY(index) => {
                let next = self.next_station(index);
                vec![
                    idx_n(self.count, index),
                    idx_n(self.count, next),
                    idx_vy(self.count, index),
                    idx_vy(self.count, next),
                    idx_dt(self.count, index),
                ]
            }
            ConstraintRow::DynVelX(index) => {
                let next = self.next_station(index);
                vec![
                    idx_vx(self.count, index),
                    idx_vx(self.count, next),
                    idx_ax(self.count, index),
                    idx_dt(self.count, index),
                ]
            }
            ConstraintRow::DynVelY(index) => {
                let next = self.next_station(index);
                vec![
                    idx_vy(self.count, index),
                    idx_vy(self.count, next),
                    idx_ay(self.count, index),
                    idx_dt(self.count, index),
                ]
            }
            ConstraintRow::SpeedSq(index) => {
                vec![idx_vx(self.count, index), idx_vy(self.count, index)]
            }
            ConstraintRow::Env {
                station, sample, ..
            } => {
                let next = self.next_station(station);
                match sample {
                    EnvelopeSample::Start => vec![
                        idx_vx(self.count, station),
                        idx_vy(self.count, station),
                        idx_ax(self.count, station),
                        idx_ay(self.count, station),
                    ],
                    EnvelopeSample::Mid => vec![
                        idx_vx(self.count, station),
                        idx_vy(self.count, station),
                        idx_vx(self.count, next),
                        idx_vy(self.count, next),
                        idx_ax(self.count, station),
                        idx_ay(self.count, station),
                    ],
                    EnvelopeSample::End => vec![
                        idx_vx(self.count, next),
                        idx_vy(self.count, next),
                        idx_ax(self.count, station),
                        idx_ay(self.count, station),
                    ],
                }
            }
        }
    }

    fn envelope_constraint(
        &self,
        station: usize,
        sample: EnvelopeSample,
        side: EnvelopeSide,
        x: &[f64],
    ) -> f64 {
        let next = self.next_station(station);
        let v_start = [
            x[idx_vx(self.count, station)],
            x[idx_vy(self.count, station)],
        ];
        let v_end = [x[idx_vx(self.count, next)], x[idx_vy(self.count, next)]];
        let velocity = match sample {
            EnvelopeSample::Start => v_start,
            EnvelopeSample::Mid => [0.5 * (v_start[0] + v_end[0]), 0.5 * (v_start[1] + v_end[1])],
            EnvelopeSample::End => v_end,
        };
        let acceleration = [
            x[idx_ax(self.count, station)],
            x[idx_ay(self.count, station)],
        ];
        let ref_speed = (velocity[0] * velocity[0] + velocity[1] * velocity[1] + 1e-8).sqrt();
        let safe_speed = ref_speed.max(1e-6);
        let ref_dir = [velocity[0] / safe_speed, velocity[1] / safe_speed];
        let a_long = acceleration[0] * ref_dir[0] + acceleration[1] * ref_dir[1];
        let a_lat = ref_dir[0] * acceleration[1] - ref_dir[1] * acceleration[0];
        let limits = self.envelope.limits(ref_speed);
        let ay_limit = if a_lat >= 0.0 {
            limits.ay_left_max_mps2
        } else {
            limits.ay_right_max_mps2
        };
        let a_lat_abs = (a_lat * a_lat + self.options.smooth_abs_eps.powi(2)).sqrt();
        let a_long_abs = (a_long * a_long + self.options.smooth_abs_eps.powi(2)).sqrt();
        let a_long_pos = 0.5 * (a_long + a_long_abs);
        let a_long_neg = 0.5 * (-a_long + a_long_abs);
        let p = self.envelope.coupling_exponent;
        let lat_term = (a_lat_abs / ay_limit.max(1e-6)).powf(p);
        match side {
            EnvelopeSide::Drive => {
                lat_term + (a_long_pos / limits.ax_drive_max_mps2.max(1e-6)).powf(p)
            }
            EnvelopeSide::Brake => {
                lat_term + (a_long_neg / limits.ax_brake_max_mps2.max(1e-6)).powf(p)
            }
        }
    }

    fn point_x(&self, index: usize, x: &[f64]) -> f64 {
        self.center[index][0] + self.normals[index][0] * x[idx_n(self.count, index)]
    }

    fn point_y(&self, index: usize, x: &[f64]) -> f64 {
        self.center[index][1] + self.normals[index][1] * x[idx_n(self.count, index)]
    }

    fn station_xy_from_x(&self, x: &[f64]) -> Vec<Point2> {
        (0..self.count)
            .map(|index| [self.point_x(index, x), self.point_y(index, x)])
            .collect()
    }

    fn station_xy(&self, n_offset: &[f64]) -> Vec<Point2> {
        self.center
            .iter()
            .zip(self.normals.iter())
            .zip(n_offset.iter())
            .map(|((center, normal), n)| [center[0] + normal[0] * *n, center[1] + normal[1] * *n])
            .collect()
    }

    fn accel_component_bound(&self) -> f64 {
        self.options.accel_component_bound_mps2.unwrap_or_else(|| {
            2.0 * self
                .envelope
                .ax_drive_max_mps2
                .iter()
                .chain(self.envelope.ax_brake_max_mps2.iter())
                .chain(self.envelope.ay_left_max_mps2.iter())
                .chain(self.envelope.ay_right_max_mps2.iter())
                .copied()
                .fold(0.0_f64, f64::max)
        })
    }

    fn clip_variables(&self, x: &mut [f64]) {
        let (lower, upper) = self.variable_bounds();
        for ((value, low), high) in x.iter_mut().zip(lower.iter()).zip(upper.iter()) {
            *value = value.clamp(*low, *high);
        }
    }

    fn to_series(&self, x: &[f64]) -> (TrajectoryResultSeriesV1, f64) {
        let station_xy = self.station_xy_from_x(x);
        let (station_s_m, _station_ds, _station_total_length) = self.station_arclength(&station_xy);
        let vx = (0..self.count)
            .map(|index| x[idx_vx(self.count, index)])
            .collect::<Vec<_>>();
        let vy = (0..self.count)
            .map(|index| x[idx_vy(self.count, index)])
            .collect::<Vec<_>>();
        let ax_world = (0..self.count)
            .map(|index| x[idx_ax(self.count, index)])
            .collect::<Vec<_>>();
        let ay_world = (0..self.count)
            .map(|index| x[idx_ay(self.count, index)])
            .collect::<Vec<_>>();
        let dt = (0..self.count)
            .map(|index| x[idx_dt(self.count, index)].max(1e-9))
            .collect::<Vec<_>>();
        let lap_time = dt.iter().take(self.interval_count).sum();

        if self.options.publish_geometry_mode == PublishGeometryMode::DenseDynamics {
            return (
                self.to_dense_series(&station_xy, &vx, &vy, &ax_world, &ay_world, &dt),
                lap_time,
            );
        }

        if self.options.publish_geometry_mode == PublishGeometryMode::SectionInterpolated {
            return (
                self.to_section_interpolated_series(x, &vx, &vy, &ax_world, &ay_world, &dt),
                lap_time,
            );
        }

        let mut speed = Vec::with_capacity(self.count);
        let mut heading = Vec::with_capacity(self.count);
        let mut ax_long = Vec::with_capacity(self.count);
        let mut ay_lat = Vec::with_capacity(self.count);
        let mut kappa = Vec::with_capacity(self.count);
        for index in 0..self.count {
            let next = if index < self.interval_count {
                self.next_station(index)
            } else {
                index
            };
            let station_speed = (vx[index] * vx[index] + vy[index] * vy[index]).sqrt();
            let avg_vx = 0.5 * (vx[index] + vx[next]);
            let avg_vy = 0.5 * (vy[index] + vy[next]);
            let segment_speed = (avg_vx * avg_vx + avg_vy * avg_vy).sqrt();
            let safe_segment_speed = segment_speed.max(1e-6);
            speed.push(station_speed);
            heading.push(vy[index].atan2(vx[index]));
            ax_long
                .push((ax_world[index] * avg_vx + ay_world[index] * avg_vy) / safe_segment_speed);
            ay_lat.push((avg_vx * ay_world[index] - avg_vy * ax_world[index]) / safe_segment_speed);
            kappa.push(ay_lat[index] / (segment_speed * segment_speed).max(1e-6));
        }
        unwrap_angles(&mut heading);
        let (cornering, longitudinal, combined) =
            envelope_utilization(&speed, &ax_long, &ay_lat, &self.envelope);
        (
            TrajectoryResultSeriesV1 {
                s_m: station_s_m,
                x_m: station_xy.iter().map(|point| point[0]).collect(),
                y_m: station_xy.iter().map(|point| point[1]).collect(),
                heading_rad: heading,
                kappa_1pm: kappa,
                v_mps: speed,
                ax_mps2: ax_long,
                ay_mps2: ay_lat,
                utilization_cornering: cornering,
                utilization_longitudinal: longitudinal,
                utilization_combined: combined,
                station_index: Some((0..self.count).map(|index| index as i64).collect()),
            },
            lap_time,
        )
    }

    fn to_section_interpolated_series(
        &self,
        x: &[f64],
        vx: &[f64],
        vy: &[f64],
        ax_world: &[f64],
        ay_world: &[f64],
        _dt: &[f64],
    ) -> TrajectoryResultSeriesV1 {
        let target_count = self.output_sample_count();
        let mut xy = Vec::with_capacity(target_count);
        let mut velocity = Vec::with_capacity(target_count);
        let mut acceleration = Vec::with_capacity(target_count);
        let mut station_index = Vec::with_capacity(target_count);

        for sample in 0..target_count {
            let sample_denominator = if self.topology.is_closed() {
                target_count as f64
            } else {
                target_count.saturating_sub(1).max(1) as f64
            };
            let scaled = self.interval_count as f64 * sample as f64 / sample_denominator;
            let segment = (scaled.floor() as usize).min(self.interval_count.saturating_sub(1));
            let next = self.next_station(segment);
            let t = scaled - segment as f64;
            let center = lerp_point(self.center[segment], self.center[next], t);
            let normal = normalize_point(
                lerp_point(self.normals[segment], self.normals[next], t),
                self.normals[segment],
            );
            let n = lerp_scalar(x[idx_n(self.count, segment)], x[idx_n(self.count, next)], t);
            let point = [center[0] + normal[0] * n, center[1] + normal[1] * n];
            let vel = [
                lerp_scalar(vx[segment], vx[next], t),
                lerp_scalar(vy[segment], vy[next], t),
            ];
            let accel = [
                lerp_scalar(ax_world[segment], ax_world[next], t),
                lerp_scalar(ay_world[segment], ay_world[next], t),
            ];

            xy.push(point);
            velocity.push(vel);
            acceleration.push(accel);
            station_index.push(segment as i64);
        }

        let (s_m, _ds, _total_length) = self.station_arclength(&xy);
        let mut speed = Vec::with_capacity(target_count);
        let mut heading = Vec::with_capacity(target_count);
        let mut ax_long = Vec::with_capacity(target_count);
        let mut ay_lat = Vec::with_capacity(target_count);

        for index in 0..target_count {
            let vel = velocity[index];
            let accel = acceleration[index];
            let station_speed = (vel[0] * vel[0] + vel[1] * vel[1]).sqrt();
            let safe_speed = station_speed.max(1e-6);

            speed.push(station_speed);
            heading.push(vel[1].atan2(vel[0]));
            ax_long.push((accel[0] * vel[0] + accel[1] * vel[1]) / safe_speed);
            ay_lat.push((vel[0] * accel[1] - vel[1] * accel[0]) / safe_speed);
        }

        unwrap_angles(&mut heading);
        let kappa_model = speed
            .iter()
            .zip(ay_lat.iter())
            .map(|(v, ay)| ay / (v * v).max(1e-6))
            .collect::<Vec<_>>();
        let (cornering, longitudinal, combined) =
            envelope_utilization(&speed, &ax_long, &ay_lat, &self.envelope);

        TrajectoryResultSeriesV1 {
            s_m,
            x_m: xy.iter().map(|point| point[0]).collect(),
            y_m: xy.iter().map(|point| point[1]).collect(),
            heading_rad: heading,
            kappa_1pm: kappa_model,
            v_mps: speed,
            ax_mps2: ax_long,
            ay_mps2: ay_lat,
            utilization_cornering: cornering,
            utilization_longitudinal: longitudinal,
            utilization_combined: combined,
            station_index: Some(station_index),
        }
    }

    fn to_dense_series(
        &self,
        station_xy: &[Point2],
        vx: &[f64],
        vy: &[f64],
        ax_world: &[f64],
        ay_world: &[f64],
        dt: &[f64],
    ) -> TrajectoryResultSeriesV1 {
        let target_count = self.output_sample_count();
        let mut cumulative_t = Vec::with_capacity(self.count + 1);
        cumulative_t.push(0.0);
        for value in dt.iter().take(self.interval_count) {
            cumulative_t.push(cumulative_t.last().copied().unwrap_or(0.0) + *value);
        }
        let total_time = cumulative_t.last().copied().unwrap_or(0.0);
        let mut xy = Vec::with_capacity(target_count);
        let mut velocity = Vec::with_capacity(target_count);
        let mut acceleration = Vec::with_capacity(target_count);
        let mut station_index = Vec::with_capacity(target_count);

        for sample in 0..target_count {
            let sample_denominator = if self.topology.is_closed() {
                target_count as f64
            } else {
                target_count.saturating_sub(1).max(1) as f64
            };
            let sample_t = if total_time <= 1e-9 {
                0.0
            } else {
                total_time * sample as f64 / sample_denominator
            };
            let mut segment = cumulative_t.partition_point(|value| *value <= sample_t);
            segment = segment
                .saturating_sub(1)
                .min(self.interval_count.saturating_sub(1));
            let tau = sample_t - cumulative_t[segment];
            let point = [
                station_xy[segment][0] + vx[segment] * tau + 0.5 * ax_world[segment] * tau * tau,
                station_xy[segment][1] + vy[segment] * tau + 0.5 * ay_world[segment] * tau * tau,
            ];
            let vel = [
                vx[segment] + ax_world[segment] * tau,
                vy[segment] + ay_world[segment] * tau,
            ];

            xy.push(point);
            velocity.push(vel);
            acceleration.push([ax_world[segment], ay_world[segment]]);
            station_index.push(segment as i64);
        }

        let (s_m, _ds, _total_length) = self.station_arclength(&xy);
        let mut speed = Vec::with_capacity(target_count);
        let mut heading = Vec::with_capacity(target_count);
        let mut ax_long = Vec::with_capacity(target_count);
        let mut ay_lat = Vec::with_capacity(target_count);

        for index in 0..target_count {
            let vel = velocity[index];
            let accel = acceleration[index];
            let station_speed = (vel[0] * vel[0] + vel[1] * vel[1]).sqrt();
            let safe_speed = station_speed.max(1e-6);

            speed.push(station_speed);
            heading.push(vel[1].atan2(vel[0]));
            ax_long.push((accel[0] * vel[0] + accel[1] * vel[1]) / safe_speed);
            ay_lat.push((vel[0] * accel[1] - vel[1] * accel[0]) / safe_speed);
        }

        unwrap_angles(&mut heading);
        let kappa_model = speed
            .iter()
            .zip(ay_lat.iter())
            .map(|(v, ay)| ay / (v * v).max(1e-6))
            .collect::<Vec<_>>();
        let (cornering, longitudinal, combined) =
            envelope_utilization(&speed, &ax_long, &ay_lat, &self.envelope);

        TrajectoryResultSeriesV1 {
            s_m,
            x_m: xy.iter().map(|point| point[0]).collect(),
            y_m: xy.iter().map(|point| point[1]).collect(),
            heading_rad: heading,
            kappa_1pm: kappa_model,
            v_mps: speed,
            ax_mps2: ax_long,
            ay_mps2: ay_lat,
            utilization_cornering: cornering,
            utilization_longitudinal: longitudinal,
            utilization_combined: combined,
            station_index: Some(station_index),
        }
    }

    fn output_sample_count(&self) -> usize {
        self.options
            .output_sample_count
            .unwrap_or_else(|| self.count * POINT_MASS_OUTPUT_SAMPLES_PER_STATION)
            .max(self.count)
            .max(1)
    }

    fn emit_iteration_preview(&mut self, x: &[f64], objective_value: f64, force: bool) {
        self.objective_eval_count = self.objective_eval_count.saturating_add(1);
        let should_emit = force
            || self.objective_eval_count == 1
            || self
                .objective_eval_count
                .saturating_sub(self.last_preview_eval_count)
                >= 12;

        if !should_emit {
            return;
        }

        if self.progress_callback.is_none() {
            return;
        }

        self.last_preview_eval_count = self.objective_eval_count;
        let (series, lap_time_s) = self.to_series(x);
        let preview = PointMassIterationPreview {
            iteration: self.last_ipopt_iteration,
            series,
            lap_time_s,
            objective_value,
            max_envelope_utilization: self.max_envelope_utilization(x).unwrap_or(f64::NAN),
        };

        let Some(callback) = self.progress_callback.as_deref_mut() else {
            return;
        };
        callback(PointMassProgressUpdate::Preview(Box::new(preview)));
    }

    fn emit_optimizer_iteration(&mut self, iter_count: i32, objective_value: f64) {
        let iteration = u32::try_from(iter_count.max(0)).unwrap_or(u32::MAX);
        self.last_ipopt_iteration = Some(iteration);
        let Some(callback) = self.progress_callback.as_deref_mut() else {
            return;
        };
        callback(PointMassProgressUpdate::OptimizerIteration {
            iteration,
            objective_value,
        });
    }
}

pub fn solve_point_mass_velocity_vector_ocp(
    view: &SectionsTrackViewV1,
    profile: &PointMassProfileV1,
    envelope: &AccelerationEnvelopeV1,
    options: PointMassSolveOptions,
) -> Result<PointMassSolveResult, String> {
    solve_point_mass_velocity_vector_ocp_with_progress(view, profile, envelope, options, None)
}

pub fn solve_point_mass_velocity_vector_ocp_with_progress(
    view: &SectionsTrackViewV1,
    profile: &PointMassProfileV1,
    envelope: &AccelerationEnvelopeV1,
    options: PointMassSolveOptions,
    progress: Option<PointMassSolveProgressCallback<'_>>,
) -> Result<PointMassSolveResult, String> {
    let mut nlp = PointMassNlp::new(view, profile, envelope, options)?;
    if let Some(callback) = progress {
        nlp.set_progress_callback(callback);
    }
    let nlp = Box::new(nlp);
    let result = solve_with_ipopt(nlp)?;
    Ok(result)
}

pub fn write_trajectory_csv(
    path: impl AsRef<Path>,
    series: &TrajectoryResultSeriesV1,
) -> Result<(), String> {
    let path = path.as_ref();
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent)
            .map_err(|error| format!("failed to create {}: {error}", parent.display()))?;
    }
    let mut body =
        "# s_m;x_m;y_m;heading_rad;kappa_1pm;v_mps;ax_mps2;ay_mps2;utilization_cornering;utilization_longitudinal;utilization_combined;station_index\n".to_owned();
    let station_index = series.station_index.as_ref();
    for index in 0..series.s_m.len() {
        body.push_str(&format!(
            "{:.9};{:.9};{:.9};{:.9};{:.9};{:.9};{:.9};{:.9};{:.9};{:.9};{:.9};{:.9}\n",
            series.s_m[index],
            series.x_m[index],
            series.y_m[index],
            series.heading_rad[index],
            series.kappa_1pm[index],
            series.v_mps[index],
            series.ax_mps2[index],
            series.ay_mps2[index],
            series.utilization_cornering[index],
            series.utilization_longitudinal[index],
            series.utilization_combined[index],
            station_index
                .and_then(|values| values.get(index).copied())
                .unwrap_or(index as i64) as f64
        ));
    }
    std::fs::write(path, body)
        .map_err(|error| format!("failed to write {}: {error}", path.display()))
}

fn solve_with_ipopt(mut nlp: Box<PointMassNlp<'_>>) -> Result<PointMassSolveResult, String> {
    let mut x = nlp.initial_solution();
    let (mut lower_x, mut upper_x) = nlp.variable_bounds();
    let (mut lower_g, mut upper_g) = nlp.constraint_bounds();
    let variable_count =
        i32::try_from(nlp.variable_count()).map_err(|_| "too many variables".to_owned())?;
    let constraint_count =
        i32::try_from(nlp.rows.len()).map_err(|_| "too many constraints".to_owned())?;
    let jac_count =
        i32::try_from(nlp.jac_pattern.len()).map_err(|_| "too many jacobian entries".to_owned())?;
    let library_path = crate::ipopt::default_library_path(nlp.options.ipopt_dll_path.clone());
    let ipopt = crate::ipopt::IpoptApi::load(&library_path)?;
    unsafe {
        let problem = (ipopt.create_problem)(
            variable_count,
            lower_x.as_mut_ptr(),
            upper_x.as_mut_ptr(),
            constraint_count,
            lower_g.as_mut_ptr(),
            upper_g.as_mut_ptr(),
            jac_count,
            0,
            0,
            eval_f_cb,
            eval_g_cb,
            eval_grad_f_cb,
            eval_jac_g_cb,
            Some(eval_h_cb),
        );
        if problem.is_null() {
            return Err("CreateIpoptProblem returned null".to_owned());
        }
        let _guard = crate::ipopt::IpoptProblemGuard::new(problem, ipopt.free_problem);
        ipopt.add_int(problem, "print_level", nlp.options.ipopt_print_level)?;
        ipopt.add_int(problem, "max_iter", nlp.options.max_iter)?;
        ipopt.add_num(problem, "tol", nlp.options.tol)?;
        ipopt.add_num(problem, "acceptable_tol", nlp.options.acceptable_tol)?;
        ipopt.add_int(problem, "acceptable_iter", nlp.options.acceptable_iter)?;
        if let Some(linear_solver) = nlp.options.ipopt_linear_solver.as_deref() {
            ipopt.add_str(problem, "linear_solver", linear_solver)?;
        }
        ipopt.add_str(problem, "hessian_approximation", "limited-memory")?;
        ipopt.add_str(problem, "mu_strategy", "adaptive")?;
        if let Some(set_intermediate_callback) = ipopt.set_intermediate_callback {
            set_intermediate_callback(problem, point_mass_intermediate_cb);
        }
        let mut g = vec![0.0; constraint_count as usize];
        let mut objective = 0.0;
        let user_data = (&mut *nlp) as *mut PointMassNlp as *mut c_void;
        let initial_objective = nlp.objective(&x);
        nlp.emit_iteration_preview(&x, initial_objective, true);
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
        let status = crate::ipopt::status_name(status_code);
        if !crate::ipopt::status_is_success(status_code) {
            return Err(format!(
                "Ipopt solve failed with status {status} ({status_code})"
            ));
        }
        let (series, lap_time_s) = nlp.to_series(&x);
        Ok(PointMassSolveResult {
            series,
            lap_time_s,
            status: status.to_owned(),
            objective_value: objective,
        })
    }
}

unsafe extern "C" fn eval_f_cb(
    _n: i32,
    x: *mut f64,
    _new_x: bool,
    obj_value: *mut f64,
    user_data: *mut c_void,
) -> bool {
    let nlp = &mut *(user_data as *mut PointMassNlp);
    let values = std::slice::from_raw_parts(x, nlp.variable_count());
    let objective = nlp.objective(values);
    *obj_value = objective;
    nlp.emit_iteration_preview(values, objective, false);
    true
}

unsafe extern "C" fn point_mass_intermediate_cb(
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
    let nlp = &mut *(user_data as *mut PointMassNlp);
    nlp.emit_optimizer_iteration(iter_count, obj_value);
    true
}

unsafe extern "C" fn eval_grad_f_cb(
    _n: i32,
    x: *mut f64,
    _new_x: bool,
    grad_f: *mut f64,
    user_data: *mut c_void,
) -> bool {
    let nlp = &*(user_data as *const PointMassNlp);
    let values = std::slice::from_raw_parts(x, nlp.variable_count());
    let grad = std::slice::from_raw_parts_mut(grad_f, nlp.variable_count());
    nlp.objective_gradient(values, grad);
    true
}

unsafe extern "C" fn eval_g_cb(
    _n: i32,
    x: *mut f64,
    _new_x: bool,
    _m: i32,
    g: *mut f64,
    user_data: *mut c_void,
) -> bool {
    let nlp = &*(user_data as *const PointMassNlp);
    let values = std::slice::from_raw_parts(x, nlp.variable_count());
    let constraints = std::slice::from_raw_parts_mut(g, nlp.rows.len());
    nlp.constraints(values, constraints);
    true
}

unsafe extern "C" fn eval_jac_g_cb(
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
    let nlp = &*(user_data as *const PointMassNlp);
    if values.is_null() {
        let rows = std::slice::from_raw_parts_mut(i_row, nlp.jac_pattern.len());
        let cols = std::slice::from_raw_parts_mut(j_col, nlp.jac_pattern.len());
        for (index, (row, col)) in nlp.jac_pattern.iter().copied().enumerate() {
            rows[index] = row;
            cols[index] = col;
        }
        return true;
    }
    let x_values = std::slice::from_raw_parts(x, nlp.variable_count());
    let jac_values = std::slice::from_raw_parts_mut(values, nlp.jac_pattern.len());
    nlp.jacobian_values(x_values, jac_values);
    true
}

unsafe extern "C" fn eval_h_cb(
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

fn profile_param(profile: &PointMassProfileV1, key: &str) -> Option<f64> {
    profile
        .params
        .iter()
        .find(|(entry_key, _)| entry_key == key)
        .and_then(|(_, value)| value.as_f64())
}

fn metadata_str<'a>(
    metadata: &'a [(String, crate::json::JsonValue)],
    key: &str,
) -> Option<&'a str> {
    metadata
        .iter()
        .find(|(entry_key, _)| entry_key == key)
        .and_then(|(_, value)| value.as_str())
}

fn point_mass_n_bounds_m(width_left_m: f64, width_right_m: f64, width_opt_m: f64) -> (f64, f64) {
    let half_width_opt_m = 0.5 * width_opt_m.max(0.0);
    let lower = -width_left_m.max(1.0e-3) + half_width_opt_m;
    let upper = width_right_m.max(1.0e-3) - half_width_opt_m;
    if lower <= upper - 1.0e-3 {
        return (lower, upper);
    }
    let midpoint = 0.5 * (lower + upper);
    (midpoint - 5.0e-4, midpoint + 5.0e-4)
}

fn idx_n(_count: usize, index: usize) -> usize {
    index
}

fn idx_vx(count: usize, index: usize) -> usize {
    count + index
}

fn idx_vy(count: usize, index: usize) -> usize {
    2 * count + index
}

fn idx_ax(count: usize, index: usize) -> usize {
    3 * count + index
}

fn idx_ay(count: usize, index: usize) -> usize {
    4 * count + index
}

fn idx_dt(count: usize, index: usize) -> usize {
    5 * count + index
}

fn initial_speed_profile(
    center: &[Point2],
    vehicle_v_max_mps: f64,
    envelope: &AccelerationEnvelopeV1,
    topology: TrackTopology,
) -> Vec<f64> {
    let (_, ds, _) = match topology {
        TrackTopology::Closed => closed_arclength(center),
        TrackTopology::Open => open_arclength(center),
    };
    let kappa = three_point_curvature(center, topology);
    let mut speed = vec![vehicle_v_max_mps; center.len()];
    for _ in 0..8 {
        let mut max_delta = 0.0_f64;
        let next_speed = speed
            .iter()
            .zip(kappa.iter())
            .map(|(v, k)| {
                let limits = envelope.limits(*v);
                if k.abs() > 1e-7 {
                    let ay_limit = limits.ay_left_max_mps2.min(limits.ay_right_max_mps2);
                    vehicle_v_max_mps.min(((0.995 * ay_limit).max(1e-9) / k.abs()).sqrt())
                } else {
                    vehicle_v_max_mps
                }
            })
            .collect::<Vec<_>>();
        for (old, new) in speed.iter().zip(next_speed.iter()) {
            max_delta = max_delta.max((old - new).abs());
        }
        speed = next_speed;
        if max_delta < 1e-6 {
            break;
        }
    }
    for _ in 0..12 {
        let drive = speed
            .iter()
            .map(|v| envelope.limits(*v).ax_drive_max_mps2)
            .collect::<Vec<_>>();
        let interval_count = match topology {
            TrackTopology::Closed => speed.len(),
            TrackTopology::Open => speed.len().saturating_sub(1),
        };
        for index in 0..interval_count {
            let next = match topology {
                TrackTopology::Closed => (index + 1) % speed.len(),
                TrackTopology::Open => index + 1,
            };
            speed[next] = speed[next].min(
                (speed[index] * speed[index] + 2.0 * drive[index] * ds[index])
                    .max(0.0)
                    .sqrt(),
            );
        }
        let brake = speed
            .iter()
            .map(|v| envelope.limits(*v).ax_brake_max_mps2)
            .collect::<Vec<_>>();
        for index in (0..interval_count).rev() {
            let next = match topology {
                TrackTopology::Closed => (index + 1) % speed.len(),
                TrackTopology::Open => index + 1,
            };
            speed[index] = speed[index].min(
                (speed[next] * speed[next] + 2.0 * brake[next] * ds[index])
                    .max(0.0)
                    .sqrt(),
            );
        }
    }
    speed
}

fn three_point_curvature(points: &[Point2], topology: TrackTopology) -> Vec<f64> {
    (0..points.len())
        .map(|index| {
            let prev = match topology {
                TrackTopology::Closed => (index + points.len() - 1) % points.len(),
                TrackTopology::Open => index.saturating_sub(1),
            };
            let next = match topology {
                TrackTopology::Closed => (index + 1) % points.len(),
                TrackTopology::Open => (index + 1).min(points.len() - 1),
            };
            let a = [
                points[index][0] - points[prev][0],
                points[index][1] - points[prev][1],
            ];
            let b = [
                points[next][0] - points[index][0],
                points[next][1] - points[index][1],
            ];
            let c = [
                points[next][0] - points[prev][0],
                points[next][1] - points[prev][1],
            ];
            let cross = a[0] * b[1] - a[1] * b[0];
            let denom = hypot(a).max(0.0) * hypot(b).max(0.0) * hypot(c).max(0.0);
            2.0 * cross / denom.max(1e-9)
        })
        .collect()
}

fn closed_arclength(points: &[Point2]) -> (Vec<f64>, Vec<f64>, f64) {
    let mut s = Vec::with_capacity(points.len());
    let mut ds = Vec::with_capacity(points.len());
    let mut total = 0.0;
    for index in 0..points.len() {
        s.push(total);
        let next = (index + 1) % points.len();
        let length = ((points[next][0] - points[index][0]).powi(2)
            + (points[next][1] - points[index][1]).powi(2))
        .sqrt();
        ds.push(length);
        total += length;
    }
    (s, ds, total)
}

fn open_arclength(points: &[Point2]) -> (Vec<f64>, Vec<f64>, f64) {
    let mut s = Vec::with_capacity(points.len());
    let mut ds = Vec::with_capacity(points.len().saturating_sub(1));
    let mut total = 0.0;
    for index in 0..points.len() {
        s.push(total);
        if index + 1 < points.len() {
            let length = ((points[index + 1][0] - points[index][0]).powi(2)
                + (points[index + 1][1] - points[index][1]).powi(2))
            .sqrt();
            ds.push(length);
            total += length;
        }
    }
    (s, ds, total)
}

fn closed_headings_from_xy(points: &[Point2]) -> Vec<f64> {
    let mut heading = (0..points.len())
        .map(|index| {
            let prev = (index + points.len() - 1) % points.len();
            let next = (index + 1) % points.len();
            (points[next][1] - points[prev][1]).atan2(points[next][0] - points[prev][0])
        })
        .collect::<Vec<_>>();
    unwrap_angles(&mut heading);
    heading
}

fn open_headings_from_xy(points: &[Point2]) -> Vec<f64> {
    let mut heading = (0..points.len())
        .map(|index| {
            if index == 0 {
                (points[1][1] - points[0][1]).atan2(points[1][0] - points[0][0])
            } else if index + 1 == points.len() {
                (points[index][1] - points[index - 1][1])
                    .atan2(points[index][0] - points[index - 1][0])
            } else {
                (points[index + 1][1] - points[index - 1][1])
                    .atan2(points[index + 1][0] - points[index - 1][0])
            }
        })
        .collect::<Vec<_>>();
    unwrap_angles(&mut heading);
    heading
}

fn unwrap_angles(values: &mut [f64]) {
    if values.is_empty() {
        return;
    }
    for index in 1..values.len() {
        let mut delta = values[index] - values[index - 1];
        while delta > std::f64::consts::PI {
            values[index] -= 2.0 * std::f64::consts::PI;
            delta -= 2.0 * std::f64::consts::PI;
        }
        while delta < -std::f64::consts::PI {
            values[index] += 2.0 * std::f64::consts::PI;
            delta += 2.0 * std::f64::consts::PI;
        }
    }
}

fn envelope_utilization(
    speed: &[f64],
    ax: &[f64],
    ay: &[f64],
    envelope: &AccelerationEnvelopeV1,
) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
    let mut cornering = Vec::with_capacity(speed.len());
    let mut longitudinal = Vec::with_capacity(speed.len());
    let mut combined = Vec::with_capacity(speed.len());
    for index in 0..speed.len() {
        let limits = envelope.limits(speed[index]);
        let ay_limit = if ay[index] >= 0.0 {
            limits.ay_left_max_mps2
        } else {
            limits.ay_right_max_mps2
        };
        let ax_limit = if ax[index] >= 0.0 {
            limits.ax_drive_max_mps2
        } else {
            limits.ax_brake_max_mps2
        };
        let lat = ay[index].abs() / ay_limit.max(1e-9);
        let long = ax[index].abs() / ax_limit.max(1e-9);
        cornering.push(lat);
        longitudinal.push(long);
        combined.push(
            (lat.powf(envelope.coupling_exponent) + long.powf(envelope.coupling_exponent))
                .powf(1.0 / envelope.coupling_exponent),
        );
    }
    (cornering, longitudinal, combined)
}

fn lerp_scalar(from: f64, to: f64, t: f64) -> f64 {
    from + (to - from) * t
}

fn lerp_point(from: Point2, to: Point2, t: f64) -> Point2 {
    [
        lerp_scalar(from[0], to[0], t),
        lerp_scalar(from[1], to[1], t),
    ]
}

fn normalize_point(vector: Point2, fallback: Point2) -> Point2 {
    let length = hypot(vector);

    if length <= 1e-9 {
        fallback
    } else {
        [vector[0] / length, vector[1] / length]
    }
}

fn hypot(vector: Point2) -> f64 {
    (vector[0] * vector[0] + vector[1] * vector[1]).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_profile() -> PointMassProfileV1 {
        PointMassProfileV1 {
            schema_version: PointMassProfileV1::SCHEMA_VERSION.to_owned(),
            profile_id: "test_point_mass".to_owned(),
            model_kind: PointMassProfileV1::MODEL_KIND.to_owned(),
            params: vec![("v_max_mps".to_owned(), 50.0.into())],
            metadata: Vec::new(),
        }
    }

    fn test_envelope() -> AccelerationEnvelopeV1 {
        AccelerationEnvelopeV1 {
            schema_version: AccelerationEnvelopeV1::SCHEMA_VERSION.to_owned(),
            envelope_id: "test_envelope".to_owned(),
            speed_mps: vec![0.0, 50.0],
            ax_drive_max_mps2: vec![10.0, 10.0],
            ax_brake_max_mps2: vec![10.0, 10.0],
            ay_left_max_mps2: vec![10.0, 10.0],
            ay_right_max_mps2: vec![10.0, 10.0],
            coupling_exponent: 2.0,
            metadata: Vec::new(),
        }
    }

    fn point_mass_open_smoke_boundaries(center: &[Point2], half_width_m: f64) -> (String, String) {
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

    fn point_mass_open_smoke_case(track_id: &str) -> Option<(usize, f64, Vec<Point2>)> {
        match track_id {
            "open_straight_lab_v1" => Some((
                20,
                4.0,
                (0..=30).map(|index| [index as f64 * 4.0, 0.0]).collect(),
            )),
            "open_s_bend_lab_v1" => Some((
                32,
                4.5,
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

    fn point_mass_open_smoke_request_json(
        track_id: &str,
        station_count: usize,
        center: Vec<Point2>,
        half_width_m: f64,
    ) -> String {
        let (left_boundary_xy_m, right_boundary_xy_m) =
            point_mass_open_smoke_boundaries(&center, half_width_m);
        format!(
            r#"{{
              "track_area": {{
                "schema_version": "TrackAreaContractV1",
                "track_id": "{track_id}",
                "units": "m",
                "trajectory_mode": "open",
                "left_boundary_xy_m": [{left_boundary_xy_m}],
                "right_boundary_xy_m": [{right_boundary_xy_m}],
                "metadata": {{}}
              }},
              "station_count": {station_count},
              "solve_options": {{
                "max_iter": 1200,
                "tol": 0.00001,
                "acceptable_tol": 0.00001,
                "acceptable_iter": 5,
                "ipopt_print_level": 0,
                "production_station_builder": "open_area_station_generator",
                "dense_count": 384,
                "target_spacing_max_adjacent_ratio": 1.35,
                "target_spacing_metric": "hybrid_area_centerline",
                "publish_geometry_mode": "section_interpolated"
              }},
              "point_mass_profile": {{
                "schema_version": "PointMassProfileV1",
                "profile_id": "point_open_smoke",
                "model_kind": "point_mass_envelope",
                "params": {{"v_max_mps": 34}},
                "metadata": {{}}
              }},
              "acceleration_envelope": {{
                "schema_version": "AccelerationEnvelopeV1",
                "envelope_id": "point_open_smoke_envelope",
                "speed_mps": [0, 34],
                "ax_drive_max_mps2": [6, 6],
                "ax_brake_max_mps2": [10, 10],
                "ay_left_max_mps2": [12, 12],
                "ay_right_max_mps2": [12, 12],
                "coupling_exponent": 2,
                "metadata": {{}}
              }}
            }}"#
        )
    }

    fn square_sections() -> SectionsTrackViewV1 {
        SectionsTrackViewV1 {
            schema_version: SectionsTrackViewV1::SCHEMA_VERSION.to_owned(),
            view_id: "test_square_sections".to_owned(),
            track_id: "test_square".to_owned(),
            station_s_m: vec![0.0, 10.0, 20.0, 30.0],
            centerline_xy_m: vec![[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]],
            left_boundary_xy_m: vec![[0.0, 1.0], [9.0, 0.0], [10.0, 9.0], [1.0, 10.0]],
            right_boundary_xy_m: vec![[0.0, -1.0], [11.0, 0.0], [10.0, 11.0], [-1.0, 10.0]],
            normals_xy: vec![[0.0, 1.0], [-1.0, 0.0], [0.0, -1.0], [1.0, 0.0]],
            width_left_m: vec![1.0; 4],
            width_right_m: vec![1.0; 4],
            section_dirs_xy: vec![[0.0, 1.0], [-1.0, 0.0], [0.0, -1.0], [1.0, 0.0]],
            quality_metrics: Vec::new(),
            metadata: Vec::new(),
        }
    }

    #[test]
    fn point_mass_intermediate_callback_reports_ipopt_iteration() {
        let sections = square_sections();
        let profile = test_profile();
        let envelope = test_envelope();
        let mut updates = Vec::new();

        {
            let mut callback = |update| updates.push(update);
            let mut nlp = PointMassNlp::new(
                &sections,
                &profile,
                &envelope,
                PointMassSolveOptions::default(),
            )
            .unwrap();
            nlp.set_progress_callback(&mut callback);
            let user_data = (&mut nlp) as *mut PointMassNlp as *mut std::ffi::c_void;

            let keep_running = unsafe {
                point_mass_intermediate_cb(
                    0, 7, 42.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, user_data,
                )
            };

            assert!(keep_running);
            assert_eq!(nlp.last_ipopt_iteration, Some(7));
        }

        assert_eq!(updates.len(), 1);
        match &updates[0] {
            PointMassProgressUpdate::OptimizerIteration {
                iteration,
                objective_value,
            } => {
                assert_eq!(*iteration, 7);
                assert!((*objective_value - 42.5).abs() < 1e-12);
            }
            PointMassProgressUpdate::Preview(_) => {
                panic!("expected optimizer iteration update")
            }
        }
    }

    fn open_straight_sections() -> SectionsTrackViewV1 {
        SectionsTrackViewV1 {
            schema_version: SectionsTrackViewV1::SCHEMA_VERSION.to_owned(),
            view_id: "test_open_straight_sections".to_owned(),
            track_id: "test_open_straight".to_owned(),
            station_s_m: vec![0.0, 10.0, 20.0, 30.0],
            centerline_xy_m: vec![[0.0, 0.0], [10.0, 0.0], [20.0, 0.0], [30.0, 0.0]],
            left_boundary_xy_m: vec![[0.0, 2.0], [10.0, 2.0], [20.0, 2.0], [30.0, 2.0]],
            right_boundary_xy_m: vec![[0.0, -2.0], [10.0, -2.0], [20.0, -2.0], [30.0, -2.0]],
            normals_xy: vec![[0.0, 1.0]; 4],
            width_left_m: vec![2.0; 4],
            width_right_m: vec![2.0; 4],
            section_dirs_xy: vec![[0.0, 1.0]; 4],
            quality_metrics: Vec::new(),
            metadata: vec![("trajectory_mode".to_owned(), "open".into())],
        }
    }

    #[test]
    fn open_topology_omits_final_to_first_point_mass_dynamics() {
        let nlp = PointMassNlp::new(
            &open_straight_sections(),
            &test_profile(),
            &test_envelope(),
            PointMassSolveOptions::default(),
        )
        .unwrap();

        assert_eq!(nlp.topology, TrackTopology::Open);
        assert_eq!(nlp.interval_count, nlp.count - 1);
        assert!(!nlp.rows.iter().any(|row| matches!(
            row,
            ConstraintRow::DynPosX(index)
                | ConstraintRow::DynPosY(index)
                | ConstraintRow::DynVelX(index)
                | ConstraintRow::DynVelY(index)
                | ConstraintRow::Env { station: index, .. }
                if *index == nlp.count - 1
        )));
    }

    #[test]
    fn open_point_mass_series_preserves_open_arclength_and_runtime() {
        let nlp = PointMassNlp::new(
            &open_straight_sections(),
            &test_profile(),
            &test_envelope(),
            PointMassSolveOptions::default(),
        )
        .unwrap();
        let mut x = nlp.initial_solution();
        for index in 0..nlp.interval_count {
            x[idx_dt(nlp.count, index)] = 1.0;
        }
        x[idx_dt(nlp.count, nlp.count - 1)] = 100.0;

        let (series, open_run_time_s) = nlp.to_series(&x);

        assert_eq!(series.s_m.first().copied(), Some(0.0));
        assert!((series.s_m.last().copied().unwrap() - 30.0).abs() < 1e-9);
        assert_eq!(open_run_time_s, 3.0);
        assert!(
            (series.x_m.last().copied().unwrap() - series.x_m.first().copied().unwrap()).abs()
                > 25.0
        );
    }

    #[test]
    #[ignore = "requires local Ipopt DLL and runs multiple open point-mass solves"]
    fn point_mass_solves_open_fixture_smokes() {
        for track_id in [
            "open_straight_lab_v1",
            "open_s_bend_lab_v1",
            "open_chicane_lab_v1",
        ] {
            let (station_count, half_width_m, center) = point_mass_open_smoke_case(track_id)
                .unwrap_or_else(|| panic!("unknown open point-mass smoke fixture {track_id}"));
            let request =
                point_mass_open_smoke_request_json(track_id, station_count, center, half_width_m);
            let response = crate::solver_api::solve_point_mass_json(&request)
                .unwrap_or_else(|error| panic!("{track_id} point-mass smoke failed: {error}"));
            let value = crate::json::parse_json_str(&response).unwrap();
            assert_eq!(
                value.get("status").and_then(crate::json::JsonValue::as_str),
                Some("Solve_Succeeded")
            );
            assert!(value
                .get("open_run_time_s")
                .and_then(crate::json::JsonValue::as_f64)
                .is_some_and(|value| value.is_finite() && value > 0.0));
            assert_eq!(
                value
                    .get("visualization")
                    .and_then(|value| value.get("display_trajectory"))
                    .and_then(|value| value.get("closed")),
                Some(&crate::json::JsonValue::Bool(false))
            );
            let trajectory = value.get("trajectory_result").unwrap();
            let x_m = trajectory
                .get("x_m")
                .and_then(crate::json::JsonValue::as_array)
                .unwrap();
            let first_x = x_m
                .first()
                .and_then(crate::json::JsonValue::as_f64)
                .unwrap();
            let last_x = x_m.last().and_then(crate::json::JsonValue::as_f64).unwrap();
            assert!((last_x - first_x).abs() > 100.0);
        }
    }

    #[test]
    fn section_interpolated_publish_mode_keeps_output_in_station_corridor() {
        let options = PointMassSolveOptions {
            publish_geometry_mode: PublishGeometryMode::SectionInterpolated,
            output_sample_count: Some(16),
            ..Default::default()
        };
        let nlp = PointMassNlp::new(
            &square_sections(),
            &test_profile(),
            &test_envelope(),
            options,
        )
        .unwrap();
        let mut x = vec![0.0; nlp.variable_count()];

        for index in 0..nlp.count {
            x[idx_n(nlp.count, index)] = 0.0;
            x[idx_vx(nlp.count, index)] = 50.0;
            x[idx_vy(nlp.count, index)] = 50.0;
            x[idx_dt(nlp.count, index)] = 1.0;
        }

        let (series, _lap_time_s) = nlp.to_series(&x);

        assert_eq!(series.x_m.len(), 16);
        for (x_m, y_m) in series.x_m.iter().zip(series.y_m.iter()) {
            assert!(
                (-1.0..=11.0).contains(x_m) && (-1.0..=11.0).contains(y_m),
                "section-interpolated point left the section corridor bounds: ({x_m}, {y_m})"
            );
        }
    }

    #[test]
    fn point_publish_acceleration_uses_model_lateral_accel() {
        for mode in [
            PublishGeometryMode::StationPolyline,
            PublishGeometryMode::SectionInterpolated,
            PublishGeometryMode::DenseDynamics,
        ] {
            let options = PointMassSolveOptions {
                publish_geometry_mode: mode,
                output_sample_count: Some(16),
                ..Default::default()
            };
            let nlp = PointMassNlp::new(
                &square_sections(),
                &test_profile(),
                &test_envelope(),
                options,
            )
            .unwrap();
            let mut x = vec![0.0; nlp.variable_count()];

            for index in 0..nlp.count {
                x[idx_n(nlp.count, index)] = 0.0;
                x[idx_vx(nlp.count, index)] = 10.0;
                x[idx_vy(nlp.count, index)] = 0.0;
                x[idx_ax(nlp.count, index)] = 0.0;
                x[idx_ay(nlp.count, index)] = 0.0;
                x[idx_dt(nlp.count, index)] = 1.0;
            }

            let (series, _lap_time_s) = nlp.to_series(&x);
            for (ay, kappa) in series.ay_mps2.iter().zip(series.kappa_1pm.iter()) {
                assert!(
                    ay.abs() < 1.0e-9,
                    "mode {mode:?} published geometric ay instead of model ay: {ay}"
                );
                assert!(
                    kappa.abs() < 1.0e-9,
                    "mode {mode:?} published geometric kappa instead of model kappa: {kappa}"
                );
            }
        }
    }

    #[test]
    fn point_width_opt_shrinks_station_offset_bounds() {
        let options = PointMassSolveOptions {
            width_opt_m: 2.0,
            ..Default::default()
        };
        let nlp = PointMassNlp::new(
            &square_sections(),
            &test_profile(),
            &test_envelope(),
            options,
        )
        .unwrap();
        let (lower, upper) = nlp.variable_bounds();

        for index in 0..nlp.count {
            assert!(
                lower[idx_n(nlp.count, index)] >= -1.0e-3,
                "left side should be shrunk by half width"
            );
            assert!(
                upper[idx_n(nlp.count, index)] <= 1.0e-3,
                "right side should be shrunk by half width"
            );
        }
    }

    #[test]
    fn default_output_sample_count_is_station_relative() {
        let options = PointMassSolveOptions {
            publish_geometry_mode: PublishGeometryMode::SectionInterpolated,
            ..Default::default()
        };
        assert_eq!(options.output_sample_count, None);

        let nlp = PointMassNlp::new(
            &square_sections(),
            &test_profile(),
            &test_envelope(),
            options,
        )
        .unwrap();

        assert_eq!(
            nlp.output_sample_count(),
            square_sections().station_s_m.len() * POINT_MASS_OUTPUT_SAMPLES_PER_STATION
        );
    }
}
