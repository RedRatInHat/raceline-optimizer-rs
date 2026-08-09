use crate::contracts::Point2;

const EPSILON: f64 = 1e-12;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ReferenceSample {
    pub s_m: f64,
    pub xy_m: Point2,
    pub tangent: Point2,
    pub normal: Point2,
    pub heading_rad: f64,
    pub kappa_1pm: f64,
    pub kappa_prime_1pm2: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DenseFrenetInput {
    pub n_m: f64,
    pub dn_ds: f64,
    pub d2n_ds2: f64,
    pub v_mps: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DenseFrenetSample {
    pub s_m: f64,
    pub x_m: f64,
    pub y_m: f64,
    pub n_m: f64,
    pub dn_ds: f64,
    pub d2n_ds2: f64,
    pub v_mps: f64,
    pub heading_geo_rad: f64,
    pub kappa_geo_1pm: f64,
    pub ay_geo_mps2: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DenseSectionFrameReference {
    pub s_m: f64,
    pub centerline_xy_m: Point2,
    pub ref_tangent: Point2,
    pub ref_left_normal: Point2,
    pub ref_kappa_1pm: f64,
    pub section_dir: Point2,
    pub section_dir_derivative: Point2,
    pub section_dir_second_derivative: Point2,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DenseSectionFrameGeometry {
    pub s_m: f64,
    pub centerline_xy_m: Point2,
    pub centerline_ds: Point2,
    pub centerline_d2s: Point2,
    pub section_dir: Point2,
    pub section_dir_ds: Point2,
    pub section_dir_d2s: Point2,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DenseSectionFrameInput {
    pub n_m: f64,
    pub dn_ds: f64,
    pub d2n_ds2: f64,
    pub v_mps: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DenseSectionFrameSample {
    pub s_m: f64,
    pub x_m: f64,
    pub y_m: f64,
    pub n_m: f64,
    pub dn_ds: f64,
    pub d2n_ds2: f64,
    pub v_mps: f64,
    pub heading_geo_rad: f64,
    pub kappa_geo_1pm: f64,
    pub ay_geo_mps2: f64,
    pub path_ds: Point2,
    pub path_d2s: Point2,
}

#[derive(Clone, Copy, Debug)]
pub struct ReferencePathView<'a> {
    pub station_s_m: &'a [f64],
    pub centerline_xy_m: &'a [Point2],
    pub tangent_xy: &'a [Point2],
    pub normal_xy: &'a [Point2],
    pub kappa_1pm: &'a [f64],
    pub closed: bool,
}

#[derive(Clone, Copy, Debug)]
pub struct DenseSectionFrameHermiteSampler<'a> {
    pub station_s_m: &'a [f64],
    pub centerline_xy_m: &'a [Point2],
    pub tangent_xy: &'a [Point2],
    pub section_dir_xy: &'a [Point2],
    pub section_dir_derivative_xy: &'a [Point2],
    pub closed: bool,
}

impl DenseSectionFrameHermiteSampler<'_> {
    pub fn sample_at_interval_tau(
        self,
        interval: usize,
        tau: f64,
    ) -> Option<DenseSectionFrameGeometry> {
        let count = self.centerline_xy_m.len();
        if count == 0
            || self.station_s_m.len() < count
            || self.tangent_xy.len() < count
            || self.section_dir_xy.len() < count
            || self.section_dir_derivative_xy.len() < count
        {
            return None;
        }
        let interval = interval.min(count.saturating_sub(1));
        let next = next_index(count, interval, self.closed);
        let ds_m = self.interval_ds_m(interval)?;
        let tau = tau.clamp(0.0, 1.0);
        let s_m = self.station_s_m[interval] + tau * ds_m;
        let (centerline_xy_m, centerline_ds, centerline_d2s) = cubic_hermite_point(
            self.centerline_xy_m[interval],
            self.tangent_xy[interval],
            self.centerline_xy_m[next],
            self.tangent_xy[next],
            ds_m,
            tau,
        );
        let (section_dir, section_dir_ds, section_dir_d2s) = cubic_hermite_point(
            self.section_dir_xy[interval],
            self.section_dir_derivative_xy[interval],
            self.section_dir_xy[next],
            self.section_dir_derivative_xy[next],
            ds_m,
            tau,
        );

        Some(DenseSectionFrameGeometry {
            s_m,
            centerline_xy_m,
            centerline_ds,
            centerline_d2s,
            section_dir,
            section_dir_ds,
            section_dir_d2s,
        })
    }

    pub fn interval_ds_m(self, interval: usize) -> Option<f64> {
        let count = self.centerline_xy_m.len();
        if count == 0 || self.station_s_m.len() < count {
            return None;
        }
        let interval = interval.min(count.saturating_sub(1));
        if let Some(next_s) = self.station_s_m.get(interval + 1).copied() {
            return Some((next_s - self.station_s_m[interval]).abs().max(EPSILON));
        }
        if self.closed {
            return median_positive_station_step_m(self.station_s_m)
                .map(|step| step.abs().max(EPSILON));
        }
        None
    }
}

impl ReferencePathView<'_> {
    pub fn sample_at_interval_tau(self, interval: usize, tau: f64) -> Option<ReferenceSample> {
        let count = self.centerline_xy_m.len();
        if count == 0
            || self.station_s_m.len() < count
            || self.tangent_xy.len() < count
            || self.normal_xy.len() < count
            || self.kappa_1pm.len() < count
        {
            return None;
        }
        let interval = interval.min(count.saturating_sub(1));
        let next = next_index(count, interval, self.closed);
        let tau = tau.clamp(0.0, 1.0);
        let ds_m = self.interval_ds_m(interval)?;
        let s_m = self.station_s_m[interval] + tau * ds_m;
        let kappa_prime_1pm2 = self.interval_kappa_prime_1pm2(interval)?;

        let tangent = normalize_or(
            lerp_point(self.tangent_xy[interval], self.tangent_xy[next], tau),
            self.tangent_xy[interval],
        );
        let normal = normalize_or(
            lerp_point(self.normal_xy[interval], self.normal_xy[next], tau),
            [-tangent[1], tangent[0]],
        );
        let heading_rad = tangent[1].atan2(tangent[0]);

        Some(ReferenceSample {
            s_m,
            xy_m: lerp_point(
                self.centerline_xy_m[interval],
                self.centerline_xy_m[next],
                tau,
            ),
            tangent,
            normal,
            heading_rad,
            kappa_1pm: lerp(self.kappa_1pm[interval], self.kappa_1pm[next], tau),
            kappa_prime_1pm2,
        })
    }

    pub fn interval_ds_m(self, interval: usize) -> Option<f64> {
        let count = self.centerline_xy_m.len();
        if count == 0 || self.station_s_m.len() < count {
            return None;
        }
        let interval = interval.min(count.saturating_sub(1));
        if let Some(next_s) = self.station_s_m.get(interval + 1).copied() {
            return Some((next_s - self.station_s_m[interval]).abs().max(EPSILON));
        }
        if self.closed {
            return median_positive_station_step_m(self.station_s_m)
                .map(|step| step.abs().max(EPSILON));
        }
        None
    }

    fn interval_kappa_prime_1pm2(self, interval: usize) -> Option<f64> {
        let count = self.centerline_xy_m.len();
        if count == 0 || self.kappa_1pm.len() < count {
            return None;
        }
        let interval = interval.min(count.saturating_sub(1));
        let next = next_index(count, interval, self.closed);
        let ds_m = self.interval_ds_m(interval)?;
        Some((self.kappa_1pm[next] - self.kappa_1pm[interval]) / ds_m)
    }
}

pub fn build_dense_frenet_sample(
    reference: ReferenceSample,
    input: DenseFrenetInput,
) -> DenseFrenetSample {
    let tangent = normalize_or(
        reference.tangent,
        [reference.heading_rad.cos(), reference.heading_rad.sin()],
    );
    let normal = normalize_or(reference.normal, [-tangent[1], tangent[0]]);
    let xy_m = add(reference.xy_m, scale(normal, input.n_m));

    let a = 1.0 - reference.kappa_1pm * input.n_m;
    let path_ds = add(scale(tangent, a), scale(normal, input.dn_ds));
    let path_d2s = add(
        scale(
            tangent,
            -reference.kappa_prime_1pm2 * input.n_m - 2.0 * reference.kappa_1pm * input.dn_ds,
        ),
        scale(normal, reference.kappa_1pm * a + input.d2n_ds2),
    );
    let heading_geo_rad = path_ds[1].atan2(path_ds[0]);
    let kappa_geo_1pm = curvature_from_derivatives(path_ds, path_d2s);

    DenseFrenetSample {
        s_m: reference.s_m,
        x_m: xy_m[0],
        y_m: xy_m[1],
        n_m: input.n_m,
        dn_ds: input.dn_ds,
        d2n_ds2: input.d2n_ds2,
        v_mps: input.v_mps,
        heading_geo_rad,
        kappa_geo_1pm,
        ay_geo_mps2: input.v_mps * input.v_mps * kappa_geo_1pm,
    }
}

pub fn build_dense_section_frame_sample(
    reference: DenseSectionFrameReference,
    input: DenseSectionFrameInput,
) -> DenseSectionFrameSample {
    let tangent = normalize_or(reference.ref_tangent, [1.0, 0.0]);
    let ref_left_normal = normalize_or(reference.ref_left_normal, [-tangent[1], tangent[0]]);
    let section_dir = normalize_or(reference.section_dir, scale(ref_left_normal, -1.0));
    let path_normal = scale(section_dir, -1.0);
    let path_normal_ds = scale(reference.section_dir_derivative, -1.0);
    let path_normal_d2s = scale(reference.section_dir_second_derivative, -1.0);

    let xy_m = add(reference.centerline_xy_m, scale(path_normal, input.n_m));
    let centerline_d2s = scale(ref_left_normal, reference.ref_kappa_1pm);
    let path_ds = add(
        tangent,
        add(
            scale(path_normal, input.dn_ds),
            scale(path_normal_ds, input.n_m),
        ),
    );
    let path_d2s = add(
        centerline_d2s,
        add(
            scale(path_normal, input.d2n_ds2),
            add(
                scale(path_normal_ds, 2.0 * input.dn_ds),
                scale(path_normal_d2s, input.n_m),
            ),
        ),
    );
    let heading_geo_rad = path_ds[1].atan2(path_ds[0]);
    let kappa_geo_1pm = curvature_from_derivatives(path_ds, path_d2s);

    DenseSectionFrameSample {
        s_m: reference.s_m,
        x_m: xy_m[0],
        y_m: xy_m[1],
        n_m: input.n_m,
        dn_ds: input.dn_ds,
        d2n_ds2: input.d2n_ds2,
        v_mps: input.v_mps,
        heading_geo_rad,
        kappa_geo_1pm,
        ay_geo_mps2: input.v_mps * input.v_mps * kappa_geo_1pm,
        path_ds,
        path_d2s,
    }
}

pub fn build_dense_section_frame_sample_from_geometry(
    geometry: DenseSectionFrameGeometry,
    input: DenseSectionFrameInput,
) -> DenseSectionFrameSample {
    let path_normal = scale(geometry.section_dir, -1.0);
    let path_normal_ds = scale(geometry.section_dir_ds, -1.0);
    let path_normal_d2s = scale(geometry.section_dir_d2s, -1.0);

    let xy_m = add(geometry.centerline_xy_m, scale(path_normal, input.n_m));
    let path_ds = add(
        geometry.centerline_ds,
        add(
            scale(path_normal, input.dn_ds),
            scale(path_normal_ds, input.n_m),
        ),
    );
    let path_d2s = add(
        geometry.centerline_d2s,
        add(
            scale(path_normal, input.d2n_ds2),
            add(
                scale(path_normal_ds, 2.0 * input.dn_ds),
                scale(path_normal_d2s, input.n_m),
            ),
        ),
    );
    let heading_geo_rad = path_ds[1].atan2(path_ds[0]);
    let kappa_geo_1pm = curvature_from_derivatives(path_ds, path_d2s);

    DenseSectionFrameSample {
        s_m: geometry.s_m,
        x_m: xy_m[0],
        y_m: xy_m[1],
        n_m: input.n_m,
        dn_ds: input.dn_ds,
        d2n_ds2: input.d2n_ds2,
        v_mps: input.v_mps,
        heading_geo_rad,
        kappa_geo_1pm,
        ay_geo_mps2: input.v_mps * input.v_mps * kappa_geo_1pm,
        path_ds,
        path_d2s,
    }
}

pub fn curvature_from_derivatives(path_ds: Point2, path_d2s: Point2) -> f64 {
    let speed_sq = dot(path_ds, path_ds);
    if speed_sq <= EPSILON {
        return 0.0;
    }
    cross(path_ds, path_d2s) / speed_sq.powf(1.5)
}

pub fn cubic_hermite_point(
    y0: Point2,
    dy0_ds: Point2,
    y1: Point2,
    dy1_ds: Point2,
    ds_m: f64,
    tau: f64,
) -> (Point2, Point2, Point2) {
    let (x, dx, d2x) = cubic_hermite_scalar(y0[0], dy0_ds[0], y1[0], dy1_ds[0], ds_m, tau);
    let (y, dy, d2y) = cubic_hermite_scalar(y0[1], dy0_ds[1], y1[1], dy1_ds[1], ds_m, tau);
    ([x, y], [dx, dy], [d2x, d2y])
}

pub fn cubic_hermite_scalar(
    y0: f64,
    dy0_ds: f64,
    y1: f64,
    dy1_ds: f64,
    ds_m: f64,
    tau: f64,
) -> (f64, f64, f64) {
    let t = tau.clamp(0.0, 1.0);
    let t2 = t * t;
    let t3 = t2 * t;
    let h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
    let h10 = t3 - 2.0 * t2 + t;
    let h01 = -2.0 * t3 + 3.0 * t2;
    let h11 = t3 - t2;
    let dh00 = 6.0 * t2 - 6.0 * t;
    let dh10 = 3.0 * t2 - 4.0 * t + 1.0;
    let dh01 = -6.0 * t2 + 6.0 * t;
    let dh11 = 3.0 * t2 - 2.0 * t;
    let d2h00 = 12.0 * t - 6.0;
    let d2h10 = 6.0 * t - 4.0;
    let d2h01 = -12.0 * t + 6.0;
    let d2h11 = 6.0 * t - 2.0;
    let ds = ds_m.max(EPSILON);
    let value = h00 * y0 + h10 * ds * dy0_ds + h01 * y1 + h11 * ds * dy1_ds;
    let d_tau = dh00 * y0 + dh10 * ds * dy0_ds + dh01 * y1 + dh11 * ds * dy1_ds;
    let d2_tau = d2h00 * y0 + d2h10 * ds * dy0_ds + d2h01 * y1 + d2h11 * ds * dy1_ds;
    (value, d_tau / ds, d2_tau / (ds * ds))
}

fn add(left: Point2, right: Point2) -> Point2 {
    [left[0] + right[0], left[1] + right[1]]
}

#[cfg(test)]
fn sub(left: Point2, right: Point2) -> Point2 {
    [left[0] - right[0], left[1] - right[1]]
}

fn lerp(left: f64, right: f64, tau: f64) -> f64 {
    left + (right - left) * tau
}

fn lerp_point(left: Point2, right: Point2, tau: f64) -> Point2 {
    [lerp(left[0], right[0], tau), lerp(left[1], right[1], tau)]
}

fn scale(point: Point2, scalar: f64) -> Point2 {
    [point[0] * scalar, point[1] * scalar]
}

fn dot(left: Point2, right: Point2) -> f64 {
    left[0] * right[0] + left[1] * right[1]
}

fn cross(left: Point2, right: Point2) -> f64 {
    left[0] * right[1] - left[1] * right[0]
}

fn normalize_or(vector: Point2, fallback: Point2) -> Point2 {
    let norm = dot(vector, vector).sqrt();
    if norm > EPSILON {
        [vector[0] / norm, vector[1] / norm]
    } else {
        let fallback_norm = dot(fallback, fallback).sqrt();
        if fallback_norm > EPSILON {
            [fallback[0] / fallback_norm, fallback[1] / fallback_norm]
        } else {
            [1.0, 0.0]
        }
    }
}

fn next_index(count: usize, interval: usize, closed: bool) -> usize {
    if count == 0 {
        0
    } else if closed {
        (interval + 1) % count
    } else {
        (interval + 1).min(count - 1)
    }
}

fn median_positive_station_step_m(station_s_m: &[f64]) -> Option<f64> {
    let mut deltas = station_s_m
        .windows(2)
        .map(|pair| (pair[1] - pair[0]).abs())
        .filter(|delta| *delta > EPSILON && delta.is_finite())
        .collect::<Vec<_>>();
    if deltas.is_empty() {
        return None;
    }
    deltas.sort_by(|left, right| left.total_cmp(right));
    Some(deltas[deltas.len() / 2])
}

#[cfg(test)]
mod tests {
    use super::*;

    const TOLERANCE: f64 = 1e-9;

    #[test]
    fn cubic_hermite_scalar_derivatives_match_endpoint_and_midpoint_basis() {
        let ds = 10.0;
        let (_, d0, _) = cubic_hermite_scalar(0.0, 2.0, 6.0, -1.0, ds, 0.0);
        let (_, d1, _) = cubic_hermite_scalar(0.0, 2.0, 6.0, -1.0, ds, 1.0);
        assert!((d0 - 2.0).abs() < TOLERANCE);
        assert!((d1 + 1.0).abs() < TOLERANCE);

        let (_, midpoint_d, _) = cubic_hermite_scalar(0.0, 0.0, 0.0, 1.0, ds, 0.5);
        assert!((midpoint_d + 0.25).abs() < TOLERANCE);
    }

    #[test]
    fn cubic_hermite_point_derivatives_match_finite_difference() {
        let ds = 8.0;
        let tau = 0.37;
        let eps = 1e-5;
        let (_, analytic_ds, analytic_d2s) =
            cubic_hermite_point([1.0, -2.0], [0.8, 0.2], [5.0, -1.0], [0.1, 0.9], ds, tau);
        let (left_xy, left_ds, _) = cubic_hermite_point(
            [1.0, -2.0],
            [0.8, 0.2],
            [5.0, -1.0],
            [0.1, 0.9],
            ds,
            tau - eps,
        );
        let (right_xy, right_ds, _) = cubic_hermite_point(
            [1.0, -2.0],
            [0.8, 0.2],
            [5.0, -1.0],
            [0.1, 0.9],
            ds,
            tau + eps,
        );
        let fd_ds = scale(sub(right_xy, left_xy), 1.0 / (2.0 * eps * ds));
        let fd_d2s = scale(sub(right_ds, left_ds), 1.0 / (2.0 * eps * ds));
        assert_point_close(fd_ds, analytic_ds, 1e-10);
        assert_point_close(fd_d2s, analytic_d2s, 1e-10);
    }

    #[test]
    fn dense_section_frame_hermite_sampler_returns_coherent_derivatives() {
        let sampler = DenseSectionFrameHermiteSampler {
            station_s_m: &[0.0, 8.0],
            centerline_xy_m: &[[0.0, 0.0], [7.0, 1.0]],
            tangent_xy: &[[1.0, 0.0], [0.8, 0.4]],
            section_dir_xy: &[[0.0, -1.0], [0.2, -0.9]],
            section_dir_derivative_xy: &[[0.0, 0.02], [0.03, -0.01]],
            closed: false,
        };
        let tau = 0.42;
        let eps = 1e-5;
        let mid = sampler.sample_at_interval_tau(0, tau).unwrap();
        let left = sampler.sample_at_interval_tau(0, tau - eps).unwrap();
        let right = sampler.sample_at_interval_tau(0, tau + eps).unwrap();
        let denom = 2.0 * eps * sampler.interval_ds_m(0).unwrap();
        let fd_center_ds = scale(
            sub(right.centerline_xy_m, left.centerline_xy_m),
            1.0 / denom,
        );
        let fd_center_d2s = scale(sub(right.centerline_ds, left.centerline_ds), 1.0 / denom);
        let fd_section_ds = scale(sub(right.section_dir, left.section_dir), 1.0 / denom);
        let fd_section_d2s = scale(sub(right.section_dir_ds, left.section_dir_ds), 1.0 / denom);

        assert_point_close(fd_center_ds, mid.centerline_ds, 1e-10);
        assert_point_close(fd_center_d2s, mid.centerline_d2s, 1e-10);
        assert_point_close(fd_section_ds, mid.section_dir_ds, 1e-10);
        assert_point_close(fd_section_d2s, mid.section_dir_d2s, 1e-10);
    }

    #[test]
    fn dense_section_frame_geometry_builder_matches_finite_difference_path() {
        let sampler = DenseSectionFrameHermiteSampler {
            station_s_m: &[0.0, 8.0],
            centerline_xy_m: &[[0.0, 0.0], [7.0, 1.0]],
            tangent_xy: &[[1.0, 0.0], [0.8, 0.4]],
            section_dir_xy: &[[0.0, -1.0], [0.2, -0.9]],
            section_dir_derivative_xy: &[[0.0, 0.02], [0.03, -0.01]],
            closed: false,
        };
        let tau = 0.45;
        let eps = 1e-5;
        let ds = sampler.interval_ds_m(0).unwrap();
        let input_at = |tau: f64| {
            let n = 1.3 + 0.2 * tau - 0.05 * tau * tau;
            let dn_ds = (0.2 - 0.1 * tau) / ds;
            let d2n_ds2 = -0.1 / (ds * ds);
            DenseSectionFrameInput {
                n_m: n,
                dn_ds,
                d2n_ds2,
                v_mps: 11.0,
            }
        };
        let left = build_dense_section_frame_sample_from_geometry(
            sampler.sample_at_interval_tau(0, tau - eps).unwrap(),
            input_at(tau - eps),
        );
        let mid = build_dense_section_frame_sample_from_geometry(
            sampler.sample_at_interval_tau(0, tau).unwrap(),
            input_at(tau),
        );
        let right = build_dense_section_frame_sample_from_geometry(
            sampler.sample_at_interval_tau(0, tau + eps).unwrap(),
            input_at(tau + eps),
        );
        let fd_heading = (right.y_m - left.y_m).atan2(right.x_m - left.x_m);
        assert!((fd_heading - mid.heading_geo_rad).abs() < 1e-7);
    }

    #[test]
    fn straight_reference_zero_offset_has_zero_curvature_and_ay() {
        let sample = build_dense_frenet_sample(
            ReferenceSample {
                s_m: 12.0,
                xy_m: [12.0, 3.0],
                tangent: [1.0, 0.0],
                normal: [0.0, 1.0],
                heading_rad: 0.0,
                kappa_1pm: 0.0,
                kappa_prime_1pm2: 0.0,
            },
            DenseFrenetInput {
                n_m: 0.0,
                dn_ds: 0.0,
                d2n_ds2: 0.0,
                v_mps: 30.0,
            },
        );

        assert!((sample.x_m - 12.0).abs() < TOLERANCE);
        assert!((sample.y_m - 3.0).abs() < TOLERANCE);
        assert!(sample.kappa_geo_1pm.abs() < TOLERANCE);
        assert!(sample.ay_geo_mps2.abs() < TOLERANCE);
    }

    #[test]
    fn circle_reference_zero_offset_matches_reference_curvature() {
        let radius = 50.0;
        let kappa = 1.0 / radius;
        let sample = build_dense_frenet_sample(
            ReferenceSample {
                s_m: 0.0,
                xy_m: [radius, 0.0],
                tangent: [0.0, 1.0],
                normal: [-1.0, 0.0],
                heading_rad: std::f64::consts::FRAC_PI_2,
                kappa_1pm: kappa,
                kappa_prime_1pm2: 0.0,
            },
            DenseFrenetInput {
                n_m: 0.0,
                dn_ds: 0.0,
                d2n_ds2: 0.0,
                v_mps: 10.0,
            },
        );

        assert!((sample.kappa_geo_1pm - kappa).abs() < TOLERANCE);
        assert!((sample.ay_geo_mps2 - 100.0 * kappa).abs() < TOLERANCE);
    }

    #[test]
    fn circle_constant_offset_matches_actual_normal_convention() {
        let radius = 50.0;
        let offset = 5.0;
        let kappa = 1.0 / radius;
        let sample = build_dense_frenet_sample(
            ReferenceSample {
                s_m: 0.0,
                xy_m: [radius, 0.0],
                tangent: [0.0, 1.0],
                normal: [-1.0, 0.0],
                heading_rad: std::f64::consts::FRAC_PI_2,
                kappa_1pm: kappa,
                kappa_prime_1pm2: 0.0,
            },
            DenseFrenetInput {
                n_m: offset,
                dn_ds: 0.0,
                d2n_ds2: 0.0,
                v_mps: 1.0,
            },
        );

        let expected = 1.0 / (radius - offset);
        assert!((sample.x_m - (radius - offset)).abs() < TOLERANCE);
        assert!((sample.kappa_geo_1pm - expected).abs() < TOLERANCE);
    }

    #[test]
    fn sinusoidal_offset_curvature_matches_finite_difference_shape() {
        let amplitude = 1.5;
        let wave = 0.07;
        let s: f64 = 11.0;
        let n = amplitude * (wave * s).sin();
        let dn = amplitude * wave * (wave * s).cos();
        let d2n = -amplitude * wave * wave * (wave * s).sin();
        let analytic = build_dense_frenet_sample(
            ReferenceSample {
                s_m: s,
                xy_m: [s, 0.0],
                tangent: [1.0, 0.0],
                normal: [0.0, 1.0],
                heading_rad: 0.0,
                kappa_1pm: 0.0,
                kappa_prime_1pm2: 0.0,
            },
            DenseFrenetInput {
                n_m: n,
                dn_ds: dn,
                d2n_ds2: d2n,
                v_mps: 1.0,
            },
        );

        let expected = d2n / (1.0 + dn * dn).powf(1.5);
        assert!((analytic.kappa_geo_1pm - expected).abs() < TOLERANCE);
    }

    #[test]
    fn section_frame_straight_zero_offset_has_zero_curvature_and_ay() {
        let sample = build_dense_section_frame_sample(
            DenseSectionFrameReference {
                s_m: 4.0,
                centerline_xy_m: [4.0, -2.0],
                ref_tangent: [1.0, 0.0],
                ref_left_normal: [0.0, 1.0],
                ref_kappa_1pm: 0.0,
                section_dir: [0.0, -1.0],
                section_dir_derivative: [0.0, 0.0],
                section_dir_second_derivative: [0.0, 0.0],
            },
            DenseSectionFrameInput {
                n_m: 0.0,
                dn_ds: 0.0,
                d2n_ds2: 0.0,
                v_mps: 20.0,
            },
        );

        assert!((sample.x_m - 4.0).abs() < TOLERANCE);
        assert!((sample.y_m + 2.0).abs() < TOLERANCE);
        assert!(sample.kappa_geo_1pm.abs() < TOLERANCE);
        assert!(sample.ay_geo_mps2.abs() < TOLERANCE);
    }

    #[test]
    fn section_frame_straight_linear_offset_has_heading_but_no_curvature() {
        let dn = 0.125;
        let sample = build_dense_section_frame_sample(
            DenseSectionFrameReference {
                s_m: 8.0,
                centerline_xy_m: [8.0, 0.0],
                ref_tangent: [1.0, 0.0],
                ref_left_normal: [0.0, 1.0],
                ref_kappa_1pm: 0.0,
                section_dir: [0.0, -1.0],
                section_dir_derivative: [0.0, 0.0],
                section_dir_second_derivative: [0.0, 0.0],
            },
            DenseSectionFrameInput {
                n_m: 2.0,
                dn_ds: dn,
                d2n_ds2: 0.0,
                v_mps: 10.0,
            },
        );

        assert!((sample.y_m - 2.0).abs() < TOLERANCE);
        assert!((sample.heading_geo_rad - dn.atan()).abs() < TOLERANCE);
        assert!(sample.kappa_geo_1pm.abs() < TOLERANCE);
    }

    #[test]
    fn section_frame_straight_sinusoidal_offset_matches_analytic_curvature() {
        let amplitude = 1.2;
        let wave = 0.09;
        let s: f64 = 7.0;
        let n = amplitude * (wave * s).sin();
        let dn = amplitude * wave * (wave * s).cos();
        let d2n = -amplitude * wave * wave * (wave * s).sin();
        let sample = build_dense_section_frame_sample(
            DenseSectionFrameReference {
                s_m: s,
                centerline_xy_m: [s, 0.0],
                ref_tangent: [1.0, 0.0],
                ref_left_normal: [0.0, 1.0],
                ref_kappa_1pm: 0.0,
                section_dir: [0.0, -1.0],
                section_dir_derivative: [0.0, 0.0],
                section_dir_second_derivative: [0.0, 0.0],
            },
            DenseSectionFrameInput {
                n_m: n,
                dn_ds: dn,
                d2n_ds2: d2n,
                v_mps: 1.0,
            },
        );

        let expected = d2n / (1.0 + dn * dn).powf(1.5);
        assert!((sample.kappa_geo_1pm - expected).abs() < TOLERANCE);
    }

    #[test]
    fn section_frame_matches_frenet_when_section_dir_is_negative_ref_normal() {
        let tangent = [0.6, 0.8];
        let normal = [-0.8, 0.6];
        let kappa = 0.03;
        let kappa_prime = -0.002;
        let input = DenseFrenetInput {
            n_m: 1.7,
            dn_ds: -0.06,
            d2n_ds2: 0.004,
            v_mps: 14.0,
        };
        let frenet = build_dense_frenet_sample(
            ReferenceSample {
                s_m: 13.0,
                xy_m: [5.0, -3.0],
                tangent,
                normal,
                heading_rad: tangent[1].atan2(tangent[0]),
                kappa_1pm: kappa,
                kappa_prime_1pm2: kappa_prime,
            },
            input,
        );
        let section = build_dense_section_frame_sample(
            DenseSectionFrameReference {
                s_m: 13.0,
                centerline_xy_m: [5.0, -3.0],
                ref_tangent: tangent,
                ref_left_normal: normal,
                ref_kappa_1pm: kappa,
                section_dir: scale(normal, -1.0),
                section_dir_derivative: scale(tangent, kappa),
                section_dir_second_derivative: add(
                    scale(tangent, kappa_prime),
                    scale(normal, kappa * kappa),
                ),
            },
            DenseSectionFrameInput {
                n_m: input.n_m,
                dn_ds: input.dn_ds,
                d2n_ds2: input.d2n_ds2,
                v_mps: input.v_mps,
            },
        );

        assert!((section.x_m - frenet.x_m).abs() < TOLERANCE);
        assert!((section.y_m - frenet.y_m).abs() < TOLERANCE);
        assert!((section.heading_geo_rad - frenet.heading_geo_rad).abs() < TOLERANCE);
        assert!((section.kappa_geo_1pm - frenet.kappa_geo_1pm).abs() < TOLERANCE);
        assert!((section.ay_geo_mps2 - frenet.ay_geo_mps2).abs() < TOLERANCE);
    }

    #[test]
    fn section_frame_boundary_is_c1_c2_clean_when_endpoint_data_matches() {
        let left = build_dense_section_frame_sample(
            DenseSectionFrameReference {
                s_m: 42.0,
                centerline_xy_m: [12.0, -5.0],
                ref_tangent: [0.8, 0.6],
                ref_left_normal: [-0.6, 0.8],
                ref_kappa_1pm: 0.018,
                section_dir: [0.6, -0.8],
                section_dir_derivative: [0.0144, 0.0108],
                section_dir_second_derivative: [-0.000_194_4, 0.000_259_2],
            },
            DenseSectionFrameInput {
                n_m: 1.25,
                dn_ds: -0.035,
                d2n_ds2: 0.004,
                v_mps: 18.0,
            },
        );
        let right = build_dense_section_frame_sample(
            DenseSectionFrameReference {
                s_m: 42.0,
                centerline_xy_m: [12.0, -5.0],
                ref_tangent: [0.8, 0.6],
                ref_left_normal: [-0.6, 0.8],
                ref_kappa_1pm: 0.018,
                section_dir: [0.6, -0.8],
                section_dir_derivative: [0.0144, 0.0108],
                section_dir_second_derivative: [-0.000_194_4, 0.000_259_2],
            },
            DenseSectionFrameInput {
                n_m: 1.25,
                dn_ds: -0.035,
                d2n_ds2: 0.004,
                v_mps: 18.0,
            },
        );

        assert!((right.x_m - left.x_m).abs() < TOLERANCE);
        assert!((right.y_m - left.y_m).abs() < TOLERANCE);
        assert!((right.heading_geo_rad - left.heading_geo_rad).abs() < TOLERANCE);
        assert!((right.kappa_geo_1pm - left.kappa_geo_1pm).abs() < TOLERANCE);
        assert!((right.ay_geo_mps2 - left.ay_geo_mps2).abs() < TOLERANCE);
    }

    #[test]
    fn section_frame_boundary_curvature_changes_when_second_derivative_jumps() {
        let reference = DenseSectionFrameReference {
            s_m: 9.0,
            centerline_xy_m: [9.0, 2.0],
            ref_tangent: [1.0, 0.0],
            ref_left_normal: [0.0, 1.0],
            ref_kappa_1pm: 0.0,
            section_dir: [0.0, -1.0],
            section_dir_derivative: [0.0, 0.0],
            section_dir_second_derivative: [0.0, 0.0],
        };
        let smooth = build_dense_section_frame_sample(
            reference,
            DenseSectionFrameInput {
                n_m: 2.0,
                dn_ds: 0.0,
                d2n_ds2: 0.0,
                v_mps: 1.0,
            },
        );
        let kinked = build_dense_section_frame_sample(
            DenseSectionFrameReference {
                section_dir_second_derivative: [0.0, 0.05],
                ..reference
            },
            DenseSectionFrameInput {
                n_m: 2.0,
                dn_ds: 0.0,
                d2n_ds2: 0.0,
                v_mps: 1.0,
            },
        );

        assert!(smooth.kappa_geo_1pm.abs() < TOLERANCE);
        assert!((kinked.kappa_geo_1pm + 0.1).abs() < TOLERANCE);
    }

    #[test]
    fn reference_path_tau_zero_and_one_match_station_endpoints() {
        let path = ReferencePathView {
            station_s_m: &[0.0, 10.0],
            centerline_xy_m: &[[0.0, 0.0], [10.0, 0.0]],
            tangent_xy: &[[1.0, 0.0], [1.0, 0.0]],
            normal_xy: &[[0.0, 1.0], [0.0, 1.0]],
            kappa_1pm: &[0.0, 0.2],
            closed: false,
        };

        let start = path.sample_at_interval_tau(0, 0.0).unwrap();
        let end = path.sample_at_interval_tau(0, 1.0).unwrap();

        assert_eq!(start.xy_m, [0.0, 0.0]);
        assert_eq!(end.xy_m, [10.0, 0.0]);
        assert!((start.s_m - 0.0).abs() < TOLERANCE);
        assert!((end.s_m - 10.0).abs() < TOLERANCE);
        assert!((start.kappa_prime_1pm2 - 0.02).abs() < TOLERANCE);
    }

    #[test]
    fn reference_path_closed_final_interval_wraps_to_first_station() {
        let path = ReferencePathView {
            station_s_m: &[0.0, 10.0, 20.0],
            centerline_xy_m: &[[0.0, 0.0], [10.0, 0.0], [10.0, 10.0]],
            tangent_xy: &[[1.0, 0.0], [0.0, 1.0], [-1.0, 0.0]],
            normal_xy: &[[0.0, 1.0], [-1.0, 0.0], [0.0, -1.0]],
            kappa_1pm: &[0.0, 0.1, 0.2],
            closed: true,
        };

        let start = path.sample_at_interval_tau(2, 0.0).unwrap();
        let end = path.sample_at_interval_tau(2, 1.0).unwrap();

        assert_eq!(start.xy_m, [10.0, 10.0]);
        assert_eq!(end.xy_m, [0.0, 0.0]);
        assert!((start.s_m - 20.0).abs() < TOLERANCE);
        assert!((end.s_m - 30.0).abs() < TOLERANCE);
        assert!((start.kappa_prime_1pm2 + 0.02).abs() < TOLERANCE);
    }

    #[test]
    fn reference_path_uses_local_nonuniform_spacing_for_kappa_prime() {
        let path = ReferencePathView {
            station_s_m: &[0.0, 7.0, 20.0],
            centerline_xy_m: &[[0.0, 0.0], [7.0, 0.0], [20.0, 0.0]],
            tangent_xy: &[[1.0, 0.0], [1.0, 0.0], [1.0, 0.0]],
            normal_xy: &[[0.0, 1.0], [0.0, 1.0], [0.0, 1.0]],
            kappa_1pm: &[0.01, 0.03, 0.095],
            closed: false,
        };

        let first = path.sample_at_interval_tau(0, 0.5).unwrap();
        let second = path.sample_at_interval_tau(1, 0.5).unwrap();

        assert!((first.kappa_prime_1pm2 - (0.02 / 7.0)).abs() < TOLERANCE);
        assert!((second.kappa_prime_1pm2 - (0.065 / 13.0)).abs() < TOLERANCE);
    }

    fn assert_point_close(actual: Point2, expected: Point2, tolerance: f64) {
        assert!(
            (actual[0] - expected[0]).abs() < tolerance,
            "x mismatch: actual={} expected={} tolerance={}",
            actual[0],
            expected[0],
            tolerance
        );
        assert!(
            (actual[1] - expected[1]).abs() < tolerance,
            "y mismatch: actual={} expected={} tolerance={}",
            actual[1],
            expected[1],
            tolerance
        );
    }
}
