use crate::contracts::Point2;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SectionFrameProgress {
    pub det_geom: f64,
    pub forward_progress_per_speed: f64,
    pub s_dot: f64,
    pub n_dot: f64,
    pub sigma_dt_ds: f64,
    pub dn_ds: f64,
}

#[must_use]
pub fn section_frame_progress(
    n_m: f64,
    v_mps: f64,
    beta_rad: f64,
    xi_rad: f64,
    tangent: Point2,
    left_normal: Point2,
    section_dir: Point2,
    section_dir_ds: Point2,
) -> SectionFrameProgress {
    section_frame_progress_from_derivatives(
        n_m,
        v_mps,
        beta_rad,
        xi_rad,
        tangent,
        left_normal,
        tangent,
        section_dir,
        section_dir_ds,
    )
}

#[must_use]
pub fn section_frame_progress_from_derivatives(
    n_m: f64,
    v_mps: f64,
    beta_rad: f64,
    xi_rad: f64,
    tangent: Point2,
    left_normal: Point2,
    centerline_ds: Point2,
    section_dir: Point2,
    section_dir_ds: Point2,
) -> SectionFrameProgress {
    let theta = xi_rad + beta_rad;
    let vel_dir = [
        theta.cos() * tangent[0] + theta.sin() * left_normal[0],
        theta.cos() * tangent[1] + theta.sin() * left_normal[1],
    ];
    let p_s = [
        centerline_ds[0] - n_m * section_dir_ds[0],
        centerline_ds[1] - n_m * section_dir_ds[1],
    ];
    let p_n = [-section_dir[0], -section_dir[1]];
    let det_geom = cross2(p_s, p_n);
    let det_for_division = signed_max_abs(det_geom, 1e-9);
    let forward_progress_per_speed = cross2(vel_dir, p_n) / det_for_division;
    let speed = v_mps.max(1e-6);
    let s_dot = speed * forward_progress_per_speed;
    let n_dot = speed * cross2(p_s, vel_dir) / det_for_division;
    let sigma_dt_ds = (1.0 / signed_max_abs(s_dot, 1e-9)).max(1e-9);

    SectionFrameProgress {
        det_geom,
        forward_progress_per_speed,
        s_dot,
        n_dot,
        sigma_dt_ds,
        dn_ds: sigma_dt_ds * n_dot,
    }
}

#[must_use]
pub fn pure_frenet_path_factor(n_m: f64, kappa_1pm: f64) -> f64 {
    1.0 - n_m * kappa_1pm
}

#[must_use]
pub fn heading_forward_projection(beta_rad: f64, xi_rad: f64) -> f64 {
    (beta_rad + xi_rad).cos()
}

#[must_use]
pub fn velocity_heading_curvature_1pm(
    v_mps: f64,
    omega_z_radps: f64,
    dbeta_ds: f64,
    sigma_dt_ds: f64,
) -> f64 {
    let speed = signed_max_abs(v_mps, 1e-6);
    let beta_dot_radps = dbeta_ds / signed_max_abs(sigma_dt_ds, 1e-9);
    (omega_z_radps + beta_dot_radps) / speed
}

#[must_use]
pub fn cross2(left: Point2, right: Point2) -> f64 {
    left[0] * right[1] - left[1] * right[0]
}

#[must_use]
pub fn signed_max_abs(value: f64, min_abs: f64) -> f64 {
    if value.abs() < min_abs {
        min_abs.copysign(value)
    } else {
        value
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn section_frame_progress_matches_pure_frenet_when_section_is_right_normal() {
        let progress = section_frame_progress(
            0.4,
            20.0,
            0.02,
            0.03,
            [1.0, 0.0],
            [0.0, 1.0],
            [0.0, -1.0],
            [0.1, 0.0],
        );

        let expected_factor = pure_frenet_path_factor(0.4, 0.1);
        let expected_forward = (0.02_f64 + 0.03).cos();
        assert!((progress.det_geom - expected_factor).abs() < 1.0e-12);
        assert!(
            (progress.forward_progress_per_speed - expected_forward / expected_factor).abs()
                < 1.0e-12
        );
        assert!(
            (progress.sigma_dt_ds - expected_factor / (20.0 * expected_forward)).abs() < 1.0e-12
        );
    }

    #[test]
    fn section_frame_can_be_regular_when_pure_frenet_factor_is_negative() {
        let pure = pure_frenet_path_factor(-4.8, -0.25);
        let progress = section_frame_progress(
            -4.8,
            12.0,
            0.0,
            0.0,
            [1.0, 0.0],
            [0.0, 1.0],
            [0.0, -1.0],
            [-0.06, 0.0],
        );

        assert!(pure < 0.0);
        assert!(progress.det_geom > 0.0);
        assert!(progress.forward_progress_per_speed > 0.0);
        assert!(progress.sigma_dt_ds.is_finite());
    }

    #[test]
    fn velocity_heading_curvature_includes_sideslip_heading_rate() {
        let curvature = velocity_heading_curvature_1pm(10.0, 0.2, 0.03, 0.1);

        assert!((curvature - 0.05).abs() < 1.0e-12);
    }
}
