use crate::contracts::Point2;
use crate::dense_frenet::{DenseSectionFrameGeometry, DenseSectionFrameHermiteSampler};

const MIN_DERIVATIVE_NORM_SQ: f64 = 1e-18;

#[derive(Clone, Copy, Debug)]
pub struct SectionFrameMapView<'a> {
    pub station_s_m: &'a [f64],
    pub centerline_xy_m: &'a [Point2],
    pub tangent_xy: &'a [Point2],
    pub section_dir_xy: &'a [Point2],
    pub section_dir_derivative_xy: &'a [Point2],
    pub closed: bool,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SectionFrameGeometry {
    pub s_m: f64,
    pub centerline_xy_m: Point2,
    pub centerline_derivative_xy: Point2,
    pub centerline_second_derivative_xy: Point2,
    pub ref_tangent_xy: Point2,
    pub ref_left_normal_xy: Point2,
    pub kappa_1pm: f64,
    pub heading_rate_per_s: f64,
    pub section_dir_xy: Point2,
    pub section_dir_derivative_xy: Point2,
    pub section_dir_second_derivative_xy: Point2,
}

impl SectionFrameMapView<'_> {
    pub fn sample_at_interval_tau(self, interval: usize, tau: f64) -> Option<SectionFrameGeometry> {
        let raw = DenseSectionFrameHermiteSampler {
            station_s_m: self.station_s_m,
            centerline_xy_m: self.centerline_xy_m,
            tangent_xy: self.tangent_xy,
            section_dir_xy: self.section_dir_xy,
            section_dir_derivative_xy: self.section_dir_derivative_xy,
            closed: self.closed,
        }
        .sample_at_interval_tau(interval, tau)?;

        SectionFrameGeometry::from_dense(raw)
    }
}

impl SectionFrameGeometry {
    pub fn from_dense(raw: DenseSectionFrameGeometry) -> Option<Self> {
        let speed_sq = dot(raw.centerline_ds, raw.centerline_ds);
        if !speed_sq.is_finite() || speed_sq <= MIN_DERIVATIVE_NORM_SQ {
            return None;
        }
        let speed = speed_sq.sqrt();
        let ref_tangent_xy = [raw.centerline_ds[0] / speed, raw.centerline_ds[1] / speed];
        let cross = cross(raw.centerline_ds, raw.centerline_d2s);
        let heading_rate_per_s = cross / speed_sq;
        let kappa_1pm = cross / speed_sq.powf(1.5);
        if !heading_rate_per_s.is_finite() || !kappa_1pm.is_finite() {
            return None;
        }

        Some(Self {
            s_m: raw.s_m,
            centerline_xy_m: raw.centerline_xy_m,
            centerline_derivative_xy: raw.centerline_ds,
            centerline_second_derivative_xy: raw.centerline_d2s,
            ref_tangent_xy,
            ref_left_normal_xy: [-ref_tangent_xy[1], ref_tangent_xy[0]],
            kappa_1pm,
            heading_rate_per_s,
            section_dir_xy: raw.section_dir,
            section_dir_derivative_xy: raw.section_dir_ds,
            section_dir_second_derivative_xy: raw.section_dir_d2s,
        })
    }

    #[must_use]
    pub fn dense_geometry(self) -> DenseSectionFrameGeometry {
        DenseSectionFrameGeometry {
            s_m: self.s_m,
            centerline_xy_m: self.centerline_xy_m,
            centerline_ds: self.centerline_derivative_xy,
            centerline_d2s: self.centerline_second_derivative_xy,
            section_dir: self.section_dir_xy,
            section_dir_ds: self.section_dir_derivative_xy,
            section_dir_d2s: self.section_dir_second_derivative_xy,
        }
    }
}

fn dot(left: Point2, right: Point2) -> f64 {
    left[0] * right[0] + left[1] * right[1]
}

fn cross(left: Point2, right: Point2) -> f64 {
    left[0] * right[1] - left[1] * right[0]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn geometry_derives_heading_rate_and_curvature_from_parameterized_centerline() {
        let raw = DenseSectionFrameGeometry {
            s_m: 1.0,
            centerline_xy_m: [2.0, 3.0],
            centerline_ds: [2.0, 0.0],
            centerline_d2s: [0.0, 1.0],
            section_dir: [0.0, -1.0],
            section_dir_ds: [0.1, 0.0],
            section_dir_d2s: [0.0, 0.2],
        };

        let geometry = SectionFrameGeometry::from_dense(raw).unwrap();

        assert_eq!(geometry.ref_tangent_xy, [1.0, 0.0]);
        assert_eq!(geometry.ref_left_normal_xy, [0.0, 1.0]);
        assert!((geometry.heading_rate_per_s - 0.5).abs() <= 1e-12);
        assert!((geometry.kappa_1pm - 0.25).abs() <= 1e-12);
        assert_eq!(geometry.dense_geometry(), raw);
    }

    #[test]
    fn geometry_rejects_degenerate_centerline_derivative() {
        let raw = DenseSectionFrameGeometry {
            s_m: 0.0,
            centerline_xy_m: [0.0, 0.0],
            centerline_ds: [0.0, 0.0],
            centerline_d2s: [0.0, 0.0],
            section_dir: [0.0, -1.0],
            section_dir_ds: [0.0, 0.0],
            section_dir_d2s: [0.0, 0.0],
        };

        assert!(SectionFrameGeometry::from_dense(raw).is_none());
    }
}
