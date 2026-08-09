use crate::contracts::{Point2, SectionsTrackViewV1, TrackAreaContractV1};
use crate::json::JsonValue;
use crate::JsonObject;
use std::fmt;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct StationGenerationCancelled;

pub(crate) type StationBuildResult<T> = Result<T, StationGenerationCancelled>;

#[derive(Clone, Copy)]
pub(crate) struct StationGenerationControl<'a> {
    cancel_check: Option<&'a dyn Fn() -> bool>,
    phase_observer: Option<&'a dyn Fn(&'static str)>,
}

impl<'a> StationGenerationControl<'a> {
    pub(crate) const fn never_cancelled() -> Self {
        Self {
            cancel_check: None,
            phase_observer: None,
        }
    }

    pub(crate) const fn cancellable(cancel_check: &'a dyn Fn() -> bool) -> Self {
        Self {
            cancel_check: Some(cancel_check),
            phase_observer: None,
        }
    }

    #[cfg(test)]
    pub(crate) const fn testable(
        cancel_check: &'a dyn Fn() -> bool,
        phase_observer: &'a dyn Fn(&'static str),
    ) -> Self {
        Self {
            cancel_check: Some(cancel_check),
            phase_observer: Some(phase_observer),
        }
    }

    pub(crate) fn checkpoint(self) -> StationBuildResult<()> {
        if self.cancel_check.is_some_and(|check| check()) {
            Err(StationGenerationCancelled)
        } else {
            Ok(())
        }
    }

    pub(crate) fn checkpoint_phase(self, phase: &'static str) -> StationBuildResult<()> {
        if let Some(observer) = self.phase_observer {
            observer(phase);
        }
        self.checkpoint()
    }
}

// TODO(station-quality): This projection-ratio gate is a pragmatic repair, not
// the final station-frame model. Repro for the bug it guards: Rice manual GT3,
// 160 production stations, pre-fix
// outputs/car_based_current_overlay_20260512/rice_gt3_current_rust_standard_debug_overlay.svg
// shows a nonphysical kink/pit around stations 21-23, with station 22 at
// raw_projection_lr_gap_ratio ~= 5.18 and center_turn_deg ~= -66.7. The clean
// Python overlay reproduced the same left/right endpoint-progress mismatch
// around reftrack 21/22, so keep validating every preset before tuning these
// thresholds or replacing this with globally synchronized station pairing.
const AREA_REPAIR_LR_PROJECTION_RATIO_SOFT_LIMIT: f64 = 2.5;
const AREA_REPAIR_LR_PROJECTION_RATIO_HARD_LIMIT: f64 = 4.0;
const AREA_REPAIR_CHORD_PERP_EPS_M: f64 = 1e-4;
const CLOSED_ENDPOINT_PLATEAU_EPS_M: f64 = 1e-4;
#[cfg(test)]
const OPEN_STATION_CELL_AREA_ADJACENT_RATIO_HARD_LIMIT: f64 = 8.0;
#[cfg(test)]
const OPEN_STATION_SPACING_ADJACENT_RATIO_HARD_LIMIT: f64 = 5.0;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DtwAlignmentRollBias {
    Auto,
    Explicit(isize),
}

impl DtwAlignmentRollBias {
    fn mode(self) -> &'static str {
        match self {
            Self::Auto => "auto",
            Self::Explicit(_) => "explicit",
        }
    }
}

impl fmt::Display for DtwAlignmentRollBias {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Auto => formatter.write_str("auto"),
            Self::Explicit(value) => write!(formatter, "{value}"),
        }
    }
}

impl From<isize> for DtwAlignmentRollBias {
    fn from(value: isize) -> Self {
        Self::Explicit(value)
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct StationBuilderOptions {
    pub sample_count: usize,
    pub smoothing_window: usize,
    pub dtw_band_ratio: f64,
    pub dtw_alignment_roll_bias: DtwAlignmentRollBias,
    pub centerline_hint_world: Option<Vec<Point2>>,
    pub dtw_centerline_normal_cost_weight: f64,
    pub dtw_slide_cost_weight: f64,
    pub turn_density_gain: f64,
    pub turn_analysis_smoothing_window: usize,
    pub turn_density_source: String,
    pub density_smooth_window: usize,
    pub density_max_adjacent_ratio: f64,
    pub density_slew_mode: String,
    pub target_spacing_max_adjacent_ratio: f64,
    pub target_spacing_metric: String,
    pub zero_station_normal_fix: bool,
}

impl Default for StationBuilderOptions {
    fn default() -> Self {
        Self {
            sample_count: 159,
            smoothing_window: 7,
            dtw_band_ratio: 0.27,
            dtw_alignment_roll_bias: DtwAlignmentRollBias::Auto,
            centerline_hint_world: None,
            dtw_centerline_normal_cost_weight: 0.0,
            dtw_slide_cost_weight: 1.0,
            turn_density_gain: 1.5,
            turn_analysis_smoothing_window: 1,
            turn_density_source: "centerline".to_owned(),
            density_smooth_window: 1,
            density_max_adjacent_ratio: 0.0,
            density_slew_mode: "log_smooth".to_owned(),
            target_spacing_max_adjacent_ratio: 0.0,
            target_spacing_metric: "centerline".to_owned(),
            zero_station_normal_fix: true,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct BoundaryPairTrack {
    pub left_world: Vec<Point2>,
    pub right_world: Vec<Point2>,
    pub left_route_progress: Vec<f64>,
    pub right_route_progress: Vec<f64>,
    pub centerline_world: Vec<Point2>,
    pub normals_world: Vec<Point2>,
    pub width_right: Vec<f64>,
    pub width_left: Vec<f64>,
    pub section_dirs: Vec<Point2>,
    pub metadata: JsonObject,
}

#[derive(Clone, Debug, PartialEq)]
struct PreparedRoutePair {
    topology: StationTopology,
    left_route: Vec<Point2>,
    right_route: Vec<Point2>,
    centerline_route: Vec<Point2>,
    normals: Vec<Point2>,
    width_left: Vec<f64>,
    width_right: Vec<f64>,
    shared_progress: Vec<f64>,
    left_progress: Vec<f64>,
    right_progress: Vec<f64>,
    metadata: JsonObject,
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct PreparedProductionStationPlan {
    prepared: PreparedRoutePair,
    complexity: StationComplexityReport,
}

impl PreparedProductionStationPlan {
    pub(crate) fn complexity(&self) -> &StationComplexityReport {
        &self.complexity
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct StationComplexityReport {
    pub recommended_station_count: usize,
    pub complexity_score: f64,
    pub route_length_m: f64,
    pub total_abs_heading_rad: f64,
    pub width_p10_m: f64,
    pub width_median_m: f64,
    pub max_segment_to_width_ratio: f64,
    pub crossing_zone_count: usize,
    pub raw_left_boundary_count: usize,
    pub raw_right_boundary_count: usize,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ReferenceTrack {
    pub centerline_world: Vec<Point2>,
    pub width_right_m: Vec<f64>,
    pub width_left_m: Vec<f64>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ProductionStationBuilder {
    CanonicalAreaStation,
    OpenAreaStation,
    GeneratedBoundaryPair,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum StationTopology {
    Closed,
    Open,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum EffectiveTrackDirection {
    Clockwise,
    Counterclockwise,
}

impl EffectiveTrackDirection {
    fn as_str(self) -> &'static str {
        match self {
            Self::Clockwise => "clockwise",
            Self::Counterclockwise => "counterclockwise",
        }
    }

    fn from_requested(value: Option<&str>) -> Option<Self> {
        match value {
            Some("clockwise") => Some(Self::Clockwise),
            Some("counterclockwise") => Some(Self::Counterclockwise),
            _ => None,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct FixedCenterlineStationOptions {
    pub sample_count: usize,
    pub dense_count: usize,
    pub dtw_frame_smoothing_window: usize,
    pub dtw_frame_turn_density_gain: f64,
    pub dtw_frame_band_ratio: f64,
    pub dtw_frame_alignment_roll_bias: DtwAlignmentRollBias,
    pub dtw_frame_centerline_normal_cost_weight: f64,
    pub dtw_frame_slide_cost_weight: f64,
    pub dtw_frame_slide_step_penalty: f64,
    pub dtw_frame_slide_repeat_penalty: f64,
    pub turn_analysis_smoothing_window: usize,
    pub turn_density_source: String,
    pub density_smooth_window: usize,
    pub density_max_adjacent_ratio: f64,
    pub density_slew_mode: String,
    pub target_spacing_max_adjacent_ratio: f64,
    pub target_spacing_metric: String,
    pub straight_weight: f64,
    pub curved_weight: f64,
    pub turn_smoothing_window: usize,
    pub curvature_low_percentile: f64,
    pub curvature_high_percentile: f64,
    pub density_area_length_cap_multiplier: f64,
    pub normal_repair_max_angle_deg: f64,
    pub normal_repair_angle_step_deg: f64,
    pub normal_repair_passes: usize,
    pub zero_station_normal_fix: bool,
}

impl Default for FixedCenterlineStationOptions {
    fn default() -> Self {
        Self {
            sample_count: 159,
            dense_count: 2400,
            dtw_frame_smoothing_window: 7,
            dtw_frame_turn_density_gain: 1.5,
            dtw_frame_band_ratio: 0.27,
            dtw_frame_alignment_roll_bias: DtwAlignmentRollBias::Auto,
            dtw_frame_centerline_normal_cost_weight: 24.0,
            dtw_frame_slide_cost_weight: 0.70,
            dtw_frame_slide_step_penalty: 4.0,
            dtw_frame_slide_repeat_penalty: 24.0,
            turn_analysis_smoothing_window: 1,
            turn_density_source: "centerline".to_owned(),
            density_smooth_window: 1,
            density_max_adjacent_ratio: 0.0,
            density_slew_mode: "log_smooth".to_owned(),
            target_spacing_max_adjacent_ratio: 0.0,
            target_spacing_metric: "centerline".to_owned(),
            straight_weight: 0.5,
            curved_weight: 1.5,
            turn_smoothing_window: 31,
            curvature_low_percentile: 35.0,
            curvature_high_percentile: 85.0,
            density_area_length_cap_multiplier: 1.5,
            normal_repair_max_angle_deg: 85.0,
            normal_repair_angle_step_deg: 5.0,
            normal_repair_passes: 4,
            zero_station_normal_fix: true,
        }
    }
}

#[must_use]
pub fn build_fixed_centerline_sections_track_view(
    track: &TrackAreaContractV1,
    reference: &ReferenceTrack,
    options: &FixedCenterlineStationOptions,
) -> SectionsTrackViewV1 {
    build_fixed_centerline_sections_track_view_with_control(
        track,
        reference,
        options,
        StationGenerationControl::never_cancelled(),
    )
    .expect("non-cancellable fixed-centerline construction cannot be cancelled")
}

fn build_fixed_centerline_sections_track_view_with_control(
    track: &TrackAreaContractV1,
    reference: &ReferenceTrack,
    options: &FixedCenterlineStationOptions,
    control: StationGenerationControl<'_>,
) -> StationBuildResult<SectionsTrackViewV1> {
    control.checkpoint()?;
    let station_frame_options = StationBuilderOptions {
        sample_count: reference.centerline_world.len(),
        smoothing_window: options.dtw_frame_smoothing_window,
        dtw_band_ratio: options.dtw_frame_band_ratio,
        dtw_alignment_roll_bias: options.dtw_frame_alignment_roll_bias,
        centerline_hint_world: Some(reference.centerline_world.clone()),
        dtw_centerline_normal_cost_weight: options.dtw_frame_centerline_normal_cost_weight,
        dtw_slide_cost_weight: options.dtw_frame_slide_cost_weight,
        turn_density_gain: options.dtw_frame_turn_density_gain,
        turn_analysis_smoothing_window: options.turn_analysis_smoothing_window,
        turn_density_source: options.turn_density_source.clone(),
        density_smooth_window: options.density_smooth_window,
        density_max_adjacent_ratio: options.density_max_adjacent_ratio,
        density_slew_mode: options.density_slew_mode.clone(),
        target_spacing_max_adjacent_ratio: options.target_spacing_max_adjacent_ratio,
        target_spacing_metric: options.target_spacing_metric.clone(),
        zero_station_normal_fix: false,
    };
    let station_frame = build_boundary_pair_track_with_control(
        &track.left_boundary_xy_m,
        &track.right_boundary_xy_m,
        &station_frame_options,
        control,
    )?;
    control.checkpoint()?;
    let (
        station_frame_left,
        station_frame_right,
        align_meta,
        frame_order_reversed,
        frame_order_shift,
    ) = align_station_frame_order_to_centerline(
        &station_frame.left_world,
        &station_frame.right_world,
        &reference.centerline_world,
    );
    let mut station_frame_left_route_progress = station_frame.left_route_progress.clone();
    let mut station_frame_right_route_progress = station_frame.right_route_progress.clone();
    if frame_order_reversed {
        station_frame_left_route_progress.reverse();
        station_frame_right_route_progress.reverse();
    }
    station_frame_left_route_progress = roll_values(
        &station_frame_left_route_progress,
        frame_order_shift as isize,
    );
    station_frame_right_route_progress = roll_values(
        &station_frame_right_route_progress,
        frame_order_shift as isize,
    );
    let (station_frame_progress, progress_meta) = station_frame_progress_for_centerline(
        &station_frame_left,
        &station_frame_right,
        &reference.centerline_world,
    );
    let dense_count = options.dense_count.max(320);
    let dense_progress = (0..dense_count)
        .map(|index| index as f64 / dense_count as f64)
        .collect::<Vec<_>>();
    let raw_left_dense = resample_closed_polyline(&track.left_boundary_xy_m, dense_count);
    let raw_right_dense = resample_closed_polyline(&track.right_boundary_xy_m, dense_count);
    control.checkpoint()?;
    let width_right = widths_or_default(
        &reference.width_right_m,
        reference.centerline_world.len(),
        4.0,
    );
    let width_left = widths_or_default(
        &reference.width_left_m,
        reference.centerline_world.len(),
        4.0,
    );
    let active_width_right = sample_ref_widths_for_centerline(
        &reference.centerline_world,
        &reference.centerline_world,
        &width_right,
    );
    let active_width_left = sample_ref_widths_for_centerline(
        &reference.centerline_world,
        &reference.centerline_world,
        &width_left,
    );
    let dense = build_normal_line_sections(
        &reference.centerline_world,
        &raw_left_dense,
        &raw_right_dense,
        &active_width_right,
        &active_width_left,
        &dense_progress,
        &station_frame_left,
        &station_frame_right,
        &station_frame_left_route_progress,
        &station_frame_right_route_progress,
        &station_frame_progress,
        options.zero_station_normal_fix,
    );
    control.checkpoint()?;
    let physical_areas = compute_section_cell_areas(&dense.left, &dense.right);
    let (density_areas, area_cap_meta) = density_areas_with_iqr_length_cap(
        &dense.left,
        &dense.right,
        options.density_area_length_cap_multiplier,
    );
    let (mut segment_weight, density_meta) = topology_curvature_segment_weight(
        StationTopology::Closed,
        &dense.center,
        &density_areas,
        options.straight_weight,
        options.curved_weight,
        options.turn_smoothing_window,
        options.curvature_low_percentile,
        options.curvature_high_percentile,
    );
    let total_area = density_areas.iter().copied().sum::<f64>();
    let mut weighted_area = density_areas
        .iter()
        .zip(&segment_weight)
        .map(|(area, weight)| area * weight)
        .collect::<Vec<_>>();
    let weighted_sum_raw = weighted_area.iter().copied().sum::<f64>();
    if weighted_sum_raw > 1e-9 && total_area > 1e-9 {
        let scale = total_area / weighted_sum_raw;
        for weight in &mut segment_weight {
            *weight *= scale;
        }
        weighted_area = density_areas
            .iter()
            .zip(&segment_weight)
            .map(|(area, weight)| area * weight)
            .collect();
    }
    let weighted_sum = weighted_area.iter().copied().sum::<f64>();
    let cumulative = cumulative_with_zero(&weighted_area);
    let progress_ext = {
        let mut result = dense_progress.clone();
        result.push(1.0);
        result
    };
    let sample_count = options.sample_count.max(3);
    let target_progress = (0..sample_count)
        .map(|index| {
            let target_weighted = weighted_sum * index as f64 / sample_count as f64;
            interp_scalar(target_weighted, &cumulative, &progress_ext).rem_euclid(1.0)
        })
        .collect::<Vec<_>>();
    let (station, repair_meta) = area_preserving_chord_repair(
        &reference.centerline_world,
        &raw_left_dense,
        &raw_right_dense,
        &track.left_boundary_xy_m,
        &track.right_boundary_xy_m,
        &active_width_right,
        &active_width_left,
        &target_progress,
        &station_frame_left,
        &station_frame_right,
        &station_frame_left_route_progress,
        &station_frame_right_route_progress,
        &station_frame_progress,
        options.zero_station_normal_fix,
        options.normal_repair_max_angle_deg,
        options.normal_repair_angle_step_deg,
        options.normal_repair_passes,
        control,
    )?;
    control.checkpoint()?;
    let (station_s, segment_lengths) = closed_polyline_arclength(&station.center);
    let total_length_m = station_s.last().copied().unwrap_or(0.0);
    let station_s_m = station_s
        .into_iter()
        .take(station.center.len())
        .collect::<Vec<_>>();
    let mut metadata = station_frame.metadata.clone();
    metadata.extend(align_meta);
    metadata.extend(progress_meta);
    metadata.extend(area_cap_meta);
    metadata.extend(density_meta);
    metadata.extend(repair_meta);
    metadata.push((
        "source".to_owned(),
        "fixed_centerline_area_station_generator".into(),
    ));
    metadata.push((
        "generator".to_owned(),
        "rlc_solver_models::station::build_fixed_centerline_sections_track_view".into(),
    ));
    metadata.push(("station_frame_source".to_owned(), "dtw_pairs".into()));
    metadata.push(("density_source".to_owned(), "baseline_curvature".into()));
    metadata.push(("endpoint_mode".to_owned(), "normal_line".into()));
    metadata.push(("placement_mode".to_owned(), "area_preserving_chords".into()));
    metadata.push(("total_length_m".to_owned(), total_length_m.into()));
    metadata.push((
        "physical_total_area_m2".to_owned(),
        physical_areas.iter().copied().sum::<f64>().into(),
    ));
    metadata.push(("density_total_area_m2".to_owned(), weighted_sum.into()));
    metadata.push((
        "dense_ray_miss_count".to_owned(),
        JsonValue::Integer(dense.miss_count),
    ));
    metadata.push((
        "station_ray_miss_count".to_owned(),
        JsonValue::Integer(station.miss_count),
    ));
    metadata.push((
        "station_count".to_owned(),
        JsonValue::Integer(station.center.len() as i64),
    ));
    metadata.push((
        "station_progress".to_owned(),
        JsonValue::Array(
            target_progress
                .iter()
                .copied()
                .map(JsonValue::from)
                .collect(),
        ),
    ));
    let (min_spacing, median_spacing) = station_spacing_metrics(&station.center);
    metadata.push(("min_station_spacing_m".to_owned(), min_spacing.into()));
    metadata.push(("median_station_spacing_m".to_owned(), median_spacing.into()));
    metadata.push((
        "max_station_spacing_m".to_owned(),
        segment_lengths.iter().copied().fold(0.0, f64::max).into(),
    ));
    metadata.push((
        "max_adjacent_normal_rotation_deg".to_owned(),
        max_adjacent_vector_rotation_deg(&station.normals).into(),
    ));
    metadata.push((
        "adjacent_section_crossing_count_horizon2".to_owned(),
        JsonValue::Integer(station_horizon_crossing_count(
            &station.left,
            &station.right,
            2,
        )),
    ));
    let quality_metrics = metadata
        .iter()
        .filter(|(key, _)| {
            matches!(
                key.as_str(),
                "min_station_spacing_m"
                    | "median_station_spacing_m"
                    | "max_station_spacing_m"
                    | "max_adjacent_normal_rotation_deg"
                    | "zero_station_normal_fix_applied_count"
                    | "density_area_length_cap_count"
                    | "density_area_length_cap_fraction"
                    | "area_preserving_repair_changed_count"
                    | "area_preserving_repair_horizon2_crossing_count"
            )
        })
        .map(|(key, value)| (key.clone(), value.clone()))
        .collect::<JsonObject>();

    Ok(SectionsTrackViewV1 {
        schema_version: SectionsTrackViewV1::SCHEMA_VERSION.to_owned(),
        view_id: format!("{}_fixed_centerline_sections_track_view_v1", track.track_id),
        track_id: track.track_id.clone(),
        station_s_m,
        centerline_xy_m: station.center,
        left_boundary_xy_m: station.left,
        right_boundary_xy_m: station.right,
        normals_xy: station.normals.clone(),
        width_left_m: station.width_left,
        width_right_m: station.width_right,
        section_dirs_xy: station.normals,
        quality_metrics,
        metadata,
    })
}

#[must_use]
pub fn build_area_station_sections_track_view(
    track: &TrackAreaContractV1,
    reference: &ReferenceTrack,
    options: &FixedCenterlineStationOptions,
) -> SectionsTrackViewV1 {
    build_area_station_sections_track_view_with_control(
        track,
        reference,
        options,
        StationGenerationControl::never_cancelled(),
    )
    .expect("non-cancellable area station construction cannot be cancelled")
}

fn build_area_station_sections_track_view_with_control(
    track: &TrackAreaContractV1,
    reference: &ReferenceTrack,
    options: &FixedCenterlineStationOptions,
    control: StationGenerationControl<'_>,
) -> StationBuildResult<SectionsTrackViewV1> {
    let mut view = build_fixed_centerline_sections_track_view_with_control(
        track, reference, options, control,
    )?;

    view.view_id = format!(
        "{}_canonical_area_station_sections_track_view_v1",
        track.track_id
    );
    upsert_metadata(
        &mut view.metadata,
        "source",
        "canonical_area_station_generator".into(),
    );
    upsert_metadata(
        &mut view.metadata,
        "generator",
        "rlc_solver_models::station::build_area_station_sections_track_view".into(),
    );
    upsert_metadata(&mut view.metadata, "centerline_mode", "fixed".into());
    upsert_metadata(
        &mut view.metadata,
        "station_builder",
        "canonical_area_station_generator".into(),
    );

    Ok(view)
}

#[must_use]
pub fn build_open_area_station_sections_track_view(
    track: &TrackAreaContractV1,
    options: &FixedCenterlineStationOptions,
) -> SectionsTrackViewV1 {
    assert_eq!(
        track.trajectory_mode, "open",
        "build_open_area_station_sections_track_view requires trajectory_mode=open"
    );

    let control = StationGenerationControl::never_cancelled();
    let prepared = prepare_open_area_route_pair(track, options, control)
        .expect("non-cancellable open station preparation cannot be cancelled");
    build_open_area_station_sections_track_view_from_prepared(track, options, prepared, control)
        .expect("non-cancellable open station construction cannot be cancelled")
}

fn build_open_area_station_sections_track_view_from_prepared(
    track: &TrackAreaContractV1,
    options: &FixedCenterlineStationOptions,
    prepared: PreparedRoutePair,
    control: StationGenerationControl<'_>,
) -> StationBuildResult<SectionsTrackViewV1> {
    control.checkpoint()?;
    let sample_count = options.sample_count.max(3);
    let dense_count = prepared.centerline_route.len();
    let mut metadata = prepared.metadata.clone();
    let dense_corridor = BoundaryPairTrack {
        left_world: prepared.left_route,
        right_world: prepared.right_route,
        left_route_progress: prepared.left_progress,
        right_route_progress: prepared.right_progress,
        centerline_world: prepared.centerline_route,
        normals_world: prepared.normals.clone(),
        width_right: prepared.width_right,
        width_left: prepared.width_left,
        section_dirs: prepared.normals,
        metadata: Vec::new(),
    };
    let station_frame_progress = station_frame_progress_for_path(
        StationTopology::Open,
        dense_corridor.centerline_world.len(),
    );
    let (target_progress, placement_meta) =
        open_area_target_progress(&dense_corridor, sample_count, options);
    let (sections, repair_meta) = open_area_preserving_chord_repair(
        &dense_corridor,
        &track.left_boundary_xy_m,
        &track.right_boundary_xy_m,
        &station_frame_progress,
        &target_progress,
        options.normal_repair_max_angle_deg,
        options.normal_repair_angle_step_deg,
        options.normal_repair_passes,
        control,
    )?;
    control.checkpoint()?;
    let corridor = BoundaryPairTrack {
        left_world: sections.left,
        right_world: sections.right,
        left_route_progress: station_frame_progress_for_path(StationTopology::Open, sample_count),
        right_route_progress: station_frame_progress_for_path(StationTopology::Open, sample_count),
        centerline_world: sections.center,
        normals_world: sections.normals.clone(),
        width_right: sections.width_right,
        width_left: sections.width_left,
        section_dirs: sections.normals,
        metadata: Vec::new(),
    };
    let (station_s_all, segment_lengths) = open_polyline_arclength(&corridor.centerline_world);
    let station_s_m = station_s_all
        .into_iter()
        .take(corridor.centerline_world.len())
        .collect::<Vec<_>>();
    let total_length_m = station_s_m.last().copied().unwrap_or(0.0);
    let first_last_gap_m = corridor
        .centerline_world
        .first()
        .zip(corridor.centerline_world.last())
        .map_or(0.0, |(first, last)| distance(*first, *last));
    let cell_areas = compute_open_section_cell_areas(&corridor.left_world, &corridor.right_world);
    let spacing_adjacent_ratio = linear_adjacent_ratio(&segment_lengths);
    let cell_area_adjacent_ratio = linear_adjacent_ratio(&cell_areas);

    metadata.extend(placement_meta);
    metadata.extend(repair_meta);
    metadata.push(("source".to_owned(), "open_area_station_generator".into()));
    metadata.push((
        "generator".to_owned(),
        "rlc_solver_models::station::build_open_area_station_sections_track_view".into(),
    ));
    metadata.push(("trajectory_mode".to_owned(), "open".into()));
    metadata.push((
        "station_builder".to_owned(),
        "open_area_station_generator".into(),
    ));
    metadata.push(("station_frame_source".to_owned(), "dtw_pairs".into()));
    metadata.push((
        "endpoint_mode".to_owned(),
        "open_normal_line_with_dtw_paired_boundary_fallback".into(),
    ));
    metadata.push((
        "placement_mode".to_owned(),
        "open_area_preserving_chords".into(),
    ));
    metadata.push((
        "chord_repair".to_owned(),
        "open_normal_line_dtw_boundary_fallback".into(),
    ));
    metadata.push((
        "dense_count".to_owned(),
        JsonValue::Integer(dense_count as i64),
    ));
    metadata.push((
        "station_count".to_owned(),
        JsonValue::Integer(sample_count as i64),
    ));
    metadata.push(("total_length_m".to_owned(), total_length_m.into()));
    metadata.push(("first_last_gap_m".to_owned(), first_last_gap_m.into()));
    metadata.push((
        "station_spacing_adjacent_ratio_max".to_owned(),
        spacing_adjacent_ratio.into(),
    ));
    metadata.push((
        "cell_area_adjacent_ratio_max".to_owned(),
        cell_area_adjacent_ratio.into(),
    ));
    metadata.push(("station_progress".to_owned(), f64s_json(&target_progress)));

    let (min_spacing, median_spacing) = open_station_spacing_metrics(&corridor.centerline_world);
    metadata.push(("min_station_spacing_m".to_owned(), min_spacing.into()));
    metadata.push(("median_station_spacing_m".to_owned(), median_spacing.into()));
    metadata.push((
        "max_station_spacing_m".to_owned(),
        segment_lengths.iter().copied().fold(0.0, f64::max).into(),
    ));
    metadata.push((
        "max_adjacent_normal_rotation_deg".to_owned(),
        max_adjacent_vector_rotation_deg_open(&corridor.normals_world).into(),
    ));
    let crossing_count =
        station_horizon_crossing_count_open(&corridor.left_world, &corridor.right_world, 2);
    let raw_boundary_crossing_count = station_raw_boundary_crossing_count_open(
        &corridor.left_world,
        &corridor.right_world,
        &track.left_boundary_xy_m,
        &track.right_boundary_xy_m,
    );
    metadata.push((
        "adjacent_section_crossing_count_horizon2".to_owned(),
        JsonValue::Integer(crossing_count),
    ));
    metadata.push((
        "station_raw_boundary_crossing_count".to_owned(),
        JsonValue::Integer(raw_boundary_crossing_count),
    ));
    let quality_metrics = metadata
        .iter()
        .filter(|(key, _)| {
            matches!(
                key.as_str(),
                "first_last_gap_m"
                    | "total_length_m"
                    | "min_station_spacing_m"
                    | "median_station_spacing_m"
                    | "max_station_spacing_m"
                    | "station_spacing_adjacent_ratio_max"
                    | "cell_area_adjacent_ratio_max"
                    | "max_adjacent_normal_rotation_deg"
                    | "adjacent_section_crossing_count_horizon2"
            )
        })
        .map(|(key, value)| (key.clone(), value.clone()))
        .collect::<JsonObject>();

    Ok(SectionsTrackViewV1 {
        schema_version: SectionsTrackViewV1::SCHEMA_VERSION.to_owned(),
        view_id: format!("{}_open_area_sections_sc{}", track.track_id, sample_count),
        track_id: track.track_id.clone(),
        station_s_m,
        centerline_xy_m: corridor.centerline_world,
        left_boundary_xy_m: corridor.left_world,
        right_boundary_xy_m: corridor.right_world,
        normals_xy: corridor.normals_world.clone(),
        width_left_m: corridor.width_left,
        width_right_m: corridor.width_right,
        section_dirs_xy: corridor.normals_world,
        quality_metrics,
        metadata,
    })
}

fn open_area_preserving_chord_repair(
    dense_corridor: &BoundaryPairTrack,
    raw_left_world: &[Point2],
    raw_right_world: &[Point2],
    station_frame_progress: &[f64],
    target_progress: &[f64],
    _max_angle_deg: f64,
    _angle_step_deg: f64,
    _passes: usize,
    control: StationGenerationControl<'_>,
) -> StationBuildResult<(BuiltSections, JsonObject)> {
    control.checkpoint()?;
    let mut centers = interp_points_by_shared_progress(
        station_frame_progress,
        &dense_corridor.centerline_world,
        target_progress,
    );
    let frame_left = interp_points_by_shared_progress(
        station_frame_progress,
        &dense_corridor.left_world,
        target_progress,
    );
    let frame_right = interp_points_by_shared_progress(
        station_frame_progress,
        &dense_corridor.right_world,
        target_progress,
    );
    let (_, tangents) = right_normals_world_open(&centers);
    let baseline_normals = tangents
        .iter()
        .map(|tangent| normalize([tangent[1], -tangent[0]], [1.0, 0.0]))
        .collect::<Vec<_>>();
    let fallback_width_left = interp_scalar_by_shared_progress(
        station_frame_progress,
        &dense_corridor.width_left,
        target_progress,
    );
    let fallback_width_right = interp_scalar_by_shared_progress(
        station_frame_progress,
        &dense_corridor.width_right,
        target_progress,
    );

    let mut normal_line_left = Vec::with_capacity(centers.len());
    let mut normal_line_right = Vec::with_capacity(centers.len());
    let mut miss_count = 0_i64;
    for index in 0..centers.len() {
        if index % 32 == 0 {
            control.checkpoint()?;
        }
        let fallback_left = point_sub(
            centers[index],
            point_scale(baseline_normals[index], fallback_width_left[index]),
        );
        let fallback_right = point_add(
            centers[index],
            point_scale(baseline_normals[index], fallback_width_right[index]),
        );
        let section = normal_line_section_open(
            centers[index],
            baseline_normals[index],
            raw_left_world,
            raw_right_world,
            fallback_left,
            fallback_right,
        );
        normal_line_left.push(section.0);
        normal_line_right.push(section.1);
        miss_count += i64::from(section.3);
    }

    let crossing_count_before =
        station_horizon_crossing_count_open(&normal_line_left, &normal_line_right, 2);
    let all_crossing_count_before =
        station_crossing_count_all(&normal_line_left, &normal_line_right);
    let raw_boundary_crossing_count_before = station_raw_boundary_crossing_count_open(
        &normal_line_left,
        &normal_line_right,
        raw_left_world,
        raw_right_world,
    );

    let mut left = normal_line_left.clone();
    let mut right = normal_line_right.clone();
    let mut normals = right
        .iter()
        .zip(&left)
        .zip(&baseline_normals)
        .map(|((r, l), fallback)| normalize(point_sub(*r, *l), *fallback))
        .collect::<Vec<_>>();
    let mut changed_total = 0_i64;
    for index in 0..centers.len() {
        if index % 32 == 0 {
            control.checkpoint()?;
        }
        if station_segment_raw_boundary_crossing_count(
            left[index],
            right[index],
            raw_left_world,
            raw_right_world,
        ) > 0
        {
            left[index] = frame_left[index];
            right[index] = frame_right[index];
            normals[index] = normalize(point_sub(right[index], left[index]), normals[index]);
            changed_total += 1;
        }
    }
    let left_projection = OpenPolylineProjection::new(raw_left_world);
    let right_projection = OpenPolylineProjection::new(raw_right_world);
    let mut left_projection_s = left
        .iter()
        .map(|point| left_projection.project_arclength(*point))
        .collect::<Vec<_>>();
    let mut right_projection_s = right
        .iter()
        .map(|point| right_projection.project_arclength(*point))
        .collect::<Vec<_>>();
    for _ in 0..4 {
        control.checkpoint_phase("open_refinement_pass")?;
        let mut changed = false;
        for index in 0..centers.len() {
            if index % 32 == 0 {
                control.checkpoint()?;
            }
            let endpoint = index == 0 || index + 1 == centers.len();
            let left_not_ordered =
                index > 0 && left_projection_s[index] <= left_projection_s[index - 1] + 1e-6;
            let right_not_ordered =
                index > 0 && right_projection_s[index] <= right_projection_s[index - 1] + 1e-6;
            if !endpoint && !left_not_ordered && !right_not_ordered {
                continue;
            }
            let start = if endpoint {
                index
            } else {
                index.saturating_sub(2)
            };
            let end = if endpoint {
                index
            } else {
                (index + 2).min(centers.len().saturating_sub(1))
            };
            for repair_index in start..=end {
                if left[repair_index] != frame_left[repair_index]
                    || right[repair_index] != frame_right[repair_index]
                {
                    left[repair_index] = frame_left[repair_index];
                    right[repair_index] = frame_right[repair_index];
                    normals[repair_index] = normalize(
                        point_sub(right[repair_index], left[repair_index]),
                        normals[repair_index],
                    );
                    left_projection_s[repair_index] =
                        left_projection.project_arclength(left[repair_index]);
                    right_projection_s[repair_index] =
                        right_projection.project_arclength(right[repair_index]);
                    changed_total += 1;
                    changed = true;
                }
            }
        }
        if !changed {
            break;
        }
    }
    let crossing_pair_replaced_count = repair_open_crossing_pairs(
        &mut left,
        &mut right,
        &mut normals,
        &mut left_projection_s,
        &mut right_projection_s,
        &left_projection,
        &right_projection,
        &centers,
        raw_left_world,
        raw_right_world,
        &fallback_width_left,
        &fallback_width_right,
        &frame_left,
        &frame_right,
        2,
        control,
    )?;
    changed_total += crossing_pair_replaced_count;
    for _ in 0..4 {
        control.checkpoint_phase("open_refinement_pass")?;
        let crossing_pairs = station_horizon_crossing_pairs_open(&left, &right, 2);
        if crossing_pairs.is_empty() {
            break;
        }
        let mut changed = false;
        for (first, second) in crossing_pairs {
            let start = first.saturating_sub(3);
            let end = (second + 3).min(centers.len().saturating_sub(1));
            for repair_index in start..=end {
                if left[repair_index] != frame_left[repair_index]
                    || right[repair_index] != frame_right[repair_index]
                {
                    left[repair_index] = frame_left[repair_index];
                    right[repair_index] = frame_right[repair_index];
                    normals[repair_index] = normalize(
                        point_sub(right[repair_index], left[repair_index]),
                        normals[repair_index],
                    );
                    left_projection_s[repair_index] =
                        left_projection.project_arclength(left[repair_index]);
                    right_projection_s[repair_index] =
                        right_projection.project_arclength(right[repair_index]);
                    changed_total += 1;
                    changed = true;
                }
            }
        }
        if !changed {
            break;
        }
    }
    if !centers.is_empty() {
        let last = centers.len() - 1;
        if let (Some(raw_left_start), Some(raw_right_start)) =
            (raw_left_world.first(), raw_right_world.first())
        {
            left[0] = *raw_left_start;
            right[0] = *raw_right_start;
            normals[0] = normalize(point_sub(right[0], left[0]), normals[0]);
            left_projection_s[0] = left_projection.project_arclength(left[0]);
            right_projection_s[0] = right_projection.project_arclength(right[0]);
        }
        if let (Some(raw_left_finish), Some(raw_right_finish)) =
            (raw_left_world.last(), raw_right_world.last())
        {
            left[last] = *raw_left_finish;
            right[last] = *raw_right_finish;
            normals[last] = normalize(point_sub(right[last], left[last]), normals[last]);
            left_projection_s[last] = left_projection.project_arclength(left[last]);
            right_projection_s[last] = right_projection.project_arclength(right[last]);
        }
    }
    let remaining_crossings = station_horizon_crossing_count_open(&left, &right, 2);
    let remaining_raw_boundary_crossings =
        station_raw_boundary_crossing_count_open(&left, &right, raw_left_world, raw_right_world);
    let mut synchronized_progress_fallback_count = 0_i64;
    if remaining_crossings > 0 || remaining_raw_boundary_crossings > 0 {
        let raw_left_progress =
            station_frame_progress_for_path(StationTopology::Open, raw_left_world.len());
        let raw_right_progress =
            station_frame_progress_for_path(StationTopology::Open, raw_right_world.len());
        let synchronized_left =
            interp_points_by_shared_progress(&raw_left_progress, raw_left_world, target_progress);
        let synchronized_right =
            interp_points_by_shared_progress(&raw_right_progress, raw_right_world, target_progress);

        for index in 0..centers
            .len()
            .min(synchronized_left.len())
            .min(synchronized_right.len())
        {
            left[index] = synchronized_left[index];
            right[index] = synchronized_right[index];
            centers[index] = midpoint(left[index], right[index]);
            normals[index] = normalize(point_sub(right[index], left[index]), normals[index]);
            left_projection_s[index] = left_projection.project_arclength(left[index]);
            right_projection_s[index] = right_projection.project_arclength(right[index]);
            synchronized_progress_fallback_count += 1;
        }
        changed_total += synchronized_progress_fallback_count;
    }
    let chord_lengths = right
        .iter()
        .zip(&left)
        .map(|(r, l)| distance(*r, *l))
        .collect::<Vec<_>>();
    let fractions = centers
        .iter()
        .zip(&left)
        .zip(&right)
        .zip(&chord_lengths)
        .map(|(((center, l), r), length)| {
            let chord = point_sub(*r, *l);
            (dot(point_sub(*center, *l), chord) / length.powi(2).max(1e-9)).clamp(0.0, 1.0)
        })
        .collect::<Vec<_>>();
    let width_left = chord_lengths
        .iter()
        .zip(&fractions)
        .map(|(length, fraction)| length * fraction)
        .collect::<Vec<_>>();
    let width_right = chord_lengths
        .iter()
        .zip(&width_left)
        .map(|(length, left_width)| length - left_width)
        .collect::<Vec<_>>();
    for index in 0..centers.len() {
        normals[index] = normalize(point_sub(right[index], left[index]), normals[index]);
    }

    let crossing_count_after = station_horizon_crossing_count_open(&left, &right, 2);
    let all_crossing_count_after = station_crossing_count_all(&left, &right);
    let raw_boundary_crossing_count_after =
        station_raw_boundary_crossing_count_open(&left, &right, raw_left_world, raw_right_world);
    let left_endpoint_spacing =
        open_endpoint_projection_spacing_stats_from_arclengths(&left_projection_s);
    let right_endpoint_spacing =
        open_endpoint_projection_spacing_stats_from_arclengths(&right_projection_s);
    let lr_projection_ratios =
        open_lr_projection_interval_ratios(&left_projection_s, &right_projection_s);
    let frame_endpoint_delta = left
        .iter()
        .zip(&right)
        .zip(&centers)
        .zip(&frame_left)
        .zip(&frame_right)
        .map(|((((left, right), center), frame_left), frame_right)| {
            distance(*left, *frame_left)
                .max(distance(*right, *frame_right))
                .max(distance(*center, midpoint(*frame_left, *frame_right)))
        })
        .collect::<Vec<_>>();
    let meta = vec![
        (
            "open_repair_endpoint_mode".to_owned(),
            "open_normal_line_with_dtw_paired_boundary_fallback".into(),
        ),
        (
            "open_repair_progress_source".to_owned(),
            "shared_dtw_frame_progress".into(),
        ),
        (
            "open_repair_changed_count".to_owned(),
            JsonValue::Integer(changed_total),
        ),
        (
            "open_repair_crossing_count_before".to_owned(),
            JsonValue::Integer(crossing_count_before),
        ),
        (
            "open_repair_crossing_count_after".to_owned(),
            JsonValue::Integer(crossing_count_after),
        ),
        (
            "open_repair_all_crossing_count_before".to_owned(),
            JsonValue::Integer(all_crossing_count_before),
        ),
        (
            "open_repair_all_crossing_count_after".to_owned(),
            JsonValue::Integer(all_crossing_count_after),
        ),
        (
            "open_repair_raw_boundary_crossing_count_before".to_owned(),
            JsonValue::Integer(raw_boundary_crossing_count_before),
        ),
        (
            "open_repair_raw_boundary_crossing_count_after".to_owned(),
            JsonValue::Integer(raw_boundary_crossing_count_after),
        ),
        (
            "open_repair_initial_miss_count".to_owned(),
            JsonValue::Integer(miss_count),
        ),
        (
            "open_repair_fallback_count".to_owned(),
            JsonValue::Integer(miss_count),
        ),
        (
            "open_repair_crossing_pair_replaced_count".to_owned(),
            JsonValue::Integer(crossing_pair_replaced_count),
        ),
        (
            "open_repair_synchronized_progress_fallback_count".to_owned(),
            JsonValue::Integer(synchronized_progress_fallback_count),
        ),
        ("open_repair_angle_abs_max_deg".to_owned(), 0.0.into()),
        ("open_repair_angle_abs_p95_deg".to_owned(), 0.0.into()),
        (
            "open_repair_left_endpoint_projection_spacing_min_m".to_owned(),
            left_endpoint_spacing.min.into(),
        ),
        (
            "open_repair_left_endpoint_projection_spacing_p05_m".to_owned(),
            left_endpoint_spacing.p05.into(),
        ),
        (
            "open_repair_left_endpoint_projection_spacing_median_m".to_owned(),
            left_endpoint_spacing.median.into(),
        ),
        (
            "open_repair_right_endpoint_projection_spacing_min_m".to_owned(),
            right_endpoint_spacing.min.into(),
        ),
        (
            "open_repair_right_endpoint_projection_spacing_p05_m".to_owned(),
            right_endpoint_spacing.p05.into(),
        ),
        (
            "open_repair_right_endpoint_projection_spacing_median_m".to_owned(),
            right_endpoint_spacing.median.into(),
        ),
        (
            "open_repair_lr_projection_ratio_max".to_owned(),
            lr_projection_ratios
                .iter()
                .copied()
                .fold(0.0, f64::max)
                .into(),
        ),
        (
            "open_repair_lr_projection_ratio_p95".to_owned(),
            percentile(lr_projection_ratios, 95.0).into(),
        ),
        (
            "open_repair_frame_endpoint_delta_max_m".to_owned(),
            frame_endpoint_delta
                .iter()
                .copied()
                .fold(0.0, f64::max)
                .into(),
        ),
        (
            "open_repair_frame_endpoint_delta_p95_m".to_owned(),
            percentile(frame_endpoint_delta, 95.0).into(),
        ),
    ];

    Ok((
        BuiltSections {
            left,
            right,
            center: centers,
            normals,
            width_right,
            width_left,
            miss_count,
        },
        meta,
    ))
}

fn interp_points_closed_by_progress(
    progress: &[f64],
    points: &[Point2],
    target_progress: &[f64],
) -> Vec<Point2> {
    if progress.len() != points.len() || points.is_empty() {
        return vec![points.first().copied().unwrap_or([0.0, 0.0]); target_progress.len()];
    }
    if points.len() == 1 {
        return vec![points[0]; target_progress.len()];
    }
    let mut source_progress = progress.to_vec();
    let mut closed_points = points.to_vec();
    source_progress.push(1.0);
    closed_points.push(points[0]);
    let xs = closed_points
        .iter()
        .map(|point| point[0])
        .collect::<Vec<_>>();
    let ys = closed_points
        .iter()
        .map(|point| point[1])
        .collect::<Vec<_>>();
    target_progress
        .iter()
        .map(|value| {
            let target = value.rem_euclid(1.0);
            [
                interp_scalar(target, &source_progress, &xs),
                interp_scalar(target, &source_progress, &ys),
            ]
        })
        .collect()
}

const OPEN_ROUTE_PREPARATION_DENSE_COUNT: usize = 1280;

fn open_area_route_preparation_dense_count(requested_dense_count: usize) -> usize {
    if requested_dense_count == FixedCenterlineStationOptions::default().dense_count {
        OPEN_ROUTE_PREPARATION_DENSE_COUNT
    } else {
        requested_dense_count.max(320)
    }
}

fn station_topology_for_track(track: &TrackAreaContractV1) -> StationTopology {
    if track.trajectory_mode == "open" {
        StationTopology::Open
    } else {
        StationTopology::Closed
    }
}

fn midref_reference_station_options() -> StationBuilderOptions {
    StationBuilderOptions {
        sample_count: 160,
        smoothing_window: 1,
        dtw_band_ratio: 0.27,
        dtw_alignment_roll_bias: DtwAlignmentRollBias::Explicit(0),
        centerline_hint_world: None,
        dtw_centerline_normal_cost_weight: 0.0,
        dtw_slide_cost_weight: 1.0,
        turn_density_gain: 1.5,
        turn_analysis_smoothing_window: 1,
        turn_density_source: "centerline".to_owned(),
        density_smooth_window: 1,
        density_max_adjacent_ratio: 0.0,
        density_slew_mode: "log_smooth".to_owned(),
        target_spacing_max_adjacent_ratio: 0.0,
        target_spacing_metric: "centerline".to_owned(),
        zero_station_normal_fix: false,
    }
}

fn prepare_closed_area_route_pair(
    track: &TrackAreaContractV1,
    control: StationGenerationControl<'_>,
) -> StationBuildResult<PreparedRoutePair> {
    let corridor = build_boundary_pair_track_with_control(
        &track.left_boundary_xy_m,
        &track.right_boundary_xy_m,
        &midref_reference_station_options(),
        control,
    )?;
    control.checkpoint()?;
    Ok(prepared_route_pair_from_boundary_pair_track(
        StationTopology::Closed,
        corridor,
    ))
}

fn prepare_open_area_route_pair(
    track: &TrackAreaContractV1,
    options: &FixedCenterlineStationOptions,
    control: StationGenerationControl<'_>,
) -> StationBuildResult<PreparedRoutePair> {
    let dense_count = open_area_route_preparation_dense_count(options.dense_count);
    let left_dense = resample_open_polyline(&track.left_boundary_xy_m, dense_count);
    control.checkpoint()?;
    let right_dense = resample_open_polyline(&track.right_boundary_xy_m, dense_count);
    control.checkpoint()?;
    let band = 20_usize.max((dense_count as f64 * options.dtw_frame_band_ratio).round() as usize);
    let (right_paired_dense, path_pairs, mut metadata) = pair_boundaries_dtw_open(
        &left_dense,
        &right_dense,
        band,
        options.dtw_frame_centerline_normal_cost_weight,
        options.dtw_frame_slide_cost_weight,
        options.dtw_frame_slide_step_penalty,
        options.dtw_frame_slide_repeat_penalty,
        control,
    )?;
    let corridor = build_section_based_corridor(
        &left_dense,
        &right_paired_dense,
        options.dtw_frame_smoothing_window,
        StationTopology::Open,
        control,
    )?;
    metadata.extend(corridor.metadata.clone());
    metadata.push(("dtw_band".to_owned(), JsonValue::Integer(band as i64)));
    metadata.push((
        "dtw_path_length".to_owned(),
        JsonValue::Integer(path_pairs.len() as i64),
    ));
    Ok(prepared_route_pair_from_boundary_pair_track(
        StationTopology::Open,
        BoundaryPairTrack {
            metadata,
            ..corridor
        },
    ))
}

fn prepare_production_area_route_pair(
    track: &TrackAreaContractV1,
    options: &FixedCenterlineStationOptions,
    control: StationGenerationControl<'_>,
) -> StationBuildResult<PreparedRoutePair> {
    control.checkpoint()?;
    match station_topology_for_track(track) {
        StationTopology::Closed => prepare_closed_area_route_pair(track, control),
        StationTopology::Open => prepare_open_area_route_pair(track, options, control),
    }
}

fn reference_track_from_prepared_route(prepared: &PreparedRoutePair) -> ReferenceTrack {
    ReferenceTrack {
        centerline_world: prepared.centerline_route.clone(),
        width_right_m: prepared.width_right.clone(),
        width_left_m: prepared.width_left.clone(),
    }
}

#[must_use]
pub fn legacy_station_builder_for_track(track: &TrackAreaContractV1) -> ProductionStationBuilder {
    if track.trajectory_mode == "open" {
        ProductionStationBuilder::OpenAreaStation
    } else {
        ProductionStationBuilder::CanonicalAreaStation
    }
}

#[must_use]
pub fn build_production_sections_track_view(
    track: &TrackAreaContractV1,
    options: &FixedCenterlineStationOptions,
) -> SectionsTrackViewV1 {
    let control = StationGenerationControl::never_cancelled();
    let prepared = prepare_production_area_route_pair(track, options, control)
        .expect("non-cancellable station preparation cannot be cancelled");
    build_production_sections_track_view_from_prepared(track, options, prepared, control)
        .expect("non-cancellable station construction cannot be cancelled")
}

pub(crate) fn prepare_production_station_plan_with_control(
    track: &TrackAreaContractV1,
    options: &FixedCenterlineStationOptions,
    control: StationGenerationControl<'_>,
) -> StationBuildResult<PreparedProductionStationPlan> {
    let prepared = prepare_production_area_route_pair(track, options, control)?;
    control.checkpoint()?;
    let complexity = station_complexity_report_from_prepared_route(track, &prepared, control)?;
    Ok(PreparedProductionStationPlan {
        prepared,
        complexity,
    })
}

#[cfg(test)]
fn prepare_production_station_plan(
    track: &TrackAreaContractV1,
    options: &FixedCenterlineStationOptions,
) -> PreparedProductionStationPlan {
    prepare_production_station_plan_with_control(
        track,
        options,
        StationGenerationControl::never_cancelled(),
    )
    .expect("non-cancellable station preparation cannot be cancelled")
}

pub(crate) fn build_production_sections_track_view_from_plan_with_control(
    track: &TrackAreaContractV1,
    options: &FixedCenterlineStationOptions,
    plan: PreparedProductionStationPlan,
    control: StationGenerationControl<'_>,
) -> StationBuildResult<SectionsTrackViewV1> {
    build_production_sections_track_view_from_prepared(track, options, plan.prepared, control)
}

#[cfg(test)]
fn build_production_sections_track_view_from_plan(
    track: &TrackAreaContractV1,
    options: &FixedCenterlineStationOptions,
    plan: PreparedProductionStationPlan,
) -> SectionsTrackViewV1 {
    build_production_sections_track_view_from_plan_with_control(
        track,
        options,
        plan,
        StationGenerationControl::never_cancelled(),
    )
    .expect("non-cancellable station construction cannot be cancelled")
}

fn build_production_sections_track_view_from_prepared(
    track: &TrackAreaContractV1,
    options: &FixedCenterlineStationOptions,
    prepared: PreparedRoutePair,
    control: StationGenerationControl<'_>,
) -> StationBuildResult<SectionsTrackViewV1> {
    control.checkpoint()?;
    let mut view = match prepared.topology {
        StationTopology::Open => build_open_area_station_sections_track_view_from_prepared(
            track, options, prepared, control,
        )?,
        StationTopology::Closed => {
            let reference = reference_track_from_prepared_route(&prepared);
            build_area_station_sections_track_view_with_control(
                track, &reference, options, control,
            )?
        }
    };
    control.checkpoint()?;

    upsert_metadata(
        &mut view.metadata,
        "station_geometry_source",
        "universal_area_route_pair".into(),
    );
    upsert_metadata(
        &mut view.metadata,
        "station_builder",
        "universal_area_route_pair".into(),
    );
    upsert_metadata(
        &mut view.metadata,
        "production_station_builder",
        "universal_area_route_pair".into(),
    );
    upsert_metadata(
        &mut view.metadata,
        "route_topology_policy",
        track.trajectory_mode.clone().into(),
    );
    if track.trajectory_mode != "open" {
        upsert_metadata(
            &mut view.metadata,
            "reference_source",
            "generated_from_raw_boundaries".into(),
        );
        upsert_metadata(
            &mut view.metadata,
            "reference_generator",
            "build_midref_reference_track_from_raw_boundaries".into(),
        );
        upsert_metadata(
            &mut view.metadata,
            "reference_sample_count",
            JsonValue::Integer(160),
        );
        upsert_metadata(
            &mut view.metadata,
            "reference_smoothing_window",
            JsonValue::Integer(1),
        );
        upsert_metadata(
            &mut view.metadata,
            "reference_turn_density_gain",
            1.5_f64.into(),
        );
        upsert_metadata(
            &mut view.metadata,
            "reference_dtw_band_ratio",
            0.27_f64.into(),
        );
        upsert_metadata(
            &mut view.metadata,
            "reference_dtw_alignment_roll_bias",
            JsonValue::Integer(0),
        );
    }

    upsert_metadata(
        &mut view.metadata,
        "trajectory_mode",
        track.trajectory_mode.clone().into(),
    );

    Ok(orient_sections_for_requested_direction(view, track))
}

/// Runs an explicitly selected historical builder for regression comparison.
/// Product station generation must use `build_production_sections_track_view`.
#[must_use]
pub fn build_legacy_sections_track_view(
    track: &TrackAreaContractV1,
    options: &FixedCenterlineStationOptions,
    builder: ProductionStationBuilder,
) -> SectionsTrackViewV1 {
    let view = match builder {
        ProductionStationBuilder::CanonicalAreaStation => {
            assert_ne!(
                track.trajectory_mode, "open",
                "open tracks must use open_area_station_generator"
            );
            let reference = build_midref_reference_track_from_raw_boundaries(track);
            let mut view = build_area_station_sections_track_view(track, &reference, options);
            upsert_metadata(
                &mut view.metadata,
                "station_geometry_source",
                "canonical_area_station_generator".into(),
            );
            upsert_metadata(
                &mut view.metadata,
                "production_station_builder",
                "canonical_area_station_generator".into(),
            );
            upsert_metadata(
                &mut view.metadata,
                "reference_source",
                "generated_from_raw_boundaries".into(),
            );
            upsert_metadata(
                &mut view.metadata,
                "reference_generator",
                "build_midref_reference_track_from_raw_boundaries".into(),
            );
            upsert_metadata(
                &mut view.metadata,
                "reference_sample_count",
                JsonValue::Integer(160),
            );
            upsert_metadata(
                &mut view.metadata,
                "reference_smoothing_window",
                JsonValue::Integer(1),
            );
            upsert_metadata(
                &mut view.metadata,
                "reference_turn_density_gain",
                1.5_f64.into(),
            );
            upsert_metadata(
                &mut view.metadata,
                "reference_dtw_band_ratio",
                0.27_f64.into(),
            );
            upsert_metadata(
                &mut view.metadata,
                "reference_dtw_alignment_roll_bias",
                JsonValue::Integer(0),
            );
            view
        }
        ProductionStationBuilder::OpenAreaStation => {
            assert_eq!(
                track.trajectory_mode, "open",
                "open_area_station_generator requires trajectory_mode=open"
            );
            let mut view = build_open_area_station_sections_track_view(track, options);
            upsert_metadata(
                &mut view.metadata,
                "station_geometry_source",
                "open_area_station_generator".into(),
            );
            upsert_metadata(
                &mut view.metadata,
                "production_station_builder",
                "open_area_station_generator".into(),
            );
            view
        }
        ProductionStationBuilder::GeneratedBoundaryPair => {
            assert_ne!(
                track.trajectory_mode, "open",
                "open tracks cannot use generated_boundary_pair"
            );
            let mut view = build_generated_boundary_pair_sections_track_view(track, options);
            upsert_metadata(
                &mut view.metadata,
                "station_geometry_source",
                "generated_boundary_pair".into(),
            );
            upsert_metadata(
                &mut view.metadata,
                "production_station_builder",
                "generated_boundary_pair".into(),
            );
            view
        }
    };

    orient_sections_for_requested_direction(view, track)
}

const AUTO_STATION_COUNTS: [usize; 9] = [32, 40, 48, 64, 80, 96, 120, 160, 200];
const AUTO_CLOSED_MIN_STATION_COUNT: usize = 64;
const AUTO_OPEN_MIN_STATION_COUNT: usize = 32;
const AUTO_STATION_WIDTH_SAMPLES_PER_CORRIDOR_WIDTH: f64 = 1.20;
const AUTO_STATION_ALLOWED_HEADING_STEP_RAD: f64 = 0.30;
const AUTO_STATION_CROSSING_ZONE_BONUS: f64 = 8.0;

#[must_use]
pub fn estimate_station_complexity(
    track: &TrackAreaContractV1,
    options: &FixedCenterlineStationOptions,
) -> StationComplexityReport {
    let prepared = prepare_area_route_pair_for_complexity(track, options);
    station_complexity_report_from_prepared_route(
        track,
        &prepared,
        StationGenerationControl::never_cancelled(),
    )
    .expect("non-cancellable complexity report cannot be cancelled")
}

fn prepare_area_route_pair_for_complexity(
    track: &TrackAreaContractV1,
    options: &FixedCenterlineStationOptions,
) -> PreparedRoutePair {
    prepare_production_area_route_pair(track, options, StationGenerationControl::never_cancelled())
        .expect("non-cancellable complexity preparation cannot be cancelled")
}

fn station_complexity_report_from_prepared_route(
    track: &TrackAreaContractV1,
    prepared: &PreparedRoutePair,
    control: StationGenerationControl<'_>,
) -> StationBuildResult<StationComplexityReport> {
    let closed = prepared.topology == StationTopology::Closed;
    let centerline = &prepared.centerline_route;
    let segment_lengths = if closed {
        closed_polyline_arclength(centerline).1
    } else {
        centerline
            .windows(2)
            .map(|pair| distance(pair[0], pair[1]))
            .collect()
    };
    let route_length_m = segment_lengths.iter().sum::<f64>();
    control.checkpoint_phase("complexity_preparation")?;
    let total_abs_heading_rad = if closed {
        let mut analysis_centerline = centerline.clone();
        smooth_area_centerline_samples(&mut analysis_centerline, 24, 0.25);
        polyline_total_abs_heading(&analysis_centerline, true)
    } else {
        polyline_total_abs_heading(centerline, false)
    };
    control.checkpoint_phase("complexity_preparation")?;
    let mut widths = prepared
        .width_left
        .iter()
        .zip(&prepared.width_right)
        .filter_map(|(left, right)| {
            let width = left + right;
            (width.is_finite() && width > 1e-6).then_some(width)
        })
        .collect::<Vec<_>>();
    widths.sort_by(f64::total_cmp);
    let width_p10_m = percentile_sorted(&widths, 0.10).max(1e-6);
    let width_median_m = percentile_sorted(&widths, 0.50).max(width_p10_m);
    let raw_max_segment_m = raw_boundary_max_segment_length(track, closed);
    let max_segment_to_width_ratio = raw_max_segment_m / width_median_m.max(1e-6);
    let crossing_zone_count = if closed {
        closed_polyline_self_intersection_count(centerline).max(0) as usize
    } else {
        0
    };

    let minimum_count = if closed {
        AUTO_CLOSED_MIN_STATION_COUNT
    } else {
        AUTO_OPEN_MIN_STATION_COUNT
    };
    let spatial_count =
        AUTO_STATION_WIDTH_SAMPLES_PER_CORRIDOR_WIDTH * route_length_m / width_p10_m;
    let heading_count = total_abs_heading_rad / AUTO_STATION_ALLOWED_HEADING_STEP_RAD;
    let raw_complexity = (minimum_count as f64).max(spatial_count).max(heading_count)
        + crossing_zone_count as f64 * AUTO_STATION_CROSSING_ZONE_BONUS;
    let recommended_station_count = quantize_station_count(raw_complexity.ceil() as usize);
    let complexity_score =
        (raw_complexity / *AUTO_STATION_COUNTS.last().unwrap() as f64).clamp(0.0, 1.0);

    Ok(StationComplexityReport {
        recommended_station_count,
        complexity_score,
        route_length_m,
        total_abs_heading_rad,
        width_p10_m,
        width_median_m,
        max_segment_to_width_ratio,
        crossing_zone_count,
        raw_left_boundary_count: track.left_boundary_xy_m.len(),
        raw_right_boundary_count: track.right_boundary_xy_m.len(),
    })
}

fn quantize_station_count(value: usize) -> usize {
    AUTO_STATION_COUNTS
        .iter()
        .copied()
        .find(|candidate| *candidate >= value)
        .unwrap_or(*AUTO_STATION_COUNTS.last().unwrap())
}

fn percentile_sorted(values: &[f64], percentile: f64) -> f64 {
    if values.is_empty() {
        return 0.0;
    }
    let index = ((values.len() - 1) as f64 * percentile.clamp(0.0, 1.0)).round() as usize;
    values[index]
}

fn polyline_total_abs_heading(points: &[Point2], closed: bool) -> f64 {
    if points.len() < 3 {
        return 0.0;
    }
    let range = if closed {
        0..points.len()
    } else {
        1..points.len() - 1
    };
    range
        .map(|index| {
            let previous = if index == 0 {
                points[points.len() - 1]
            } else {
                points[index - 1]
            };
            let next = if index + 1 == points.len() {
                points[0]
            } else {
                points[index + 1]
            };
            let incoming = point_sub(points[index], previous);
            let outgoing = point_sub(next, points[index]);
            cross(incoming, outgoing)
                .atan2(dot(incoming, outgoing))
                .abs()
        })
        .sum()
}

fn raw_boundary_max_segment_length(track: &TrackAreaContractV1, closed: bool) -> f64 {
    track
        .left_boundary_xy_m
        .iter()
        .zip(
            track
                .left_boundary_xy_m
                .iter()
                .skip(1)
                .chain(closed.then(|| &track.left_boundary_xy_m[0])),
        )
        .chain(
            track.right_boundary_xy_m.iter().zip(
                track
                    .right_boundary_xy_m
                    .iter()
                    .skip(1)
                    .chain(closed.then(|| &track.right_boundary_xy_m[0])),
            ),
        )
        .map(|(a, b)| distance(*a, *b))
        .fold(0.0, f64::max)
}

fn closed_centerline_direction(points: &[Point2]) -> Option<EffectiveTrackDirection> {
    if points.len() < 3 {
        return None;
    }

    let signed_double_area = points
        .iter()
        .zip(points.iter().cycle().skip(1))
        .take(points.len())
        .map(|(current, next)| current[0] * next[1] - next[0] * current[1])
        .sum::<f64>();

    if signed_double_area.abs() <= 1e-9 {
        None
    } else if signed_double_area > 0.0 {
        Some(EffectiveTrackDirection::Counterclockwise)
    } else {
        Some(EffectiveTrackDirection::Clockwise)
    }
}

fn reverse_closed_series_preserving_anchor<T>(values: &mut [T]) {
    if values.len() > 1 {
        values[1..].reverse();
    }
}

fn orient_sections_for_requested_direction(
    mut view: SectionsTrackViewV1,
    track: &TrackAreaContractV1,
) -> SectionsTrackViewV1 {
    if track.trajectory_mode == "open" {
        return view;
    }

    let source = closed_centerline_direction(&view.centerline_xy_m);
    let Some(requested) = EffectiveTrackDirection::from_requested(track.direction.as_deref())
    else {
        if let Some(source) = source {
            upsert_metadata(
                &mut view.metadata,
                "requested_direction",
                source.as_str().into(),
            );
            upsert_metadata(
                &mut view.metadata,
                "source_station_direction",
                source.as_str().into(),
            );
            upsert_metadata(
                &mut view.metadata,
                "effective_direction",
                source.as_str().into(),
            );
            upsert_metadata(&mut view.metadata, "direction_reversed", false.into());
        }
        return view;
    };

    upsert_metadata(
        &mut view.metadata,
        "requested_direction",
        requested.as_str().into(),
    );
    if let Some(source) = source {
        upsert_metadata(
            &mut view.metadata,
            "source_station_direction",
            source.as_str().into(),
        );
    }

    let should_reverse = source.is_some_and(|source| source != requested);
    if should_reverse {
        reverse_closed_series_preserving_anchor(&mut view.centerline_xy_m);

        std::mem::swap(&mut view.left_boundary_xy_m, &mut view.right_boundary_xy_m);
        reverse_closed_series_preserving_anchor(&mut view.left_boundary_xy_m);
        reverse_closed_series_preserving_anchor(&mut view.right_boundary_xy_m);

        std::mem::swap(&mut view.width_left_m, &mut view.width_right_m);
        reverse_closed_series_preserving_anchor(&mut view.width_left_m);
        reverse_closed_series_preserving_anchor(&mut view.width_right_m);

        reverse_closed_series_preserving_anchor(&mut view.normals_xy);
        for normal in &mut view.normals_xy {
            *normal = [-normal[0], -normal[1]];
        }
        reverse_closed_series_preserving_anchor(&mut view.section_dirs_xy);
        for direction in &mut view.section_dirs_xy {
            *direction = [-direction[0], -direction[1]];
        }

        let (station_s_m, _) = closed_polyline_arclength(&view.centerline_xy_m);
        view.station_s_m = station_s_m
            .into_iter()
            .take(view.centerline_xy_m.len())
            .collect();
    }

    let effective = closed_centerline_direction(&view.centerline_xy_m);
    if let Some(effective) = effective {
        upsert_metadata(
            &mut view.metadata,
            "effective_direction",
            effective.as_str().into(),
        );
        view.view_id = format!("{}_dir_{}", view.view_id, effective.as_str());
    }
    upsert_metadata(
        &mut view.metadata,
        "direction_reversed",
        should_reverse.into(),
    );

    view
}

#[must_use]
pub fn build_curvature_weighted_sections_track_view(
    track: &TrackAreaContractV1,
    options: &FixedCenterlineStationOptions,
) -> SectionsTrackViewV1 {
    let mut reference_options = StationBuilderOptions {
        sample_count: options.dense_count.max(320),
        smoothing_window: options.dtw_frame_smoothing_window,
        dtw_band_ratio: options.dtw_frame_band_ratio,
        dtw_alignment_roll_bias: options.dtw_frame_alignment_roll_bias,
        centerline_hint_world: None,
        dtw_centerline_normal_cost_weight: 0.0,
        dtw_slide_cost_weight: 1.0,
        turn_density_gain: options.dtw_frame_turn_density_gain,
        turn_analysis_smoothing_window: options.turn_analysis_smoothing_window,
        turn_density_source: options.turn_density_source.clone(),
        density_smooth_window: options.density_smooth_window,
        density_max_adjacent_ratio: options.density_max_adjacent_ratio,
        density_slew_mode: options.density_slew_mode.clone(),
        target_spacing_max_adjacent_ratio: options.target_spacing_max_adjacent_ratio,
        target_spacing_metric: options.target_spacing_metric.clone(),
        zero_station_normal_fix: false,
    };
    reference_options.sample_count = reference_options.sample_count.max(options.sample_count * 4);
    let reference_frame = build_boundary_pair_track(
        &track.left_boundary_xy_m,
        &track.right_boundary_xy_m,
        &reference_options,
    );
    let reference = ReferenceTrack {
        centerline_world: reference_frame.centerline_world,
        width_right_m: reference_frame.width_right,
        width_left_m: reference_frame.width_left,
    };
    let mut view = build_fixed_centerline_sections_track_view(track, &reference, options);

    view.view_id = format!(
        "{}_curvature_weighted_sections_sc{}_dense{}",
        track.track_id, options.sample_count, options.dense_count
    );
    view.metadata.push((
        "station_builder".to_owned(),
        "curvature_weighted_generated_reference".into(),
    ));
    upsert_metadata(
        &mut view.metadata,
        "source",
        "curvature_weighted_generated_reference".into(),
    );

    view
}

#[must_use]
pub fn build_generated_boundary_pair_sections_track_view(
    track: &TrackAreaContractV1,
    options: &FixedCenterlineStationOptions,
) -> SectionsTrackViewV1 {
    let station_options = StationBuilderOptions {
        sample_count: options.sample_count,
        smoothing_window: options.dtw_frame_smoothing_window,
        dtw_band_ratio: options.dtw_frame_band_ratio,
        dtw_alignment_roll_bias: options.dtw_frame_alignment_roll_bias,
        centerline_hint_world: None,
        dtw_centerline_normal_cost_weight: 0.0,
        dtw_slide_cost_weight: 1.0,
        turn_density_gain: options.dtw_frame_turn_density_gain,
        turn_analysis_smoothing_window: options.turn_analysis_smoothing_window,
        turn_density_source: options.turn_density_source.clone(),
        density_smooth_window: options.density_smooth_window,
        density_max_adjacent_ratio: options.density_max_adjacent_ratio,
        density_slew_mode: options.density_slew_mode.clone(),
        target_spacing_max_adjacent_ratio: options.target_spacing_max_adjacent_ratio,
        target_spacing_metric: options.target_spacing_metric.clone(),
        zero_station_normal_fix: options.zero_station_normal_fix,
    };
    let mut view = build_sections_track_view(track, &station_options);
    view.view_id = format!(
        "{}_generated_boundary_pair_sections_sc{}_sw{}_g{}_roll{}",
        track.track_id,
        station_options.sample_count,
        station_options.smoothing_window,
        station_options.turn_density_gain,
        station_options.dtw_alignment_roll_bias
    );
    view.metadata.push((
        "station_builder".to_owned(),
        "generated_boundary_pair".into(),
    ));
    view
}

#[must_use]
pub fn build_sections_track_view(
    track: &TrackAreaContractV1,
    options: &StationBuilderOptions,
) -> SectionsTrackViewV1 {
    let prepared = normalize_closed_boundary_pair_to_route_pair(
        &track.left_boundary_xy_m,
        &track.right_boundary_xy_m,
        options,
    );
    let (station_s, segment_lengths) = closed_polyline_arclength(&prepared.centerline_route);
    let total_length = station_s.last().copied().unwrap_or(0.0);
    let mut metadata = prepared.metadata.clone();
    metadata.push(("total_length_m".to_owned(), total_length.into()));
    let quality_metrics = metadata
        .iter()
        .filter(|(key, _)| {
            matches!(
                key.as_str(),
                "adjacent_section_crossing_count"
                    | "min_station_spacing_m"
                    | "median_station_spacing_m"
                    | "max_adjacent_normal_rotation_deg"
                    | "zero_station_normal_fix_applied"
            )
        })
        .map(|(key, value)| (key.clone(), value.clone()))
        .collect::<JsonObject>();

    SectionsTrackViewV1 {
        schema_version: SectionsTrackViewV1::SCHEMA_VERSION.to_owned(),
        view_id: format!(
            "{}_sections_sc{}_sw{}_g{}_roll{}",
            track.track_id,
            options.sample_count,
            options.smoothing_window,
            options.turn_density_gain,
            options.dtw_alignment_roll_bias
        ),
        track_id: track.track_id.clone(),
        station_s_m: station_s
            .into_iter()
            .take(prepared.centerline_route.len())
            .collect(),
        centerline_xy_m: prepared.centerline_route,
        left_boundary_xy_m: prepared.left_route,
        right_boundary_xy_m: prepared.right_route,
        normals_xy: prepared.normals.clone(),
        width_left_m: prepared.width_left,
        width_right_m: prepared.width_right,
        section_dirs_xy: prepared.normals,
        quality_metrics: {
            let mut result = quality_metrics;
            if !segment_lengths.is_empty() {
                result.push((
                    "min_segment_length_m".to_owned(),
                    segment_lengths
                        .iter()
                        .copied()
                        .fold(f64::INFINITY, f64::min)
                        .into(),
                ));
                result.push((
                    "median_segment_length_m".to_owned(),
                    median(segment_lengths.clone()).into(),
                ));
            }
            result
        },
        metadata,
    }
}

fn prepared_route_pair_from_boundary_pair_track(
    topology: StationTopology,
    corridor: BoundaryPairTrack,
) -> PreparedRoutePair {
    let progress = station_frame_progress_for_path(topology, corridor.centerline_world.len());

    PreparedRoutePair {
        topology,
        left_route: corridor.left_world,
        right_route: corridor.right_world,
        left_progress: corridor.left_route_progress,
        right_progress: corridor.right_route_progress,
        centerline_route: corridor.centerline_world,
        normals: corridor.normals_world,
        width_left: corridor.width_left,
        width_right: corridor.width_right,
        shared_progress: progress,
        metadata: corridor.metadata,
    }
}

fn normalize_closed_boundary_pair_to_route_pair(
    left_world: &[Point2],
    right_world: &[Point2],
    options: &StationBuilderOptions,
) -> PreparedRoutePair {
    let corridor = build_boundary_pair_track(left_world, right_world, options);
    prepared_route_pair_from_boundary_pair_track(StationTopology::Closed, corridor)
}

#[must_use]
pub fn build_boundary_pair_track(
    left_world: &[Point2],
    right_world: &[Point2],
    options: &StationBuilderOptions,
) -> BoundaryPairTrack {
    build_boundary_pair_track_with_control(
        left_world,
        right_world,
        options,
        StationGenerationControl::never_cancelled(),
    )
    .expect("non-cancellable boundary pairing cannot be cancelled")
}

fn build_boundary_pair_track_with_control(
    left_world: &[Point2],
    right_world: &[Point2],
    options: &StationBuilderOptions,
    control: StationGenerationControl<'_>,
) -> StationBuildResult<BoundaryPairTrack> {
    control.checkpoint()?;
    let sample_count = options.sample_count.max(20);
    let pairing_sample_count = sample_count.max((sample_count * 4).clamp(320, 1200));
    let (left_pairing, left_route_progress) =
        resample_closed_polyline_with_route_progress(left_world, pairing_sample_count, control)?;
    let (right_pairing, right_route_progress) =
        resample_closed_polyline_with_route_progress(right_world, pairing_sample_count, control)?;
    let mut prepared = build_boundary_pair_track_from_pairing_samples(
        left_pairing,
        right_pairing,
        left_route_progress,
        right_route_progress,
        vec![(
            "closed_pair_resample_mode".to_owned(),
            "canonical_route_arclength".into(),
        )],
        options,
        control,
    )?;
    prepared.metadata.push((
        "closed_pair_normalizer".to_owned(),
        "canonical_route_anchored_arclength_dtw".into(),
    ));
    Ok(prepared)
}

fn resample_closed_polyline_with_route_progress(
    points: &[Point2],
    sample_count: usize,
    control: StationGenerationControl<'_>,
) -> StationBuildResult<(Vec<Point2>, Vec<f64>)> {
    if points.is_empty() || sample_count == 0 {
        return Ok((Vec::new(), Vec::new()));
    }
    if points.len() == 1 {
        return Ok((vec![points[0]; sample_count], vec![0.0; sample_count]));
    }

    let (_, segment_lengths) = closed_polyline_arclength(points);
    let total = segment_lengths.iter().sum::<f64>();
    if !total.is_finite() || total <= 1e-9 {
        return Ok((vec![points[0]; sample_count], vec![0.0; sample_count]));
    }

    let mut samples = Vec::with_capacity(sample_count);
    let mut route_progress = Vec::with_capacity(sample_count);
    let mut segment_index = 0_usize;
    let mut segment_start_s = 0.0;
    for sample_index in 0..sample_count {
        if sample_index % 64 == 0 {
            control.checkpoint_phase("adaptive_resampling")?;
        }
        let target_s = total * sample_index as f64 / sample_count as f64;
        while segment_index + 1 < segment_lengths.len()
            && target_s > segment_start_s + segment_lengths[segment_index]
        {
            segment_start_s += segment_lengths[segment_index];
            segment_index += 1;
        }
        let segment_length = segment_lengths[segment_index];
        let fraction = if segment_length <= 1e-12 {
            0.0
        } else {
            ((target_s - segment_start_s) / segment_length).clamp(0.0, 1.0)
        };
        samples.push(point_add(
            points[segment_index],
            point_scale(
                point_sub(
                    points[(segment_index + 1) % points.len()],
                    points[segment_index],
                ),
                fraction,
            ),
        ));
        route_progress.push(target_s / total);
    }
    Ok((samples, route_progress))
}

fn build_boundary_pair_track_from_pairing_samples(
    left_pairing: Vec<Point2>,
    right_pairing: Vec<Point2>,
    left_route_progress: Vec<f64>,
    right_route_progress: Vec<f64>,
    pair_resample_meta: JsonObject,
    options: &StationBuilderOptions,
    control: StationGenerationControl<'_>,
) -> StationBuildResult<BoundaryPairTrack> {
    let sample_count = options.sample_count.max(20);
    let pairing_sample_count = left_pairing
        .len()
        .max(right_pairing.len())
        .max(sample_count);
    let band =
        20_usize.max((pairing_sample_count as f64 * options.dtw_band_ratio).round() as usize);
    let (right_aligned, path_pairs, right_route_progress_aligned, mut meta) = pair_boundaries_dtw(
        &left_pairing,
        &right_pairing,
        &left_route_progress,
        &right_route_progress,
        band,
        options.dtw_alignment_roll_bias,
        options.centerline_hint_world.as_deref(),
        options.dtw_centerline_normal_cost_weight,
        options.dtw_slide_cost_weight,
        control,
    )?;
    control.checkpoint()?;
    let (
        mut sampled_left,
        mut sampled_right,
        sampled_left_route_progress,
        sampled_right_route_progress,
        adaptive_meta,
    ) = resample_paired_boundaries_adaptive(
        &left_pairing,
        &right_aligned,
        &left_route_progress,
        &right_route_progress_aligned,
        &path_pairs,
        sample_count,
        options.turn_density_gain,
        options.turn_analysis_smoothing_window,
        &options.turn_density_source,
        options.density_smooth_window,
        options.density_max_adjacent_ratio,
        &options.density_slew_mode,
        options.target_spacing_max_adjacent_ratio,
        &options.target_spacing_metric,
        control,
    )?;
    control.checkpoint()?;
    let endpoint_plateau_meta = repair_closed_endpoint_plateaus_by_route_progress(
        &mut sampled_left,
        &mut sampled_right,
        &left_pairing,
        &right_aligned,
        control,
    )?;
    meta.push((
        "adaptive_sampled_pair_crossing_count".to_owned(),
        JsonValue::Integer(station_horizon_crossing_count(
            &sampled_left,
            &sampled_right,
            2,
        )),
    ));
    meta.push((
        "adaptive_sampled_left_progress_max_step".to_owned(),
        closed_progress_max_step(&sampled_left_route_progress).into(),
    ));
    meta.push((
        "adaptive_sampled_right_progress_max_step".to_owned(),
        closed_progress_max_step(&sampled_right_route_progress).into(),
    ));
    let mut corridor = build_section_based_corridor(
        &sampled_left,
        &sampled_right,
        options.smoothing_window,
        StationTopology::Closed,
        control,
    )?;
    corridor.left_route_progress = sampled_left_route_progress;
    corridor.right_route_progress = sampled_right_route_progress;

    if options.zero_station_normal_fix {
        let applied = apply_zero_station_normal_fix(&mut corridor);
        meta.push(("zero_station_normal_fix".to_owned(), true.into()));
        meta.push(("zero_station_normal_fix_applied".to_owned(), applied.into()));
    } else {
        meta.push(("zero_station_normal_fix".to_owned(), false.into()));
        meta.push(("zero_station_normal_fix_applied".to_owned(), false.into()));
    }

    meta.push(("dtw_band".to_owned(), JsonValue::Integer(band as i64)));
    meta.push((
        "pairing_sample_count".to_owned(),
        JsonValue::Integer(pairing_sample_count as i64),
    ));
    meta.push((
        "smoothing_window".to_owned(),
        JsonValue::Integer(options.smoothing_window as i64),
    ));
    meta.push((
        "turn_density_gain".to_owned(),
        options.turn_density_gain.into(),
    ));
    meta.push((
        "turn_analysis_smoothing_window".to_owned(),
        JsonValue::Integer(options.turn_analysis_smoothing_window as i64),
    ));
    meta.push((
        "turn_density_source".to_owned(),
        options.turn_density_source.clone().into(),
    ));
    meta.push(("solution_density_gain".to_owned(), 0.0.into()));
    meta.push((
        "density_smooth_window".to_owned(),
        JsonValue::Integer(options.density_smooth_window as i64),
    ));
    meta.push((
        "density_max_adjacent_ratio".to_owned(),
        options.density_max_adjacent_ratio.into(),
    ));
    meta.push((
        "density_slew_mode".to_owned(),
        options.density_slew_mode.clone().into(),
    ));
    meta.push((
        "target_spacing_max_adjacent_ratio".to_owned(),
        options.target_spacing_max_adjacent_ratio.into(),
    ));
    meta.push((
        "target_spacing_metric".to_owned(),
        options.target_spacing_metric.clone().into(),
    ));
    meta.extend(pair_resample_meta);
    meta.extend(adaptive_meta);
    meta.extend(endpoint_plateau_meta);
    let (min_spacing, median_spacing) = station_spacing_metrics(&corridor.centerline_world);
    meta.push(("min_station_spacing_m".to_owned(), min_spacing.into()));
    meta.push(("median_station_spacing_m".to_owned(), median_spacing.into()));
    meta.push((
        "max_adjacent_normal_rotation_deg".to_owned(),
        max_adjacent_vector_rotation_deg(&corridor.normals_world).into(),
    ));
    meta.push((
        "adjacent_section_crossing_count".to_owned(),
        JsonValue::Integer(station_horizon_crossing_count(
            &corridor.left_world,
            &corridor.right_world,
            2,
        )),
    ));
    corridor.metadata = meta;
    Ok(corridor)
}

fn route_sampled_closed_boundary_pair_by_midpoint_progress(
    left_world: &[Point2],
    right_world: &[Point2],
    target_progress: &[f64],
) -> (Vec<Point2>, Vec<Point2>) {
    let count = left_world.len().min(right_world.len());
    if count < 3 || target_progress.is_empty() {
        return (
            vec![left_world.first().copied().unwrap_or([0.0, 0.0]); target_progress.len()],
            vec![right_world.first().copied().unwrap_or([0.0, 0.0]); target_progress.len()],
        );
    }

    let left = &left_world[..count];
    let right = &right_world[..count];
    let center = left
        .iter()
        .zip(right)
        .map(|(left, right)| midpoint(*left, *right))
        .collect::<Vec<_>>();
    let source_progress = closed_path_progress(&center);

    (
        interp_points_closed_by_progress(&source_progress, left, target_progress),
        interp_points_closed_by_progress(&source_progress, right, target_progress),
    )
}

fn repair_closed_endpoint_plateaus_by_route_progress(
    left: &mut [Point2],
    right: &mut [Point2],
    route_left: &[Point2],
    route_right: &[Point2],
    control: StationGenerationControl<'_>,
) -> StationBuildResult<JsonObject> {
    control.checkpoint_phase("closed_endpoint_repair")?;
    let count = left.len().min(right.len());
    let mut meta = Vec::new();
    let before_left_min = closed_endpoint_spacing_min(left);
    let before_right_min = closed_endpoint_spacing_min(right);
    meta.push((
        "closed_endpoint_plateau_repair_threshold_m".to_owned(),
        CLOSED_ENDPOINT_PLATEAU_EPS_M.into(),
    ));
    meta.push((
        "closed_endpoint_plateau_repair_left_spacing_before_min_m".to_owned(),
        before_left_min.into(),
    ));
    meta.push((
        "closed_endpoint_plateau_repair_right_spacing_before_min_m".to_owned(),
        before_right_min.into(),
    ));

    if count < 4 || route_left.len().min(route_right.len()) < 3 {
        meta.push((
            "closed_endpoint_plateau_repair_candidate_count".to_owned(),
            JsonValue::Integer(0),
        ));
        meta.push((
            "closed_endpoint_plateau_repair_replaced_count".to_owned(),
            JsonValue::Integer(0),
        ));
        meta.push((
            "closed_endpoint_plateau_repair_accepted".to_owned(),
            JsonValue::Bool(false),
        ));
        return Ok(meta);
    }

    let mut replace = vec![false; count];
    for index in 0..count {
        let next = (index + 1) % count;
        if distance(left[index], left[next]) <= CLOSED_ENDPOINT_PLATEAU_EPS_M
            || distance(right[index], right[next]) <= CLOSED_ENDPOINT_PLATEAU_EPS_M
        {
            replace[index] = true;
            replace[next] = true;
        }
    }
    let candidate_count = replace.iter().filter(|value| **value).count();
    meta.push((
        "closed_endpoint_plateau_repair_candidate_count".to_owned(),
        JsonValue::Integer(candidate_count as i64),
    ));
    if candidate_count == 0 {
        meta.push((
            "closed_endpoint_plateau_repair_replaced_count".to_owned(),
            JsonValue::Integer(0),
        ));
        meta.push((
            "closed_endpoint_plateau_repair_accepted".to_owned(),
            JsonValue::Bool(false),
        ));
        return Ok(meta);
    }

    let centers = left
        .iter()
        .zip(right.iter())
        .take(count)
        .map(|(left, right)| midpoint(*left, *right))
        .collect::<Vec<_>>();
    let target_progress = closed_path_progress(&centers);
    let (route_sampled_left, route_sampled_right) =
        route_sampled_closed_boundary_pair_by_midpoint_progress(
            route_left,
            route_right,
            &target_progress,
        );
    if route_sampled_left.len() < count || route_sampled_right.len() < count {
        meta.push((
            "closed_endpoint_plateau_repair_replaced_count".to_owned(),
            JsonValue::Integer(0),
        ));
        meta.push((
            "closed_endpoint_plateau_repair_accepted".to_owned(),
            JsonValue::Bool(false),
        ));
        meta.push((
            "closed_endpoint_plateau_repair_rejected_reason".to_owned(),
            "route_sample_count_mismatch".into(),
        ));
        return Ok(meta);
    }

    let mut route_candidate_left = left[..count].to_vec();
    let mut route_candidate_right = right[..count].to_vec();
    for index in 0..count {
        if replace[index] {
            route_candidate_left[index] = route_sampled_left[index];
            route_candidate_right[index] = route_sampled_right[index];
        }
    }
    let mut spread_candidate_left = left[..count].to_vec();
    let mut spread_candidate_right = right[..count].to_vec();
    let spread_left_count =
        spread_closed_endpoint_plateaus_by_neighbors(&mut spread_candidate_left);
    let spread_right_count =
        spread_closed_endpoint_plateaus_by_neighbors(&mut spread_candidate_right);

    let before_score = closed_endpoint_plateau_topology_score(left, right);
    let route_after_score =
        closed_endpoint_plateau_topology_score(&route_candidate_left, &route_candidate_right);
    let route_after_left_min = closed_endpoint_spacing_min(&route_candidate_left);
    let route_after_right_min = closed_endpoint_spacing_min(&route_candidate_right);
    let spread_after_score =
        closed_endpoint_plateau_topology_score(&spread_candidate_left, &spread_candidate_right);
    let spread_after_left_min = closed_endpoint_spacing_min(&spread_candidate_left);
    let spread_after_right_min = closed_endpoint_spacing_min(&spread_candidate_right);
    let route_after_min = route_after_left_min.min(route_after_right_min);
    let route_spacing_improved = route_after_min > before_left_min.min(before_right_min) + 1.0e-9
        && route_after_left_min > CLOSED_ENDPOINT_PLATEAU_EPS_M
        && route_after_right_min > CLOSED_ENDPOINT_PLATEAU_EPS_M;
    let spread_spacing_improved = (spread_left_count > 0
        && spread_after_left_min > before_left_min + 1.0e-9)
        || (spread_right_count > 0 && spread_after_right_min > before_right_min + 1.0e-9);
    let spread_spacing_improved = spread_spacing_improved
        && spread_after_left_min > CLOSED_ENDPOINT_PLATEAU_EPS_M
        && spread_after_right_min > CLOSED_ENDPOINT_PLATEAU_EPS_M;
    let route_ok = route_after_score <= before_score && route_spacing_improved;
    let spread_ok = spread_after_score <= before_score && spread_spacing_improved;

    meta.push((
        "closed_endpoint_plateau_repair_route_left_spacing_after_min_m".to_owned(),
        route_after_left_min.into(),
    ));
    meta.push((
        "closed_endpoint_plateau_repair_route_right_spacing_after_min_m".to_owned(),
        route_after_right_min.into(),
    ));
    meta.push((
        "closed_endpoint_plateau_repair_spread_left_spacing_after_min_m".to_owned(),
        spread_after_left_min.into(),
    ));
    meta.push((
        "closed_endpoint_plateau_repair_spread_right_spacing_after_min_m".to_owned(),
        spread_after_right_min.into(),
    ));
    meta.push((
        "closed_endpoint_plateau_repair_topology_before_score".to_owned(),
        JsonValue::Integer(before_score),
    ));
    meta.push((
        "closed_endpoint_plateau_repair_route_topology_after_score".to_owned(),
        JsonValue::Integer(route_after_score),
    ));
    meta.push((
        "closed_endpoint_plateau_repair_spread_topology_after_score".to_owned(),
        JsonValue::Integer(spread_after_score),
    ));
    meta.push((
        "closed_endpoint_plateau_repair_spread_left_count".to_owned(),
        JsonValue::Integer(spread_left_count as i64),
    ));
    meta.push((
        "closed_endpoint_plateau_repair_spread_right_count".to_owned(),
        JsonValue::Integer(spread_right_count as i64),
    ));

    let use_spread = spread_ok && (!route_ok || spread_after_score <= route_after_score);
    if route_ok || use_spread {
        if use_spread {
            left[..count].copy_from_slice(&spread_candidate_left);
            right[..count].copy_from_slice(&spread_candidate_right);
        } else {
            left[..count].copy_from_slice(&route_candidate_left);
            right[..count].copy_from_slice(&route_candidate_right);
        }
        meta.push((
            "closed_endpoint_plateau_repair_replaced_count".to_owned(),
            JsonValue::Integer(if use_spread {
                (spread_left_count + spread_right_count) as i64
            } else {
                candidate_count as i64
            }),
        ));
        meta.push((
            "closed_endpoint_plateau_repair_accepted".to_owned(),
            JsonValue::Bool(true),
        ));
        meta.push((
            "closed_endpoint_plateau_repair_mode".to_owned(),
            if use_spread {
                "neighbor_spread"
            } else {
                "route_progress"
            }
            .into(),
        ));
        let topology_replaced_count = repair_projection_topology_from_prepared_frame(
            &mut left[..count],
            &mut right[..count],
            &route_sampled_left,
            &route_sampled_right,
            control,
        )?;
        meta.push((
            "closed_endpoint_plateau_repair_topology_replaced_count".to_owned(),
            JsonValue::Integer(topology_replaced_count),
        ));
    } else {
        meta.push((
            "closed_endpoint_plateau_repair_replaced_count".to_owned(),
            JsonValue::Integer(0),
        ));
        meta.push((
            "closed_endpoint_plateau_repair_accepted".to_owned(),
            JsonValue::Bool(false),
        ));
        meta.push((
            "closed_endpoint_plateau_repair_rejected_reason".to_owned(),
            if !route_spacing_improved && !spread_spacing_improved {
                "spacing_not_improved"
            } else {
                "topology_would_worsen"
            }
            .into(),
        ));
    }

    Ok(meta)
}

fn closed_endpoint_spacing_min(points: &[Point2]) -> f64 {
    if points.len() < 2 {
        return 0.0;
    }
    (0..points.len())
        .map(|index| distance(points[index], points[(index + 1) % points.len()]))
        .fold(f64::INFINITY, f64::min)
}

fn spread_closed_endpoint_plateaus_by_neighbors(points: &mut [Point2]) -> usize {
    let count = points.len();
    if count < 4 {
        return 0;
    }

    let Some(break_edge) = (0..count).find(|index| {
        distance(points[*index], points[(*index + 1) % count]) > CLOSED_ENDPOINT_PLATEAU_EPS_M
    }) else {
        return 0;
    };
    let offset = (break_edge + 1) % count;
    let mut rotated = (0..count)
        .map(|index| points[(offset + index) % count])
        .collect::<Vec<_>>();

    let mut replaced_count = 0_usize;
    let mut index = 0_usize;
    while index + 1 < count {
        if distance(rotated[index], rotated[index + 1]) > CLOSED_ENDPOINT_PLATEAU_EPS_M {
            index += 1;
            continue;
        }

        let start = index;
        let mut end = index + 1;
        while end + 1 < count
            && distance(rotated[end], rotated[end + 1]) <= CLOSED_ENDPOINT_PLATEAU_EPS_M
        {
            end += 1;
        }

        if start == 0 || end + 1 >= count {
            index = end + 1;
            continue;
        }

        let previous = rotated[start - 1];
        let next = rotated[end + 1];
        let run_len = end - start + 1;
        for (run_offset, point_index) in (start..=end).enumerate() {
            let tau = (run_offset + 1) as f64 / (run_len + 1) as f64;
            rotated[point_index] = point_add(previous, point_scale(point_sub(next, previous), tau));
            replaced_count += 1;
        }

        index = end + 1;
    }

    for index in 0..count {
        points[(offset + index) % count] = rotated[index];
    }
    replaced_count
}

fn closed_endpoint_plateau_topology_score(left: &[Point2], right: &[Point2]) -> i64 {
    1_000_000 * station_horizon_crossing_count(left, right, 2)
        + 10_000 * station_crossing_count_all(left, right)
        + 100 * closed_polyline_self_intersection_count(left)
        + 100 * closed_polyline_self_intersection_count(right)
        + closed_polyline_pair_intersection_count(left, right)
}

fn build_section_based_corridor(
    left_world: &[Point2],
    paired_right_world: &[Point2],
    smoothing_window: usize,
    topology: StationTopology,
    control: StationGenerationControl<'_>,
) -> StationBuildResult<BoundaryPairTrack> {
    control.checkpoint()?;
    let effective_left = left_world.to_vec();
    let effective_right = paired_right_world.to_vec();
    let raw_centerline = effective_left
        .iter()
        .zip(&effective_right)
        .map(|(left, right)| midpoint(*left, *right))
        .collect::<Vec<_>>();
    let centerline_candidate = if smoothing_window > 1 {
        match topology {
            StationTopology::Closed => circular_smooth_points(&raw_centerline, smoothing_window),
            StationTopology::Open => open_smooth_points(&raw_centerline, smoothing_window),
        }
    } else {
        raw_centerline
    };
    let projected = project_centerline_to_paired_chords(
        &centerline_candidate,
        &effective_left,
        &effective_right,
        control,
    )?;
    let section_dirs = effective_left
        .iter()
        .zip(&effective_right)
        .map(|(left, right)| normalize(point_sub(*right, *left), [1.0, 0.0]))
        .collect::<Vec<_>>();

    Ok(BoundaryPairTrack {
        left_world: effective_left,
        right_world: effective_right,
        left_route_progress: station_frame_progress_for_path(topology, projected.centerline.len()),
        right_route_progress: station_frame_progress_for_path(topology, projected.centerline.len()),
        centerline_world: projected.centerline,
        normals_world: projected.normals,
        width_right: projected.width_right,
        width_left: projected.width_left,
        section_dirs,
        metadata: vec![(
            "centerline_projection_clamped_count".to_owned(),
            JsonValue::Integer(projected.clamped_count as i64),
        )],
    })
}

struct ProjectedCorridor {
    centerline: Vec<Point2>,
    normals: Vec<Point2>,
    width_right: Vec<f64>,
    width_left: Vec<f64>,
    clamped_count: usize,
}

fn project_centerline_to_paired_chords(
    centerline_candidate: &[Point2],
    left_world: &[Point2],
    right_world: &[Point2],
    control: StationGenerationControl<'_>,
) -> StationBuildResult<ProjectedCorridor> {
    let mut centerline = Vec::with_capacity(left_world.len());
    let mut normals = Vec::with_capacity(left_world.len());
    let mut width_right = Vec::with_capacity(left_world.len());
    let mut width_left = Vec::with_capacity(left_world.len());
    let mut clamped_count = 0_usize;

    for (index, left) in left_world.iter().copied().enumerate() {
        if index % 64 == 0 {
            control.checkpoint()?;
        }
        let right = right_world[index];
        let chord = point_sub(right, left);
        let chord_length = hypot(chord);
        let fraction = if chord_length <= 1e-9 {
            0.5
        } else {
            let raw = dot(point_sub(centerline_candidate[index], left), chord)
                / (chord_length * chord_length);
            let clamped = raw.clamp(0.0, 1.0);
            if clamped <= 0.0 || clamped >= 1.0 {
                clamped_count += 1;
            }
            clamped
        };

        centerline.push(point_add(left, point_scale(chord, fraction)));
        normals.push(normalize(chord, [1.0, 0.0]));
        width_left.push(chord_length * fraction);
        width_right.push(chord_length - chord_length * fraction);
    }

    Ok(ProjectedCorridor {
        centerline,
        normals,
        width_right,
        width_left,
        clamped_count,
    })
}

fn apply_zero_station_normal_fix(track: &mut BoundaryPairTrack) -> bool {
    if track.centerline_world.len() < 3 {
        return false;
    }

    let tangent = normalize(
        point_sub(
            track.centerline_world[1],
            *track.centerline_world.last().unwrap(),
        ),
        [0.0, 0.0],
    );

    if hypot(tangent) <= 1e-9 {
        return false;
    }

    let mut fixed_normal = [tangent[1], -tangent[0]];
    if dot(fixed_normal, track.normals_world[0]) < 0.0 {
        fixed_normal = point_scale(fixed_normal, -1.0);
    }

    let baseline_score = projection_topology_score(&track.left_world, &track.right_world);
    let original_left = track.left_world[0];
    let original_right = track.right_world[0];
    let chord_normal = normalize(
        point_sub(original_right, original_left),
        track.normals_world[0],
    );
    let mut best_normal = chord_normal;
    let mut best_left = original_left;
    let mut best_right = original_right;
    let mut best_alignment = dot(chord_normal, fixed_normal).abs();
    for step in 1..=20 {
        let tau = step as f64 / 20.0;
        let candidate_normal = normalize(
            point_add(
                point_scale(chord_normal, 1.0 - tau),
                point_scale(fixed_normal, tau),
            ),
            chord_normal,
        );
        let candidate_left = point_sub(
            track.centerline_world[0],
            point_scale(candidate_normal, track.width_left[0]),
        );
        let candidate_right = point_add(
            track.centerline_world[0],
            point_scale(candidate_normal, track.width_right[0]),
        );
        let mut candidate_left_route = track.left_world.clone();
        let mut candidate_right_route = track.right_world.clone();
        candidate_left_route[0] = candidate_left;
        candidate_right_route[0] = candidate_right;
        let candidate_score =
            projection_topology_score(&candidate_left_route, &candidate_right_route);
        let candidate_alignment = dot(candidate_normal, fixed_normal).abs();
        if candidate_score.0 <= baseline_score.0 && candidate_alignment > best_alignment + 1.0e-12 {
            best_normal = candidate_normal;
            best_left = candidate_left;
            best_right = candidate_right;
            best_alignment = candidate_alignment;
        }
    }
    track.normals_world[0] = best_normal;
    track.section_dirs[0] = best_normal;
    track.left_world[0] = best_left;
    track.right_world[0] = best_right;
    distance(best_left, original_left) > 1.0e-12 || distance(best_right, original_right) > 1.0e-12
}

type AdaptiveBoundaryResampleResult = (Vec<Point2>, Vec<Point2>, Vec<f64>, Vec<f64>, JsonObject);

fn resample_paired_boundaries_adaptive(
    left_world_dense: &[Point2],
    right_aligned_world_dense: &[Point2],
    left_route_progress_dense: &[f64],
    right_route_progress_dense: &[f64],
    path_pairs: &[(usize, usize)],
    sample_count: usize,
    turn_density_gain: f64,
    turn_analysis_smoothing_window: usize,
    turn_density_source: &str,
    density_smooth_window: usize,
    density_max_adjacent_ratio: f64,
    density_slew_mode: &str,
    target_spacing_max_adjacent_ratio: f64,
    target_spacing_metric: &str,
    control: StationGenerationControl<'_>,
) -> StationBuildResult<AdaptiveBoundaryResampleResult> {
    control.checkpoint()?;
    if path_pairs.len() < 4 {
        return Ok((
            resample_closed_polyline(left_world_dense, sample_count),
            resample_closed_polyline(right_aligned_world_dense, sample_count),
            station_frame_progress_for_path(StationTopology::Closed, sample_count),
            station_frame_progress_for_path(StationTopology::Closed, sample_count),
            vec![
                (
                    "adaptive_progress_unique_count".to_owned(),
                    JsonValue::Integer(path_pairs.len() as i64),
                ),
                (
                    "adaptive_turn_full_weight_count".to_owned(),
                    JsonValue::Integer(0),
                ),
            ],
        ));
    }

    let (left_s, _) = closed_polyline_arclength(left_world_dense);
    let (right_s, _) = closed_polyline_arclength(right_aligned_world_dense);
    let left_total = left_s.last().copied().unwrap_or(0.0).max(1e-9);
    let right_total = right_s.last().copied().unwrap_or(0.0).max(1e-9);
    let left_path = path_pairs
        .iter()
        .map(|(left_index, _)| left_world_dense[*left_index])
        .collect::<Vec<_>>();
    let right_path = path_pairs
        .iter()
        .map(|(_, right_index)| right_aligned_world_dense[*right_index])
        .collect::<Vec<_>>();
    let left_route_path = path_pairs
        .iter()
        .map(|(left_index, _)| {
            left_route_progress_dense
                .get(*left_index)
                .copied()
                .unwrap_or(0.0)
        })
        .collect::<Vec<_>>();
    let right_route_path = path_pairs
        .iter()
        .map(|(_, right_index)| {
            right_route_progress_dense
                .get(*right_index)
                .copied()
                .unwrap_or(0.0)
        })
        .collect::<Vec<_>>();
    let center_path = left_path
        .iter()
        .zip(&right_path)
        .map(|(left, right)| midpoint(*left, *right))
        .collect::<Vec<_>>();
    let (mut turn_angles, turn_density_source_used, turn_analysis_window) =
        density_turn_angles_from_source(
            &center_path,
            &left_path,
            &right_path,
            turn_density_source,
            turn_analysis_smoothing_window,
        );
    let turn_smooth_window = normalize_odd_window(9, turn_angles.len());

    if turn_smooth_window >= 3 {
        turn_angles = circular_smooth_1d(&turn_angles, turn_smooth_window);
    }

    let mut left_progress = path_pairs
        .iter()
        .map(|(left_index, _)| left_s[*left_index] / left_total)
        .collect::<Vec<_>>();
    let mut right_progress = path_pairs
        .iter()
        .map(|(_, right_index)| right_s[*right_index] / right_total)
        .collect::<Vec<_>>();
    let left_progress_plateau_repair_count =
        spread_closed_route_progress_plateaus(&mut left_progress);
    let right_progress_plateau_repair_count =
        spread_closed_route_progress_plateaus(&mut right_progress);
    let straight_angle = 2.0_f64.to_radians();
    let full_turn_angle = 10.0_f64.to_radians();
    let turn_weight = turn_angles
        .iter()
        .map(|angle| {
            ((angle.abs() - straight_angle) / (full_turn_angle - straight_angle).max(1e-9))
                .clamp(0.0, 1.0)
        })
        .collect::<Vec<_>>();
    let mut shared_progress = turn_angles
        .iter()
        .enumerate()
        .map(|(index, angle)| {
            let inner_progress = if *angle >= 0.0 {
                left_progress[index]
            } else {
                right_progress[index]
            };
            let average_progress = 0.5 * (left_progress[index] + right_progress[index]);
            let inner_blend = 0.5 * turn_weight[index];
            (1.0 - inner_blend) * average_progress + inner_blend * inner_progress
        })
        .collect::<Vec<_>>();

    for index in 1..shared_progress.len() {
        shared_progress[index] = shared_progress[index].max(shared_progress[index - 1]);
    }

    let first_shared = shared_progress.first().copied().unwrap_or(0.0);
    for value in &mut shared_progress {
        *value -= first_shared;
    }

    if shared_progress.last().copied().unwrap_or(0.0) <= 1e-9 {
        shared_progress = left_progress
            .iter()
            .zip(&right_progress)
            .map(|(left, right)| 0.5 * (left + right))
            .collect();
        for index in 1..shared_progress.len() {
            shared_progress[index] = shared_progress[index].max(shared_progress[index - 1]);
        }
    }

    let last_shared = shared_progress.last().copied().unwrap_or(0.0);
    if last_shared <= 1e-9 {
        return Ok((
            resample_closed_polyline(left_world_dense, sample_count),
            resample_closed_polyline(right_aligned_world_dense, sample_count),
            station_frame_progress_for_path(StationTopology::Closed, sample_count),
            station_frame_progress_for_path(StationTopology::Closed, sample_count),
            vec![(
                "adaptive_progress_unique_count".to_owned(),
                JsonValue::Integer(0),
            )],
        ));
    }

    for value in &mut shared_progress {
        *value /= last_shared;
    }

    let shared_delta = shared_progress
        .windows(2)
        .map(|values| values[1] - values[0])
        .collect::<Vec<_>>();
    let segment_turn_weight = turn_weight
        .windows(2)
        .map(|values| 0.5 * (values[0] + values[1]))
        .collect::<Vec<_>>();
    let raw_density = segment_turn_weight
        .iter()
        .map(|weight| 1.0 + turn_density_gain.max(0.0) * weight)
        .collect::<Vec<_>>();
    let (density, density_slew_meta) = smooth_density_profile(
        &raw_density,
        density_smooth_window,
        density_max_adjacent_ratio,
        density_slew_mode,
    );
    let mut weighted_progress = vec![0.0];
    for (index, delta) in shared_delta.iter().copied().enumerate() {
        if index % 64 == 0 {
            control.checkpoint_phase("adaptive_resampling")?;
        }
        weighted_progress
            .push(weighted_progress.last().copied().unwrap_or(0.0) + delta * density[index]);
    }

    let weighted_total = weighted_progress.last().copied().unwrap_or(0.0);
    let target_shared_progress = if weighted_total <= 1e-9 {
        (0..sample_count)
            .map(|index| index as f64 / sample_count as f64)
            .collect::<Vec<_>>()
    } else {
        let weighted_normalized = weighted_progress
            .iter()
            .map(|value| *value / weighted_total)
            .collect::<Vec<_>>();
        (0..sample_count)
            .map(|index| {
                interp_scalar(
                    index as f64 / sample_count as f64,
                    &weighted_normalized,
                    &shared_progress,
                )
            })
            .collect::<Vec<_>>()
    };
    let (target_shared_progress, target_spacing_meta) = limit_target_progress_spacing(
        &shared_progress,
        &left_path,
        &right_path,
        &center_path,
        &target_shared_progress,
        target_spacing_max_adjacent_ratio,
        target_spacing_metric,
    );

    let unique_progress_count = collapse_progress_samples(&shared_progress, &left_path)
        .0
        .len();
    let mut meta = vec![
        (
            "adaptive_progress_unique_count".to_owned(),
            JsonValue::Integer(unique_progress_count as i64),
        ),
        (
            "adaptive_turn_full_weight_count".to_owned(),
            JsonValue::Integer(turn_weight.iter().filter(|value| **value >= 0.999).count() as i64),
        ),
        (
            "adaptive_left_progress_plateau_repair_count".to_owned(),
            JsonValue::Integer(left_progress_plateau_repair_count as i64),
        ),
        (
            "adaptive_right_progress_plateau_repair_count".to_owned(),
            JsonValue::Integer(right_progress_plateau_repair_count as i64),
        ),
        (
            "adaptive_turn_density_gain".to_owned(),
            turn_density_gain.into(),
        ),
        (
            "turn_density_source".to_owned(),
            turn_density_source_used.into(),
        ),
        (
            "turn_analysis_smoothing_window".to_owned(),
            JsonValue::Integer(turn_analysis_window as i64),
        ),
        (
            "adaptive_density_max_before_slew".to_owned(),
            raw_density.iter().copied().fold(1.0, f64::max).into(),
        ),
        (
            "adaptive_density_max".to_owned(),
            density.iter().copied().fold(1.0, f64::max).into(),
        ),
        ("adaptive_solution_density_gain".to_owned(), 0.0.into()),
        ("adaptive_solution_density_max".to_owned(), 0.0.into()),
    ];
    meta.extend(density_slew_meta);
    meta.extend(target_spacing_meta);

    let mut sampled_left_route_progress =
        periodic_interp_route_progress(&shared_progress, &left_route_path, &target_shared_progress);
    let mut sampled_right_route_progress = periodic_interp_route_progress(
        &shared_progress,
        &right_route_path,
        &target_shared_progress,
    );
    let sampled_left_route_plateau_repair_count =
        spread_closed_route_progress_plateaus(&mut sampled_left_route_progress);
    let sampled_right_route_plateau_repair_count =
        spread_closed_route_progress_plateaus(&mut sampled_right_route_progress);
    meta.push((
        "adaptive_sampled_left_route_plateau_repair_count".to_owned(),
        JsonValue::Integer(sampled_left_route_plateau_repair_count as i64),
    ));
    meta.push((
        "adaptive_sampled_right_route_plateau_repair_count".to_owned(),
        JsonValue::Integer(sampled_right_route_plateau_repair_count as i64),
    ));
    let mut sampled_left =
        periodic_interp_points(&shared_progress, &left_path, &target_shared_progress);
    let mut sampled_right =
        periodic_interp_points(&shared_progress, &right_path, &target_shared_progress);
    let path_order_progress =
        station_frame_progress_for_path(StationTopology::Closed, left_path.len());
    let uniform_target_progress =
        station_frame_progress_for_path(StationTopology::Closed, sample_count);
    let route_sampled_left =
        periodic_interp_points(&path_order_progress, &left_path, &uniform_target_progress);
    let route_sampled_right =
        periodic_interp_points(&path_order_progress, &right_path, &uniform_target_progress);
    let mut topology_candidate_left = sampled_left.clone();
    let mut topology_candidate_right = sampled_right.clone();
    let mut adaptive_route_replaced_count = 0_i64;
    for _ in 0..4 {
        control.checkpoint_phase("closed_refinement_pass")?;
        let replaced = repair_projection_topology_from_prepared_frame(
            &mut topology_candidate_left,
            &mut topology_candidate_right,
            &route_sampled_left,
            &route_sampled_right,
            control,
        )?;
        if replaced == 0 {
            break;
        }
        adaptive_route_replaced_count += replaced;
    }
    let adaptive_route_repair_accepted =
        projection_topology_score(&topology_candidate_left, &topology_candidate_right).0 == 0;
    if adaptive_route_repair_accepted {
        sampled_left = topology_candidate_left;
        sampled_right = topology_candidate_right;
    } else {
        adaptive_route_replaced_count = 0;
    }
    meta.push((
        "adaptive_route_topology_replaced_count".to_owned(),
        JsonValue::Integer(adaptive_route_replaced_count),
    ));
    meta.push((
        "adaptive_route_topology_repair_accepted".to_owned(),
        JsonValue::Bool(adaptive_route_repair_accepted),
    ));

    Ok((
        sampled_left,
        sampled_right,
        sampled_left_route_progress,
        sampled_right_route_progress,
        meta,
    ))
}

fn spread_closed_route_progress_plateaus(progress: &mut [f64]) -> usize {
    if progress.len() < 3 {
        return 0;
    }

    let mut repaired_count = 0_usize;
    let mut index = 1_usize;
    while index < progress.len() {
        if progress[index] - progress[index - 1] > 1.0e-10 {
            index += 1;
            continue;
        }

        let start = index;
        let mut end = index;
        while end + 1 < progress.len() && progress[end + 1] - progress[end] <= 1.0e-10 {
            end += 1;
        }

        if end + 1 >= progress.len() {
            index = end + 1;
            continue;
        }

        let before = progress[start - 1];
        let after = progress[end + 1];
        if after <= before + 1.0e-10 {
            index = end + 1;
            continue;
        }

        let run_len = end - start + 1;
        for (offset, item) in progress[start..=end].iter_mut().enumerate() {
            let tau = (offset + 1) as f64 / (run_len + 1) as f64;
            *item = before + (after - before) * tau;
            repaired_count += 1;
        }

        index = end + 1;
    }

    repaired_count
}

fn closed_progress_max_step(progress: &[f64]) -> f64 {
    if progress.len() < 2 {
        return 0.0;
    }
    (0..progress.len())
        .map(|index| (progress[(index + 1) % progress.len()] - progress[index]).rem_euclid(1.0))
        .fold(0.0, f64::max)
}

fn rebase_closed_progress(progress: &mut [f64]) {
    let Some(first) = progress.first().copied() else {
        return;
    };
    for value in progress {
        *value = (*value - first).rem_euclid(1.0);
    }
}

fn density_turn_angles_from_source(
    center_path: &[Point2],
    left_path: &[Point2],
    right_path: &[Point2],
    source: &str,
    smoothing_window: usize,
) -> (Vec<f64>, String, usize) {
    let turn_analysis_window = normalize_odd_window(smoothing_window, center_path.len());
    let center_path_for_turn = if turn_analysis_window > 1 {
        circular_smooth_points(center_path, turn_analysis_window)
    } else {
        center_path.to_vec()
    };
    let center_turn = local_turn_angles(&center_path_for_turn);

    if !matches!(
        source,
        "boundary_curvature" | "boundary_curvature_integrated"
    ) {
        return (center_turn, "centerline".to_owned(), turn_analysis_window);
    }

    let left_path_for_turn = if turn_analysis_window > 1 {
        circular_smooth_points(left_path, turn_analysis_window)
    } else {
        left_path.to_vec()
    };
    let right_path_for_turn = if turn_analysis_window > 1 {
        circular_smooth_points(right_path, turn_analysis_window)
    } else {
        right_path.to_vec()
    };
    let left_turn = local_turn_angles(&left_path_for_turn);
    let right_turn = local_turn_angles(&right_path_for_turn);
    let mut boundary_abs = left_turn
        .iter()
        .zip(&right_turn)
        .map(|(left, right)| left.abs().max(right.abs()))
        .collect::<Vec<_>>();

    if source == "boundary_curvature_integrated" {
        let integration_window =
            normalize_odd_window(3.max(turn_analysis_window), boundary_abs.len());
        if integration_window > 1 {
            let integrated = circular_smooth_1d(&boundary_abs, integration_window)
                .into_iter()
                .map(|value| value * (integration_window as f64).sqrt())
                .collect::<Vec<_>>();
            for (value, integrated_value) in boundary_abs.iter_mut().zip(integrated) {
                *value = value.max(integrated_value);
            }
        }
    }

    let mut signed = Vec::with_capacity(boundary_abs.len());
    for index in 0..boundary_abs.len() {
        let mut sign = center_turn[index].signum();
        if sign.abs() < 1e-12 {
            sign = (left_turn[index] + right_turn[index]).signum();
        }
        if sign.abs() < 1e-12 {
            sign = 1.0;
        }
        signed.push(sign * boundary_abs[index]);
    }

    (signed, source.to_owned(), turn_analysis_window)
}

fn smooth_density_profile(
    density: &[f64],
    smooth_window: usize,
    max_adjacent_ratio: f64,
    slew_mode: &str,
) -> (Vec<f64>, JsonObject) {
    if density.is_empty() {
        return (
            Vec::new(),
            density_slew_metadata(1, max_adjacent_ratio, slew_mode, 1.0, 1.0, 1.0),
        );
    }

    let original_mean = density.iter().sum::<f64>() / density.len() as f64;
    let mut density_work = density
        .iter()
        .map(|value| value.max(1e-12))
        .collect::<Vec<_>>();
    let ratio_before = circular_adjacent_ratio(&density_work);
    let actual_window = normalize_odd_window(smooth_window, density_work.len());
    let ratio_after_smooth;

    if slew_mode == "peak_preserve" {
        if actual_window > 1 {
            let log_density = density_work
                .iter()
                .map(|value| value.ln())
                .collect::<Vec<_>>();
            let smoothed = circular_smooth_1d(&log_density, actual_window)
                .into_iter()
                .map(f64::exp)
                .collect::<Vec<_>>();
            for (value, smoothed_value) in density_work.iter_mut().zip(smoothed) {
                *value = value.max(smoothed_value);
            }
        }
        ratio_after_smooth = circular_adjacent_ratio(&density_work);
        density_work =
            raise_positive_circular_floor_for_slew(&density_work, max_adjacent_ratio, 32);
    } else if actual_window > 1 {
        let log_density = density_work
            .iter()
            .map(|value| value.ln())
            .collect::<Vec<_>>();
        density_work = circular_smooth_1d(&log_density, actual_window)
            .into_iter()
            .map(f64::exp)
            .collect::<Vec<_>>();
        let smoothed_mean = density_work.iter().sum::<f64>() / density_work.len() as f64;
        if smoothed_mean > 1e-12 && original_mean > 1e-12 {
            for value in &mut density_work {
                *value *= original_mean / smoothed_mean;
            }
        }
        ratio_after_smooth = circular_adjacent_ratio(&density_work);
        density_work = limit_positive_circular_slew(&density_work, max_adjacent_ratio, 32);
    } else {
        ratio_after_smooth = circular_adjacent_ratio(&density_work);
        density_work = limit_positive_circular_slew(&density_work, max_adjacent_ratio, 32);
    }

    let ratio_after = circular_adjacent_ratio(&density_work);
    (
        density_work,
        density_slew_metadata(
            actual_window,
            max_adjacent_ratio,
            slew_mode,
            ratio_before,
            ratio_after_smooth,
            ratio_after,
        ),
    )
}

fn density_slew_metadata(
    smooth_window: usize,
    max_adjacent_ratio: f64,
    slew_mode: &str,
    ratio_before: f64,
    ratio_after_smooth: f64,
    ratio_after: f64,
) -> JsonObject {
    vec![
        (
            "density_slew_smooth_window".to_owned(),
            JsonValue::Integer(smooth_window as i64),
        ),
        (
            "density_slew_max_adjacent_ratio_limit".to_owned(),
            max_adjacent_ratio.into(),
        ),
        ("density_slew_mode".to_owned(), slew_mode.to_owned().into()),
        (
            "density_max_adjacent_ratio_before".to_owned(),
            ratio_before.into(),
        ),
        (
            "density_max_adjacent_ratio_after_smooth".to_owned(),
            ratio_after_smooth.into(),
        ),
        (
            "density_max_adjacent_ratio_after".to_owned(),
            ratio_after.into(),
        ),
    ]
}

fn circular_adjacent_ratio(values: &[f64]) -> f64 {
    if values.len() < 2 {
        return 1.0;
    }
    values
        .iter()
        .enumerate()
        .map(|(index, value)| {
            let safe = value.max(1e-12);
            let next_safe = values[(index + 1) % values.len()].max(1e-12);
            (safe / next_safe).max(next_safe / safe)
        })
        .fold(1.0, f64::max)
}

fn linear_adjacent_ratio(values: &[f64]) -> f64 {
    if values.len() < 2 {
        return 1.0;
    }
    values
        .windows(2)
        .map(|pair| {
            let safe = pair[0].max(1e-12);
            let next_safe = pair[1].max(1e-12);
            (safe / next_safe).max(next_safe / safe)
        })
        .fold(1.0, f64::max)
}

fn limit_positive_circular_slew(
    values: &[f64],
    max_adjacent_ratio: f64,
    iterations: usize,
) -> Vec<f64> {
    if values.len() < 2 || max_adjacent_ratio <= 1.0 {
        return values.to_vec();
    }

    let original_mean = values.iter().sum::<f64>() / values.len() as f64;
    let mut log_values = values
        .iter()
        .map(|value| value.max(1e-12).ln())
        .collect::<Vec<_>>();
    let max_step = max_adjacent_ratio.ln();
    let count = log_values.len();

    for _ in 0..iterations.max(1) {
        let mut changed = false;
        for index in 0..count {
            let next = (index + 1) % count;
            let upper = log_values[index] + max_step;
            let lower = log_values[index] - max_step;
            if log_values[next] > upper {
                log_values[next] = upper;
                changed = true;
            } else if log_values[next] < lower {
                log_values[next] = lower;
                changed = true;
            }
        }
        for index in (0..count).rev() {
            let previous = (index + count - 1) % count;
            let upper = log_values[index] + max_step;
            let lower = log_values[index] - max_step;
            if log_values[previous] > upper {
                log_values[previous] = upper;
                changed = true;
            } else if log_values[previous] < lower {
                log_values[previous] = lower;
                changed = true;
            }
        }
        if !changed {
            break;
        }
    }

    let mut limited = log_values.into_iter().map(f64::exp).collect::<Vec<_>>();
    let limited_mean = limited.iter().sum::<f64>() / limited.len() as f64;
    if limited_mean > 1e-12 && original_mean > 1e-12 {
        for value in &mut limited {
            *value *= original_mean / limited_mean;
        }
    }
    limited
}

fn limit_positive_linear_slew(
    values: &[f64],
    max_adjacent_ratio: f64,
    iterations: usize,
) -> Vec<f64> {
    if values.len() < 2 || max_adjacent_ratio <= 1.0 {
        return values.to_vec();
    }

    let original_mean = values.iter().sum::<f64>() / values.len() as f64;
    let mut log_values = values
        .iter()
        .map(|value| value.max(1e-12).ln())
        .collect::<Vec<_>>();
    let max_step = max_adjacent_ratio.ln();

    for _ in 0..iterations.max(1) {
        let mut changed = false;
        for index in 1..log_values.len() {
            let upper = log_values[index - 1] + max_step;
            let lower = log_values[index - 1] - max_step;
            if log_values[index] > upper {
                log_values[index] = upper;
                changed = true;
            } else if log_values[index] < lower {
                log_values[index] = lower;
                changed = true;
            }
        }
        for index in (0..log_values.len() - 1).rev() {
            let upper = log_values[index + 1] + max_step;
            let lower = log_values[index + 1] - max_step;
            if log_values[index] > upper {
                log_values[index] = upper;
                changed = true;
            } else if log_values[index] < lower {
                log_values[index] = lower;
                changed = true;
            }
        }
        if !changed {
            break;
        }
    }

    let mut limited = log_values.into_iter().map(f64::exp).collect::<Vec<_>>();
    let limited_mean = limited.iter().sum::<f64>() / limited.len() as f64;
    if limited_mean > 1e-12 && original_mean > 1e-12 {
        for value in &mut limited {
            *value *= original_mean / limited_mean;
        }
    }
    limited
}

fn raise_positive_circular_floor_for_slew(
    values: &[f64],
    max_adjacent_ratio: f64,
    iterations: usize,
) -> Vec<f64> {
    if values.len() < 2 || max_adjacent_ratio <= 1.0 {
        return values.to_vec();
    }

    let mut result = values
        .iter()
        .map(|value| value.max(1e-12))
        .collect::<Vec<_>>();
    let count = result.len();
    let ratio = max_adjacent_ratio;

    for _ in 0..iterations.max(1) {
        let mut changed = false;
        for index in 0..count {
            let next = (index + 1) % count;
            if result[index] > result[next] * ratio {
                result[next] = result[index] / ratio;
                changed = true;
            } else if result[next] > result[index] * ratio {
                result[index] = result[next] / ratio;
                changed = true;
            }
        }
        for index in (0..count).rev() {
            let previous = (index + count - 1) % count;
            if result[index] > result[previous] * ratio {
                result[previous] = result[index] / ratio;
                changed = true;
            } else if result[previous] > result[index] * ratio {
                result[index] = result[previous] / ratio;
                changed = true;
            }
        }
        if !changed {
            break;
        }
    }

    result
}

fn limit_target_progress_spacing(
    shared_progress: &[f64],
    left_path: &[Point2],
    right_path: &[Point2],
    center_path: &[Point2],
    target_shared_progress: &[f64],
    max_adjacent_ratio: f64,
    metric: &str,
) -> (Vec<f64>, JsonObject) {
    let metric = metric.to_owned();
    let (adjusted, mut meta) = match metric.as_str() {
        "section_area" => limit_target_area_slew(
            shared_progress,
            left_path,
            right_path,
            target_shared_progress,
            max_adjacent_ratio,
        ),
        "hybrid_area_centerline" => {
            let (area_adjusted, area_meta) = limit_target_area_slew(
                shared_progress,
                left_path,
                right_path,
                target_shared_progress,
                max_adjacent_ratio,
            );
            let (center_adjusted, center_meta) = limit_target_spacing_slew(
                shared_progress,
                center_path,
                &area_adjusted,
                max_adjacent_ratio,
            );
            let area_before = json_f64(&area_meta, "target_spacing_max_adjacent_ratio_before", 1.0);
            let area_after = json_f64(&area_meta, "target_spacing_max_adjacent_ratio_after", 1.0);
            let center_before = json_f64(
                &center_meta,
                "target_spacing_max_adjacent_ratio_before",
                1.0,
            );
            let center_after =
                json_f64(&center_meta, "target_spacing_max_adjacent_ratio_after", 1.0);
            (
                center_adjusted,
                vec![
                    (
                        "target_spacing_max_adjacent_ratio_limit".to_owned(),
                        max_adjacent_ratio.into(),
                    ),
                    (
                        "target_spacing_max_adjacent_ratio_before".to_owned(),
                        area_before.into(),
                    ),
                    (
                        "target_spacing_max_adjacent_ratio_after".to_owned(),
                        center_after.into(),
                    ),
                    (
                        "target_area_spacing_ratio_before".to_owned(),
                        area_before.into(),
                    ),
                    (
                        "target_area_spacing_ratio_after".to_owned(),
                        area_after.into(),
                    ),
                    (
                        "target_centerline_spacing_ratio_before".to_owned(),
                        center_before.into(),
                    ),
                    (
                        "target_centerline_spacing_ratio_after".to_owned(),
                        center_after.into(),
                    ),
                ],
            )
        }
        _ => limit_target_spacing_slew(
            shared_progress,
            center_path,
            target_shared_progress,
            max_adjacent_ratio,
        ),
    };
    meta.push(("target_spacing_metric".to_owned(), metric.into()));

    (adjusted, meta)
}

fn limit_target_progress_spacing_for_topology(
    topology: StationTopology,
    shared_progress: &[f64],
    left_path: &[Point2],
    right_path: &[Point2],
    center_path: &[Point2],
    target_shared_progress: &[f64],
    max_adjacent_ratio: f64,
    metric: &str,
) -> (Vec<f64>, JsonObject) {
    match topology {
        StationTopology::Closed => limit_target_progress_spacing(
            shared_progress,
            left_path,
            right_path,
            center_path,
            target_shared_progress,
            max_adjacent_ratio,
            metric,
        ),
        StationTopology::Open => {
            let metric = metric.to_owned();
            let (adjusted, mut meta) = match metric.as_str() {
                "section_area" => limit_open_target_area_slew(
                    shared_progress,
                    left_path,
                    right_path,
                    target_shared_progress,
                    max_adjacent_ratio,
                ),
                "hybrid_area_centerline" => {
                    let (area_adjusted, area_meta) = limit_open_target_area_slew(
                        shared_progress,
                        left_path,
                        right_path,
                        target_shared_progress,
                        max_adjacent_ratio,
                    );
                    let (center_adjusted, center_meta) = limit_open_target_progress_spacing(
                        shared_progress,
                        center_path,
                        &area_adjusted,
                        max_adjacent_ratio,
                    );
                    let area_before =
                        json_f64(&area_meta, "target_spacing_max_adjacent_ratio_before", 1.0);
                    let area_after =
                        json_f64(&area_meta, "target_spacing_max_adjacent_ratio_after", 1.0);
                    let center_before = json_f64(
                        &center_meta,
                        "target_spacing_max_adjacent_ratio_before",
                        1.0,
                    );
                    let center_after =
                        json_f64(&center_meta, "target_spacing_max_adjacent_ratio_after", 1.0);
                    (
                        center_adjusted,
                        vec![
                            (
                                "target_spacing_max_adjacent_ratio_limit".to_owned(),
                                max_adjacent_ratio.into(),
                            ),
                            (
                                "target_spacing_max_adjacent_ratio_before".to_owned(),
                                area_before.into(),
                            ),
                            (
                                "target_spacing_max_adjacent_ratio_after".to_owned(),
                                center_after.into(),
                            ),
                            (
                                "target_area_spacing_ratio_before".to_owned(),
                                area_before.into(),
                            ),
                            (
                                "target_area_spacing_ratio_after".to_owned(),
                                area_after.into(),
                            ),
                            (
                                "target_centerline_spacing_ratio_before".to_owned(),
                                center_before.into(),
                            ),
                            (
                                "target_centerline_spacing_ratio_after".to_owned(),
                                center_after.into(),
                            ),
                        ],
                    )
                }
                _ => limit_open_target_progress_spacing(
                    shared_progress,
                    center_path,
                    target_shared_progress,
                    max_adjacent_ratio,
                ),
            };
            meta.push(("target_spacing_metric".to_owned(), metric.into()));
            (adjusted, meta)
        }
    }
}

fn limit_open_target_progress_spacing(
    shared_progress: &[f64],
    center_path: &[Point2],
    target_progress: &[f64],
    max_adjacent_ratio: f64,
) -> (Vec<f64>, JsonObject) {
    if target_progress.len() < 3 || max_adjacent_ratio <= 1.0 || center_path.len() < 3 {
        return (
            target_progress.to_vec(),
            target_spacing_meta(max_adjacent_ratio, 1.0, 1.0),
        );
    }
    let (progress_unique, center_unique) = collapse_progress_samples(shared_progress, center_path);
    if progress_unique.len() < 3 {
        return (
            target_progress.to_vec(),
            target_spacing_meta(max_adjacent_ratio, 1.0, 1.0),
        );
    }
    let (center_s, _) = open_polyline_arclength(&center_unique);
    let total_length = center_s.last().copied().unwrap_or(0.0);
    if total_length <= 1e-9 {
        return (
            target_progress.to_vec(),
            target_spacing_meta(max_adjacent_ratio, 1.0, 1.0),
        );
    }
    let center_s_norm = center_s
        .into_iter()
        .map(|value| value / total_length)
        .collect::<Vec<_>>();
    let target_center_s = target_progress
        .iter()
        .map(|target| interp_scalar(*target, &progress_unique, &center_s_norm))
        .collect::<Vec<_>>();
    let lengths = target_center_s
        .windows(2)
        .map(|pair| (pair[1] - pair[0]).max(1e-12))
        .collect::<Vec<_>>();
    let ratio_before = linear_adjacent_ratio(&lengths);
    let limited_lengths = limit_positive_linear_slew(&lengths, max_adjacent_ratio, 32);
    let ratio_after = linear_adjacent_ratio(&limited_lengths);
    let limited_total = limited_lengths.iter().sum::<f64>();
    if limited_total <= 1e-9 {
        return (
            target_progress.to_vec(),
            target_spacing_meta(max_adjacent_ratio, ratio_before, ratio_after),
        );
    }
    let mut adjusted_center_s = Vec::with_capacity(target_progress.len());
    adjusted_center_s.push(0.0);
    for length in limited_lengths {
        adjusted_center_s
            .push(adjusted_center_s.last().copied().unwrap_or(0.0) + length / limited_total);
    }
    if let Some(last) = adjusted_center_s.last_mut() {
        *last = 1.0;
    }
    let adjusted_progress = adjusted_center_s
        .iter()
        .map(|target| interp_scalar(*target, &center_s_norm, &progress_unique))
        .collect::<Vec<_>>();
    (
        adjusted_progress,
        target_spacing_meta(max_adjacent_ratio, ratio_before, ratio_after),
    )
}

fn limit_target_spacing_slew(
    shared_progress: &[f64],
    center_path: &[Point2],
    target_shared_progress: &[f64],
    max_adjacent_ratio: f64,
) -> (Vec<f64>, JsonObject) {
    if target_shared_progress.len() < 3 || max_adjacent_ratio <= 1.0 {
        return (
            target_shared_progress.to_vec(),
            target_spacing_meta(max_adjacent_ratio, 1.0, 1.0),
        );
    }

    let (progress_unique, center_unique) = collapse_progress_samples(shared_progress, center_path);
    if progress_unique.len() < 3 {
        return (
            target_shared_progress.to_vec(),
            target_spacing_meta(max_adjacent_ratio, 1.0, 1.0),
        );
    }

    let (center_s, _) = closed_polyline_arclength(&center_unique);
    let total_length = center_s.last().copied().unwrap_or(0.0);
    if total_length <= 1e-9 {
        return (
            target_shared_progress.to_vec(),
            target_spacing_meta(max_adjacent_ratio, 1.0, 1.0),
        );
    }

    let center_s_norm = center_s
        .into_iter()
        .take(progress_unique.len())
        .map(|value| value / total_length)
        .collect::<Vec<_>>();
    let mut target_center_s = target_shared_progress
        .iter()
        .map(|target| interp_scalar(*target, &progress_unique, &center_s_norm))
        .collect::<Vec<_>>();
    for index in 1..target_center_s.len() {
        target_center_s[index] = target_center_s[index].max(target_center_s[index - 1]);
    }
    let intervals = circular_intervals(&target_center_s);
    let ratio_before = circular_adjacent_ratio(&intervals);
    let limited_intervals =
        lower_positive_circular_ceiling_for_slew(&intervals, max_adjacent_ratio, 32);
    let ratio_after = circular_adjacent_ratio(&limited_intervals);

    let mut adjusted_center_s = Vec::with_capacity(target_center_s.len());
    adjusted_center_s.push(target_center_s[0]);
    for interval in limited_intervals
        .iter()
        .take(limited_intervals.len().saturating_sub(1))
    {
        adjusted_center_s.push(adjusted_center_s.last().copied().unwrap_or(0.0) + interval);
    }
    for value in &mut adjusted_center_s {
        *value = value.rem_euclid(1.0);
    }
    adjusted_center_s.sort_by(f64::total_cmp);
    let adjusted_progress = adjusted_center_s
        .iter()
        .map(|target| interp_scalar(*target, &center_s_norm, &progress_unique))
        .collect::<Vec<_>>();

    (
        adjusted_progress,
        target_spacing_meta(max_adjacent_ratio, ratio_before, ratio_after),
    )
}

fn limit_target_area_slew(
    shared_progress: &[f64],
    left_path: &[Point2],
    right_path: &[Point2],
    target_shared_progress: &[f64],
    max_adjacent_ratio: f64,
) -> (Vec<f64>, JsonObject) {
    if target_shared_progress.len() < 3 || max_adjacent_ratio <= 1.0 {
        return (
            target_shared_progress.to_vec(),
            target_spacing_meta(max_adjacent_ratio, 1.0, 1.0),
        );
    }

    let (progress_unique, left_unique) = collapse_progress_samples(shared_progress, left_path);
    let (_, right_unique) = collapse_progress_samples(shared_progress, right_path);
    if progress_unique.len() < 3 || right_unique.len() != left_unique.len() {
        return (
            target_shared_progress.to_vec(),
            target_spacing_meta(max_adjacent_ratio, 1.0, 1.0),
        );
    }

    let source_areas = compute_section_cell_areas(&left_unique, &right_unique);
    let total_area = source_areas.iter().sum::<f64>();
    if total_area <= 1e-9 {
        return (
            target_shared_progress.to_vec(),
            target_spacing_meta(max_adjacent_ratio, 1.0, 1.0),
        );
    }

    let mut progress_closed = progress_unique.clone();
    progress_closed.push(1.0);
    let mut area_norm = Vec::with_capacity(source_areas.len() + 1);
    area_norm.push(0.0);
    let mut area_sum = 0.0;
    for area in source_areas {
        area_sum += area;
        area_norm.push(area_sum / total_area);
    }
    let mut target_area = target_shared_progress
        .iter()
        .map(|target| interp_scalar(*target, &progress_closed, &area_norm))
        .collect::<Vec<_>>();
    for index in 1..target_area.len() {
        while target_area[index] <= target_area[index - 1] {
            target_area[index] += 1.0;
        }
    }

    let intervals = circular_intervals(&target_area);
    let ratio_before = circular_adjacent_ratio(&intervals);
    let limited_intervals =
        lower_positive_circular_ceiling_for_slew(&intervals, max_adjacent_ratio, 32);
    let ratio_after = circular_adjacent_ratio(&limited_intervals);

    let mut adjusted_area = Vec::with_capacity(target_area.len());
    adjusted_area.push(target_area[0]);
    for interval in limited_intervals
        .iter()
        .take(limited_intervals.len().saturating_sub(1))
    {
        adjusted_area.push(adjusted_area.last().copied().unwrap_or(0.0) + interval);
    }
    for value in &mut adjusted_area {
        *value = value.rem_euclid(1.0);
    }
    adjusted_area.sort_by(f64::total_cmp);
    let adjusted_progress = adjusted_area
        .iter()
        .map(|target| interp_scalar(*target, &area_norm, &progress_closed).rem_euclid(1.0))
        .collect::<Vec<_>>();

    (
        adjusted_progress,
        target_spacing_meta(max_adjacent_ratio, ratio_before, ratio_after),
    )
}

fn limit_open_target_area_slew(
    shared_progress: &[f64],
    left_path: &[Point2],
    right_path: &[Point2],
    target_shared_progress: &[f64],
    max_adjacent_ratio: f64,
) -> (Vec<f64>, JsonObject) {
    if target_shared_progress.len() < 3 || max_adjacent_ratio <= 1.0 {
        return (
            target_shared_progress.to_vec(),
            target_spacing_meta(max_adjacent_ratio, 1.0, 1.0),
        );
    }

    let (progress_unique, left_unique) = collapse_progress_samples(shared_progress, left_path);
    let (_, right_unique) = collapse_progress_samples(shared_progress, right_path);
    if progress_unique.len() < 3 || right_unique.len() != left_unique.len() {
        return (
            target_shared_progress.to_vec(),
            target_spacing_meta(max_adjacent_ratio, 1.0, 1.0),
        );
    }

    let source_areas = compute_open_section_cell_areas(&left_unique, &right_unique);
    let total_area = source_areas.iter().sum::<f64>();
    if total_area <= 1e-9 {
        return (
            target_shared_progress.to_vec(),
            target_spacing_meta(max_adjacent_ratio, 1.0, 1.0),
        );
    }

    let mut area_norm = Vec::with_capacity(source_areas.len() + 1);
    area_norm.push(0.0);
    let mut area_sum = 0.0;
    for area in source_areas {
        area_sum += area;
        area_norm.push(area_sum / total_area);
    }

    let mut target_area = target_shared_progress
        .iter()
        .map(|target| interp_scalar(*target, &progress_unique, &area_norm))
        .collect::<Vec<_>>();
    if let Some(first) = target_area.first_mut() {
        *first = 0.0;
    }
    if let Some(last) = target_area.last_mut() {
        *last = 1.0;
    }

    let intervals = target_area
        .windows(2)
        .map(|pair| (pair[1] - pair[0]).max(1e-12))
        .collect::<Vec<_>>();
    let ratio_before = linear_adjacent_ratio(&intervals);
    let limited_intervals = limit_positive_linear_slew(&intervals, max_adjacent_ratio, 32);
    let ratio_after = linear_adjacent_ratio(&limited_intervals);
    let limited_sum = limited_intervals.iter().sum::<f64>();
    if limited_sum <= 1e-12 {
        return (
            target_shared_progress.to_vec(),
            target_spacing_meta(max_adjacent_ratio, ratio_before, ratio_after),
        );
    }

    let mut adjusted_area = Vec::with_capacity(target_area.len());
    adjusted_area.push(0.0);
    for interval in limited_intervals {
        let next = adjusted_area.last().copied().unwrap_or(0.0) + interval / limited_sum;
        adjusted_area.push(next);
    }
    if let Some(last) = adjusted_area.last_mut() {
        *last = 1.0;
    }

    let adjusted_progress = adjusted_area
        .iter()
        .map(|target| interp_scalar(*target, &area_norm, &progress_unique))
        .collect::<Vec<_>>();

    (
        adjusted_progress,
        target_spacing_meta(max_adjacent_ratio, ratio_before, ratio_after),
    )
}

fn target_spacing_meta(limit: f64, before: f64, after: f64) -> JsonObject {
    vec![
        (
            "target_spacing_max_adjacent_ratio_limit".to_owned(),
            limit.into(),
        ),
        (
            "target_spacing_max_adjacent_ratio_before".to_owned(),
            before.into(),
        ),
        (
            "target_spacing_max_adjacent_ratio_after".to_owned(),
            after.into(),
        ),
    ]
}

fn circular_intervals(values: &[f64]) -> Vec<f64> {
    if values.is_empty() {
        return Vec::new();
    }
    values
        .iter()
        .enumerate()
        .map(|(index, value)| {
            let next = if index + 1 == values.len() {
                values[0] + 1.0
            } else {
                values[index + 1]
            };
            (next - *value).max(1e-12)
        })
        .collect()
}

fn lower_positive_circular_ceiling_for_slew(
    values: &[f64],
    max_adjacent_ratio: f64,
    iterations: usize,
) -> Vec<f64> {
    if values.len() < 2 || max_adjacent_ratio <= 1.0 {
        return values.to_vec();
    }

    let original_sum = values.iter().sum::<f64>();
    let mut result = values
        .iter()
        .map(|value| value.max(1e-12))
        .collect::<Vec<_>>();
    let count = result.len();
    let ratio = max_adjacent_ratio;

    for _ in 0..iterations.max(1) {
        let mut changed = false;
        for index in 0..count {
            let next = (index + 1) % count;
            if result[index] > result[next] * ratio {
                result[index] = result[next] * ratio;
                changed = true;
            } else if result[next] > result[index] * ratio {
                result[next] = result[index] * ratio;
                changed = true;
            }
        }
        for index in (0..count).rev() {
            let previous = (index + count - 1) % count;
            if result[index] > result[previous] * ratio {
                result[index] = result[previous] * ratio;
                changed = true;
            } else if result[previous] > result[index] * ratio {
                result[previous] = result[index] * ratio;
                changed = true;
            }
        }
        if !changed {
            break;
        }
    }

    let limited_sum = result.iter().sum::<f64>();
    if limited_sum > 1e-12 && original_sum > 1e-12 {
        for value in &mut result {
            *value *= original_sum / limited_sum;
        }
    }
    result
}

fn json_f64(metadata: &JsonObject, key: &str, default: f64) -> f64 {
    metadata
        .iter()
        .find_map(|(entry_key, value)| {
            (entry_key == key).then_some(match value {
                JsonValue::Integer(value) => *value as f64,
                JsonValue::Number(value) => *value,
                _ => default,
            })
        })
        .unwrap_or(default)
}

struct RollCandidate {
    pairing_mode: &'static str,
    roll_bias: isize,
    right_aligned: Vec<Point2>,
    right_route_progress: Vec<f64>,
    path: Vec<(usize, usize)>,
    path_length: usize,
    score: f64,
    seam_angle_p95_deg: f64,
    seam_angle_max_deg: f64,
    global_angle_p95_deg: f64,
    global_angle_max_deg: f64,
    progress_delta_abs_p95: f64,
    crossing_count: i64,
    width_median_m: f64,
    width_max_m: f64,
    width_ratio_max_to_median: f64,
    centerline_length_m: f64,
    centerline_length_ratio_abs_log: f64,
}

type BoundaryPairingResult = (Vec<Point2>, Vec<(usize, usize)>, Vec<f64>, JsonObject);
type OpenBoundaryPairingResult = (Vec<Point2>, Vec<(usize, usize)>, JsonObject);

fn pair_boundaries_dtw(
    left_world: &[Point2],
    right_world: &[Point2],
    left_route_progress: &[f64],
    right_route_progress: &[f64],
    band: usize,
    alignment_roll_bias: DtwAlignmentRollBias,
    centerline_hint_world: Option<&[Point2]>,
    centerline_normal_cost_weight: f64,
    slide_cost_weight: f64,
    control: StationGenerationControl<'_>,
) -> StationBuildResult<BoundaryPairingResult> {
    control.checkpoint()?;
    let (left_inward_normals, left_tangents) = right_normals_world(left_world);
    let (right_aligned_base, alignment_meta, alignment_reversed, alignment_shift) =
        align_closed_boundaries(left_world, right_world);
    let count = left_world.len();
    let left_route_progress = &left_route_progress[..count.min(left_route_progress.len())];
    let mut right_route_progress_aligned = right_route_progress.to_vec();
    if alignment_reversed {
        right_route_progress_aligned.reverse();
    }
    right_route_progress_aligned =
        shifted_closed_values(&right_route_progress_aligned, alignment_shift);
    rebase_closed_progress(&mut right_route_progress_aligned);
    let inf = 1e18;
    let has_centerline_hint = centerline_hint_world.is_some_and(|points| points.len() >= 3);
    let normal_weight = centerline_normal_cost_weight.max(0.0);
    let slide_weight = slide_cost_weight.max(0.0);
    let boundary_length_target_m = 0.5
        * (closed_polyline_arclength(left_world).1.iter().sum::<f64>()
            + closed_polyline_arclength(&right_aligned_base)
                .1
                .iter()
                .sum::<f64>());

    let score_pairing = |pairing_mode: &'static str,
                         roll_bias: isize,
                         right_aligned: Vec<Point2>,
                         right_aligned_route_progress: &[f64],
                         path: Vec<(usize, usize)>|
     -> RollCandidate {
        let (right_right_normals, _) = right_normals_world(&right_aligned);
        let right_inward_normals = right_right_normals
            .iter()
            .map(|normal| point_scale(*normal, -1.0))
            .collect::<Vec<_>>();
        let path_length = path.len();
        let mut matches = vec![Vec::<usize>::new(); count];
        for (left_index, right_index) in &path {
            if *left_index < count && *right_index < count {
                matches[*left_index].push(*right_index);
            }
        }

        let mut paired_right = vec![[0.0, 0.0]; count];
        let mut paired_right_indices = vec![0_usize; count];
        for (index, right_indices) in matches.iter().enumerate() {
            if right_indices.is_empty() {
                paired_right_indices[index] = index;
                paired_right[index] = right_aligned[index];
            } else {
                let mean = right_indices.iter().copied().sum::<usize>() as f64
                    / right_indices.len() as f64;
                let right_index = mean.round() as usize % count;
                paired_right_indices[index] = right_index;
                paired_right[index] = right_aligned[right_index];
            }
        }

        let mut angle_deg = Vec::with_capacity(count);
        let mut widths = Vec::with_capacity(count);
        for index in 0..count {
            let chord = point_sub(paired_right[index], left_world[index]);
            let width = hypot(chord);
            widths.push(width);
            let chord_dir = if width > 1e-9 {
                point_scale(chord, 1.0 / width)
            } else {
                [0.0, 0.0]
            };
            if has_centerline_hint {
                let midpoint = midpoint(left_world[index], paired_right[index]);
                let hint_normal =
                    centerline_projected_normal(centerline_hint_world.unwrap(), midpoint);
                angle_deg.push(
                    dot(chord_dir, hint_normal)
                        .abs()
                        .clamp(0.0, 1.0)
                        .acos()
                        .to_degrees(),
                );
            } else {
                let left_angle = dot(chord_dir, left_inward_normals[index])
                    .clamp(-1.0, 1.0)
                    .acos()
                    .to_degrees();
                let right_angle =
                    (-dot(chord_dir, right_inward_normals[paired_right_indices[index]]))
                        .clamp(-1.0, 1.0)
                        .acos()
                        .to_degrees();
                angle_deg.push(0.5 * (left_angle + right_angle));
            }
        }

        let seam_window = 6_usize.max((count / 24).min(24));
        let mut seam_angle = angle_deg
            .iter()
            .take(seam_window)
            .copied()
            .collect::<Vec<_>>();
        seam_angle.extend(angle_deg.iter().rev().take(seam_window).copied());

        let progress_delta_abs = paired_right_indices
            .iter()
            .enumerate()
            .map(|(index, right_index)| {
                cyclic_unit_distance(
                    left_route_progress.get(index).copied().unwrap_or(0.0),
                    right_aligned_route_progress
                        .get(*right_index)
                        .copied()
                        .unwrap_or(0.0),
                )
            })
            .collect::<Vec<_>>();
        let crossing_count = station_horizon_crossing_count(left_world, &paired_right, 2);
        let width_median = median(widths.clone());
        let width_max = widths.iter().copied().fold(0.0, f64::max);
        let width_ratio = width_max / width_median.max(1e-9);
        let paired_centerline = left_world
            .iter()
            .zip(&paired_right)
            .map(|(left, right)| midpoint(*left, *right))
            .collect::<Vec<_>>();
        let centerline_length_m = closed_polyline_arclength(&paired_centerline)
            .1
            .iter()
            .sum::<f64>();
        let centerline_length_ratio_abs_log = (centerline_length_m
            / boundary_length_target_m.max(1e-9))
        .max(1e-9)
        .ln()
        .abs();
        let width_penalty = (width_ratio - 3.0).max(0.0) * 25.0;
        let crossing_penalty = crossing_count as f64 * 1_000_000.0;
        let seam_angle_max = seam_angle.iter().copied().fold(0.0, f64::max);
        let global_angle_max = angle_deg.iter().copied().fold(0.0, f64::max);
        let global_angle_p95 = percentile(angle_deg.clone(), 95.0);
        let progress_delta_abs_p95 = percentile(progress_delta_abs, 95.0);
        let score = 2.0 * seam_angle_max
            + global_angle_p95
            + global_angle_max
            + 25.0 * progress_delta_abs_p95
            + 500.0 * centerline_length_ratio_abs_log
            + width_penalty
            + crossing_penalty;

        RollCandidate {
            pairing_mode,
            roll_bias,
            right_aligned,
            right_route_progress: right_aligned_route_progress.to_vec(),
            path,
            path_length,
            score,
            seam_angle_p95_deg: percentile(seam_angle, 95.0),
            seam_angle_max_deg: seam_angle_max,
            global_angle_p95_deg: global_angle_p95,
            global_angle_max_deg: global_angle_max,
            progress_delta_abs_p95,
            crossing_count,
            width_median_m: width_median,
            width_max_m: width_max,
            width_ratio_max_to_median: width_ratio,
            centerline_length_m,
            centerline_length_ratio_abs_log,
        }
    };

    let evaluate_roll = |roll_bias: isize| -> StationBuildResult<RollCandidate> {
        control.checkpoint()?;
        let right_aligned = roll_points(&right_aligned_base, roll_bias);
        let mut right_aligned_route_progress =
            roll_values(&right_route_progress_aligned, roll_bias);
        rebase_closed_progress(&mut right_aligned_route_progress);
        let (right_right_normals, right_tangents) = right_normals_world(&right_aligned);
        let right_inward_normals = right_right_normals
            .iter()
            .map(|normal| point_scale(*normal, -1.0))
            .collect::<Vec<_>>();
        let mut cost = vec![inf; count * count];

        for i in 0..count {
            if i % 32 == 0 {
                control.checkpoint_phase("closed_dtw")?;
            }
            let j0 = i.saturating_sub(band);
            let j1 = count.min(i + band + 1);
            for j in j0..j1 {
                let chord = point_sub(right_aligned[j], left_world[i]);
                let distance = hypot(chord);
                let chord_dir = if distance > 1e-9 {
                    point_scale(chord, 1.0 / distance)
                } else {
                    [0.0, 0.0]
                };
                let mut local_cost = distance;
                local_cost += 3.0 * (1.0 - dot(chord_dir, left_inward_normals[i]).clamp(-1.0, 1.0));
                local_cost +=
                    3.0 * (1.0 + dot(chord_dir, right_inward_normals[j]).clamp(-1.0, 1.0));
                local_cost +=
                    1.0 * (1.0 - dot(left_tangents[i], right_tangents[j]).clamp(-1.0, 1.0));
                let route_progress_mismatch = cyclic_unit_distance(
                    left_route_progress.get(i).copied().unwrap_or(0.0),
                    right_aligned_route_progress.get(j).copied().unwrap_or(0.0),
                );
                local_cost += 0.03 * (j as f64 - i as f64).abs();
                local_cost += 0.0 * route_progress_mismatch;
                if has_centerline_hint && normal_weight > 0.0 {
                    let midpoint = midpoint(left_world[i], right_aligned[j]);
                    let hint_normal =
                        centerline_projected_normal(centerline_hint_world.unwrap(), midpoint);
                    local_cost +=
                        normal_weight * (1.0 - dot(chord_dir, hint_normal).abs().clamp(0.0, 1.0));
                }
                cost[i * count + j] = local_cost;
            }
        }

        let dp_size = count + 1;
        let mut dp = vec![inf; dp_size * dp_size];
        let mut prev = vec![(usize::MAX, usize::MAX); dp_size * dp_size];
        dp[0] = 0.0;

        for i in 1..=count {
            if i % 32 == 0 {
                control.checkpoint_phase("closed_dtw")?;
            }
            let j0 = 1_usize.max(i.saturating_sub(band));
            let j1 = count.min(i + band);
            for j in j0..=j1 {
                let cell_cost = cost[(i - 1) * count + (j - 1)];
                let candidates = [
                    (dp[(i - 1) * dp_size + (j - 1)] + cell_cost, i - 1, j - 1),
                    (
                        dp[(i - 1) * dp_size + j] + slide_weight * cell_cost,
                        i - 1,
                        j,
                    ),
                    (
                        dp[i * dp_size + (j - 1)] + slide_weight * cell_cost,
                        i,
                        j - 1,
                    ),
                ];
                let (best_total, best_i, best_j) = candidates
                    .into_iter()
                    .min_by(|left, right| left.0.total_cmp(&right.0))
                    .unwrap();
                if !best_total.is_finite() || best_total >= inf {
                    continue;
                }
                let index = i * dp_size + j;
                dp[index] = best_total;
                prev[index] = (best_i, best_j);
            }
        }

        let mut path = Vec::<(usize, usize)>::new();
        let mut i = count;
        let mut j = count;
        while i > 0 || j > 0 {
            if i > 0 && j > 0 {
                path.push((i - 1, j - 1));
            }
            let (next_i, next_j) = prev[i * dp_size + j];
            if next_i == usize::MAX {
                break;
            }
            i = next_i;
            j = next_j;
        }
        path.reverse();

        Ok(score_pairing(
            "dtw",
            roll_bias,
            right_aligned,
            &right_aligned_route_progress,
            path,
        ))
    };

    let mut candidates = Vec::new();
    match alignment_roll_bias {
        DtwAlignmentRollBias::Auto => {
            for roll_bias in -8..=8 {
                candidates.push(evaluate_roll(roll_bias)?);
            }
        }
        DtwAlignmentRollBias::Explicit(value) => candidates.push(evaluate_roll(value)?),
    }
    let selected = candidates
        .iter()
        .min_by(|left, right| left.score.total_cmp(&right.score))
        .expect("DTW roll selection requires at least one candidate");
    let score_table = format_roll_score_table_json(&candidates);

    Ok((
        selected.right_aligned.clone(),
        selected.path.clone(),
        selected.right_route_progress.clone(),
        [
            alignment_meta,
            vec![
                (
                    "dtw_alignment_roll_bias".to_owned(),
                    JsonValue::Integer(selected.roll_bias as i64),
                ),
                ("dtw_pairing_mode".to_owned(), selected.pairing_mode.into()),
                (
                    "dtw_alignment_roll_bias_mode".to_owned(),
                    alignment_roll_bias.mode().into(),
                ),
                (
                    "dtw_alignment_roll_bias_requested".to_owned(),
                    alignment_roll_bias.to_string().into(),
                ),
                (
                    "dtw_alignment_roll_bias_auto_window".to_owned(),
                    match alignment_roll_bias {
                        DtwAlignmentRollBias::Auto => "[-8,8]",
                        DtwAlignmentRollBias::Explicit(_) => "",
                    }
                    .into(),
                ),
                (
                    "dtw_alignment_roll_bias_auto_score_table".to_owned(),
                    score_table.into(),
                ),
                (
                    "dtw_alignment_roll_bias_selected_score".to_owned(),
                    selected.score.into(),
                ),
                (
                    "dtw_alignment_roll_bias_selected_seam_angle_max_deg".to_owned(),
                    selected.seam_angle_max_deg.into(),
                ),
                (
                    "dtw_alignment_roll_bias_selected_global_angle_p95_deg".to_owned(),
                    selected.global_angle_p95_deg.into(),
                ),
                (
                    "dtw_alignment_roll_bias_selected_global_angle_max_deg".to_owned(),
                    selected.global_angle_max_deg.into(),
                ),
                (
                    "dtw_alignment_roll_bias_selected_seam_angle_p95_deg".to_owned(),
                    selected.seam_angle_p95_deg.into(),
                ),
                (
                    "dtw_alignment_roll_bias_selected_progress_delta_abs_p95".to_owned(),
                    selected.progress_delta_abs_p95.into(),
                ),
                (
                    "dtw_alignment_roll_bias_selected_crossing_count".to_owned(),
                    JsonValue::Integer(selected.crossing_count),
                ),
                (
                    "dtw_alignment_roll_bias_selected_width_median_m".to_owned(),
                    selected.width_median_m.into(),
                ),
                (
                    "dtw_alignment_roll_bias_selected_width_max_m".to_owned(),
                    selected.width_max_m.into(),
                ),
                (
                    "dtw_alignment_roll_bias_selected_width_ratio_max_to_median".to_owned(),
                    selected.width_ratio_max_to_median.into(),
                ),
                (
                    "dtw_alignment_roll_bias_selected_centerline_length_m".to_owned(),
                    selected.centerline_length_m.into(),
                ),
                (
                    "dtw_alignment_roll_bias_selected_centerline_length_ratio_abs_log".to_owned(),
                    selected.centerline_length_ratio_abs_log.into(),
                ),
                (
                    "dtw_centerline_normal_cost_weight".to_owned(),
                    normal_weight.into(),
                ),
                ("dtw_slide_cost_weight".to_owned(), slide_weight.into()),
                (
                    "dtw_path_length".to_owned(),
                    JsonValue::Integer(selected.path_length as i64),
                ),
                (
                    "dtw_path_pair_checksum".to_owned(),
                    JsonValue::Integer(dtw_path_pair_checksum(&selected.path)),
                ),
            ],
        ]
        .concat(),
    ))
}

fn pair_boundaries_dtw_open(
    left_world: &[Point2],
    right_world: &[Point2],
    band: usize,
    centerline_normal_cost_weight: f64,
    slide_cost_weight: f64,
    slide_step_penalty: f64,
    slide_repeat_penalty: f64,
    control: StationGenerationControl<'_>,
) -> StationBuildResult<OpenBoundaryPairingResult> {
    control.checkpoint()?;
    let count = left_world.len().min(right_world.len());
    if count == 0 {
        return Ok((
            Vec::new(),
            Vec::new(),
            vec![("dtw_topology".to_owned(), "open".into())],
        ));
    }
    let left_world = &left_world[..count];
    let right_world = &right_world[..count];
    let (left_inward_normals, left_tangents) = right_normals_world_open(left_world);
    let (right_right_normals, right_tangents) = right_normals_world_open(right_world);
    let centerline_hint = left_world
        .iter()
        .zip(right_world)
        .map(|(left, right)| midpoint(*left, *right))
        .collect::<Vec<_>>();
    let right_inward_normals = right_right_normals
        .iter()
        .map(|normal| point_scale(*normal, -1.0))
        .collect::<Vec<_>>();
    let normal_weight = centerline_normal_cost_weight.max(0.0);
    let slide_weight = slide_cost_weight.max(0.0);
    let slide_step_penalty = slide_step_penalty.max(0.0);
    let slide_repeat_penalty = slide_repeat_penalty.max(0.0);
    let inf = 1e18;
    let band = band.max(1);
    let mut cost = vec![inf; count * count];

    for i in 0..count {
        if i % 32 == 0 {
            control.checkpoint_phase("open_dtw")?;
        }
        let j0 = i.saturating_sub(band);
        let j1 = count.min(i + band + 1);
        for j in j0..j1 {
            let chord = point_sub(right_world[j], left_world[i]);
            let distance = hypot(chord);
            let chord_dir = if distance > 1e-9 {
                point_scale(chord, 1.0 / distance)
            } else {
                [0.0, 0.0]
            };
            let normal_mismatch = 1.0 - dot(chord_dir, left_inward_normals[i]).clamp(-1.0, 1.0);
            let right_mismatch = 1.0 + dot(chord_dir, right_inward_normals[j]).clamp(-1.0, 1.0);
            let tangent_mismatch = 1.0 - dot(left_tangents[i], right_tangents[j]).clamp(-1.0, 1.0);
            let progress_mismatch = (j as f64 - i as f64).abs() / count.max(1) as f64;
            let hint_normal = centerline_projected_normal_open(
                &centerline_hint,
                midpoint(left_world[i], right_world[j]),
            );
            cost[i * count + j] = distance
                + (3.0 + normal_weight) * normal_mismatch
                + 3.0 * right_mismatch
                + tangent_mismatch
                + normal_weight * (1.0 - dot(chord_dir, hint_normal).abs().clamp(0.0, 1.0))
                + 0.30 * progress_mismatch;
        }
    }

    let dp_size = count;
    const DTW_STATE_DIAG: usize = 0;
    const DTW_STATE_VERTICAL: usize = 1;
    const DTW_STATE_HORIZONTAL: usize = 2;
    const DTW_STATE_COUNT: usize = 3;
    let state_index =
        |i: usize, j: usize, state: usize| (i * dp_size + j) * DTW_STATE_COUNT + state;
    let mut dp = vec![inf; dp_size * dp_size * DTW_STATE_COUNT];
    let mut prev_state = vec![u8::MAX; dp_size * dp_size * DTW_STATE_COUNT];
    dp[state_index(0, 0, DTW_STATE_DIAG)] = cost[0];

    for i in 0..count {
        if i % 32 == 0 {
            control.checkpoint_phase("open_dtw")?;
        }
        for j in 0..count {
            if i == 0 && j == 0 {
                continue;
            }
            if j + band < i || i + band < j {
                continue;
            }
            let cell_cost = cost[i * count + j];
            if !cell_cost.is_finite() || cell_cost >= inf {
                continue;
            }

            if i > 0 && j > 0 {
                let (best_prev_state, best_prev_cost) =
                    best_dtw_prev_state(&dp, &state_index, i - 1, j - 1);
                let total = best_prev_cost + cell_cost;
                if total.is_finite() && total < inf {
                    let index = state_index(i, j, DTW_STATE_DIAG);
                    dp[index] = total;
                    prev_state[index] = best_prev_state as u8;
                }
            }

            if i > 0 {
                let base = slide_weight * cell_cost + slide_step_penalty;
                let candidates = [
                    (
                        dp[state_index(i - 1, j, DTW_STATE_DIAG)] + base,
                        DTW_STATE_DIAG,
                    ),
                    (
                        dp[state_index(i - 1, j, DTW_STATE_VERTICAL)] + base + slide_repeat_penalty,
                        DTW_STATE_VERTICAL,
                    ),
                    (
                        dp[state_index(i - 1, j, DTW_STATE_HORIZONTAL)] + base,
                        DTW_STATE_HORIZONTAL,
                    ),
                ];
                let (best_total, best_prev_state) = candidates
                    .into_iter()
                    .min_by(|left, right| left.0.total_cmp(&right.0))
                    .unwrap();
                if best_total.is_finite() && best_total < inf {
                    let index = state_index(i, j, DTW_STATE_VERTICAL);
                    dp[index] = best_total;
                    prev_state[index] = best_prev_state as u8;
                }
            }

            if j > 0 {
                let base = slide_weight * cell_cost + slide_step_penalty;
                let candidates = [
                    (
                        dp[state_index(i, j - 1, DTW_STATE_DIAG)] + base,
                        DTW_STATE_DIAG,
                    ),
                    (
                        dp[state_index(i, j - 1, DTW_STATE_VERTICAL)] + base,
                        DTW_STATE_VERTICAL,
                    ),
                    (
                        dp[state_index(i, j - 1, DTW_STATE_HORIZONTAL)]
                            + base
                            + slide_repeat_penalty,
                        DTW_STATE_HORIZONTAL,
                    ),
                ];
                let (best_total, best_prev_state) = candidates
                    .into_iter()
                    .min_by(|left, right| left.0.total_cmp(&right.0))
                    .unwrap();
                if best_total.is_finite() && best_total < inf {
                    let index = state_index(i, j, DTW_STATE_HORIZONTAL);
                    dp[index] = best_total;
                    prev_state[index] = best_prev_state as u8;
                }
            }
        }
    }

    let mut path = Vec::<(usize, usize)>::new();
    let mut i = count - 1;
    let mut j = count - 1;
    let (mut state, _) = best_dtw_prev_state(&dp, &state_index, i, j);
    loop {
        path.push((i, j));
        if i == 0 && j == 0 {
            break;
        }
        let index = state_index(i, j, state);
        let next_state = prev_state[index];
        if next_state == u8::MAX {
            break;
        }
        match state {
            DTW_STATE_DIAG => {
                i = i.saturating_sub(1);
                j = j.saturating_sub(1);
            }
            DTW_STATE_VERTICAL => {
                i = i.saturating_sub(1);
            }
            DTW_STATE_HORIZONTAL => {
                j = j.saturating_sub(1);
            }
            _ => break,
        }
        state = next_state as usize;
    }
    path.reverse();
    if path.first().copied() != Some((0, 0)) {
        path.insert(0, (0, 0));
    }
    if path.last().copied() != Some((count - 1, count - 1)) {
        path.push((count - 1, count - 1));
    }

    let mut matches = vec![Vec::<usize>::new(); count];
    for (left_index, right_index) in &path {
        matches[*left_index].push(*right_index);
    }
    let mut previous_right_index = 0_usize;
    let mut paired_right = vec![[0.0, 0.0]; count];
    let mut paired_right_indices = vec![0_usize; count];
    for (index, right_indices) in matches.iter().enumerate() {
        let right_index = if right_indices.is_empty() {
            previous_right_index.min(count - 1)
        } else {
            let mean =
                right_indices.iter().copied().sum::<usize>() as f64 / right_indices.len() as f64;
            mean.round() as usize
        }
        .min(count - 1);
        previous_right_index = right_index;
        paired_right_indices[index] = right_index;
        paired_right[index] = right_world[right_index];
    }

    #[derive(Clone, Copy)]
    struct OpenPairingQuality {
        score: f64,
        crossing_count: i64,
        width_collapse_m: f64,
    }

    let open_pairing_quality_score = |candidate_right: &[Point2]| -> OpenPairingQuality {
        let crossings = station_horizon_crossing_count_open(left_world, candidate_right, 2);
        let widths = left_world
            .iter()
            .zip(candidate_right)
            .map(|(left, right)| distance(*left, *right))
            .collect::<Vec<_>>();
        let width_median = median(widths.clone()).max(1e-9);
        let width_min = widths.iter().copied().fold(f64::INFINITY, f64::min);
        let width_max = widths.iter().copied().fold(0.0, f64::max);
        let width_spread_ratio = width_max / width_min.max(1e-9);
        let width_collapse = (0.20 * width_median - width_min).max(0.0);
        let score = 1_000_000_000.0 * crossings as f64
            + 10_000_000.0 * width_collapse.powi(2)
            + 1_000_000.0 * (width_spread_ratio - 1.8).max(0.0).powi(2);
        OpenPairingQuality {
            score,
            crossing_count: crossings,
            width_collapse_m: width_collapse,
        }
    };
    let dtw_pairing_score = open_pairing_quality_score(&paired_right);
    let progress_paired_right = right_world.to_vec();
    let progress_pairing_score = open_pairing_quality_score(&progress_paired_right);
    let progress_fixes_hard_gate = progress_pairing_score.crossing_count
        < dtw_pairing_score.crossing_count
        || (progress_pairing_score.crossing_count == dtw_pairing_score.crossing_count
            && progress_pairing_score.width_collapse_m + 1e-9 < dtw_pairing_score.width_collapse_m);
    let (pairing_mode, selected_pairing_score, rejected_pairing_score) = if progress_fixes_hard_gate
    {
        paired_right = progress_paired_right;
        paired_right_indices = (0..count).collect();
        path = (0..count).map(|index| (index, index)).collect();
        (
            "progress",
            progress_pairing_score.score,
            dtw_pairing_score.score,
        )
    } else {
        ("dtw", dtw_pairing_score.score, progress_pairing_score.score)
    };

    let angle_deg = (0..count)
        .map(|index| {
            let chord = point_sub(paired_right[index], left_world[index]);
            let width = hypot(chord);
            let chord_dir = if width > 1e-9 {
                point_scale(chord, 1.0 / width)
            } else {
                [0.0, 0.0]
            };
            let left_angle = dot(chord_dir, left_inward_normals[index])
                .clamp(-1.0, 1.0)
                .acos()
                .to_degrees();
            let right_angle = (-dot(chord_dir, right_inward_normals[paired_right_indices[index]]))
                .clamp(-1.0, 1.0)
                .acos()
                .to_degrees();
            0.5 * (left_angle + right_angle)
        })
        .collect::<Vec<_>>();
    let progress_delta_abs = paired_right_indices
        .iter()
        .enumerate()
        .map(|(index, right_index)| (*right_index as isize - index as isize).unsigned_abs() as f64)
        .collect::<Vec<_>>();
    let (vertical_step_count, horizontal_step_count) = dtw_path_slide_counts(&path);
    let slide_step_count = vertical_step_count + horizontal_step_count;
    let same_right_run_max = longest_same_right_run(&path);
    let same_left_run_max = longest_same_left_run(&path);
    let paired_right_plateau_run_max = longest_equal_run(&paired_right_indices);
    let paired_right_progress_delta_min = paired_right_indices
        .windows(2)
        .map(|pair| pair[1] as isize - pair[0] as isize)
        .min()
        .unwrap_or(0);
    let slide_step_fraction = if path.len() > 1 {
        slide_step_count as f64 / (path.len() - 1) as f64
    } else {
        0.0
    };
    let crossing_count = station_horizon_crossing_count_open(left_world, &paired_right, 2);
    let path_checksum = dtw_path_pair_checksum(&path);

    Ok((
        paired_right,
        path,
        vec![
            ("dtw_topology".to_owned(), "open".into()),
            ("dtw_pairing_mode".to_owned(), pairing_mode.into()),
            (
                "dtw_pairing_selected_score".to_owned(),
                selected_pairing_score.into(),
            ),
            (
                "dtw_pairing_rejected_score".to_owned(),
                rejected_pairing_score.into(),
            ),
            ("orientation".to_owned(), "same".into()),
            ("shift".to_owned(), JsonValue::Integer(0)),
            (
                "dtw_centerline_normal_cost_weight".to_owned(),
                normal_weight.into(),
            ),
            ("dtw_slide_cost_weight".to_owned(), slide_weight.into()),
            (
                "dtw_slide_step_penalty".to_owned(),
                slide_step_penalty.into(),
            ),
            (
                "dtw_slide_repeat_penalty".to_owned(),
                slide_repeat_penalty.into(),
            ),
            ("dtw_alignment_roll_bias".to_owned(), JsonValue::Integer(0)),
            (
                "dtw_alignment_roll_bias_mode".to_owned(),
                "open_none".into(),
            ),
            (
                "dtw_alignment_roll_bias_selected_global_angle_p95_deg".to_owned(),
                percentile(angle_deg.clone(), 95.0).into(),
            ),
            (
                "dtw_alignment_roll_bias_selected_global_angle_max_deg".to_owned(),
                angle_deg.iter().copied().fold(0.0, f64::max).into(),
            ),
            (
                "dtw_alignment_roll_bias_selected_progress_delta_abs_p95".to_owned(),
                percentile(progress_delta_abs, 95.0).into(),
            ),
            (
                "dtw_alignment_roll_bias_selected_crossing_count".to_owned(),
                JsonValue::Integer(crossing_count),
            ),
            (
                "dtw_same_right_run_max".to_owned(),
                JsonValue::Integer(same_right_run_max as i64),
            ),
            (
                "dtw_same_left_run_max".to_owned(),
                JsonValue::Integer(same_left_run_max as i64),
            ),
            (
                "dtw_paired_right_plateau_run_max".to_owned(),
                JsonValue::Integer(paired_right_plateau_run_max as i64),
            ),
            (
                "dtw_vertical_step_count".to_owned(),
                JsonValue::Integer(vertical_step_count as i64),
            ),
            (
                "dtw_horizontal_step_count".to_owned(),
                JsonValue::Integer(horizontal_step_count as i64),
            ),
            (
                "dtw_slide_step_fraction".to_owned(),
                slide_step_fraction.into(),
            ),
            (
                "dtw_paired_right_progress_delta_min".to_owned(),
                JsonValue::Integer(paired_right_progress_delta_min as i64),
            ),
            (
                "dtw_path_pair_checksum".to_owned(),
                JsonValue::Integer(path_checksum),
            ),
        ],
    ))
}

fn best_dtw_prev_state<F>(dp: &[f64], state_index: &F, i: usize, j: usize) -> (usize, f64)
where
    F: Fn(usize, usize, usize) -> usize,
{
    const DTW_STATE_DIAG: usize = 0;
    const DTW_STATE_VERTICAL: usize = 1;
    const DTW_STATE_HORIZONTAL: usize = 2;
    [
        (DTW_STATE_DIAG, dp[state_index(i, j, DTW_STATE_DIAG)]),
        (
            DTW_STATE_VERTICAL,
            dp[state_index(i, j, DTW_STATE_VERTICAL)],
        ),
        (
            DTW_STATE_HORIZONTAL,
            dp[state_index(i, j, DTW_STATE_HORIZONTAL)],
        ),
    ]
    .into_iter()
    .min_by(|left, right| left.1.total_cmp(&right.1))
    .unwrap()
}

fn dtw_path_slide_counts(path: &[(usize, usize)]) -> (usize, usize) {
    let mut vertical = 0_usize;
    let mut horizontal = 0_usize;
    for pair in path.windows(2) {
        let (prev_i, prev_j) = pair[0];
        let (next_i, next_j) = pair[1];
        if next_i > prev_i && next_j == prev_j {
            vertical += 1;
        } else if next_i == prev_i && next_j > prev_j {
            horizontal += 1;
        }
    }
    (vertical, horizontal)
}

fn longest_same_right_run(path: &[(usize, usize)]) -> usize {
    longest_run_by(path, |pair| pair.1)
}

fn longest_same_left_run(path: &[(usize, usize)]) -> usize {
    longest_run_by(path, |pair| pair.0)
}

fn longest_run_by<F>(path: &[(usize, usize)], mut key: F) -> usize
where
    F: FnMut((usize, usize)) -> usize,
{
    let mut best = 0_usize;
    let mut previous = None::<usize>;
    let mut current = 0_usize;
    for pair in path.iter().copied() {
        let value = key(pair);
        if previous == Some(value) {
            current += 1;
        } else {
            current = 1;
            previous = Some(value);
        }
        best = best.max(current);
    }
    best
}

fn longest_equal_run(values: &[usize]) -> usize {
    let mut best = 0_usize;
    let mut previous = None::<usize>;
    let mut current = 0_usize;
    for value in values.iter().copied() {
        if previous == Some(value) {
            current += 1;
        } else {
            current = 1;
            previous = Some(value);
        }
        best = best.max(current);
    }
    best
}

fn dtw_path_pair_checksum(path: &[(usize, usize)]) -> i64 {
    path.iter().fold(0_i64, |checksum, (left, right)| {
        checksum
            .wrapping_mul(1_000_003)
            .wrapping_add((left + 1) as i64 * 9_176)
            .wrapping_add((right + 1) as i64)
    })
}

fn align_closed_boundaries(
    left_world: &[Point2],
    right_world: &[Point2],
) -> (Vec<Point2>, JsonObject, bool, usize) {
    let count = left_world.len();
    let mut best_score = f64::INFINITY;
    let mut best_crossing_count = i64::MAX;
    let mut best_right = right_world.to_vec();
    let best_orientation = "same";
    let mut best_shift = 0_usize;
    for shift in 0..count {
        let shifted = shifted_closed_points(right_world, shift);
        let crossing_count = station_horizon_crossing_count(left_world, &shifted, 2);
        let score = shifted
            .iter()
            .enumerate()
            .map(|(index, point)| distance(left_world[index], *point))
            .sum::<f64>()
            / count.max(1) as f64;
        if score < best_score {
            best_crossing_count = crossing_count;
            best_score = score;
            best_right = shifted;
            best_shift = shift;
        }
    }

    (
        best_right,
        vec![
            ("orientation".to_owned(), best_orientation.into()),
            ("shift".to_owned(), JsonValue::Integer(best_shift as i64)),
            ("mean_pair_distance_m".to_owned(), best_score.into()),
            (
                "alignment_horizon2_crossing_count".to_owned(),
                JsonValue::Integer(best_crossing_count),
            ),
        ],
        best_orientation == "reversed",
        best_shift,
    )
}

fn format_roll_score_table_json(candidates: &[RollCandidate]) -> String {
    let entries = candidates
        .iter()
        .map(|candidate| {
            format!(
                "{{\"pairing_mode\":\"{}\",\"roll_bias\":{},\"score\":{},\"seam_angle_p95_deg\":{},\"seam_angle_max_deg\":{},\"global_angle_p95_deg\":{},\"global_angle_max_deg\":{},\"progress_delta_abs_p95\":{},\"crossing_count\":{},\"width_median_m\":{},\"width_max_m\":{},\"width_ratio_max_to_median\":{},\"centerline_length_m\":{},\"centerline_length_ratio_abs_log\":{}}}",
                candidate.pairing_mode,
                candidate.roll_bias,
                json_number(candidate.score),
                json_number(candidate.seam_angle_p95_deg),
                json_number(candidate.seam_angle_max_deg),
                json_number(candidate.global_angle_p95_deg),
                json_number(candidate.global_angle_max_deg),
                json_number(candidate.progress_delta_abs_p95),
                candidate.crossing_count,
                json_number(candidate.width_median_m),
                json_number(candidate.width_max_m),
                json_number(candidate.width_ratio_max_to_median),
                json_number(candidate.centerline_length_m),
                json_number(candidate.centerline_length_ratio_abs_log),
            )
        })
        .collect::<Vec<_>>()
        .join(",");

    format!("[{entries}]")
}

fn json_number(value: f64) -> String {
    if value.is_finite() {
        value.to_string()
    } else {
        "null".to_owned()
    }
}

fn f64s_json(values: &[f64]) -> JsonValue {
    JsonValue::Array(values.iter().copied().map(JsonValue::from).collect())
}

fn centerline_projected_normal(centerline: &[Point2], point: Point2) -> Point2 {
    if centerline.len() < 2 {
        return [1.0, 0.0];
    }

    let mut best_distance_sq = f64::INFINITY;
    let mut best_tangent = [1.0, 0.0];
    for index in 0..centerline.len() {
        let start = centerline[index];
        let end = centerline[(index + 1) % centerline.len()];
        let segment = point_sub(end, start);
        let length_sq = dot(segment, segment);
        let t = if length_sq > 1e-12 {
            (dot(point_sub(point, start), segment) / length_sq).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let projection = point_add(start, point_scale(segment, t));
        let delta = point_sub(point, projection);
        let distance_sq = dot(delta, delta);
        if distance_sq < best_distance_sq {
            best_distance_sq = distance_sq;
            best_tangent = normalize(segment, [1.0, 0.0]);
        }
    }

    normalize([best_tangent[1], -best_tangent[0]], [1.0, 0.0])
}

fn centerline_projected_normal_open(centerline: &[Point2], point: Point2) -> Point2 {
    if centerline.len() < 2 {
        return [1.0, 0.0];
    }

    let mut best_distance_sq = f64::INFINITY;
    let mut best_tangent = [1.0, 0.0];
    for index in 0..centerline.len() - 1 {
        let start = centerline[index];
        let end = centerline[index + 1];
        let segment = point_sub(end, start);
        let length_sq = dot(segment, segment);
        let t = if length_sq > 1e-12 {
            (dot(point_sub(point, start), segment) / length_sq).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let projection = point_add(start, point_scale(segment, t));
        let delta = point_sub(point, projection);
        let distance_sq = dot(delta, delta);
        if distance_sq < best_distance_sq {
            best_distance_sq = distance_sq;
            best_tangent = normalize(segment, [1.0, 0.0]);
        }
    }

    normalize([best_tangent[1], -best_tangent[0]], [1.0, 0.0])
}

fn shifted_closed_points(points: &[Point2], shift: usize) -> Vec<Point2> {
    points
        .iter()
        .enumerate()
        .map(|(index, _)| points[(index + shift) % points.len()])
        .collect()
}

fn roll_points(points: &[Point2], shift: isize) -> Vec<Point2> {
    let count = points.len() as isize;
    if count == 0 {
        return Vec::new();
    }
    points
        .iter()
        .enumerate()
        .map(|(index, _)| {
            let source = (index as isize - shift).rem_euclid(count) as usize;
            points[source]
        })
        .collect()
}

fn shifted_closed_values<T: Copy>(values: &[T], shift: usize) -> Vec<T> {
    if values.is_empty() {
        return Vec::new();
    }
    values
        .iter()
        .enumerate()
        .map(|(index, _)| values[(index + shift) % values.len()])
        .collect()
}

fn roll_values<T: Copy>(values: &[T], shift: isize) -> Vec<T> {
    let count = values.len() as isize;
    if count == 0 {
        return Vec::new();
    }
    values
        .iter()
        .enumerate()
        .map(|(index, _)| values[(index as isize - shift).rem_euclid(count) as usize])
        .collect()
}

fn cyclic_unit_distance(left: f64, right: f64) -> f64 {
    let delta = (left.rem_euclid(1.0) - right.rem_euclid(1.0)).abs();
    delta.min(1.0 - delta)
}

fn resample_closed_polyline(points: &[Point2], sample_count: usize) -> Vec<Point2> {
    if points.is_empty() {
        return Vec::new();
    }
    if points.len() == 1 {
        return vec![points[0]; sample_count];
    }

    let (s, _) = closed_polyline_arclength(points);
    let total = s.last().copied().unwrap_or(0.0);
    if !total.is_finite() || total <= 1e-9 {
        return vec![points[0]; sample_count];
    }

    let mut closed_points = points.to_vec();
    closed_points.push(points[0]);
    let xs = closed_points
        .iter()
        .map(|point| point[0])
        .collect::<Vec<_>>();
    let ys = closed_points
        .iter()
        .map(|point| point[1])
        .collect::<Vec<_>>();

    (0..sample_count)
        .map(|index| {
            let target_s = total * index as f64 / sample_count as f64;
            [
                interp_scalar(target_s, &s, &xs),
                interp_scalar(target_s, &s, &ys),
            ]
        })
        .collect()
}

fn resample_open_polyline(points: &[Point2], sample_count: usize) -> Vec<Point2> {
    if points.is_empty() || sample_count == 0 {
        return Vec::new();
    }
    if points.len() == 1 {
        return vec![points[0]; sample_count];
    }
    if sample_count == 1 {
        return vec![points[0]];
    }

    let (s, _) = open_polyline_arclength(points);
    let total = s.last().copied().unwrap_or(0.0);
    if !total.is_finite() || total <= 1e-9 {
        return vec![points[0]; sample_count];
    }
    let xs = points.iter().map(|point| point[0]).collect::<Vec<_>>();
    let ys = points.iter().map(|point| point[1]).collect::<Vec<_>>();

    (0..sample_count)
        .map(|index| {
            let target_s = total * index as f64 / (sample_count - 1) as f64;
            [
                interp_scalar(target_s, &s, &xs),
                interp_scalar(target_s, &s, &ys),
            ]
        })
        .collect()
}

fn station_frame_progress_for_path(topology: StationTopology, count: usize) -> Vec<f64> {
    match count {
        0 => Vec::new(),
        1 => vec![0.0],
        _ => match topology {
            StationTopology::Closed => (0..count)
                .map(|index| index as f64 / count as f64)
                .collect(),
            StationTopology::Open => (0..count)
                .map(|index| index as f64 / (count - 1) as f64)
                .collect(),
        },
    }
}

fn interp_points_by_shared_progress(
    progress: &[f64],
    points: &[Point2],
    target_progress: &[f64],
) -> Vec<Point2> {
    if progress.len() != points.len() || points.is_empty() {
        return vec![points.first().copied().unwrap_or([0.0, 0.0]); target_progress.len()];
    }
    let (progress_unique, points_unique) = collapse_progress_samples(progress, points);
    if points_unique.len() < 2 {
        return vec![points.first().copied().unwrap_or([0.0, 0.0]); target_progress.len()];
    }
    let xs = points_unique
        .iter()
        .map(|point| point[0])
        .collect::<Vec<_>>();
    let ys = points_unique
        .iter()
        .map(|point| point[1])
        .collect::<Vec<_>>();

    target_progress
        .iter()
        .map(|target| {
            let clamped = target.clamp(0.0, 1.0);
            [
                interp_scalar(clamped, &progress_unique, &xs),
                interp_scalar(clamped, &progress_unique, &ys),
            ]
        })
        .collect()
}

fn interp_scalar_by_shared_progress(
    progress: &[f64],
    values: &[f64],
    target_progress: &[f64],
) -> Vec<f64> {
    if progress.len() != values.len() || values.is_empty() {
        return vec![values.first().copied().unwrap_or(0.0); target_progress.len()];
    }
    let (progress_unique, values_unique) = collapse_progress_scalar_samples(progress, values);
    if values_unique.len() < 2 {
        return vec![values.first().copied().unwrap_or(0.0); target_progress.len()];
    }
    target_progress
        .iter()
        .map(|target| interp_scalar(target.clamp(0.0, 1.0), &progress_unique, &values_unique))
        .collect()
}

fn periodic_interp_points(
    progress: &[f64],
    points: &[Point2],
    target_progress: &[f64],
) -> Vec<Point2> {
    if progress.len() != points.len() {
        return vec![points.first().copied().unwrap_or([0.0, 0.0]); target_progress.len()];
    }

    let (progress_unique, points_unique) = collapse_progress_samples(progress, points);

    if points_unique.len() < 2 {
        return vec![points.first().copied().unwrap_or([0.0, 0.0]); target_progress.len()];
    }

    let mut progress_ext = Vec::with_capacity(progress_unique.len() + 2);
    progress_ext.push(progress_unique[progress_unique.len() - 1] - 1.0);
    progress_ext.extend(progress_unique.iter().copied());
    progress_ext.push(progress_unique[0] + 1.0);
    let mut points_ext = Vec::with_capacity(points_unique.len() + 2);
    points_ext.push(points_unique[points_unique.len() - 1]);
    points_ext.extend(points_unique.iter().copied());
    points_ext.push(points_unique[0]);
    let xs = points_ext.iter().map(|point| point[0]).collect::<Vec<_>>();
    let ys = points_ext.iter().map(|point| point[1]).collect::<Vec<_>>();

    target_progress
        .iter()
        .map(|target| {
            [
                interp_scalar_numpy(*target, &progress_ext, &xs),
                interp_scalar_numpy(*target, &progress_ext, &ys),
            ]
        })
        .collect()
}

fn periodic_interp_route_progress(
    shared_progress: &[f64],
    route_progress: &[f64],
    target_progress: &[f64],
) -> Vec<f64> {
    if shared_progress.len() != route_progress.len() || route_progress.is_empty() {
        return vec![0.0; target_progress.len()];
    }
    let mut unwrapped = Vec::with_capacity(route_progress.len());
    unwrapped.push(route_progress[0].rem_euclid(1.0));
    for value in route_progress.iter().copied().skip(1) {
        let previous = *unwrapped.last().unwrap();
        let previous_wrapped = previous.rem_euclid(1.0);
        let forward_delta = (value.rem_euclid(1.0) - previous_wrapped).rem_euclid(1.0);
        unwrapped.push(previous + forward_delta);
    }
    interp_scalar_by_shared_progress(shared_progress, &unwrapped, target_progress)
        .into_iter()
        .map(|value| value.rem_euclid(1.0))
        .collect()
}

fn interp_scalar_numpy(x: f64, xp: &[f64], fp: &[f64]) -> f64 {
    if xp.is_empty() || fp.is_empty() {
        return 0.0;
    }
    let last = xp.len() - 1;
    if x < xp[0] {
        return fp[0];
    }
    if x > xp[last] {
        return fp[last];
    }
    let high = xp.partition_point(|value| *value <= x);
    if high == 0 {
        return fp[0];
    }
    if high >= xp.len() {
        return fp[last];
    }
    let low = high - 1;
    let denom = xp[high] - xp[low];
    let t = if denom.abs() <= 1e-12 {
        0.0
    } else {
        (x - xp[low]) / denom
    };
    fp[low] + (fp[high] - fp[low]) * t
}

fn collapse_progress_samples(progress: &[f64], points: &[Point2]) -> (Vec<f64>, Vec<Point2>) {
    let mut order = progress
        .iter()
        .copied()
        .enumerate()
        .map(|(index, value)| (value, index))
        .collect::<Vec<_>>();
    order.sort_by(|left, right| {
        left.0
            .total_cmp(&right.0)
            .then_with(|| left.1.cmp(&right.1))
    });

    let mut collapsed_progress = Vec::new();
    let mut collapsed_points = Vec::new();
    let mut start = 0_usize;
    while start < order.len() {
        let mut end = start + 1;
        while end < order.len() && order[end].0 - order[start].0 <= 1e-8 {
            end += 1;
        }
        let mut progress_sum = 0.0;
        let mut point_sum = [0.0, 0.0];
        for item in &order[start..end] {
            progress_sum += item.0;
            point_sum = point_add(point_sum, points[item.1]);
        }
        let count = (end - start) as f64;
        collapsed_progress.push(progress_sum / count);
        collapsed_points.push(point_scale(point_sum, 1.0 / count));
        start = end;
    }
    (collapsed_progress, collapsed_points)
}

fn collapse_progress_scalar_samples(progress: &[f64], values: &[f64]) -> (Vec<f64>, Vec<f64>) {
    let mut order = progress
        .iter()
        .copied()
        .enumerate()
        .map(|(index, value)| (value, index))
        .collect::<Vec<_>>();
    order.sort_by(|left, right| {
        left.0
            .total_cmp(&right.0)
            .then_with(|| left.1.cmp(&right.1))
    });

    let mut collapsed_progress = Vec::new();
    let mut collapsed_values = Vec::new();
    let mut start = 0_usize;
    while start < order.len() {
        let mut end = start + 1;
        while end < order.len() && order[end].0 - order[start].0 <= 1e-8 {
            end += 1;
        }
        let mut progress_sum = 0.0;
        let mut value_sum = 0.0;
        for item in &order[start..end] {
            progress_sum += item.0;
            value_sum += values[item.1];
        }
        let count = (end - start) as f64;
        collapsed_progress.push(progress_sum / count);
        collapsed_values.push(value_sum / count);
        start = end;
    }
    (collapsed_progress, collapsed_values)
}

fn closed_polyline_arclength(points: &[Point2]) -> (Vec<f64>, Vec<f64>) {
    if points.is_empty() {
        return (vec![0.0], Vec::new());
    }

    let mut s = Vec::with_capacity(points.len() + 1);
    let mut lengths = Vec::with_capacity(points.len());
    let mut total = 0.0;
    s.push(0.0);
    for index in 0..points.len() {
        let next = (index + 1) % points.len();
        let length = distance(points[index], points[next]);
        total += length;
        lengths.push(length);
        s.push(total);
    }
    (s, lengths)
}

fn open_polyline_arclength(points: &[Point2]) -> (Vec<f64>, Vec<f64>) {
    if points.is_empty() {
        return (vec![0.0], Vec::new());
    }

    let mut s = Vec::with_capacity(points.len());
    let mut lengths = Vec::with_capacity(points.len().saturating_sub(1));
    let mut total = 0.0;
    s.push(0.0);
    for index in 0..points.len().saturating_sub(1) {
        let length = distance(points[index], points[index + 1]);
        total += length;
        lengths.push(length);
        s.push(total);
    }
    (s, lengths)
}

fn right_normals_world(path_world: &[Point2]) -> (Vec<Point2>, Vec<Point2>) {
    let count = path_world.len();
    let mut normals = Vec::with_capacity(count);
    let mut tangents = Vec::with_capacity(count);
    for index in 0..count {
        let previous = path_world[(index + count - 1) % count];
        let next = path_world[(index + 1) % count];
        let tangent = normalize(point_sub(next, previous), [1.0, 0.0]);
        tangents.push(tangent);
        normals.push([tangent[1], -tangent[0]]);
    }
    (normals, tangents)
}

fn right_normals_world_open(path_world: &[Point2]) -> (Vec<Point2>, Vec<Point2>) {
    let count = path_world.len();
    let mut normals = Vec::with_capacity(count);
    let mut tangents = Vec::with_capacity(count);
    for index in 0..count {
        let tangent = if count <= 1 {
            [1.0, 0.0]
        } else if index == 0 {
            normalize(point_sub(path_world[1], path_world[0]), [1.0, 0.0])
        } else if index + 1 == count {
            normalize(
                point_sub(path_world[index], path_world[index - 1]),
                [1.0, 0.0],
            )
        } else {
            normalize(
                point_sub(path_world[index + 1], path_world[index - 1]),
                [1.0, 0.0],
            )
        };
        tangents.push(tangent);
        normals.push([tangent[1], -tangent[0]]);
    }
    (normals, tangents)
}

fn local_turn_angles(path_world: &[Point2]) -> Vec<f64> {
    let count = path_world.len();
    (0..count)
        .map(|index| {
            let previous = path_world[(index + count - 1) % count];
            let current = path_world[index];
            let next = path_world[(index + 1) % count];
            let prev_tangent = normalize(point_sub(current, previous), [1.0, 0.0]);
            let next_tangent = normalize(point_sub(next, current), prev_tangent);
            cross(prev_tangent, next_tangent)
                .atan2(dot(prev_tangent, next_tangent).clamp(-1.0, 1.0))
        })
        .collect()
}

fn local_turn_angles_open(path_world: &[Point2]) -> Vec<f64> {
    let count = path_world.len();
    if count == 0 {
        return Vec::new();
    }
    if count < 3 {
        return vec![0.0; count];
    }
    (0..count)
        .map(|index| {
            if index == 0 || index + 1 == count {
                return 0.0;
            }
            let previous = path_world[index - 1];
            let current = path_world[index];
            let next = path_world[index + 1];
            let prev_tangent = normalize(point_sub(current, previous), [1.0, 0.0]);
            let next_tangent = normalize(point_sub(next, current), prev_tangent);
            cross(prev_tangent, next_tangent)
                .atan2(dot(prev_tangent, next_tangent).clamp(-1.0, 1.0))
        })
        .collect()
}

fn circular_smooth_points(points: &[Point2], window: usize) -> Vec<Point2> {
    if window <= 1 || points.len() < 3 {
        return points.to_vec();
    }
    let window = normalize_odd_window(window, points.len());
    if window <= 1 {
        return points.to_vec();
    }
    let xs = points.iter().map(|point| point[0]).collect::<Vec<_>>();
    let ys = points.iter().map(|point| point[1]).collect::<Vec<_>>();
    let sx = circular_smooth_1d(&xs, window);
    let sy = circular_smooth_1d(&ys, window);
    sx.into_iter().zip(sy).map(|(x, y)| [x, y]).collect()
}

fn open_smooth_points(points: &[Point2], window: usize) -> Vec<Point2> {
    if window <= 1 || points.len() < 3 {
        return points.to_vec();
    }
    let window = normalize_odd_window(window, points.len());
    if window <= 1 {
        return points.to_vec();
    }
    let xs = points.iter().map(|point| point[0]).collect::<Vec<_>>();
    let ys = points.iter().map(|point| point[1]).collect::<Vec<_>>();
    let sx = open_smooth_1d(&xs, window);
    let sy = open_smooth_1d(&ys, window);
    sx.into_iter().zip(sy).map(|(x, y)| [x, y]).collect()
}

fn circular_smooth_1d(values: &[f64], window: usize) -> Vec<f64> {
    if window <= 1 || values.len() < 3 {
        return values.to_vec();
    }
    let window = normalize_odd_window(window, values.len());
    if window <= 1 {
        return values.to_vec();
    }
    let pad = window / 2;
    (0..values.len())
        .map(|index| {
            let mut sum = 0.0;
            for offset in 0..window {
                let source = (index + values.len() + offset - pad) % values.len();
                sum += values[source];
            }
            sum / window as f64
        })
        .collect()
}

fn open_smooth_1d(values: &[f64], window: usize) -> Vec<f64> {
    if window <= 1 || values.len() < 3 {
        return values.to_vec();
    }
    let window = normalize_odd_window(window, values.len());
    if window <= 1 {
        return values.to_vec();
    }
    let radius = window / 2;
    (0..values.len())
        .map(|index| {
            let start = index.saturating_sub(radius);
            let end = (index + radius + 1).min(values.len());
            values[start..end].iter().sum::<f64>() / (end - start) as f64
        })
        .collect()
}

fn open_smooth_positive_profile(values: &[f64], window: usize) -> Vec<f64> {
    if values.is_empty() {
        return Vec::new();
    }
    let original_mean = values.iter().sum::<f64>() / values.len() as f64;
    let log_values = values
        .iter()
        .map(|value| value.max(1e-12).ln())
        .collect::<Vec<_>>();
    let mut smoothed = open_smooth_1d(&log_values, window)
        .into_iter()
        .map(f64::exp)
        .collect::<Vec<_>>();
    let smoothed_mean = smoothed.iter().sum::<f64>() / smoothed.len() as f64;
    if smoothed_mean > 1e-12 && original_mean > 1e-12 {
        for value in &mut smoothed {
            *value *= original_mean / smoothed_mean;
        }
    }
    smoothed
}

fn normalize_odd_window(window: usize, sample_count: usize) -> usize {
    if window <= 1 || sample_count < 3 {
        return 1;
    }
    let max_window = if sample_count % 2 == 1 {
        sample_count
    } else {
        sample_count - 1
    };
    let mut result = window.min(max_window);
    if result % 2 == 0 {
        result += 1;
    }
    result.max(1).min(max_window)
}

fn interp_scalar(x: f64, xp: &[f64], fp: &[f64]) -> f64 {
    if xp.is_empty() || fp.is_empty() {
        return 0.0;
    }
    let last = xp.len() - 1;
    if x <= xp[0] {
        return fp[0];
    }
    if x >= xp[last] {
        return fp[last];
    }
    let mut low = 0_usize;
    let mut high = last;
    while high - low > 1 {
        let middle = (low + high) / 2;
        if xp[middle] <= x {
            low = middle;
        } else {
            high = middle;
        }
    }
    let denom = xp[high] - xp[low];
    let t = if denom.abs() <= 1e-12 {
        0.0
    } else {
        (x - xp[low]) / denom
    };
    fp[low] + (fp[high] - fp[low]) * t
}

fn station_spacing_metrics(centerline: &[Point2]) -> (f64, f64) {
    let (_, lengths) = closed_polyline_arclength(centerline);
    if lengths.is_empty() {
        return (0.0, 0.0);
    }
    (
        lengths.iter().copied().fold(f64::INFINITY, f64::min),
        median(lengths),
    )
}

fn open_station_spacing_metrics(centerline: &[Point2]) -> (f64, f64) {
    let (_, lengths) = open_polyline_arclength(centerline);
    if lengths.is_empty() {
        return (0.0, 0.0);
    }
    (
        lengths.iter().copied().fold(f64::INFINITY, f64::min),
        median(lengths),
    )
}

fn max_adjacent_vector_rotation_deg(vectors: &[Point2]) -> f64 {
    if vectors.is_empty() {
        return 0.0;
    }
    (0..vectors.len())
        .map(|index| {
            let next = (index + 1) % vectors.len();
            dot(
                normalize(vectors[index], [1.0, 0.0]),
                normalize(vectors[next], [1.0, 0.0]),
            )
            .clamp(-1.0, 1.0)
            .acos()
            .to_degrees()
        })
        .fold(0.0, f64::max)
}

fn max_adjacent_vector_rotation_deg_open(vectors: &[Point2]) -> f64 {
    if vectors.len() < 2 {
        return 0.0;
    }
    vectors
        .windows(2)
        .map(|pair| {
            dot(
                normalize(pair[0], [1.0, 0.0]),
                normalize(pair[1], [1.0, 0.0]),
            )
            .clamp(-1.0, 1.0)
            .acos()
            .to_degrees()
        })
        .fold(0.0, f64::max)
}

fn station_horizon_crossing_count(left: &[Point2], right: &[Point2], horizon: usize) -> i64 {
    let mut count = 0_i64;
    for index in 0..left.len() {
        for offset in 1..=horizon.max(1) {
            let next = (index + offset) % left.len();
            if segment_intersects(left[index], right[index], left[next], right[next]) {
                count += 1;
            }
        }
    }
    count
}

fn station_horizon_crossing_count_open(left: &[Point2], right: &[Point2], horizon: usize) -> i64 {
    station_horizon_crossing_pairs_open(left, right, horizon).len() as i64
}

fn station_horizon_crossing_pairs_open(
    left: &[Point2],
    right: &[Point2],
    horizon: usize,
) -> Vec<(usize, usize)> {
    let count_len = left.len().min(right.len());
    let mut pairs = Vec::new();
    for index in 0..count_len {
        for offset in 1..=horizon.max(1) {
            let next = index + offset;
            if next >= count_len {
                continue;
            }
            if segment_intersects(left[index], right[index], left[next], right[next]) {
                pairs.push((index, next));
            }
        }
    }
    pairs
}

fn station_raw_boundary_crossing_count_open(
    left: &[Point2],
    right: &[Point2],
    raw_left_world: &[Point2],
    raw_right_world: &[Point2],
) -> i64 {
    left.iter()
        .zip(right)
        .map(|(station_left, station_right)| {
            station_segment_raw_boundary_crossing_count(
                *station_left,
                *station_right,
                raw_left_world,
                raw_right_world,
            )
        })
        .sum()
}

fn station_segment_raw_boundary_crossing_count(
    station_left: Point2,
    station_right: Point2,
    raw_left_world: &[Point2],
    raw_right_world: &[Point2],
) -> i64 {
    let left_count =
        open_polyline_segment_intersection_count(station_left, station_right, raw_left_world);
    let right_count =
        open_polyline_segment_intersection_count(station_left, station_right, raw_right_world);
    (left_count - 1).max(0) + (right_count - 1).max(0)
}

fn open_polyline_segment_intersection_count(a: Point2, b: Point2, polyline: &[Point2]) -> i64 {
    if polyline.len() < 2 {
        return 0;
    }
    polyline
        .windows(2)
        .filter(|segment| segment_intersects(a, b, segment[0], segment[1]))
        .count() as i64
}

fn segment_intersects(p0: Point2, p1: Point2, q0: Point2, q1: Point2) -> bool {
    let r = point_sub(p1, p0);
    let s = point_sub(q1, q0);
    let denom = cross(r, s);
    if denom.abs() <= 1e-9 {
        return false;
    }
    let qp = point_sub(q0, p0);
    let t = cross(qp, s) / denom;
    let u = cross(qp, r) / denom;
    1e-9 < t && t < 1.0 - 1e-9 && 1e-9 < u && u < 1.0 - 1e-9
}

fn segment_intersects_inclusive(p0: Point2, p1: Point2, q0: Point2, q1: Point2) -> bool {
    let o1 = orient2d(p0, p1, q0);
    let o2 = orient2d(p0, p1, q1);
    let o3 = orient2d(q0, q1, p0);
    let o4 = orient2d(q0, q1, p1);
    let eps = 1e-9;
    if o1 * o2 < -eps && o3 * o4 < -eps {
        return true;
    }
    (o1.abs() <= eps && point_on_segment(p0, p1, q0))
        || (o2.abs() <= eps && point_on_segment(p0, p1, q1))
        || (o3.abs() <= eps && point_on_segment(q0, q1, p0))
        || (o4.abs() <= eps && point_on_segment(q0, q1, p1))
}

fn orient2d(a: Point2, b: Point2, c: Point2) -> f64 {
    cross(point_sub(b, a), point_sub(c, a))
}

fn point_on_segment(a: Point2, b: Point2, c: Point2) -> bool {
    let eps = 1e-9;
    c[0] >= a[0].min(b[0]) - eps
        && c[0] <= a[0].max(b[0]) + eps
        && c[1] >= a[1].min(b[1]) - eps
        && c[1] <= a[1].max(b[1]) + eps
        && orient2d(a, b, c).abs() <= eps
}

struct BuiltSections {
    left: Vec<Point2>,
    right: Vec<Point2>,
    center: Vec<Point2>,
    normals: Vec<Point2>,
    width_right: Vec<f64>,
    width_left: Vec<f64>,
    miss_count: i64,
}

#[derive(Clone, Copy)]
struct StationCandidateScoreBreakdown {
    total: f64,
    crossings: i64,
    boundary_crossings: i64,
    miss: i32,
    max_rotation_deg: f64,
    rotation_violation_deg: f64,
    width_m: f64,
    width_violation_m: f64,
    endpoint_spacing_violation_m: f64,
    lr_projection_ratio: f64,
    lr_projection_soft_violation: f64,
    lr_projection_hard_violation: f64,
    angle_deg: f64,
}

fn widths_or_default(values: &[f64], count: usize, fallback: f64) -> Vec<f64> {
    if values.len() == count {
        values.to_vec()
    } else {
        vec![fallback; count]
    }
}

fn cumulative_with_zero(values: &[f64]) -> Vec<f64> {
    let mut result = Vec::with_capacity(values.len() + 1);
    let mut total = 0.0;
    result.push(0.0);
    for value in values {
        total += *value;
        result.push(total);
    }
    result
}

fn closed_path_progress(points: &[Point2]) -> Vec<f64> {
    let (station, _) = closed_polyline_arclength(points);
    let total = station.last().copied().unwrap_or(0.0).max(1e-9);
    station
        .iter()
        .take(points.len())
        .map(|value| *value / total)
        .collect()
}

fn closed_interp_points(points: &[Point2], progress: &[f64]) -> Vec<Point2> {
    if points.is_empty() {
        return Vec::new();
    }
    if points.len() == 1 {
        return vec![points[0]; progress.len()];
    }
    let (station, _) = closed_polyline_arclength(points);
    let total = station.last().copied().unwrap_or(0.0).max(1e-9);
    let mut source_progress = station
        .iter()
        .take(points.len())
        .map(|value| *value / total)
        .collect::<Vec<_>>();
    source_progress.push(1.0);
    let mut closed_points = points.to_vec();
    closed_points.push(points[0]);
    let xs = closed_points
        .iter()
        .map(|point| point[0])
        .collect::<Vec<_>>();
    let ys = closed_points
        .iter()
        .map(|point| point[1])
        .collect::<Vec<_>>();
    progress
        .iter()
        .map(|value| {
            let target = value.rem_euclid(1.0);
            [
                interp_scalar(target, &source_progress, &xs),
                interp_scalar(target, &source_progress, &ys),
            ]
        })
        .collect()
}

fn closed_interp_scalar(values: &[f64], source_points: &[Point2], progress: &[f64]) -> Vec<f64> {
    if values.is_empty() {
        return vec![0.0; progress.len()];
    }
    let (station, _) = closed_polyline_arclength(source_points);
    let total = station.last().copied().unwrap_or(0.0).max(1e-9);
    let mut source_progress = station
        .iter()
        .take(source_points.len())
        .map(|value| *value / total)
        .collect::<Vec<_>>();
    source_progress.push(1.0);
    let mut closed_values = values.to_vec();
    closed_values.push(values[0]);
    progress
        .iter()
        .map(|value| interp_scalar(value.rem_euclid(1.0), &source_progress, &closed_values))
        .collect()
}

fn sample_ref_widths_for_centerline(
    centerline: &[Point2],
    reference: &[Point2],
    reference_widths: &[f64],
) -> Vec<f64> {
    let progress = project_points_to_closed_progress(centerline, reference);
    closed_interp_scalar(reference_widths, reference, &progress)
}

fn align_station_frame_order_to_centerline(
    station_frame_left: &[Point2],
    station_frame_right: &[Point2],
    centerline: &[Point2],
) -> (Vec<Point2>, Vec<Point2>, JsonObject, bool, usize) {
    let mut left = station_frame_left.to_vec();
    let mut right = station_frame_right.to_vec();
    let frame_center = left
        .iter()
        .zip(&right)
        .map(|(l, r)| midpoint(*l, *r))
        .collect::<Vec<_>>();
    if frame_center.len() != centerline.len() || frame_center.len() <= 1 {
        return (
            left,
            right,
            vec![
                (
                    "dtw_station_frame_centerline_align_shift".to_owned(),
                    JsonValue::Integer(0),
                ),
                (
                    "dtw_station_frame_centerline_align_reversed".to_owned(),
                    JsonValue::Integer(0),
                ),
                (
                    "dtw_station_frame_centerline_same_index_rms_m".to_owned(),
                    (-1.0).into(),
                ),
            ],
            false,
            0,
        );
    }

    let mut best_shift = 0_usize;
    let mut best_rms = f64::INFINITY;
    for shift in 0..frame_center.len() {
        let candidate = roll_usize(&frame_center, shift);
        let rms = closed_curve_rms(&candidate, centerline);
        if rms < best_rms {
            best_rms = rms;
            best_shift = shift;
        }
    }
    if best_shift != 0 {
        left = roll_usize(&left, best_shift);
        right = roll_usize(&right, best_shift);
    }
    (
        left,
        right,
        vec![
            (
                "dtw_station_frame_centerline_align_shift".to_owned(),
                JsonValue::Integer(best_shift as i64),
            ),
            (
                "dtw_station_frame_centerline_align_reversed".to_owned(),
                JsonValue::Integer(0),
            ),
            (
                "dtw_station_frame_centerline_same_index_rms_m".to_owned(),
                best_rms.into(),
            ),
        ],
        false,
        best_shift,
    )
}

fn station_frame_progress_for_centerline(
    station_frame_left: &[Point2],
    station_frame_right: &[Point2],
    centerline: &[Point2],
) -> (Vec<f64>, JsonObject) {
    let frame_center = station_frame_left
        .iter()
        .zip(station_frame_right)
        .map(|(left, right)| midpoint(*left, *right))
        .collect::<Vec<_>>();
    let same_index_rms = if frame_center.len() == centerline.len() {
        closed_curve_rms(&frame_center, centerline)
    } else {
        -1.0
    };
    let progress = if frame_center.len() == centerline.len() {
        project_points_to_closed_progress_route_window(
            &frame_center,
            centerline,
            (centerline.len() / 4).clamp(8, 64),
        )
    } else {
        closed_path_progress(&frame_center)
    };
    let source = if frame_center.len() == centerline.len() {
        "route_window_projected_midpoint_to_centerline"
    } else {
        "route_ordered_frame_arclength"
    };
    let mut sorted = progress.clone();
    sorted.sort_by(f64::total_cmp);
    let mut diffs = sorted
        .windows(2)
        .map(|pair| pair[1] - pair[0])
        .collect::<Vec<_>>();
    if let (Some(first), Some(last)) = (sorted.first(), sorted.last()) {
        diffs.push(1.0 - *last + *first);
    }
    (
        progress,
        vec![
            (
                "dtw_station_frame_progress_source".to_owned(),
                source.into(),
            ),
            (
                "dtw_station_frame_progress_same_index_rms_m".to_owned(),
                same_index_rms.into(),
            ),
            (
                "dtw_station_frame_progress_min_step".to_owned(),
                diffs.iter().copied().fold(f64::INFINITY, f64::min).into(),
            ),
            (
                "dtw_station_frame_progress_max_step".to_owned(),
                diffs.iter().copied().fold(0.0, f64::max).into(),
            ),
        ],
    )
}

fn build_normal_line_sections(
    centerline: &[Point2],
    raw_left_world: &[Point2],
    raw_right_world: &[Point2],
    fallback_width_right: &[f64],
    fallback_width_left: &[f64],
    progress: &[f64],
    station_frame_left: &[Point2],
    station_frame_right: &[Point2],
    station_frame_left_route_progress: &[f64],
    station_frame_right_route_progress: &[f64],
    station_frame_progress: &[f64],
    zero_station_normal_fix: bool,
) -> BuiltSections {
    let allow_wide_progress_search = !closed_polyline_self_intersects(raw_left_world)
        && !closed_polyline_self_intersects(raw_right_world);
    let centers = closed_interp_points(centerline, progress);
    let frame_left = periodic_interp_points(station_frame_progress, station_frame_left, progress);
    let frame_right = periodic_interp_points(station_frame_progress, station_frame_right, progress);
    let frame_left_route_progress = periodic_interp_route_progress(
        station_frame_progress,
        station_frame_left_route_progress,
        progress,
    );
    let frame_right_route_progress = periodic_interp_route_progress(
        station_frame_progress,
        station_frame_right_route_progress,
        progress,
    );
    let mut normals = frame_left
        .iter()
        .zip(&frame_right)
        .map(|(left, right)| normalize(point_sub(*right, *left), [1.0, 0.0]))
        .collect::<Vec<_>>();
    apply_progress_zero_station_normal_fix(
        centerline,
        progress,
        &mut normals,
        zero_station_normal_fix,
    );
    let width_right = closed_interp_scalar(fallback_width_right, centerline, progress);
    let width_left = closed_interp_scalar(fallback_width_left, centerline, progress);
    let left_projection = ClosedPolylineProjection::new(raw_left_world);
    let right_projection = ClosedPolylineProjection::new(raw_right_world);
    let mut left = Vec::with_capacity(centers.len());
    let mut right = Vec::with_capacity(centers.len());
    let mut section_normals = Vec::with_capacity(centers.len());
    let mut miss_count = 0_i64;
    for index in 0..centers.len() {
        let fallback_left = point_sub(
            centers[index],
            point_scale(normals[index], width_left[index]),
        );
        let fallback_right = point_add(
            centers[index],
            point_scale(normals[index], width_right[index]),
        );
        let section = normal_line_section_at_progress(
            centers[index],
            normals[index],
            raw_left_world,
            raw_right_world,
            &left_projection,
            &right_projection,
            fallback_left,
            fallback_right,
            frame_left_route_progress[index],
            frame_right_route_progress[index],
            allow_wide_progress_search,
        );
        left.push(section.left);
        right.push(section.right);
        section_normals.push(section.normal);
        miss_count += i64::from(section.miss);
    }
    let width_right = right
        .iter()
        .zip(&centers)
        .map(|(right, center)| distance(*right, *center))
        .collect::<Vec<_>>();
    let width_left = left
        .iter()
        .zip(&centers)
        .map(|(left, center)| distance(*left, *center))
        .collect::<Vec<_>>();
    BuiltSections {
        left,
        right,
        center: centers,
        normals: section_normals,
        width_right,
        width_left,
        miss_count,
    }
}

fn area_preserving_chord_repair(
    centerline: &[Point2],
    raw_left_world: &[Point2],
    raw_right_world: &[Point2],
    raw_left_route_world: &[Point2],
    raw_right_route_world: &[Point2],
    fallback_width_right: &[f64],
    fallback_width_left: &[f64],
    progress: &[f64],
    station_frame_left: &[Point2],
    station_frame_right: &[Point2],
    station_frame_left_route_progress: &[f64],
    station_frame_right_route_progress: &[f64],
    station_frame_progress: &[f64],
    zero_station_normal_fix: bool,
    max_angle_deg: f64,
    angle_step_deg: f64,
    passes: usize,
    control: StationGenerationControl<'_>,
) -> StationBuildResult<(BuiltSections, JsonObject)> {
    control.checkpoint()?;
    let allow_wide_progress_search = !closed_polyline_self_intersects(raw_left_world)
        && !closed_polyline_self_intersects(raw_right_world);
    let mut centers = closed_interp_points(centerline, progress);
    smooth_area_centerline_samples(&mut centers, 2, 0.5);
    let frame_left = if !allow_wide_progress_search && station_frame_left.len() == progress.len() {
        station_frame_left.to_vec()
    } else {
        periodic_interp_points(station_frame_progress, station_frame_left, progress)
    };
    let frame_right = if !allow_wide_progress_search && station_frame_right.len() == progress.len()
    {
        station_frame_right.to_vec()
    } else {
        periodic_interp_points(station_frame_progress, station_frame_right, progress)
    };
    let left_route_progress = periodic_interp_route_progress(
        station_frame_progress,
        station_frame_left_route_progress,
        progress,
    );
    let right_route_progress = periodic_interp_route_progress(
        station_frame_progress,
        station_frame_right_route_progress,
        progress,
    );
    let (route_left, route_right) = route_sampled_closed_boundary_pair_by_midpoint_progress(
        raw_left_route_world,
        raw_right_route_world,
        progress,
    );
    let mut baseline_normals = frame_left
        .iter()
        .zip(&frame_right)
        .map(|(left, right)| normalize(point_sub(*right, *left), [1.0, 0.0]))
        .collect::<Vec<_>>();
    let (zero_fix_count, zero_fixed_normal) = apply_progress_zero_station_normal_fix(
        centerline,
        progress,
        &mut baseline_normals,
        zero_station_normal_fix,
    );
    let width_right = closed_interp_scalar(fallback_width_right, centerline, progress);
    let width_left = closed_interp_scalar(fallback_width_left, centerline, progress);
    let left_projection = ClosedPolylineProjection::new(raw_left_world);
    let right_projection = ClosedPolylineProjection::new(raw_right_world);

    let mut left = Vec::with_capacity(centers.len());
    let mut right = Vec::with_capacity(centers.len());
    let mut normals = Vec::with_capacity(centers.len());
    let mut initial_miss_by_station = Vec::with_capacity(centers.len());
    let mut left_projection_s = Vec::with_capacity(centers.len());
    let mut right_projection_s = Vec::with_capacity(centers.len());
    let mut miss_count = 0_i64;
    for index in 0..centers.len() {
        if index % 32 == 0 {
            control.checkpoint()?;
        }
        let fallback_left = point_sub(
            centers[index],
            point_scale(baseline_normals[index], width_left[index]),
        );
        let fallback_right = point_add(
            centers[index],
            point_scale(baseline_normals[index], width_right[index]),
        );
        let section = normal_line_section_at_progress(
            centers[index],
            baseline_normals[index],
            raw_left_world,
            raw_right_world,
            &left_projection,
            &right_projection,
            fallback_left,
            fallback_right,
            left_route_progress[index],
            right_route_progress[index],
            allow_wide_progress_search,
        );
        left.push(section.left);
        right.push(section.right);
        normals.push(section.normal);
        left_projection_s.push(
            section
                .left_s_m
                .unwrap_or_else(|| left_projection.project_arclength(section.left)),
        );
        right_projection_s.push(
            section
                .right_s_m
                .unwrap_or_else(|| right_projection.project_arclength(section.right)),
        );
        initial_miss_by_station.push(section.miss);
        miss_count += i64::from(section.miss);
    }

    let initial_centers = centers.clone();
    let initial_left = left.clone();
    let initial_right = right.clone();
    let initial_normals = normals.clone();
    let initial_left_projection_s = left_projection_s.clone();
    let initial_right_projection_s = right_projection_s.clone();

    let widths = right
        .iter()
        .zip(&left)
        .map(|(r, l)| distance(*r, *l))
        .collect::<Vec<_>>();
    let median_width = median(widths);
    let left_endpoint_spacing_threshold_m =
        endpoint_projection_spacing_threshold_m(&left_projection_s, left_projection.total);
    let right_endpoint_spacing_threshold_m =
        endpoint_projection_spacing_threshold_m(&right_projection_s, right_projection.total);
    let chord_perp_limit_m = AREA_REPAIR_CHORD_PERP_EPS_M;
    let angle_step_deg = angle_step_deg.max(1.0);
    let mut angle_values = vec![0.0];
    let mut angle = angle_step_deg;
    while angle <= max_angle_deg + 0.5 * angle_step_deg {
        angle_values.push(angle);
        angle_values.push(-angle);
        angle += angle_step_deg;
    }

    let mut selected_angle_deg = vec![0.0; centers.len()];
    let mut selected_miss = initial_miss_by_station.clone();
    let mut local_rejected_miss_count = 0_i64;
    let mut local_rejected_off_chord_count = 0_i64;
    let mut local_rejected_crossing_count = 0_i64;
    let mut local_reverted_off_chord_count = 0_i64;
    let mut selected_score_breakdown = (0..centers.len())
        .map(|index| {
            station_candidate_score_breakdown(
                index,
                left[index],
                right[index],
                normals[index],
                Some(left_projection_s[index]),
                Some(right_projection_s[index]),
                &left,
                &right,
                &normals,
                selected_angle_deg[index],
                selected_miss[index],
                median_width,
                &left_projection,
                &right_projection,
                &left_projection_s,
                &right_projection_s,
                left_endpoint_spacing_threshold_m,
                right_endpoint_spacing_threshold_m,
            )
        })
        .collect::<Vec<_>>();
    let mut changed_total = 0_i64;
    for _ in 0..passes {
        control.checkpoint_phase("closed_refinement_pass")?;
        let mut changed = false;
        for index in 0..centers.len() {
            if index % 16 == 0 {
                control.checkpoint()?;
            }
            let current_breakdown = station_candidate_score_breakdown(
                index,
                left[index],
                right[index],
                normals[index],
                Some(left_projection_s[index]),
                Some(right_projection_s[index]),
                &left,
                &right,
                &normals,
                selected_angle_deg[index],
                selected_miss[index],
                median_width,
                &left_projection,
                &right_projection,
                &left_projection_s,
                &right_projection_s,
                left_endpoint_spacing_threshold_m,
                right_endpoint_spacing_threshold_m,
            );
            let current_score = current_breakdown.total;
            let mut best_left = left[index];
            let mut best_right = right[index];
            let mut best_normal = normals[index];
            let mut best_angle = selected_angle_deg[index];
            let mut best_miss = selected_miss[index];
            let mut best_breakdown = current_breakdown;
            let mut best_score = current_score;
            let mut best_left_s = left_projection_s[index];
            let mut best_right_s = right_projection_s[index];

            for angle_deg in &angle_values {
                let candidate_normal =
                    rotate_vector(baseline_normals[index], angle_deg.to_radians());
                let fallback_left = point_sub(
                    centers[index],
                    point_scale(candidate_normal, width_left[index]),
                );
                let fallback_right = point_add(
                    centers[index],
                    point_scale(candidate_normal, width_right[index]),
                );
                let section = normal_line_section_at_progress(
                    centers[index],
                    candidate_normal,
                    raw_left_world,
                    raw_right_world,
                    &left_projection,
                    &right_projection,
                    fallback_left,
                    fallback_right,
                    left_route_progress[index],
                    right_route_progress[index],
                    allow_wide_progress_search,
                );
                if selected_miss[index] == 0 && section.miss != 0 {
                    local_rejected_miss_count += 1;
                    continue;
                }
                if center_to_chord_perp_error_m(centers[index], section.left, section.right)
                    > chord_perp_limit_m
                {
                    local_rejected_off_chord_count += 1;
                    continue;
                }
                let candidate_left_s = section
                    .left_s_m
                    .unwrap_or_else(|| left_projection.project_arclength(section.left));
                let candidate_right_s = section
                    .right_s_m
                    .unwrap_or_else(|| right_projection.project_arclength(section.right));
                let breakdown = station_candidate_score_breakdown(
                    index,
                    section.left,
                    section.right,
                    section.normal,
                    Some(candidate_left_s),
                    Some(candidate_right_s),
                    &left,
                    &right,
                    &normals,
                    *angle_deg,
                    section.miss,
                    median_width,
                    &left_projection,
                    &right_projection,
                    &left_projection_s,
                    &right_projection_s,
                    left_endpoint_spacing_threshold_m,
                    right_endpoint_spacing_threshold_m,
                );
                if breakdown.crossings > current_breakdown.crossings {
                    local_rejected_crossing_count += 1;
                    continue;
                }
                let score = breakdown.total;
                if score + 1e-9 < best_score {
                    best_left = section.left;
                    best_right = section.right;
                    best_normal = section.normal;
                    best_angle = *angle_deg;
                    best_miss = section.miss;
                    best_breakdown = breakdown;
                    best_score = score;
                    best_left_s = candidate_left_s;
                    best_right_s = candidate_right_s;
                }
            }

            if best_score + 1e-9 < current_score {
                left[index] = best_left;
                right[index] = best_right;
                left_projection_s[index] = best_left_s;
                right_projection_s[index] = best_right_s;
                normals[index] = best_normal;
                selected_angle_deg[index] = best_angle;
                selected_miss[index] = best_miss;
                selected_score_breakdown[index] = best_breakdown;
                changed = true;
                changed_total += 1;
            }
        }
        if !changed {
            break;
        }
    }

    for index in 0..centers.len() {
        if index % 32 == 0 {
            control.checkpoint()?;
        }
        let current_perp = center_to_chord_perp_error_m(centers[index], left[index], right[index]);
        let initial_perp = center_to_chord_perp_error_m(
            initial_centers[index],
            initial_left[index],
            initial_right[index],
        );
        if current_perp <= chord_perp_limit_m
            || initial_perp > chord_perp_limit_m
            || initial_miss_by_station[index] != 0
        {
            continue;
        }
        left[index] = initial_left[index];
        right[index] = initial_right[index];
        normals[index] = initial_normals[index];
        left_projection_s[index] = initial_left_projection_s[index];
        right_projection_s[index] = initial_right_projection_s[index];
        selected_angle_deg[index] = 0.0;
        selected_miss[index] = initial_miss_by_station[index];
        local_reverted_off_chord_count += 1;
    }

    selected_score_breakdown = (0..centers.len())
        .map(|index| {
            station_candidate_score_breakdown(
                index,
                left[index],
                right[index],
                normals[index],
                Some(left_projection_s[index]),
                Some(right_projection_s[index]),
                &left,
                &right,
                &normals,
                selected_angle_deg[index],
                selected_miss[index],
                median_width,
                &left_projection,
                &right_projection,
                &left_projection_s,
                &right_projection_s,
                left_endpoint_spacing_threshold_m,
                right_endpoint_spacing_threshold_m,
            )
        })
        .collect();
    let post_local_centers = centers.clone();
    let post_local_left = left.clone();
    let post_local_right = right.clone();
    let post_local_normals = normals.clone();
    let post_local_left_projection_s = left_projection_s.clone();
    let post_local_right_projection_s = right_projection_s.clone();
    let post_local_lr_projection_ratios = lr_projection_interval_ratios(
        &post_local_left_projection_s,
        &post_local_right_projection_s,
        left_projection.total,
        right_projection.total,
    );
    let (lr_repair_replaced_count, lr_repair_replaced, lr_repair_rejected_topology_count) =
        repair_lr_projection_mismatch_with_frame(
            &mut centers,
            &mut left,
            &mut right,
            &mut normals,
            &mut left_projection_s,
            &mut right_projection_s,
            &left_projection,
            &right_projection,
            &frame_left,
            &frame_right,
            progress,
            AREA_REPAIR_LR_PROJECTION_RATIO_HARD_LIMIT,
            control,
        )?;
    let mut frame_projection_replaced_count = 0_i64;
    for _ in 0..4 {
        control.checkpoint()?;
        let replaced = repair_projection_topology_from_prepared_frame(
            &mut left,
            &mut right,
            &frame_left,
            &frame_right,
            control,
        )?;
        if replaced == 0 {
            break;
        }
        frame_projection_replaced_count += replaced;
    }
    if frame_projection_replaced_count > 0 {
        for index in 0..centers.len() {
            centers[index] = midpoint(left[index], right[index]);
            normals[index] = normalize(point_sub(right[index], left[index]), normals[index]);
            left_projection_s[index] = left_projection.project_arclength(left[index]);
            right_projection_s[index] = right_projection.project_arclength(right[index]);
        }
    }
    let final_fact_topology_replaced_count = repair_closed_fact_topology_with_route_pair(
        &mut left,
        &mut right,
        &route_left,
        &route_right,
        raw_left_route_world,
        raw_right_route_world,
        control,
    )?;
    let final_fact_width_replaced_count = stabilize_closed_fact_width_with_route_pair(
        &mut left,
        &mut right,
        &route_left,
        &route_right,
        raw_left_route_world,
        raw_right_route_world,
        control,
    )?;
    if final_fact_topology_replaced_count > 0 || final_fact_width_replaced_count > 0 {
        for index in 0..centers.len() {
            centers[index] = midpoint(left[index], right[index]);
            normals[index] = normalize(point_sub(right[index], left[index]), normals[index]);
            left_projection_s[index] = left_projection.project_arclength(left[index]);
            right_projection_s[index] = right_projection.project_arclength(right[index]);
        }
    }
    let final_lr_projection_ratios = lr_projection_interval_ratios(
        &left_projection_s,
        &right_projection_s,
        left_projection.total,
        right_projection.total,
    );
    let chord_lengths = right
        .iter()
        .zip(&left)
        .map(|(r, l)| distance(*r, *l))
        .collect::<Vec<_>>();
    let fractions = centers
        .iter()
        .zip(&left)
        .zip(&right)
        .zip(&chord_lengths)
        .map(|(((center, l), r), length)| {
            let chord = point_sub(*r, *l);
            (dot(point_sub(*center, *l), chord) / length.powi(2).max(1e-9)).clamp(0.0, 1.0)
        })
        .collect::<Vec<_>>();
    let width_left = chord_lengths
        .iter()
        .zip(&fractions)
        .map(|(length, fraction)| length * fraction)
        .collect::<Vec<_>>();
    let width_right = chord_lengths
        .iter()
        .zip(&width_left)
        .map(|(length, left_width)| length - left_width)
        .collect::<Vec<_>>();

    let final_horizon_crossings = station_horizon_crossing_count(&left, &right, 2);
    let final_all_crossings = station_crossing_count_all(&left, &right);
    let left_endpoint_spacing = endpoint_projection_spacing_stats_from_arclengths(
        &left_projection_s,
        left_projection.total,
    );
    let right_endpoint_spacing = endpoint_projection_spacing_stats_from_arclengths(
        &right_projection_s,
        right_projection.total,
    );
    let lr_projection_ratios = lr_projection_interval_ratios(
        &left_projection_s,
        &right_projection_s,
        left_projection.total,
        right_projection.total,
    );
    let abs_angles = selected_angle_deg
        .iter()
        .map(|value| value.abs())
        .collect::<Vec<_>>();
    let mut meta = vec![
        (
            "area_preserving_repair_changed_count".to_owned(),
            JsonValue::Integer(changed_total),
        ),
        (
            "area_preserving_repair_angle_abs_max_deg".to_owned(),
            abs_angles.iter().copied().fold(0.0, f64::max).into(),
        ),
        (
            "area_preserving_repair_angle_abs_p95_deg".to_owned(),
            percentile(abs_angles, 95.0).into(),
        ),
        (
            "area_preserving_repair_all_crossing_count".to_owned(),
            JsonValue::Integer(final_all_crossings),
        ),
        (
            "area_preserving_repair_horizon2_crossing_count".to_owned(),
            JsonValue::Integer(final_horizon_crossings),
        ),
        (
            "area_preserving_repair_initial_miss_count".to_owned(),
            JsonValue::Integer(miss_count),
        ),
        (
            "area_repair_final_fact_topology_replaced_count".to_owned(),
            JsonValue::Integer(final_fact_topology_replaced_count),
        ),
        (
            "area_repair_final_fact_width_replaced_count".to_owned(),
            JsonValue::Integer(final_fact_width_replaced_count),
        ),
        (
            "area_repair_left_endpoint_projection_spacing_min_m".to_owned(),
            left_endpoint_spacing.min.into(),
        ),
        (
            "area_repair_left_endpoint_projection_spacing_p05_m".to_owned(),
            left_endpoint_spacing.p05.into(),
        ),
        (
            "area_repair_left_endpoint_projection_spacing_median_m".to_owned(),
            left_endpoint_spacing.median.into(),
        ),
        (
            "area_repair_right_endpoint_projection_spacing_min_m".to_owned(),
            right_endpoint_spacing.min.into(),
        ),
        (
            "area_repair_right_endpoint_projection_spacing_p05_m".to_owned(),
            right_endpoint_spacing.p05.into(),
        ),
        (
            "area_repair_right_endpoint_projection_spacing_median_m".to_owned(),
            right_endpoint_spacing.median.into(),
        ),
        (
            "area_repair_lr_projection_ratio_max".to_owned(),
            lr_projection_ratios
                .iter()
                .copied()
                .fold(0.0, f64::max)
                .into(),
        ),
        (
            "area_repair_lr_projection_ratio_p95".to_owned(),
            percentile(lr_projection_ratios.clone(), 95.0).into(),
        ),
        (
            "area_repair_lr_projection_ratio_violation_count".to_owned(),
            JsonValue::Integer(
                lr_projection_ratios
                    .iter()
                    .filter(|ratio| **ratio > AREA_REPAIR_LR_PROJECTION_RATIO_HARD_LIMIT)
                    .count() as i64,
            ),
        ),
        (
            "area_repair_lr_projection_fallback_replaced_count".to_owned(),
            JsonValue::Integer(lr_repair_replaced_count),
        ),
        (
            "area_repair_lr_projection_rejected_topology_count".to_owned(),
            JsonValue::Integer(lr_repair_rejected_topology_count),
        ),
        (
            "area_repair_prepared_frame_topology_replaced_count".to_owned(),
            JsonValue::Integer(frame_projection_replaced_count),
        ),
        (
            "area_repair_centerline_smooth_passes".to_owned(),
            JsonValue::Integer(2),
        ),
        (
            "area_repair_centerline_smooth_alpha".to_owned(),
            0.5_f64.into(),
        ),
        (
            "area_repair_chord_perp_epsilon_m".to_owned(),
            AREA_REPAIR_CHORD_PERP_EPS_M.into(),
        ),
        (
            "area_repair_chord_perp_limit_m".to_owned(),
            chord_perp_limit_m.into(),
        ),
        (
            "area_repair_local_rejected_miss_count".to_owned(),
            JsonValue::Integer(local_rejected_miss_count),
        ),
        (
            "area_repair_local_rejected_off_chord_count".to_owned(),
            JsonValue::Integer(local_rejected_off_chord_count),
        ),
        (
            "area_repair_local_rejected_crossing_count".to_owned(),
            JsonValue::Integer(local_rejected_crossing_count),
        ),
        (
            "area_repair_local_reverted_off_chord_count".to_owned(),
            JsonValue::Integer(local_reverted_off_chord_count),
        ),
        (
            "zero_station_normal_fix_applied_count".to_owned(),
            JsonValue::Integer(zero_fix_count),
        ),
        (
            "area_repair_station_trace".to_owned(),
            area_repair_station_trace_json(
                progress,
                &initial_centers,
                &initial_left,
                &initial_right,
                &initial_normals,
                &post_local_centers,
                &post_local_left,
                &post_local_right,
                &post_local_normals,
                &centers,
                &left,
                &right,
                &normals,
                &initial_miss_by_station,
                &selected_miss,
                &selected_angle_deg,
                &selected_score_breakdown,
                &lr_repair_replaced,
                &post_local_left_projection_s,
                &post_local_right_projection_s,
                &left_projection_s,
                &right_projection_s,
                &post_local_lr_projection_ratios,
                &final_lr_projection_ratios,
            ),
        ),
    ];
    if let Some(normal) = zero_fixed_normal {
        meta.push(("zero_station_fixed_normal_x".to_owned(), normal[0].into()));
        meta.push(("zero_station_fixed_normal_y".to_owned(), normal[1].into()));
    }

    Ok((
        BuiltSections {
            left,
            right,
            center: centers,
            normals,
            width_right,
            width_left,
            miss_count,
        },
        meta,
    ))
}

fn area_repair_station_trace_json(
    progress: &[f64],
    initial_centers: &[Point2],
    initial_left: &[Point2],
    initial_right: &[Point2],
    initial_normals: &[Point2],
    post_local_centers: &[Point2],
    post_local_left: &[Point2],
    post_local_right: &[Point2],
    post_local_normals: &[Point2],
    final_centers: &[Point2],
    final_left: &[Point2],
    final_right: &[Point2],
    final_normals: &[Point2],
    initial_miss_by_station: &[i32],
    selected_miss: &[i32],
    selected_angle_deg: &[f64],
    selected_score_breakdown: &[StationCandidateScoreBreakdown],
    lr_repair_replaced: &[bool],
    post_local_left_projection_s: &[f64],
    post_local_right_projection_s: &[f64],
    final_left_projection_s: &[f64],
    final_right_projection_s: &[f64],
    post_local_lr_projection_ratios: &[f64],
    final_lr_projection_ratios: &[f64],
) -> JsonValue {
    let count = final_centers
        .len()
        .min(final_left.len())
        .min(final_right.len())
        .min(selected_score_breakdown.len());
    JsonValue::Array(
        (0..count)
            .map(|index| {
                let previous = if count == 0 {
                    0
                } else {
                    (index + count - 1) % count
                };
                let score = selected_score_breakdown[index];
                JsonValue::Object(vec![
                    ("index".to_owned(), JsonValue::Integer(index as i64)),
                    (
                        "progress".to_owned(),
                        finite_json(progress.get(index).copied().unwrap_or(0.0)),
                    ),
                    (
                        "initial_miss".to_owned(),
                        JsonValue::Integer(i64::from(
                            initial_miss_by_station.get(index).copied().unwrap_or(0),
                        )),
                    ),
                    (
                        "selected_miss".to_owned(),
                        JsonValue::Integer(i64::from(
                            selected_miss.get(index).copied().unwrap_or(0),
                        )),
                    ),
                    (
                        "selected_angle_deg".to_owned(),
                        finite_json(selected_angle_deg.get(index).copied().unwrap_or(0.0)),
                    ),
                    (
                        "lr_projection_replaced".to_owned(),
                        JsonValue::Bool(lr_repair_replaced.get(index).copied().unwrap_or(false)),
                    ),
                    (
                        "score".to_owned(),
                        station_candidate_score_breakdown_json(score),
                    ),
                    (
                        "initial".to_owned(),
                        station_trace_geometry_json(
                            initial_centers,
                            initial_left,
                            initial_right,
                            initial_normals,
                            index,
                        ),
                    ),
                    (
                        "post_local".to_owned(),
                        station_trace_geometry_json(
                            post_local_centers,
                            post_local_left,
                            post_local_right,
                            post_local_normals,
                            index,
                        ),
                    ),
                    (
                        "final".to_owned(),
                        station_trace_geometry_json(
                            final_centers,
                            final_left,
                            final_right,
                            final_normals,
                            index,
                        ),
                    ),
                    (
                        "post_local_left_projection_s".to_owned(),
                        finite_json(
                            post_local_left_projection_s
                                .get(index)
                                .copied()
                                .unwrap_or(0.0),
                        ),
                    ),
                    (
                        "post_local_right_projection_s".to_owned(),
                        finite_json(
                            post_local_right_projection_s
                                .get(index)
                                .copied()
                                .unwrap_or(0.0),
                        ),
                    ),
                    (
                        "final_left_projection_s".to_owned(),
                        finite_json(final_left_projection_s.get(index).copied().unwrap_or(0.0)),
                    ),
                    (
                        "final_right_projection_s".to_owned(),
                        finite_json(final_right_projection_s.get(index).copied().unwrap_or(0.0)),
                    ),
                    (
                        "post_local_lr_ratio_prev".to_owned(),
                        finite_json(
                            post_local_lr_projection_ratios
                                .get(previous)
                                .copied()
                                .unwrap_or(0.0),
                        ),
                    ),
                    (
                        "post_local_lr_ratio_next".to_owned(),
                        finite_json(
                            post_local_lr_projection_ratios
                                .get(index)
                                .copied()
                                .unwrap_or(0.0),
                        ),
                    ),
                    (
                        "final_lr_ratio_prev".to_owned(),
                        finite_json(
                            final_lr_projection_ratios
                                .get(previous)
                                .copied()
                                .unwrap_or(0.0),
                        ),
                    ),
                    (
                        "final_lr_ratio_next".to_owned(),
                        finite_json(
                            final_lr_projection_ratios
                                .get(index)
                                .copied()
                                .unwrap_or(0.0),
                        ),
                    ),
                ])
            })
            .collect(),
    )
}

fn station_trace_geometry_json(
    centers: &[Point2],
    left: &[Point2],
    right: &[Point2],
    normals: &[Point2],
    index: usize,
) -> JsonValue {
    let center = centers.get(index).copied().unwrap_or([0.0, 0.0]);
    let left_point = left.get(index).copied().unwrap_or([0.0, 0.0]);
    let right_point = right.get(index).copied().unwrap_or([0.0, 0.0]);
    let normal = normals.get(index).copied().unwrap_or([1.0, 0.0]);
    JsonValue::Object(vec![
        ("center_xy_m".to_owned(), point_json(center)),
        ("left_xy_m".to_owned(), point_json(left_point)),
        ("right_xy_m".to_owned(), point_json(right_point)),
        ("normal_xy".to_owned(), point_json(normal)),
        (
            "width_m".to_owned(),
            finite_json(distance(left_point, right_point)),
        ),
        (
            "center_to_chord_perp_error_m".to_owned(),
            finite_json(center_to_chord_perp_error_m(
                center,
                left_point,
                right_point,
            )),
        ),
        (
            "center_chord_fraction".to_owned(),
            finite_json(center_chord_fraction(center, left_point, right_point)),
        ),
    ])
}

fn station_candidate_score_breakdown_json(score: StationCandidateScoreBreakdown) -> JsonValue {
    JsonValue::Object(vec![
        ("total".to_owned(), finite_json(score.total)),
        ("crossings".to_owned(), JsonValue::Integer(score.crossings)),
        (
            "boundary_crossings".to_owned(),
            JsonValue::Integer(score.boundary_crossings),
        ),
        ("miss".to_owned(), JsonValue::Integer(i64::from(score.miss))),
        (
            "max_rotation_deg".to_owned(),
            finite_json(score.max_rotation_deg),
        ),
        (
            "rotation_violation_deg".to_owned(),
            finite_json(score.rotation_violation_deg),
        ),
        ("width_m".to_owned(), finite_json(score.width_m)),
        (
            "width_violation_m".to_owned(),
            finite_json(score.width_violation_m),
        ),
        (
            "endpoint_spacing_violation_m".to_owned(),
            finite_json(score.endpoint_spacing_violation_m),
        ),
        (
            "lr_projection_ratio".to_owned(),
            finite_json(score.lr_projection_ratio),
        ),
        (
            "lr_projection_soft_violation".to_owned(),
            finite_json(score.lr_projection_soft_violation),
        ),
        (
            "lr_projection_hard_violation".to_owned(),
            finite_json(score.lr_projection_hard_violation),
        ),
        ("angle_deg".to_owned(), finite_json(score.angle_deg)),
    ])
}

fn point_json(point: Point2) -> JsonValue {
    JsonValue::Array(vec![finite_json(point[0]), finite_json(point[1])])
}

fn finite_json(value: f64) -> JsonValue {
    if value.is_finite() {
        value.into()
    } else {
        JsonValue::Null
    }
}

fn center_to_chord_perp_error_m(center: Point2, left: Point2, right: Point2) -> f64 {
    let chord = point_sub(right, left);
    let length = hypot(chord);
    if length <= 1e-12 {
        return distance(center, left);
    }
    cross(chord, point_sub(center, left)).abs() / length
}

fn center_chord_fraction(center: Point2, left: Point2, right: Point2) -> f64 {
    let chord = point_sub(right, left);
    let length_sq = dot(chord, chord);
    if length_sq <= 1e-12 {
        return 0.0;
    }
    dot(point_sub(center, left), chord) / length_sq
}

fn station_candidate_score_breakdown(
    index: usize,
    candidate_left: Point2,
    candidate_right: Point2,
    candidate_normal: Point2,
    candidate_left_s_m: Option<f64>,
    candidate_right_s_m: Option<f64>,
    left: &[Point2],
    right: &[Point2],
    normals: &[Point2],
    angle_deg: f64,
    miss: i32,
    median_width: f64,
    raw_left_projection: &ClosedPolylineProjection<'_>,
    raw_right_projection: &ClosedPolylineProjection<'_>,
    left_projection_s: &[f64],
    right_projection_s: &[f64],
    left_endpoint_spacing_threshold_m: f64,
    right_endpoint_spacing_threshold_m: f64,
) -> StationCandidateScoreBreakdown {
    let n = left.len();
    let previous = (index + n - 1) % n;
    let next = (index + 1) % n;
    let candidate_normal = normalize(candidate_normal, [1.0, 0.0]);
    let previous_dot = dot(normals[previous], candidate_normal).clamp(-1.0, 1.0);
    let next_dot = dot(candidate_normal, normals[next]).clamp(-1.0, 1.0);
    let max_rotation_deg = previous_dot
        .acos()
        .to_degrees()
        .max(next_dot.acos().to_degrees());
    let rotation_violation = (max_rotation_deg - 60.0).max(0.0);
    let width = distance(candidate_right, candidate_left);
    let min_width = 0.20_f64.max(0.20 * median_width);
    let max_width = (min_width * 2.0).max(3.0 * median_width);
    let width_violation = (min_width - width).max(0.0) + (width - max_width).max(0.0);
    let crossings =
        candidate_station_crossings(index, candidate_left, candidate_right, left, right);
    let boundary_crossings = candidate_closed_boundary_crossings(index, candidate_left, left)
        + candidate_closed_boundary_crossings(index, candidate_right, right);
    let candidate_left_s_m =
        candidate_left_s_m.unwrap_or_else(|| raw_left_projection.project_arclength(candidate_left));
    let candidate_right_s_m = candidate_right_s_m
        .unwrap_or_else(|| raw_right_projection.project_arclength(candidate_right));
    let endpoint_spacing_violation = endpoint_projection_spacing_violation_m(
        index,
        candidate_left_s_m,
        left,
        left_projection_s,
        raw_left_projection.total,
        left_endpoint_spacing_threshold_m,
    ) + endpoint_projection_spacing_violation_m(
        index,
        candidate_right_s_m,
        right,
        right_projection_s,
        raw_right_projection.total,
        right_endpoint_spacing_threshold_m,
    );
    let lr_projection_ratio = candidate_lr_projection_gap_ratio(
        index,
        candidate_left_s_m,
        candidate_right_s_m,
        left_projection_s,
        right_projection_s,
        raw_left_projection.total,
        raw_right_projection.total,
    );
    let lr_projection_soft_violation =
        (lr_projection_ratio - AREA_REPAIR_LR_PROJECTION_RATIO_SOFT_LIMIT).max(0.0);
    let lr_projection_hard_violation =
        (lr_projection_ratio - AREA_REPAIR_LR_PROJECTION_RATIO_HARD_LIMIT).max(0.0);
    let total = 1_000_000_000.0 * crossings as f64
        + 500_000_000.0 * boundary_crossings as f64
        + 10_000_000.0 * f64::from(miss)
        + 100_000_000.0 * lr_projection_hard_violation.powi(2)
        + 2_000_000.0 * lr_projection_soft_violation.powi(2)
        + 100_000.0 * rotation_violation.powi(2)
        + 100_000.0 * endpoint_spacing_violation.powi(2)
        + 1_000.0 * width_violation.powi(2)
        + 0.25 * angle_deg.powi(2);

    StationCandidateScoreBreakdown {
        total,
        crossings,
        boundary_crossings,
        miss,
        max_rotation_deg,
        rotation_violation_deg: rotation_violation,
        width_m: width,
        width_violation_m: width_violation,
        endpoint_spacing_violation_m: endpoint_spacing_violation,
        lr_projection_ratio,
        lr_projection_soft_violation,
        lr_projection_hard_violation,
        angle_deg,
    }
}

#[derive(Debug, Clone, Copy)]
struct EndpointProjectionSpacingStats {
    min: f64,
    p05: f64,
    median: f64,
}

struct ClosedPolylineProjection<'a> {
    points: &'a [Point2],
    segment_lengths: Vec<f64>,
    total: f64,
}

impl<'a> ClosedPolylineProjection<'a> {
    fn new(points: &'a [Point2]) -> Self {
        let (_, segment_lengths) = closed_polyline_arclength(points);
        let total = segment_lengths.iter().sum::<f64>().max(1e-9);
        Self {
            points,
            segment_lengths,
            total,
        }
    }

    fn project_arclength(&self, point: Point2) -> f64 {
        if self.points.is_empty() {
            return 0.0;
        }
        let mut best_dist2 = f64::INFINITY;
        let mut best_s = 0.0;
        let mut segment_start_s = 0.0;
        for index in 0..self.points.len() {
            let a = self.points[index];
            let b = self.points[(index + 1) % self.points.len()];
            let ab = point_sub(b, a);
            let denom = dot(ab, ab).max(1e-12);
            let t = (dot(point_sub(point, a), ab) / denom).clamp(0.0, 1.0);
            let projection = point_add(a, point_scale(ab, t));
            let delta = point_sub(point, projection);
            let dist2 = dot(delta, delta);
            if dist2 < best_dist2 {
                best_dist2 = dist2;
                best_s = segment_start_s + t * self.segment_lengths[index];
            }
            segment_start_s += self.segment_lengths[index];
        }
        best_s.rem_euclid(self.total)
    }

    fn candidate_arclength(&self, candidate: LineIntersectionCandidate) -> f64 {
        if self.segment_lengths.is_empty() {
            return 0.0;
        }
        let segment_index = candidate.segment_index % self.segment_lengths.len();
        let segment_start_s = self.segment_lengths.iter().take(segment_index).sum::<f64>();
        (segment_start_s
            + candidate.segment_u.clamp(0.0, 1.0) * self.segment_lengths[segment_index])
            .rem_euclid(self.total)
    }
}

struct OpenPolylineProjection<'a> {
    points: &'a [Point2],
    segment_lengths: Vec<f64>,
}

impl<'a> OpenPolylineProjection<'a> {
    fn new(points: &'a [Point2]) -> Self {
        let (_, segment_lengths) = open_polyline_arclength(points);
        Self {
            points,
            segment_lengths,
        }
    }

    fn project_arclength(&self, point: Point2) -> f64 {
        if self.points.len() < 2 {
            return 0.0;
        }
        let mut best_dist2 = f64::INFINITY;
        let mut best_s = 0.0;
        let mut segment_start_s = 0.0;
        for index in 0..(self.points.len() - 1) {
            let a = self.points[index];
            let b = self.points[index + 1];
            let ab = point_sub(b, a);
            let denom = dot(ab, ab).max(1e-12);
            let t = (dot(point_sub(point, a), ab) / denom).clamp(0.0, 1.0);
            let projection = point_add(a, point_scale(ab, t));
            let delta = point_sub(point, projection);
            let dist2 = dot(delta, delta);
            if dist2 < best_dist2 {
                best_dist2 = dist2;
                best_s = segment_start_s + t * self.segment_lengths[index];
            }
            segment_start_s += self.segment_lengths[index];
        }
        best_s
    }
}

fn endpoint_projection_spacing_threshold_m(
    endpoint_projection_s: &[f64],
    raw_boundary_total_m: f64,
) -> f64 {
    let stats = endpoint_projection_spacing_stats_from_arclengths(
        endpoint_projection_s,
        raw_boundary_total_m,
    );
    (0.20 * stats.median).max(0.75)
}

fn endpoint_projection_spacing_stats_from_arclengths(
    endpoint_projection_s: &[f64],
    raw_boundary_total_m: f64,
) -> EndpointProjectionSpacingStats {
    if endpoint_projection_s.len() < 2 {
        return EndpointProjectionSpacingStats {
            min: 0.0,
            p05: 0.0,
            median: 0.0,
        };
    }
    let mut spacing = (0..endpoint_projection_s.len())
        .map(|index| {
            cyclic_forward_distance_m(
                endpoint_projection_s[index],
                endpoint_projection_s[(index + 1) % endpoint_projection_s.len()],
                raw_boundary_total_m,
            )
        })
        .collect::<Vec<_>>();
    spacing.sort_by(|left, right| left.partial_cmp(right).unwrap_or(std::cmp::Ordering::Equal));
    EndpointProjectionSpacingStats {
        min: spacing.first().copied().unwrap_or(0.0),
        p05: percentile(spacing.clone(), 5.0),
        median: median(spacing),
    }
}

fn open_endpoint_projection_spacing_stats_from_arclengths(
    endpoint_projection_s: &[f64],
) -> EndpointProjectionSpacingStats {
    if endpoint_projection_s.len() < 2 {
        return EndpointProjectionSpacingStats {
            min: 0.0,
            p05: 0.0,
            median: 0.0,
        };
    }
    let mut spacing = endpoint_projection_s
        .windows(2)
        .map(|pair| (pair[1] - pair[0]).abs())
        .collect::<Vec<_>>();
    spacing.sort_by(f64::total_cmp);
    EndpointProjectionSpacingStats {
        min: spacing.first().copied().unwrap_or(0.0),
        p05: percentile(spacing.clone(), 5.0),
        median: median(spacing),
    }
}

fn endpoint_projection_spacing_violation_m(
    index: usize,
    candidate_s: f64,
    endpoints: &[Point2],
    endpoint_projection_s: &[f64],
    raw_boundary_total_m: f64,
    threshold_m: f64,
) -> f64 {
    if endpoints.len() < 3 || threshold_m <= 0.0 {
        return 0.0;
    }
    let previous = (index + endpoints.len() - 1) % endpoints.len();
    let next = (index + 1) % endpoints.len();
    let previous_s = endpoint_projection_s[previous];
    let next_s = endpoint_projection_s[next];
    let previous_gap = cyclic_forward_distance_m(previous_s, candidate_s, raw_boundary_total_m);
    let next_gap = cyclic_forward_distance_m(candidate_s, next_s, raw_boundary_total_m);
    let nominal_gap = raw_boundary_total_m / endpoints.len().max(1) as f64;
    let max_gap = (8.0 * nominal_gap).max(4.0 * threshold_m);
    (threshold_m - previous_gap).max(0.0)
        + (threshold_m - next_gap).max(0.0)
        + (previous_gap - max_gap).max(0.0)
        + (next_gap - max_gap).max(0.0)
}

fn candidate_lr_projection_gap_ratio(
    index: usize,
    left_candidate_s: f64,
    right_candidate_s: f64,
    left_projection_s: &[f64],
    right_projection_s: &[f64],
    left_total_m: f64,
    right_total_m: f64,
) -> f64 {
    if left_projection_s.len() < 3 || right_projection_s.len() != left_projection_s.len() {
        return 1.0;
    }
    let count = left_projection_s.len();
    let previous = (index + count - 1) % count;
    let next = (index + 1) % count;
    let previous_ratio = normalized_lr_gap_ratio(
        cyclic_forward_distance_m(left_projection_s[previous], left_candidate_s, left_total_m),
        cyclic_forward_distance_m(
            right_projection_s[previous],
            right_candidate_s,
            right_total_m,
        ),
        left_total_m,
        right_total_m,
    );
    let next_ratio = normalized_lr_gap_ratio(
        cyclic_forward_distance_m(left_candidate_s, left_projection_s[next], left_total_m),
        cyclic_forward_distance_m(right_candidate_s, right_projection_s[next], right_total_m),
        left_total_m,
        right_total_m,
    );
    previous_ratio.max(next_ratio)
}

fn lr_projection_interval_ratios(
    left_projection_s: &[f64],
    right_projection_s: &[f64],
    left_total_m: f64,
    right_total_m: f64,
) -> Vec<f64> {
    if left_projection_s.len() < 2 || right_projection_s.len() != left_projection_s.len() {
        return Vec::new();
    }
    (0..left_projection_s.len())
        .map(|index| {
            let next = (index + 1) % left_projection_s.len();
            normalized_lr_gap_ratio(
                cyclic_forward_distance_m(
                    left_projection_s[index],
                    left_projection_s[next],
                    left_total_m,
                ),
                cyclic_forward_distance_m(
                    right_projection_s[index],
                    right_projection_s[next],
                    right_total_m,
                ),
                left_total_m,
                right_total_m,
            )
        })
        .collect()
}

fn open_lr_projection_interval_ratios(
    left_projection_s: &[f64],
    right_projection_s: &[f64],
) -> Vec<f64> {
    if left_projection_s.len() < 2 || right_projection_s.len() != left_projection_s.len() {
        return Vec::new();
    }
    left_projection_s
        .windows(2)
        .zip(right_projection_s.windows(2))
        .map(|(left_pair, right_pair)| {
            let dl = (left_pair[1] - left_pair[0]).abs();
            let dr = (right_pair[1] - right_pair[0]).abs();
            dl.max(dr) / dl.min(dr).max(1e-6)
        })
        .collect()
}

fn normalized_lr_gap_ratio(
    left_gap_m: f64,
    right_gap_m: f64,
    left_total_m: f64,
    right_total_m: f64,
) -> f64 {
    let left = (left_gap_m / left_total_m.max(1e-9)).max(1e-12);
    let right = (right_gap_m / right_total_m.max(1e-9)).max(1e-12);
    left.max(right) / left.min(right)
}

fn smooth_area_centerline_samples(points: &mut [Point2], passes: usize, alpha: f64) {
    let count = points.len();
    if count < 4 || passes == 0 || alpha <= 0.0 {
        return;
    }
    let alpha = alpha.clamp(0.0, 1.0);
    let mut scratch = points.to_vec();
    for _ in 0..passes {
        for index in 0..count {
            let previous = points[(index + count - 1) % count];
            let next = points[(index + 1) % count];
            let target = midpoint(previous, next);
            scratch[index] = point_add(
                points[index],
                point_scale(point_sub(target, points[index]), alpha),
            );
        }
        points.copy_from_slice(&scratch);
    }
}

fn repair_lr_projection_mismatch_with_frame(
    centers: &mut [Point2],
    left: &mut [Point2],
    right: &mut [Point2],
    normals: &mut [Point2],
    left_projection_s: &mut [f64],
    right_projection_s: &mut [f64],
    left_projection: &ClosedPolylineProjection<'_>,
    right_projection: &ClosedPolylineProjection<'_>,
    frame_left: &[Point2],
    frame_right: &[Point2],
    progress: &[f64],
    hard_limit: f64,
    control: StationGenerationControl<'_>,
) -> StationBuildResult<(i64, Vec<bool>, i64)> {
    control.checkpoint_phase("closed_lr_projection_repair")?;
    let count = left.len();
    if count < 3
        || right.len() != count
        || normals.len() != count
        || left_projection_s.len() != count
        || right_projection_s.len() != count
        || frame_left.len() != count
        || frame_right.len() != count
        || progress.len() != count
    {
        return Ok((0, Vec::new(), 0));
    }

    let ratios = lr_projection_interval_ratios(
        left_projection_s,
        right_projection_s,
        left_projection.total,
        right_projection.total,
    );
    let mut replace = vec![false; count];
    for (index, ratio) in ratios.iter().copied().enumerate() {
        if ratio > hard_limit {
            replace[index] = true;
            replace[(index + 1) % count] = true;
        }
    }
    let mut replaced_count = 0_i64;
    let mut rejected_topology_count = 0_i64;
    for index in 0..count {
        if index % 8 == 0 {
            control.checkpoint_phase("closed_lr_projection_repair")?;
        }
        if !replace[index] {
            continue;
        }
        let candidate_left = frame_left[index];
        let candidate_right = frame_right[index];
        let current_station_crossings =
            candidate_station_crossings(index, left[index], right[index], left, right);
        let candidate_station_crossings =
            candidate_station_crossings(index, candidate_left, candidate_right, left, right);
        let current_boundary_crossings =
            candidate_closed_boundary_crossings(index, left[index], left)
                + candidate_closed_boundary_crossings(index, right[index], right);
        let candidate_boundary_crossings =
            candidate_closed_boundary_crossings(index, candidate_left, left)
                + candidate_closed_boundary_crossings(index, candidate_right, right);
        if candidate_station_crossings > current_station_crossings
            || candidate_boundary_crossings > current_boundary_crossings
        {
            replace[index] = false;
            rejected_topology_count += 1;
            continue;
        }
        left[index] = candidate_left;
        right[index] = candidate_right;
        centers[index] = midpoint(left[index], right[index]);
        left_projection_s[index] = left_projection.project_arclength(candidate_left);
        right_projection_s[index] = right_projection.project_arclength(candidate_right);
        let chord = point_sub(right[index], left[index]);
        normals[index] = normalize(chord, normals[index]);
        replaced_count += 1;
    }
    Ok((replaced_count, replace, rejected_topology_count))
}

fn cyclic_forward_distance_m(from_s: f64, to_s: f64, total_m: f64) -> f64 {
    (to_s - from_s).rem_euclid(total_m.max(1e-9))
}

fn candidate_station_crossings(
    index: usize,
    candidate_left: Point2,
    candidate_right: Point2,
    left: &[Point2],
    right: &[Point2],
) -> i64 {
    let count = left.len();
    left.iter()
        .zip(right)
        .enumerate()
        .filter(|(other_index, (other_left, other_right))| {
            if *other_index == index {
                return false;
            }
            let forward_delta = index.abs_diff(*other_index);
            let route_delta = forward_delta.min(count.saturating_sub(forward_delta));
            route_delta <= 2
                && segment_intersects(candidate_left, candidate_right, **other_left, **other_right)
        })
        .count() as i64
}

fn candidate_closed_boundary_crossings(
    index: usize,
    candidate_point: Point2,
    boundary: &[Point2],
) -> i64 {
    let count = boundary.len();
    if count < 4 || index >= count {
        return 0;
    }
    let previous = (index + count - 1) % count;
    let next = (index + 1) % count;
    let modified_segments = [
        (boundary[previous], candidate_point),
        (candidate_point, boundary[next]),
    ];

    let mut crossing_count = 0_i64;
    for (a, b) in modified_segments {
        for segment_start in 0..count {
            let segment_end = (segment_start + 1) % count;
            if segment_start == previous
                || segment_start == index
                || segment_start == next
                || segment_end == previous
                || segment_end == index
                || segment_end == next
            {
                continue;
            }
            if segment_intersects_inclusive(a, b, boundary[segment_start], boundary[segment_end]) {
                crossing_count += 1;
            }
        }
    }
    crossing_count
}

fn station_crossing_count_all(left: &[Point2], right: &[Point2]) -> i64 {
    let mut count = 0_i64;
    for i in 0..left.len() {
        for j in (i + 1)..left.len() {
            if segment_intersects(left[i], right[i], left[j], right[j]) {
                count += 1;
            }
        }
    }
    count
}

fn closed_polyline_self_intersects(points: &[Point2]) -> bool {
    closed_polyline_self_intersection_count(points) > 0
}

fn closed_polyline_self_intersection_count(points: &[Point2]) -> i64 {
    let count = points.len();
    if count < 4 {
        return 0;
    }
    let mut intersections = 0_i64;
    for i in 0..count {
        let i_next = (i + 1) % count;
        for j in (i + 1)..count {
            let j_next = (j + 1) % count;
            if i == j || i_next == j || j_next == i {
                continue;
            }
            if i == 0 && j_next == 0 {
                continue;
            }
            if segment_intersects_inclusive(points[i], points[i_next], points[j], points[j_next]) {
                intersections += 1;
            }
        }
    }
    intersections
}

fn closed_polyline_pair_intersection_count(left: &[Point2], right: &[Point2]) -> i64 {
    if left.len() < 2 || right.len() < 2 {
        return 0;
    }
    let mut intersections = 0_i64;
    for left_index in 0..left.len() {
        let left_next = (left_index + 1) % left.len();
        for right_index in 0..right.len() {
            let right_next = (right_index + 1) % right.len();
            if segment_intersects_inclusive(
                left[left_index],
                left[left_next],
                right[right_index],
                right[right_next],
            ) {
                intersections += 1;
            }
        }
    }
    intersections
}

fn repair_closed_fact_topology_with_route_pair(
    left: &mut [Point2],
    right: &mut [Point2],
    route_left: &[Point2],
    route_right: &[Point2],
    raw_left: &[Point2],
    raw_right: &[Point2],
    control: StationGenerationControl<'_>,
) -> StationBuildResult<i64> {
    control.checkpoint_phase("closed_fact_topology_repair")?;
    let count = left
        .len()
        .min(right.len())
        .min(route_left.len())
        .min(route_right.len());
    if count < 4 {
        return Ok(0);
    }

    let raw_left_self = closed_polyline_self_intersection_count(raw_left);
    let raw_right_self = closed_polyline_self_intersection_count(raw_right);
    let raw_left_right = closed_polyline_pair_intersection_count(raw_left, raw_right);
    let initial_score =
        closed_fact_topology_score(left, right, raw_left_self, raw_right_self, raw_left_right);
    let route_score = closed_fact_topology_score(
        &route_left[..count],
        &route_right[..count],
        raw_left_self,
        raw_right_self,
        raw_left_right,
    );
    if route_score < initial_score {
        left[..count].copy_from_slice(&route_left[..count]);
        right[..count].copy_from_slice(&route_right[..count]);
        return Ok((2 * count) as i64);
    }
    let mut replaced_count = 0_i64;

    for _ in 0..4 {
        control.checkpoint_phase("closed_fact_topology_repair")?;
        let baseline_score =
            closed_fact_topology_score(left, right, raw_left_self, raw_right_self, raw_left_right);
        let mut candidate_indices = closed_fact_topology_candidate_indices(left, right);
        candidate_indices.sort_unstable();
        candidate_indices.dedup();
        if candidate_indices.is_empty() {
            break;
        }

        let mut best_left = left.to_vec();
        let mut best_right = right.to_vec();
        let mut best_score = baseline_score;
        let mut best_replaced = 0_i64;

        for center_index in candidate_indices {
            control.checkpoint_phase("closed_fact_topology_repair")?;
            let first_start = center_index.saturating_sub(4);
            let last_start = (center_index + 4).min(count - 1);
            for start in first_start..=last_start {
                for length in 1..=8_usize {
                    let end = (start + length - 1).min(count - 1);
                    for mode in 0..3 {
                        let mut candidate_left = left.to_vec();
                        let mut candidate_right = right.to_vec();
                        let mut changed = 0_i64;
                        if mode == 0 || mode == 2 {
                            for index in start..=end {
                                if candidate_left[index] != route_left[index] {
                                    candidate_left[index] = route_left[index];
                                    changed += 1;
                                }
                            }
                        }
                        if mode == 1 || mode == 2 {
                            for index in start..=end {
                                if candidate_right[index] != route_right[index] {
                                    candidate_right[index] = route_right[index];
                                    changed += 1;
                                }
                            }
                        }
                        if changed == 0 {
                            continue;
                        }
                        let score = closed_fact_topology_score(
                            &candidate_left,
                            &candidate_right,
                            raw_left_self,
                            raw_right_self,
                            raw_left_right,
                        );
                        if score < best_score {
                            best_score = score;
                            best_left = candidate_left;
                            best_right = candidate_right;
                            best_replaced = changed;
                        }
                    }
                }
            }
        }

        if best_score >= baseline_score {
            break;
        }
        left[..count].copy_from_slice(&best_left[..count]);
        right[..count].copy_from_slice(&best_right[..count]);
        replaced_count += best_replaced;
    }

    Ok(replaced_count)
}

fn stabilize_closed_fact_width_with_route_pair(
    left: &mut [Point2],
    right: &mut [Point2],
    route_left: &[Point2],
    route_right: &[Point2],
    raw_left: &[Point2],
    raw_right: &[Point2],
    control: StationGenerationControl<'_>,
) -> StationBuildResult<i64> {
    control.checkpoint_phase("closed_fact_width_repair")?;
    let count = left
        .len()
        .min(right.len())
        .min(route_left.len())
        .min(route_right.len());
    if count < 4 {
        return Ok(0);
    }

    let width_deviation = |candidate_left: &[Point2], candidate_right: &[Point2]| {
        (0..count)
            .map(|index| {
                let width = distance(candidate_left[index], candidate_right[index]).max(1e-9);
                let route_width = distance(route_left[index], route_right[index]).max(1e-9);
                (width / route_width).max(route_width / width)
            })
            .fold(1.0, f64::max)
    };
    let baseline_width_deviation = width_deviation(left, right);
    if baseline_width_deviation <= 1.2 {
        return Ok(0);
    }

    let raw_left_self = closed_polyline_self_intersection_count(raw_left);
    let raw_right_self = closed_polyline_self_intersection_count(raw_right);
    let raw_left_right = closed_polyline_pair_intersection_count(raw_left, raw_right);
    let baseline_topology =
        closed_fact_topology_score(left, right, raw_left_self, raw_right_self, raw_left_right);
    let route_topology = closed_fact_topology_score(
        &route_left[..count],
        &route_right[..count],
        raw_left_self,
        raw_right_self,
        raw_left_right,
    );
    if route_topology <= baseline_topology
        && width_deviation(&route_left[..count], &route_right[..count]) < baseline_width_deviation
    {
        left[..count].copy_from_slice(&route_left[..count]);
        right[..count].copy_from_slice(&route_right[..count]);
        return Ok((2 * count) as i64);
    }

    Ok(0)
}

fn repair_projection_topology_from_prepared_frame(
    left: &mut [Point2],
    right: &mut [Point2],
    frame_left: &[Point2],
    frame_right: &[Point2],
    control: StationGenerationControl<'_>,
) -> StationBuildResult<i64> {
    control.checkpoint_phase("projection_topology_repair")?;
    let count = left
        .len()
        .min(right.len())
        .min(frame_left.len())
        .min(frame_right.len());
    if count < 4 {
        return Ok(0);
    }

    let baseline_score = projection_topology_score(&left[..count], &right[..count]);
    if baseline_score.0 == 0 {
        return Ok(0);
    }

    let mut seeds = Vec::new();
    for (first, second) in station_horizon_crossing_pairs_closed(&left[..count], &right[..count], 2)
    {
        seeds.push(first);
        seeds.push(second);
    }
    for index in 0..count {
        let next = (index + 1) % count;
        if distance(left[index], left[next]) <= CLOSED_ENDPOINT_PLATEAU_EPS_M
            || distance(right[index], right[next]) <= CLOSED_ENDPOINT_PLATEAU_EPS_M
        {
            seeds.push(index);
            seeds.push(next);
        }
    }
    seeds.sort_unstable();
    seeds.dedup();
    if seeds.is_empty() {
        return Ok(0);
    }
    let mut best_left = left[..count].to_vec();
    let mut best_right = right[..count].to_vec();
    let mut best_score = baseline_score;
    let mut best_replaced = 0_i64;

    for radius in 0..=12_usize {
        control.checkpoint_phase("projection_topology_repair")?;
        let mut replace = vec![false; count];
        for seed in seeds.iter().copied() {
            for offset in 0..=(2 * radius) {
                replace[(seed + count + offset - radius) % count] = true;
            }
        }
        let mut candidate_left = left[..count].to_vec();
        let mut candidate_right = right[..count].to_vec();
        let mut replaced = 0_i64;
        for index in 0..count {
            if replace[index] {
                candidate_left[index] = frame_left[index];
                candidate_right[index] = frame_right[index];
                replaced += 1;
            }
        }
        let score = projection_topology_score(&candidate_left, &candidate_right);
        if score < best_score {
            best_score = score;
            best_left = candidate_left;
            best_right = candidate_right;
            best_replaced = replaced;
        }
        if best_score.0 == 0 {
            break;
        }
    }

    if best_score >= baseline_score {
        return Ok(0);
    }
    left[..count].copy_from_slice(&best_left);
    right[..count].copy_from_slice(&best_right);
    Ok(best_replaced)
}

fn projection_topology_score(left: &[Point2], right: &[Point2]) -> (i64, i64, i64, i64) {
    let horizon_crossings = station_horizon_crossing_count(left, right, 2);
    let endpoint_plateaus = (0..left.len())
        .filter(|index| {
            let next = (*index + 1) % left.len();
            distance(left[*index], left[next]) <= CLOSED_ENDPOINT_PLATEAU_EPS_M
                || distance(right[*index], right[next]) <= CLOSED_ENDPOINT_PLATEAU_EPS_M
        })
        .count() as i64;
    (
        horizon_crossings + endpoint_plateaus,
        horizon_crossings,
        endpoint_plateaus,
        station_crossing_count_all(left, right),
    )
}

fn closed_fact_topology_score(
    left: &[Point2],
    right: &[Point2],
    raw_left_self: i64,
    raw_right_self: i64,
    raw_left_right: i64,
) -> (i64, i64, i64, i64) {
    let left_self = closed_polyline_self_intersection_count(left);
    let right_self = closed_polyline_self_intersection_count(right);
    let left_right = closed_polyline_pair_intersection_count(left, right);
    (
        station_horizon_crossing_count(left, right, 2),
        (left_self - raw_left_self).max(0)
            + (right_self - raw_right_self).max(0)
            + (left_right - raw_left_right).max(0),
        station_crossing_count_all(left, right),
        left_self + right_self + left_right,
    )
}

fn closed_fact_topology_candidate_indices(left: &[Point2], right: &[Point2]) -> Vec<usize> {
    let mut indices = Vec::new();
    for (a, b) in closed_polyline_self_intersection_pairs(left) {
        push_local_topology_indices(&mut indices, left.len(), a);
        push_local_topology_indices(&mut indices, left.len(), b);
    }
    for (a, b) in closed_polyline_self_intersection_pairs(right) {
        push_local_topology_indices(&mut indices, right.len(), a);
        push_local_topology_indices(&mut indices, right.len(), b);
    }
    for (a, b) in station_horizon_crossing_pairs_closed(left, right, 2) {
        push_local_topology_indices(&mut indices, left.len(), a);
        push_local_topology_indices(&mut indices, left.len(), b);
    }
    indices
}

fn closed_polyline_self_intersection_pairs(points: &[Point2]) -> Vec<(usize, usize)> {
    let count = points.len();
    if count < 4 {
        return Vec::new();
    }
    let mut intersections = Vec::new();
    for i in 0..count {
        let i_next = (i + 1) % count;
        for j in (i + 1)..count {
            let j_next = (j + 1) % count;
            if i == j || i_next == j || j_next == i {
                continue;
            }
            if i == 0 && j_next == 0 {
                continue;
            }
            if segment_intersects_inclusive(points[i], points[i_next], points[j], points[j_next]) {
                intersections.push((i, j));
            }
        }
    }
    intersections
}

fn station_horizon_crossing_pairs_closed(
    left: &[Point2],
    right: &[Point2],
    horizon: usize,
) -> Vec<(usize, usize)> {
    let mut pairs = Vec::new();
    for index in 0..left.len() {
        for offset in 1..=horizon.max(1) {
            let next = (index + offset) % left.len();
            if segment_intersects(left[index], right[index], left[next], right[next]) {
                pairs.push((index, next));
            }
        }
    }
    pairs
}

fn push_local_topology_indices(indices: &mut Vec<usize>, count: usize, index: usize) {
    if count == 0 {
        return;
    }
    for offset in 0..=2 {
        indices.push((index + count - offset) % count);
        indices.push((index + offset) % count);
    }
}

fn repair_open_crossing_pairs(
    left: &mut [Point2],
    right: &mut [Point2],
    normals: &mut [Point2],
    left_projection_s: &mut [f64],
    right_projection_s: &mut [f64],
    left_projection: &OpenPolylineProjection<'_>,
    right_projection: &OpenPolylineProjection<'_>,
    centers: &[Point2],
    raw_left_world: &[Point2],
    raw_right_world: &[Point2],
    width_left: &[f64],
    width_right: &[f64],
    frame_left: &[Point2],
    frame_right: &[Point2],
    passes: usize,
    control: StationGenerationControl<'_>,
) -> StationBuildResult<i64> {
    control.checkpoint_phase("open_crossing_repair")?;
    let count = left
        .len()
        .min(right.len())
        .min(frame_left.len())
        .min(frame_right.len());
    if count < 2 {
        return Ok(0);
    }
    let mut replaced_count = 0_i64;
    for _ in 0..passes {
        control.checkpoint_phase("open_crossing_repair")?;
        let mut changed = false;
        'outer: for index in 0..count {
            if index % 8 == 0 {
                control.checkpoint_phase("open_crossing_repair")?;
            }
            for offset in 1..=2 {
                let next = index + offset;
                if next >= count
                    || !segment_intersects(left[index], right[index], left[next], right[next])
                {
                    continue;
                }
                let baseline =
                    open_station_crossing_score(left, right, raw_left_world, raw_right_world);
                let mut best_left = left.to_vec();
                let mut best_right = right.to_vec();
                let mut best_score = baseline;
                for (first, second) in [
                    (Some(index), None),
                    (Some(next), None),
                    (Some(index), Some(next)),
                ] {
                    let mut candidate_left = left.to_vec();
                    let mut candidate_right = right.to_vec();
                    for candidate_index in [first, second].into_iter().flatten() {
                        candidate_left[candidate_index] = frame_left[candidate_index];
                        candidate_right[candidate_index] = frame_right[candidate_index];
                    }
                    let score = open_station_crossing_score(
                        &candidate_left,
                        &candidate_right,
                        raw_left_world,
                        raw_right_world,
                    );
                    if score < best_score {
                        best_score = score;
                        best_left = candidate_left;
                        best_right = candidate_right;
                    }
                }
                let mut smoothed_normal = point_add(normals[index], normals[next]);
                if index > 0 {
                    smoothed_normal = point_add(smoothed_normal, normals[index - 1]);
                }
                if next + 1 < count {
                    smoothed_normal = point_add(smoothed_normal, normals[next + 1]);
                }
                smoothed_normal = normalize(smoothed_normal, normals[index]);
                for (first, second) in [
                    (Some(index), None),
                    (Some(next), None),
                    (Some(index), Some(next)),
                ] {
                    let mut candidate_left = left.to_vec();
                    let mut candidate_right = right.to_vec();
                    for candidate_index in [first, second].into_iter().flatten() {
                        let fallback_left = point_sub(
                            centers[candidate_index],
                            point_scale(smoothed_normal, width_left[candidate_index]),
                        );
                        let fallback_right = point_add(
                            centers[candidate_index],
                            point_scale(smoothed_normal, width_right[candidate_index]),
                        );
                        let section = normal_line_section_open(
                            centers[candidate_index],
                            smoothed_normal,
                            raw_left_world,
                            raw_right_world,
                            fallback_left,
                            fallback_right,
                        );
                        candidate_left[candidate_index] = section.0;
                        candidate_right[candidate_index] = section.1;
                    }
                    let score = open_station_crossing_score(
                        &candidate_left,
                        &candidate_right,
                        raw_left_world,
                        raw_right_world,
                    );
                    if score < best_score {
                        best_score = score;
                        best_left = candidate_left;
                        best_right = candidate_right;
                    }
                }
                for radius in [1_usize, 2, 3] {
                    let start = index.saturating_sub(radius);
                    let end = (next + radius).min(count - 1);
                    let mut candidate_left = left.to_vec();
                    let mut candidate_right = right.to_vec();
                    candidate_left[start..(end + 1)].copy_from_slice(&frame_left[start..(end + 1)]);
                    candidate_right[start..(end + 1)]
                        .copy_from_slice(&frame_right[start..(end + 1)]);
                    let score = open_station_crossing_score(
                        &candidate_left,
                        &candidate_right,
                        raw_left_world,
                        raw_right_world,
                    );
                    if score < best_score {
                        best_score = score;
                        best_left = candidate_left;
                        best_right = candidate_right;
                    }
                }
                if best_score < baseline {
                    for repair_index in 0..count {
                        if left[repair_index] != best_left[repair_index]
                            || right[repair_index] != best_right[repair_index]
                        {
                            left[repair_index] = best_left[repair_index];
                            right[repair_index] = best_right[repair_index];
                            let chord = point_sub(right[repair_index], left[repair_index]);
                            normals[repair_index] = normalize(chord, normals[repair_index]);
                            left_projection_s[repair_index] =
                                left_projection.project_arclength(left[repair_index]);
                            right_projection_s[repair_index] =
                                right_projection.project_arclength(right[repair_index]);
                            replaced_count += 1;
                        }
                    }
                    changed = true;
                    break 'outer;
                }
            }
        }
        if !changed {
            break;
        }
    }
    Ok(replaced_count)
}

fn open_station_crossing_score(
    left: &[Point2],
    right: &[Point2],
    raw_left_world: &[Point2],
    raw_right_world: &[Point2],
) -> (i64, i64, i64) {
    (
        station_horizon_crossing_count_open(left, right, 2),
        station_raw_boundary_crossing_count_open(left, right, raw_left_world, raw_right_world),
        station_crossing_count_all(left, right),
    )
}

fn rotate_vector(vector: Point2, angle_rad: f64) -> Point2 {
    let (sin, cos) = angle_rad.sin_cos();
    [
        cos * vector[0] - sin * vector[1],
        sin * vector[0] + cos * vector[1],
    ]
}

fn upsert_metadata(metadata: &mut JsonObject, key: &str, value: JsonValue) {
    if let Some((_, existing)) = metadata
        .iter_mut()
        .find(|(existing_key, _)| existing_key == key)
    {
        *existing = value;
    } else {
        metadata.push((key.to_owned(), value));
    }
}

fn build_midref_reference_track_from_raw_boundaries(track: &TrackAreaContractV1) -> ReferenceTrack {
    let reference_frame = build_boundary_pair_track(
        &track.left_boundary_xy_m,
        &track.right_boundary_xy_m,
        &midref_reference_station_options(),
    );
    ReferenceTrack {
        centerline_world: reference_frame.centerline_world,
        width_right_m: reference_frame.width_right,
        width_left_m: reference_frame.width_left,
    }
}

fn apply_progress_zero_station_normal_fix(
    centerline: &[Point2],
    progress: &[f64],
    normals: &mut [Point2],
    enabled: bool,
) -> (i64, Option<Point2>) {
    if !enabled || centerline.len() < 3 || normals.is_empty() {
        return (0, None);
    }
    let tangent = normalize(
        point_sub(centerline[1], *centerline.last().unwrap()),
        [0.0, 0.0],
    );
    if hypot(tangent) <= 1e-9 {
        return (0, None);
    }
    let fixed_base = [tangent[1], -tangent[0]];
    let mut count = 0_i64;
    for (index, normal) in normals.iter_mut().enumerate() {
        let p = progress[index].rem_euclid(1.0);
        if p.min(1.0 - p) <= 1e-12 {
            let mut fixed = fixed_base;
            if dot(fixed, *normal) < 0.0 {
                fixed = point_scale(fixed, -1.0);
            }
            *normal = fixed;
            count += 1;
        }
    }
    (count, Some(fixed_base))
}

fn compute_section_cell_areas(left_world: &[Point2], right_world: &[Point2]) -> Vec<f64> {
    (0..left_world.len())
        .map(|index| {
            let next = (index + 1) % left_world.len();
            polygon_area_abs(&[
                left_world[index],
                left_world[next],
                right_world[next],
                right_world[index],
            ])
        })
        .collect()
}

fn compute_open_section_cell_areas(left_world: &[Point2], right_world: &[Point2]) -> Vec<f64> {
    let count = left_world.len().min(right_world.len());
    if count < 2 {
        return Vec::new();
    }
    (0..count - 1)
        .map(|index| {
            polygon_area_abs(&[
                left_world[index],
                left_world[index + 1],
                right_world[index + 1],
                right_world[index],
            ])
        })
        .collect()
}

fn polygon_area_abs(points: &[Point2]) -> f64 {
    if points.len() < 3 {
        return 0.0;
    }
    let mut area = 0.0;
    for index in 0..points.len() {
        let next = (index + 1) % points.len();
        area += points[index][0] * points[next][1] - points[index][1] * points[next][0];
    }
    0.5 * area.abs()
}

fn open_area_target_progress(
    dense_corridor: &BoundaryPairTrack,
    sample_count: usize,
    options: &FixedCenterlineStationOptions,
) -> (Vec<f64>, JsonObject) {
    let sample_count = sample_count.max(3);
    let real_areas =
        compute_open_section_cell_areas(&dense_corridor.left_world, &dense_corridor.right_world);
    if real_areas.is_empty() {
        return (
            (0..sample_count)
                .map(|index| index as f64 / (sample_count - 1) as f64)
                .collect(),
            vec![(
                "density_source".to_owned(),
                "open_arclength_fallback".into(),
            )],
        );
    }
    let (density_areas, area_cap_meta) = open_density_areas_with_iqr_length_cap(
        &dense_corridor.left_world,
        &dense_corridor.right_world,
        options.density_area_length_cap_multiplier,
    );
    let (segment_weight, curvature_meta) = topology_curvature_segment_weight(
        StationTopology::Open,
        &dense_corridor.centerline_world,
        &density_areas,
        options.straight_weight,
        options.curved_weight,
        options.turn_smoothing_window,
        options.curvature_low_percentile,
        options.curvature_high_percentile,
    );
    let mut weighted_area = density_areas
        .iter()
        .zip(&segment_weight)
        .map(|(area, weight)| area * weight)
        .collect::<Vec<_>>();
    let weighted_sum_raw = weighted_area.iter().copied().sum::<f64>();
    let total_area = density_areas.iter().copied().sum::<f64>();
    if weighted_sum_raw > 1e-9 && total_area > 1e-9 {
        let scale = total_area / weighted_sum_raw;
        for value in &mut weighted_area {
            *value *= scale;
        }
    }
    let weighted_sum = weighted_area.iter().copied().sum::<f64>();
    let cumulative = cumulative_with_zero(&weighted_area);
    let dense_count = dense_corridor.centerline_world.len();
    let progress_edges = (0..dense_count)
        .map(|index| index as f64 / (dense_count - 1).max(1) as f64)
        .collect::<Vec<_>>();
    let area_progress = if weighted_sum <= 1e-9 {
        (0..sample_count)
            .map(|index| index as f64 / (sample_count - 1) as f64)
            .collect::<Vec<_>>()
    } else {
        (0..sample_count)
            .map(|index| {
                let target_weighted = weighted_sum * index as f64 / (sample_count - 1) as f64;
                interp_scalar(target_weighted, &cumulative, &progress_edges)
            })
            .collect::<Vec<_>>()
    };
    let arclength_progress = (0..sample_count)
        .map(|index| index as f64 / (sample_count - 1) as f64)
        .collect::<Vec<_>>();
    let blended = area_progress
        .iter()
        .zip(&arclength_progress)
        .map(|(area, arc)| 0.85 * area + 0.15 * arc)
        .collect::<Vec<_>>();
    let (target_progress, target_spacing_meta) = limit_target_progress_spacing_for_topology(
        StationTopology::Open,
        &progress_edges,
        &dense_corridor.left_world,
        &dense_corridor.right_world,
        &dense_corridor.centerline_world,
        &blended,
        options.target_spacing_max_adjacent_ratio,
        &options.target_spacing_metric,
    );
    let mut meta = vec![
        (
            "density_source".to_owned(),
            "open_baseline_curvature_area".into(),
        ),
        ("placement_arclength_blend".to_owned(), 0.15_f64.into()),
        (
            "physical_total_area_m2".to_owned(),
            real_areas.iter().copied().sum::<f64>().into(),
        ),
        ("density_total_area_m2".to_owned(), weighted_sum.into()),
    ];
    meta.extend(area_cap_meta);
    meta.extend(curvature_meta);
    meta.extend(target_spacing_meta);
    (target_progress, meta)
}

fn open_density_areas_with_iqr_length_cap(
    left: &[Point2],
    right: &[Point2],
    multiplier: f64,
) -> (Vec<f64>, JsonObject) {
    let real_areas = compute_open_section_cell_areas(left, right);
    let lengths = right
        .iter()
        .zip(left)
        .map(|(r, l)| distance(*r, *l))
        .collect::<Vec<_>>();
    if lengths.len() < 4 {
        return (
            real_areas,
            vec![("density_area_length_cap_mode".to_owned(), "none".into())],
        );
    }
    let q1 = percentile(lengths.clone(), 25.0);
    let q3 = percentile(lengths.clone(), 75.0);
    let median_length = median(lengths.clone());
    let mut cap = q3 + multiplier * (q3 - q1);
    if !cap.is_finite() || cap <= 1e-9 {
        cap = lengths.iter().copied().fold(0.0, f64::max);
    }
    cap = cap.max(median_length);
    let capped_left = left
        .iter()
        .zip(right)
        .zip(&lengths)
        .map(|((left, right), length)| {
            let scale = 1.0_f64.min(cap / length.max(1e-9));
            let center = midpoint(*left, *right);
            let half = point_scale(point_sub(*right, *left), 0.5 * scale);
            point_sub(center, half)
        })
        .collect::<Vec<_>>();
    let capped_right = left
        .iter()
        .zip(right)
        .zip(&lengths)
        .map(|((left, right), length)| {
            let scale = 1.0_f64.min(cap / length.max(1e-9));
            let center = midpoint(*left, *right);
            let half = point_scale(point_sub(*right, *left), 0.5 * scale);
            point_add(center, half)
        })
        .collect::<Vec<_>>();
    let effective_areas = compute_open_section_cell_areas(&capped_left, &capped_right);
    let capped_count = lengths.iter().filter(|value| **value > cap).count();
    (
        effective_areas,
        vec![
            ("density_area_length_cap_mode".to_owned(), "open_iqr".into()),
            (
                "density_area_length_cap_multiplier".to_owned(),
                multiplier.into(),
            ),
            (
                "density_area_length_median_m".to_owned(),
                median_length.into(),
            ),
            ("density_area_length_cap_m".to_owned(), cap.into()),
            (
                "density_area_length_cap_count".to_owned(),
                JsonValue::Integer(capped_count as i64),
            ),
            (
                "density_area_length_cap_fraction".to_owned(),
                (capped_count as f64 / lengths.len() as f64).into(),
            ),
        ],
    )
}

fn open_curvature_segment_weight(
    centerline: &[Point2],
    areas: &[f64],
    straight_weight: f64,
    curved_weight: f64,
    smoothing_window: usize,
    low_percentile: f64,
    high_percentile: f64,
) -> (Vec<f64>, JsonObject) {
    if areas.is_empty() {
        return (Vec::new(), Vec::new());
    }
    let mut abs_turn = local_turn_angles_open(centerline)
        .into_iter()
        .map(f64::abs)
        .collect::<Vec<_>>();
    abs_turn = open_smooth_1d(&abs_turn, smoothing_window);
    let segment_score_raw = areas
        .iter()
        .enumerate()
        .map(|(index, _)| {
            let a = abs_turn.get(index).copied().unwrap_or(0.0);
            let b = abs_turn.get(index + 1).copied().unwrap_or(a);
            0.5 * (a + b)
        })
        .collect::<Vec<_>>();
    let lo = percentile(segment_score_raw.clone(), low_percentile);
    let mut hi = percentile(segment_score_raw.clone(), high_percentile);
    if hi <= lo + 1e-12 {
        hi = lo + (segment_score_raw.iter().copied().fold(0.0, f64::max) - lo).max(1e-9);
    }
    let raw_weight = segment_score_raw
        .iter()
        .map(|value| {
            let score = ((*value - lo) / (hi - lo).max(1e-12)).clamp(0.0, 1.0);
            straight_weight + (curved_weight - straight_weight) * score
        })
        .collect::<Vec<_>>();
    let weight = limit_positive_linear_slew(
        &open_smooth_positive_profile(&raw_weight, smoothing_window),
        1.35,
        32,
    );
    (
        weight,
        vec![
            (
                "curvature_normalization".to_owned(),
                "open_percentile".into(),
            ),
            ("curvature_abs_low_1pm".to_owned(), lo.into()),
            ("curvature_abs_high_1pm".to_owned(), hi.into()),
        ],
    )
}

fn density_areas_with_iqr_length_cap(
    left: &[Point2],
    right: &[Point2],
    multiplier: f64,
) -> (Vec<f64>, JsonObject) {
    let real_areas = compute_section_cell_areas(left, right);
    let lengths = right
        .iter()
        .zip(left)
        .map(|(r, l)| distance(*r, *l))
        .collect::<Vec<_>>();
    if lengths.len() < 4 {
        return (
            real_areas.clone(),
            vec![
                ("density_area_length_cap_mode".to_owned(), "none".into()),
                (
                    "density_area_length_cap_count".to_owned(),
                    JsonValue::Integer(0),
                ),
            ],
        );
    }
    let q1 = percentile(lengths.clone(), 25.0);
    let q3 = percentile(lengths.clone(), 75.0);
    let median_length = median(lengths.clone());
    let mut cap = q3 + multiplier * (q3 - q1);
    if !cap.is_finite() || cap <= 1e-9 {
        cap = lengths.iter().copied().fold(0.0, f64::max);
    }
    cap = cap.max(median_length);
    let mut capped_left = Vec::with_capacity(left.len());
    let mut capped_right = Vec::with_capacity(right.len());
    for index in 0..left.len() {
        let scale = 1.0_f64.min(cap / lengths[index].max(1e-9));
        let center = midpoint(left[index], right[index]);
        let half = point_scale(point_sub(right[index], left[index]), 0.5 * scale);
        capped_left.push(point_sub(center, half));
        capped_right.push(point_add(center, half));
    }
    let effective_areas = compute_section_cell_areas(&capped_left, &capped_right);
    let real_total = real_areas.iter().copied().sum::<f64>();
    let effective_total = effective_areas.iter().copied().sum::<f64>();
    (
        effective_areas,
        vec![
            ("density_area_length_cap_mode".to_owned(), "iqr".into()),
            (
                "density_area_length_cap_multiplier".to_owned(),
                multiplier.into(),
            ),
            ("density_area_real_total_m2".to_owned(), real_total.into()),
            (
                "density_area_length_min_m".to_owned(),
                lengths.iter().copied().fold(f64::INFINITY, f64::min).into(),
            ),
            (
                "density_area_length_median_m".to_owned(),
                median_length.into(),
            ),
            (
                "density_area_length_max_m".to_owned(),
                lengths.iter().copied().fold(0.0, f64::max).into(),
            ),
            ("density_area_length_cap_m".to_owned(), cap.into()),
            (
                "density_area_length_cap_count".to_owned(),
                JsonValue::Integer(lengths.iter().filter(|value| **value > cap).count() as i64),
            ),
            (
                "density_area_length_cap_fraction".to_owned(),
                (lengths.iter().filter(|value| **value > cap).count() as f64
                    / lengths.len() as f64)
                    .into(),
            ),
            (
                "density_area_effective_total_m2".to_owned(),
                effective_total.into(),
            ),
            (
                "density_area_effective_to_real_total_ratio".to_owned(),
                (effective_total / real_total.max(1e-9)).into(),
            ),
        ],
    )
}

fn baseline_curvature_segment_weight(
    baseline_dense: &[Point2],
    areas: &[f64],
    straight_weight: f64,
    curved_weight: f64,
    smoothing_window: usize,
    low_percentile: f64,
    high_percentile: f64,
) -> (Vec<f64>, JsonObject) {
    let mut abs_curvature = three_point_curvature(baseline_dense)
        .into_iter()
        .map(f64::abs)
        .collect::<Vec<_>>();
    abs_curvature = circular_smooth_1d(&abs_curvature, smoothing_window);
    let lo = percentile(abs_curvature.clone(), low_percentile);
    let mut hi = percentile(abs_curvature.clone(), high_percentile);
    if hi <= lo + 1e-12 {
        hi = lo + (abs_curvature.iter().copied().fold(0.0, f64::max) - lo).max(1e-9);
    }
    let curvature01 = abs_curvature
        .iter()
        .map(|value| ((*value - lo) / (hi - lo).max(1e-12)).clamp(0.0, 1.0))
        .collect::<Vec<_>>();
    let segment_score = (0..curvature01.len())
        .map(|index| 0.5 * (curvature01[index] + curvature01[(index + 1) % curvature01.len()]))
        .collect::<Vec<_>>();
    let weight_span = curved_weight - straight_weight;
    let target_score_mean = (1.0 - straight_weight) / weight_span.max(1e-12);
    let (balanced, gamma, balanced_mean) =
        balance_unit_score_mean_bounded(&segment_score, areas, target_score_mean);
    let weights = balanced
        .iter()
        .map(|score| straight_weight + weight_span * score)
        .collect::<Vec<_>>();
    let total_area = areas.iter().copied().sum::<f64>();
    let curvature_area_weighted_mean = if total_area > 1e-9 {
        areas
            .iter()
            .zip(&weights)
            .map(|(area, weight)| area * weight)
            .sum::<f64>()
            / total_area
    } else {
        0.0
    };
    (
        weights,
        vec![
            ("curvature_normalization".to_owned(), "bounded_power".into()),
            (
                "curvature_abs_min_1pm".to_owned(),
                abs_curvature
                    .iter()
                    .copied()
                    .fold(f64::INFINITY, f64::min)
                    .into(),
            ),
            (
                "curvature_abs_median_1pm".to_owned(),
                median(abs_curvature.clone()).into(),
            ),
            (
                "curvature_abs_max_1pm".to_owned(),
                abs_curvature.iter().copied().fold(0.0, f64::max).into(),
            ),
            ("curvature_low_percentile_value_1pm".to_owned(), lo.into()),
            ("curvature_high_percentile_value_1pm".to_owned(), hi.into()),
            (
                "curvature_segment_score_area_weighted_mean_raw".to_owned(),
                area_weighted_mean(&segment_score, areas).into(),
            ),
            (
                "curvature_segment_score_target_mean".to_owned(),
                target_score_mean.into(),
            ),
            (
                "curvature_segment_score_balance_gamma".to_owned(),
                gamma.into(),
            ),
            (
                "curvature_segment_score_area_weighted_mean_balanced".to_owned(),
                balanced_mean.into(),
            ),
            (
                "curvature_area_weighted_mean_after_normalization".to_owned(),
                curvature_area_weighted_mean.into(),
            ),
        ],
    )
}

fn topology_curvature_segment_weight(
    topology: StationTopology,
    centerline: &[Point2],
    areas: &[f64],
    straight_weight: f64,
    curved_weight: f64,
    smoothing_window: usize,
    low_percentile: f64,
    high_percentile: f64,
) -> (Vec<f64>, JsonObject) {
    match topology {
        StationTopology::Closed => baseline_curvature_segment_weight(
            centerline,
            areas,
            straight_weight,
            curved_weight,
            smoothing_window,
            low_percentile,
            high_percentile,
        ),
        StationTopology::Open => open_curvature_segment_weight(
            centerline,
            areas,
            straight_weight,
            curved_weight,
            smoothing_window,
            low_percentile,
            high_percentile,
        ),
    }
}

fn three_point_curvature(points: &[Point2]) -> Vec<f64> {
    (0..points.len())
        .map(|index| {
            let prev = points[(index + points.len() - 1) % points.len()];
            let current = points[index];
            let next = points[(index + 1) % points.len()];
            let a = point_sub(current, prev);
            let b = point_sub(next, current);
            let c = point_sub(next, prev);
            let la = hypot(a);
            let lb = hypot(b);
            let lc = hypot(c);
            let denom = (la * lb * lc).max(1e-12);
            2.0 * cross(a, b) / denom
        })
        .collect()
}

fn balance_unit_score_mean_bounded(
    score: &[f64],
    areas: &[f64],
    target_mean: f64,
) -> (Vec<f64>, f64, f64) {
    let target = target_mean.clamp(0.0, 1.0);
    if target <= 1e-12 {
        return (vec![0.0; score.len()], 1.0, 0.0);
    }
    if target >= 1.0 - 1e-12 {
        return (vec![1.0; score.len()], 1.0, 1.0);
    }
    let mean_at_one = area_weighted_mean(score, areas);
    if (mean_at_one - target).abs() <= 1e-9 {
        return (score.to_vec(), 1.0, mean_at_one);
    }
    let mean_for_gamma = |gamma: f64| -> f64 {
        area_weighted_mean(
            &score
                .iter()
                .map(|value| value.clamp(0.0, 1.0).powf(gamma))
                .collect::<Vec<_>>(),
            areas,
        )
    };
    let gamma = if mean_at_one < target {
        let mut lo = 1e-3;
        let mut hi = 1.0;
        if mean_for_gamma(lo) < target {
            lo
        } else {
            for _ in 0..80 {
                let mid = 0.5 * (lo + hi);
                if mean_for_gamma(mid) > target {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            0.5 * (lo + hi)
        }
    } else {
        let mut lo = 1.0;
        let mut hi = 64.0;
        if mean_for_gamma(hi) > target {
            hi
        } else {
            for _ in 0..80 {
                let mid = 0.5 * (lo + hi);
                if mean_for_gamma(mid) > target {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            0.5 * (lo + hi)
        }
    };
    let balanced = score
        .iter()
        .map(|value| value.clamp(0.0, 1.0).powf(gamma))
        .collect::<Vec<_>>();
    let mean = area_weighted_mean(&balanced, areas);
    (balanced, gamma, mean)
}

fn area_weighted_mean(values: &[f64], areas: &[f64]) -> f64 {
    let total = areas.iter().copied().sum::<f64>();
    if total <= 1e-12 {
        return 0.0;
    }
    values
        .iter()
        .zip(areas)
        .map(|(value, area)| value * area)
        .sum::<f64>()
        / total
}

fn percentile(mut values: Vec<f64>, percentile: f64) -> f64 {
    values.retain(|value| value.is_finite());
    if values.is_empty() {
        return 0.0;
    }
    values.sort_by(f64::total_cmp);
    let rank = (percentile / 100.0).clamp(0.0, 1.0) * (values.len() - 1) as f64;
    let low = rank.floor() as usize;
    let high = rank.ceil() as usize;
    if low == high {
        values[low]
    } else {
        let t = rank - low as f64;
        values[low] + (values[high] - values[low]) * t
    }
}

#[derive(Debug, Clone, Copy)]
struct NormalLineSectionAtProgress {
    left: Point2,
    right: Point2,
    normal: Point2,
    miss: i32,
    left_s_m: Option<f64>,
    right_s_m: Option<f64>,
}

fn normal_line_section_at_progress(
    origin: Point2,
    normal: Point2,
    raw_left_world: &[Point2],
    raw_right_world: &[Point2],
    left_projection: &ClosedPolylineProjection<'_>,
    right_projection: &ClosedPolylineProjection<'_>,
    fallback_left: Point2,
    fallback_right: Point2,
    left_route_progress: f64,
    right_route_progress: f64,
    allow_wide_progress_search: bool,
) -> NormalLineSectionAtProgress {
    if allow_wide_progress_search {
        let (left, right, normal, miss) = normal_line_section_nearest(
            origin,
            normal,
            raw_left_world,
            raw_right_world,
            fallback_left,
            fallback_right,
        );
        return NormalLineSectionAtProgress {
            left,
            right,
            normal,
            miss,
            left_s_m: None,
            right_s_m: None,
        };
    }
    let normal = normalize(normal, [1.0, 0.0]);
    let search_first_successful_window = |windows: &[Option<usize>]| -> Option<(
        LineIntersectionCandidate,
        LineIntersectionCandidate,
    )> {
        for window in windows {
            let left_indices = window.map(|value| {
                local_segment_indices_by_progress(raw_left_world.len(), left_route_progress, value)
            });
            let right_indices = window.map(|value| {
                local_segment_indices_by_progress(
                    raw_right_world.len(),
                    right_route_progress,
                    value,
                )
            });
            let left_params = line_intersection_candidates(
                origin,
                normal,
                raw_left_world,
                left_indices.as_deref(),
            );
            let right_params = line_intersection_candidates(
                origin,
                normal,
                raw_right_world,
                right_indices.as_deref(),
            );
            if left_params.is_empty() || right_params.is_empty() {
                continue;
            }
            let mut best_pair: Option<(LineIntersectionCandidate, LineIntersectionCandidate)> =
                None;
            let mut best_score = f64::INFINITY;
            for left_hit in &left_params {
                for right_hit in &right_params {
                    let width = (right_hit.line_t - left_hit.line_t).abs();
                    if width <= 0.10 {
                        continue;
                    }
                    let candidate_left = point_add(origin, point_scale(normal, left_hit.line_t));
                    let candidate_right = point_add(origin, point_scale(normal, right_hit.line_t));
                    let anchor_dist = distance(candidate_left, fallback_left)
                        + distance(candidate_right, fallback_right);
                    let brackets_origin = (left_hit.line_t <= 1e-6 && 1e-6 <= right_hit.line_t)
                        || (right_hit.line_t <= 1e-6 && 1e-6 <= left_hit.line_t);
                    let midpoint_offset = (0.5 * (left_hit.line_t + right_hit.line_t)).abs();
                    let left_progress_error = cyclic_progress_distance(
                        left_hit.progress(raw_left_world.len()),
                        left_route_progress,
                    );
                    let right_progress_error = cyclic_progress_distance(
                        right_hit.progress(raw_right_world.len()),
                        right_route_progress,
                    );
                    let score = if brackets_origin { 0.0 } else { 1_000_000.0 }
                        + 1_000.0 * (left_progress_error + right_progress_error)
                        + 2.0 * anchor_dist
                        + 0.10 * midpoint_offset
                        + 0.01 * width;
                    if score < best_score {
                        best_score = score;
                        best_pair = Some((*left_hit, *right_hit));
                    }
                }
            }
            if best_pair.is_some() {
                return best_pair;
            }
        }
        None
    };
    let best_pair = search_first_successful_window(&[
        Some(1),
        Some(2),
        Some(4),
        Some(8),
        Some(16),
        Some(32),
        None,
    ]);
    if let Some((left_hit, right_hit)) = best_pair {
        let left = point_add(origin, point_scale(normal, left_hit.line_t));
        let right = point_add(origin, point_scale(normal, right_hit.line_t));
        let chord = point_sub(right, left);
        if hypot(chord) > 1e-12 {
            return NormalLineSectionAtProgress {
                left,
                right,
                normal: normalize(chord, normal),
                miss: 0,
                left_s_m: Some(left_projection.candidate_arclength(left_hit)),
                right_s_m: Some(right_projection.candidate_arclength(right_hit)),
            };
        }
    }
    let fallback_window = 32;
    let left_indices = local_segment_indices_by_progress(
        raw_left_world.len(),
        left_route_progress,
        fallback_window,
    );
    let right_indices = local_segment_indices_by_progress(
        raw_right_world.len(),
        right_route_progress,
        fallback_window,
    );
    let left =
        project_point_to_closed_polyline_indices(fallback_left, raw_left_world, &left_indices);
    let right =
        project_point_to_closed_polyline_indices(fallback_right, raw_right_world, &right_indices);
    let chord = point_sub(right, left);
    NormalLineSectionAtProgress {
        left,
        right,
        normal: normalize(chord, normal),
        miss: 1,
        left_s_m: None,
        right_s_m: None,
    }
}

#[derive(Debug, Clone, Copy)]
struct LineIntersectionCandidate {
    line_t: f64,
    segment_index: usize,
    segment_u: f64,
}

impl LineIntersectionCandidate {
    fn progress(self, segment_count: usize) -> f64 {
        if segment_count == 0 {
            return 0.0;
        }
        ((self.segment_index as f64 + self.segment_u.clamp(0.0, 1.0)) / segment_count as f64)
            .rem_euclid(1.0)
    }
}

fn normal_line_section_nearest(
    origin: Point2,
    normal: Point2,
    raw_left_world: &[Point2],
    raw_right_world: &[Point2],
    fallback_left: Point2,
    fallback_right: Point2,
) -> (Point2, Point2, Point2, i32) {
    let normal = normalize(normal, [1.0, 0.0]);
    for window in [Some(64_usize), Some(128), Some(256), None] {
        let left_indices =
            window.map(|value| local_segment_indices(raw_left_world, fallback_left, value));
        let right_indices =
            window.map(|value| local_segment_indices(raw_right_world, fallback_right, value));
        let left_params =
            line_intersection_params(origin, normal, raw_left_world, left_indices.as_deref());
        let right_params =
            line_intersection_params(origin, normal, raw_right_world, right_indices.as_deref());
        if left_params.is_empty() || right_params.is_empty() {
            continue;
        }
        let mut best_pair: Option<(f64, f64)> = None;
        let mut best_score = f64::INFINITY;
        for t_left in &left_params {
            for t_right in &right_params {
                let width = (t_right - t_left).abs();
                if width <= 0.10 {
                    continue;
                }
                let candidate_left = point_add(origin, point_scale(normal, *t_left));
                let candidate_right = point_add(origin, point_scale(normal, *t_right));
                let anchor_dist = distance(candidate_left, fallback_left)
                    + distance(candidate_right, fallback_right);
                let brackets_origin =
                    (*t_left <= 1e-6 && 1e-6 <= *t_right) || (*t_right <= 1e-6 && 1e-6 <= *t_left);
                let midpoint_offset = (0.5 * (*t_left + *t_right)).abs();
                let score = if brackets_origin { 0.0 } else { 1_000_000.0 }
                    + 2.0 * anchor_dist
                    + 0.10 * midpoint_offset
                    + 0.01 * width;
                if score < best_score {
                    best_score = score;
                    best_pair = Some((*t_left, *t_right));
                }
            }
        }
        if let Some((t_left, t_right)) = best_pair {
            let left = point_add(origin, point_scale(normal, t_left));
            let right = point_add(origin, point_scale(normal, t_right));
            let chord = point_sub(right, left);
            if hypot(chord) > 1e-12 {
                return (left, right, normalize(chord, normal), 0);
            }
        }
    }
    let left = project_point_to_closed_polyline(fallback_left, raw_left_world);
    let right = project_point_to_closed_polyline(fallback_right, raw_right_world);
    let chord = point_sub(right, left);
    (left, right, normalize(chord, normal), 1)
}

fn normal_line_section_open(
    origin: Point2,
    normal: Point2,
    raw_left_world: &[Point2],
    raw_right_world: &[Point2],
    fallback_left: Point2,
    fallback_right: Point2,
) -> (Point2, Point2, Point2, i32) {
    let normal = normalize(normal, [1.0, 0.0]);
    for window in [Some(64_usize), Some(128), Some(256), None] {
        let left_indices =
            window.map(|value| local_segment_indices_open(raw_left_world, fallback_left, value));
        let right_indices =
            window.map(|value| local_segment_indices_open(raw_right_world, fallback_right, value));
        let left_params =
            line_intersection_params_open(origin, normal, raw_left_world, left_indices.as_deref());
        let right_params = line_intersection_params_open(
            origin,
            normal,
            raw_right_world,
            right_indices.as_deref(),
        );
        if left_params.is_empty() || right_params.is_empty() {
            continue;
        }
        let mut best_pair: Option<(f64, f64)> = None;
        let mut best_score = f64::INFINITY;
        for t_left in &left_params {
            for t_right in &right_params {
                let width = (t_right - t_left).abs();
                if width <= 0.10 {
                    continue;
                }
                let candidate_left = point_add(origin, point_scale(normal, *t_left));
                let candidate_right = point_add(origin, point_scale(normal, *t_right));
                let anchor_dist = distance(candidate_left, fallback_left)
                    + distance(candidate_right, fallback_right);
                let brackets_origin =
                    (*t_left <= 1e-6 && 1e-6 <= *t_right) || (*t_right <= 1e-6 && 1e-6 <= *t_left);
                let midpoint_offset = (0.5 * (*t_left + *t_right)).abs();
                let score = if brackets_origin { 0.0 } else { 1_000_000.0 }
                    + 2.0 * anchor_dist
                    + 0.10 * midpoint_offset
                    + 0.01 * width;
                if score < best_score {
                    best_score = score;
                    best_pair = Some((*t_left, *t_right));
                }
            }
        }
        if let Some((t_left, t_right)) = best_pair {
            let left = point_add(origin, point_scale(normal, t_left));
            let right = point_add(origin, point_scale(normal, t_right));
            let chord = point_sub(right, left);
            if hypot(chord) > 1e-12 {
                return (left, right, normalize(chord, normal), 0);
            }
        }
    }
    let left = project_point_to_open_polyline(fallback_left, raw_left_world);
    let right = project_point_to_open_polyline(fallback_right, raw_right_world);
    let chord = point_sub(right, left);
    (left, right, normalize(chord, normal), 1)
}

fn local_segment_indices_by_progress(
    polyline_len: usize,
    progress: f64,
    window: usize,
) -> Vec<usize> {
    if polyline_len == 0 {
        return Vec::new();
    }
    let anchor = (progress.rem_euclid(1.0) * polyline_len as f64).round() as isize;
    let max_local_radius = (polyline_len / 4).max(1);
    let radius = window.min(max_local_radius) as isize;
    (-radius..=radius)
        .map(|offset| (anchor + offset).rem_euclid(polyline_len as isize) as usize)
        .collect()
}

fn local_segment_indices(polyline: &[Point2], anchor_point: Point2, window: usize) -> Vec<usize> {
    if polyline.is_empty() {
        return Vec::new();
    }
    let anchor = nearest_polyline_index(polyline, anchor_point);
    (-(window as isize)..=(window as isize))
        .map(|offset| (anchor as isize + offset).rem_euclid(polyline.len() as isize) as usize)
        .collect()
}

fn local_segment_indices_open(
    polyline: &[Point2],
    anchor_point: Point2,
    window: usize,
) -> Vec<usize> {
    if polyline.len() < 2 {
        return Vec::new();
    }
    let anchor = nearest_polyline_index(polyline, anchor_point);
    let start = anchor.saturating_sub(window);
    let end = (anchor + window).min(polyline.len() - 2);
    (start..=end).collect()
}

fn nearest_polyline_index(polyline: &[Point2], point: Point2) -> usize {
    polyline
        .iter()
        .enumerate()
        .min_by(|(_, left), (_, right)| {
            distance(**left, point).total_cmp(&distance(**right, point))
        })
        .map(|(index, _)| index)
        .unwrap_or(0)
}

fn line_intersection_params_open(
    origin: Point2,
    direction: Point2,
    polyline: &[Point2],
    segment_indices: Option<&[usize]>,
) -> Vec<f64> {
    const EPS: f64 = 1e-8;
    if polyline.len() < 2 {
        return Vec::new();
    }
    let indices = segment_indices
        .map(|values| values.to_vec())
        .unwrap_or_else(|| (0..(polyline.len() - 1)).collect::<Vec<_>>());
    let mut params = Vec::new();
    for index in indices {
        if index + 1 >= polyline.len() {
            continue;
        }
        let seg_start = polyline[index];
        let seg_vec = point_sub(polyline[index + 1], seg_start);
        let denom = cross(direction, seg_vec);
        let delta = point_sub(seg_start, origin);
        if denom.abs() <= EPS {
            if cross(delta, direction).abs() <= 1e-5 {
                let t0 = dot(point_sub(seg_start, origin), direction);
                let t1 = dot(point_sub(point_add(seg_start, seg_vec), origin), direction);
                if t0.min(t1) <= EPS && t0.max(t1) >= -EPS {
                    params.push(0.0);
                }
                params.push(t0);
                params.push(t1);
            }
            continue;
        }
        let t = cross(delta, seg_vec) / denom;
        let u = cross(delta, direction) / denom;
        if (-EPS..=1.0 + EPS).contains(&u) {
            params.push(t);
        }
    }
    if params.is_empty() {
        return params;
    }

    params.sort_by(f64::total_cmp);
    let mut deduped = Vec::with_capacity(params.len());
    for value in params {
        if deduped
            .last()
            .is_none_or(|previous: &f64| (value - *previous).abs() > 1e-5)
        {
            deduped.push(value);
        }
    }
    deduped
}

fn line_intersection_params(
    origin: Point2,
    direction: Point2,
    polyline: &[Point2],
    segment_indices: Option<&[usize]>,
) -> Vec<f64> {
    const EPS: f64 = 1e-8;
    let indices = segment_indices
        .map(|values| values.to_vec())
        .unwrap_or_else(|| (0..polyline.len()).collect::<Vec<_>>());
    let mut params = Vec::new();
    for index in indices {
        let seg_start = polyline[index % polyline.len()];
        let seg_vec = point_sub(polyline[(index + 1) % polyline.len()], seg_start);
        let denom = cross(direction, seg_vec);
        let delta = point_sub(seg_start, origin);
        if denom.abs() <= EPS {
            if cross(delta, direction).abs() <= 1e-5 {
                let t0 = dot(point_sub(seg_start, origin), direction);
                let t1 = dot(point_sub(point_add(seg_start, seg_vec), origin), direction);
                if t0.min(t1) <= EPS && t0.max(t1) >= -EPS {
                    params.push(0.0);
                }
                params.push(t0);
                params.push(t1);
            }
            continue;
        }
        let t = cross(delta, seg_vec) / denom;
        let u = cross(delta, direction) / denom;
        if (-EPS..=1.0 + EPS).contains(&u) {
            params.push(t);
        }
    }
    if params.is_empty() {
        return params;
    }

    params.sort_by(f64::total_cmp);
    let mut deduped = Vec::with_capacity(params.len());
    for value in params {
        if deduped
            .last()
            .is_none_or(|previous: &f64| (value - *previous).abs() > 1e-5)
        {
            deduped.push(value);
        }
    }
    deduped
}

fn line_intersection_candidates(
    origin: Point2,
    direction: Point2,
    polyline: &[Point2],
    segment_indices: Option<&[usize]>,
) -> Vec<LineIntersectionCandidate> {
    const EPS: f64 = 1e-8;
    let indices = segment_indices
        .map(|values| values.to_vec())
        .unwrap_or_else(|| (0..polyline.len()).collect::<Vec<_>>());
    let mut candidates = Vec::new();
    for index in indices {
        let seg_index = index % polyline.len();
        let seg_start = polyline[seg_index];
        let seg_vec = point_sub(polyline[(seg_index + 1) % polyline.len()], seg_start);
        let denom = cross(direction, seg_vec);
        let delta = point_sub(seg_start, origin);
        if denom.abs() <= EPS {
            if cross(delta, direction).abs() <= 1e-5 {
                let t0 = dot(point_sub(seg_start, origin), direction);
                let t1 = dot(point_sub(point_add(seg_start, seg_vec), origin), direction);
                if t0.min(t1) <= EPS && t0.max(t1) >= -EPS {
                    candidates.push(LineIntersectionCandidate {
                        line_t: 0.0,
                        segment_index: seg_index,
                        segment_u: 0.0,
                    });
                }
                candidates.push(LineIntersectionCandidate {
                    line_t: t0,
                    segment_index: seg_index,
                    segment_u: 0.0,
                });
                candidates.push(LineIntersectionCandidate {
                    line_t: t1,
                    segment_index: seg_index,
                    segment_u: 1.0,
                });
            }
            continue;
        }
        let t = cross(delta, seg_vec) / denom;
        let u = cross(delta, direction) / denom;
        if (-EPS..=1.0 + EPS).contains(&u) {
            candidates.push(LineIntersectionCandidate {
                line_t: t,
                segment_index: seg_index,
                segment_u: u,
            });
        }
    }
    if candidates.is_empty() {
        return candidates;
    }

    candidates.sort_by(|left, right| {
        left.line_t
            .total_cmp(&right.line_t)
            .then(left.segment_index.cmp(&right.segment_index))
    });
    let mut deduped = Vec::with_capacity(candidates.len());
    for candidate in candidates {
        if deduped
            .last()
            .is_none_or(|previous: &LineIntersectionCandidate| {
                (candidate.line_t - previous.line_t).abs() > 1e-5
                    || candidate.segment_index != previous.segment_index
            })
        {
            deduped.push(candidate);
        }
    }
    deduped
}

fn cyclic_progress_distance(left: f64, right: f64) -> f64 {
    let delta = (left - right).rem_euclid(1.0).abs();
    delta.min(1.0 - delta)
}

fn project_point_to_open_polyline(point: Point2, polyline: &[Point2]) -> Point2 {
    if polyline.is_empty() {
        return point;
    }
    if polyline.len() == 1 {
        return polyline[0];
    }
    let mut best = polyline[0];
    let mut best_dist2 = f64::INFINITY;
    for index in 0..(polyline.len() - 1) {
        let a = polyline[index];
        let b = polyline[index + 1];
        let ab = point_sub(b, a);
        let denom = dot(ab, ab).max(1e-12);
        let t = (dot(point_sub(point, a), ab) / denom).clamp(0.0, 1.0);
        let projection = point_add(a, point_scale(ab, t));
        let dist2 = dot(point_sub(point, projection), point_sub(point, projection));
        if dist2 < best_dist2 {
            best_dist2 = dist2;
            best = projection;
        }
    }
    best
}

fn project_point_to_closed_polyline(point: Point2, polyline: &[Point2]) -> Point2 {
    if polyline.is_empty() {
        return point;
    }
    let mut best = polyline[0];
    let mut best_dist2 = f64::INFINITY;
    for index in 0..polyline.len() {
        let a = polyline[index];
        let b = polyline[(index + 1) % polyline.len()];
        let ab = point_sub(b, a);
        let denom = dot(ab, ab).max(1e-12);
        let t = (dot(point_sub(point, a), ab) / denom).clamp(0.0, 1.0);
        let projection = point_add(a, point_scale(ab, t));
        let dist2 = dot(point_sub(point, projection), point_sub(point, projection));
        if dist2 < best_dist2 {
            best_dist2 = dist2;
            best = projection;
        }
    }
    best
}

fn project_point_to_closed_polyline_indices(
    point: Point2,
    polyline: &[Point2],
    segment_indices: &[usize],
) -> Point2 {
    if polyline.is_empty() {
        return point;
    }
    if segment_indices.is_empty() {
        return project_point_to_closed_polyline(point, polyline);
    }
    let mut best = polyline[segment_indices[0] % polyline.len()];
    let mut best_dist2 = f64::INFINITY;
    for index in segment_indices {
        let a = polyline[*index % polyline.len()];
        let b = polyline[(*index + 1) % polyline.len()];
        let ab = point_sub(b, a);
        let denom = dot(ab, ab).max(1e-12);
        let t = (dot(point_sub(point, a), ab) / denom).clamp(0.0, 1.0);
        let projection = point_add(a, point_scale(ab, t));
        let delta = point_sub(point, projection);
        let dist2 = dot(delta, delta);
        if dist2 < best_dist2 {
            best_dist2 = dist2;
            best = projection;
        }
    }
    best
}

fn project_points_to_closed_progress(points: &[Point2], reference: &[Point2]) -> Vec<f64> {
    let (station, _) = closed_polyline_arclength(reference);
    let total = station.last().copied().unwrap_or(0.0).max(1e-9);
    points
        .iter()
        .map(|point| {
            let mut best_dist2 = f64::INFINITY;
            let mut best_progress = 0.0;
            for index in 0..reference.len() {
                let a = reference[index];
                let b = reference[(index + 1) % reference.len()];
                let ab = point_sub(b, a);
                let denom = dot(ab, ab).max(1e-12);
                let t = (dot(point_sub(*point, a), ab) / denom).clamp(0.0, 1.0);
                let projection = point_add(a, point_scale(ab, t));
                let dist2 = dot(point_sub(*point, projection), point_sub(*point, projection));
                if dist2 < best_dist2 {
                    let next_s = if index == reference.len() - 1 {
                        total
                    } else {
                        station[index + 1]
                    };
                    best_dist2 = dist2;
                    best_progress = (station[index] + t * (next_s - station[index])) / total;
                }
            }
            best_progress.rem_euclid(1.0)
        })
        .collect()
}

fn project_points_to_closed_progress_route_window(
    points: &[Point2],
    reference: &[Point2],
    half_window: usize,
) -> Vec<f64> {
    if reference.is_empty() {
        return vec![0.0; points.len()];
    }
    let (station, _) = closed_polyline_arclength(reference);
    let total = station.last().copied().unwrap_or(0.0).max(1e-9);
    let reference_count = reference.len();
    let point_count = points.len().max(1);

    points
        .iter()
        .enumerate()
        .map(|(point_index, point)| {
            let route_index = point_index * reference_count / point_count;
            let mut best_dist2 = f64::INFINITY;
            let mut best_progress = route_index as f64 / reference_count as f64;
            for offset in 0..=(2 * half_window) {
                let index =
                    (route_index + reference_count + offset - half_window) % reference_count;
                let a = reference[index];
                let b = reference[(index + 1) % reference_count];
                let ab = point_sub(b, a);
                let denominator = dot(ab, ab).max(1e-12);
                let fraction = (dot(point_sub(*point, a), ab) / denominator).clamp(0.0, 1.0);
                let projection = point_add(a, point_scale(ab, fraction));
                let delta = point_sub(*point, projection);
                let dist2 = dot(delta, delta);
                if dist2 < best_dist2 {
                    let next_s = if index + 1 == reference_count {
                        total
                    } else {
                        station[index + 1]
                    };
                    best_dist2 = dist2;
                    best_progress = (station[index] + fraction * (next_s - station[index])) / total;
                }
            }
            best_progress.rem_euclid(1.0)
        })
        .collect()
}

fn closed_curve_rms(left: &[Point2], right: &[Point2]) -> f64 {
    if left.is_empty() || right.is_empty() {
        return 0.0;
    }
    let count = left.len().min(right.len());
    (left
        .iter()
        .zip(right)
        .take(count)
        .map(|(l, r)| {
            let d = point_sub(*l, *r);
            dot(d, d)
        })
        .sum::<f64>()
        / count as f64)
        .sqrt()
}

fn roll_usize(points: &[Point2], shift: usize) -> Vec<Point2> {
    points
        .iter()
        .enumerate()
        .map(|(index, _)| points[(index + points.len() - shift % points.len()) % points.len()])
        .collect()
}

fn median(mut values: Vec<f64>) -> f64 {
    if values.is_empty() {
        return 0.0;
    }
    values.sort_by(f64::total_cmp);
    let middle = values.len() / 2;
    if values.len() & 1 == 0 {
        0.5 * (values[middle - 1] + values[middle])
    } else {
        values[middle]
    }
}

fn midpoint(left: Point2, right: Point2) -> Point2 {
    [(left[0] + right[0]) * 0.5, (left[1] + right[1]) * 0.5]
}

fn point_add(left: Point2, right: Point2) -> Point2 {
    [left[0] + right[0], left[1] + right[1]]
}

fn point_sub(left: Point2, right: Point2) -> Point2 {
    [left[0] - right[0], left[1] - right[1]]
}

fn point_scale(point: Point2, scale: f64) -> Point2 {
    [point[0] * scale, point[1] * scale]
}

fn dot(left: Point2, right: Point2) -> f64 {
    left[0] * right[0] + left[1] * right[1]
}

fn cross(left: Point2, right: Point2) -> f64 {
    left[0] * right[1] - left[1] * right[0]
}

fn hypot(point: Point2) -> f64 {
    point[0].hypot(point[1])
}

fn distance(left: Point2, right: Point2) -> f64 {
    hypot(point_sub(left, right))
}

fn normalize(vector: Point2, fallback: Point2) -> Point2 {
    let length = hypot(vector);
    if length <= 1e-9 {
        fallback
    } else {
        [vector[0] / length, vector[1] / length]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;
    use std::path::Path;

    use crate::json::parse_json_str;

    fn oval_track() -> TrackAreaContractV1 {
        TrackAreaContractV1::new(
            "oval",
            vec![
                [0.0, 0.0],
                [20.0, 0.0],
                [30.0, 8.0],
                [20.0, 16.0],
                [0.0, 16.0],
                [-10.0, 8.0],
            ],
            vec![
                [4.0, 4.0],
                [18.0, 4.0],
                [23.0, 8.0],
                [18.0, 12.0],
                [4.0, 12.0],
                [-3.0, 8.0],
            ],
        )
    }

    fn square_sections() -> SectionsTrackViewV1 {
        SectionsTrackViewV1 {
            schema_version: SectionsTrackViewV1::SCHEMA_VERSION.to_owned(),
            view_id: "square_sections".to_owned(),
            track_id: "square".to_owned(),
            station_s_m: vec![0.0, 2.0, 3.0, 5.0],
            centerline_xy_m: vec![[0.0, 0.0], [2.0, 0.0], [2.0, 1.0], [0.0, 1.0]],
            left_boundary_xy_m: vec![[0.0, -1.0], [3.0, 0.0], [2.0, 2.0], [-1.0, 1.0]],
            right_boundary_xy_m: vec![[0.0, 1.0], [1.0, 0.0], [2.0, 0.0], [1.0, 1.0]],
            normals_xy: vec![[0.0, 1.0], [-1.0, 0.0], [0.0, -1.0], [1.0, 0.0]],
            width_left_m: vec![1.0, 2.0, 3.0, 4.0],
            width_right_m: vec![5.0, 6.0, 7.0, 8.0],
            section_dirs_xy: vec![[0.0, 1.0], [-1.0, 0.0], [0.0, -1.0], [1.0, 0.0]],
            quality_metrics: Vec::new(),
            metadata: Vec::new(),
        }
    }

    #[test]
    fn closed_station_direction_reverses_all_directional_series_consistently() {
        let mut track = oval_track();
        track.direction = Some("clockwise".to_owned());
        let source = square_sections();

        let oriented = orient_sections_for_requested_direction(source.clone(), &track);

        assert_eq!(
            closed_centerline_direction(&source.centerline_xy_m),
            Some(EffectiveTrackDirection::Counterclockwise)
        );
        assert_eq!(
            closed_centerline_direction(&oriented.centerline_xy_m),
            Some(EffectiveTrackDirection::Clockwise)
        );
        assert_eq!(oriented.centerline_xy_m[0], source.centerline_xy_m[0]);
        assert_eq!(
            oriented.centerline_xy_m,
            vec![[0.0, 0.0], [0.0, 1.0], [2.0, 1.0], [2.0, 0.0]]
        );
        assert_eq!(
            oriented.left_boundary_xy_m,
            vec![[0.0, 1.0], [1.0, 1.0], [2.0, 0.0], [1.0, 0.0]]
        );
        assert_eq!(
            oriented.right_boundary_xy_m,
            vec![[0.0, -1.0], [-1.0, 1.0], [2.0, 2.0], [3.0, 0.0]]
        );
        assert_eq!(oriented.width_left_m, vec![5.0, 8.0, 7.0, 6.0]);
        assert_eq!(oriented.width_right_m, vec![1.0, 4.0, 3.0, 2.0]);
        assert_eq!(
            oriented.normals_xy,
            vec![[0.0, -1.0], [-1.0, 0.0], [0.0, 1.0], [1.0, 0.0]]
        );
        assert_eq!(oriented.section_dirs_xy, oriented.normals_xy);
        assert_eq!(oriented.station_s_m, vec![0.0, 1.0, 3.0, 4.0]);
        assert!(oriented
            .station_s_m
            .windows(2)
            .all(|window| window[1] > window[0]));
        assert_eq!(
            metadata_str(&oriented.metadata, "requested_direction"),
            "clockwise"
        );
        assert_eq!(
            metadata_str(&oriented.metadata, "source_station_direction"),
            "counterclockwise"
        );
        assert_eq!(
            metadata_str(&oriented.metadata, "effective_direction"),
            "clockwise"
        );
        assert!(metadata_bool(&oriented.metadata, "direction_reversed"));
    }

    #[test]
    fn matching_closed_station_direction_preserves_station_arrays() {
        let mut track = oval_track();
        track.direction = Some("counterclockwise".to_owned());
        let source = square_sections();

        let oriented = orient_sections_for_requested_direction(source.clone(), &track);

        assert_eq!(oriented.centerline_xy_m, source.centerline_xy_m);
        assert_eq!(oriented.left_boundary_xy_m, source.left_boundary_xy_m);
        assert_eq!(oriented.right_boundary_xy_m, source.right_boundary_xy_m);
        assert_eq!(oriented.width_left_m, source.width_left_m);
        assert_eq!(oriented.width_right_m, source.width_right_m);
        assert_eq!(oriented.normals_xy, source.normals_xy);
        assert_eq!(oriented.section_dirs_xy, source.section_dirs_xy);
        assert_eq!(oriented.station_s_m, source.station_s_m);
        assert!(!metadata_bool(&oriented.metadata, "direction_reversed"));
    }

    #[test]
    fn degenerate_closed_station_direction_does_not_reverse_or_panic() {
        let mut track = oval_track();
        track.direction = Some("clockwise".to_owned());
        let mut source = square_sections();
        source.centerline_xy_m = vec![[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [3.0, 0.0]];

        let oriented = orient_sections_for_requested_direction(source.clone(), &track);

        assert_eq!(oriented.centerline_xy_m, source.centerline_xy_m);
        assert_eq!(
            metadata_str(&oriented.metadata, "requested_direction"),
            "clockwise"
        );
        assert!(!metadata_bool(&oriented.metadata, "direction_reversed"));
        assert!(oriented
            .metadata
            .iter()
            .all(|(key, _)| key != "effective_direction"));
    }

    #[test]
    fn open_station_direction_preserves_one_way_order() {
        let mut track = oval_track();
        track.trajectory_mode = "open".to_owned();
        track.direction = Some("clockwise".to_owned());
        let source = square_sections();

        let oriented = orient_sections_for_requested_direction(source.clone(), &track);

        assert_eq!(oriented, source);
    }

    #[test]
    fn production_station_builder_honors_requested_closed_direction() {
        let mut clockwise_track = oval_track();
        clockwise_track.direction = Some("clockwise".to_owned());
        let mut counterclockwise_track = clockwise_track.clone();
        counterclockwise_track.direction = Some("counterclockwise".to_owned());
        let options = FixedCenterlineStationOptions {
            sample_count: 48,
            dense_count: 480,
            ..FixedCenterlineStationOptions::default()
        };

        let clockwise = build_production_sections_track_view(&clockwise_track, &options);
        let counterclockwise =
            build_production_sections_track_view(&counterclockwise_track, &options);

        assert_eq!(
            closed_centerline_direction(&clockwise.centerline_xy_m),
            Some(EffectiveTrackDirection::Clockwise)
        );
        assert_eq!(
            closed_centerline_direction(&counterclockwise.centerline_xy_m),
            Some(EffectiveTrackDirection::Counterclockwise)
        );
        assert_ne!(clockwise.centerline_xy_m, counterclockwise.centerline_xy_m);
        assert_ne!(clockwise.view_id, counterclockwise.view_id);
        assert_ne!(
            crate::contracts::sections_track_view_hash_v1(&clockwise),
            crate::contracts::sections_track_view_hash_v1(&counterclockwise)
        );

        let station_count = clockwise.centerline_xy_m.len();
        assert_eq!(station_count, counterclockwise.centerline_xy_m.len());
        let clockwise_curvature = view_kappa_1pm(&clockwise);
        let counterclockwise_curvature = view_kappa_1pm(&counterclockwise);
        for index in 0..station_count {
            let opposite_index = (station_count - index) % station_count;
            assert_eq!(
                clockwise.centerline_xy_m[index],
                counterclockwise.centerline_xy_m[opposite_index]
            );
            assert_eq!(
                clockwise.left_boundary_xy_m[index],
                counterclockwise.right_boundary_xy_m[opposite_index]
            );
            assert_eq!(
                clockwise.right_boundary_xy_m[index],
                counterclockwise.left_boundary_xy_m[opposite_index]
            );
            assert!(
                (clockwise_curvature[index] + counterclockwise_curvature[opposite_index]).abs()
                    <= 1e-10
            );
        }
    }

    #[test]
    fn asymmetric_loop_production_sections_are_exactly_direction_reversible() {
        let mut clockwise_track = read_track_area_contract(&crate_path(
            "tests/public-fixtures/asymmetric-loop-track-area-v1.json",
        ));
        clockwise_track.direction = Some("clockwise".to_owned());
        let mut counterclockwise_track = clockwise_track.clone();
        counterclockwise_track.direction = Some("counterclockwise".to_owned());
        let options = FixedCenterlineStationOptions {
            sample_count: 64,
            dense_count: 512,
            ..FixedCenterlineStationOptions::default()
        };

        let clockwise = build_production_sections_track_view(&clockwise_track, &options);
        let counterclockwise =
            build_production_sections_track_view(&counterclockwise_track, &options);
        let clockwise_kappa = view_kappa_1pm(&clockwise);
        let counterclockwise_kappa = view_kappa_1pm(&counterclockwise);

        assert_eq!(clockwise.centerline_xy_m.len(), 64);
        assert_eq!(
            clockwise.centerline_xy_m.len(),
            counterclockwise.centerline_xy_m.len()
        );
        for index in 0..clockwise.centerline_xy_m.len() {
            let reverse_index =
                (clockwise.centerline_xy_m.len() - index) % clockwise.centerline_xy_m.len();
            assert!(
                distance(
                    clockwise.centerline_xy_m[index],
                    counterclockwise.centerline_xy_m[reverse_index],
                ) <= 1.0e-9
            );
            assert!(
                distance(
                    clockwise.left_boundary_xy_m[index],
                    counterclockwise.right_boundary_xy_m[reverse_index],
                ) <= 1.0e-9
            );
            assert!(
                distance(
                    clockwise.right_boundary_xy_m[index],
                    counterclockwise.left_boundary_xy_m[reverse_index],
                ) <= 1.0e-9
            );
            assert!(
                (clockwise.width_left_m[index] - counterclockwise.width_right_m[reverse_index])
                    .abs()
                    <= 1.0e-9
            );
            assert!(
                (clockwise.width_right_m[index] - counterclockwise.width_left_m[reverse_index])
                    .abs()
                    <= 1.0e-9
            );
            assert!(
                (clockwise_kappa[index] + counterclockwise_kappa[reverse_index]).abs() <= 1.0e-9
            );
        }
    }

    fn open_lab_track_from_center(
        track_id: &str,
        center: Vec<Point2>,
        half_width_m: f64,
    ) -> TrackAreaContractV1 {
        let (_, tangents) = right_normals_world_open(&center);
        let mut left = Vec::with_capacity(center.len());
        let mut right = Vec::with_capacity(center.len());
        for (point, tangent) in center.iter().zip(tangents) {
            let right_normal = [tangent[1], -tangent[0]];
            left.push(point_sub(*point, point_scale(right_normal, half_width_m)));
            right.push(point_add(*point, point_scale(right_normal, half_width_m)));
        }
        let mut track = TrackAreaContractV1::new(track_id, left.clone(), right.clone());
        track.trajectory_mode = "open".to_owned();
        track.start_finish_xy_m = Some(crate::contracts::StartFinish {
            p1_m: left[0],
            p2_m: right[0],
        });
        track.finish_line_xy_m = Some(crate::contracts::StartFinish {
            p1_m: *left.last().unwrap(),
            p2_m: *right.last().unwrap(),
        });
        track.metadata = vec![("fixture".to_owned(), track_id.into())];
        track
    }

    fn append_arc_points(
        points: &mut Vec<Point2>,
        center: Point2,
        radius_m: f64,
        start_angle_rad: f64,
        end_angle_rad: f64,
        segment_count: usize,
        clockwise: bool,
        skip_first: bool,
    ) {
        let mut delta = end_angle_rad - start_angle_rad;
        if clockwise {
            while delta >= 0.0 {
                delta -= 2.0 * std::f64::consts::PI;
            }
        } else {
            while delta <= 0.0 {
                delta += 2.0 * std::f64::consts::PI;
            }
        }

        let start_index = if skip_first { 1 } else { 0 };
        for index in start_index..=segment_count {
            let t = index as f64 / segment_count as f64;
            let angle = start_angle_rad + delta * t;
            points.push([
                center[0] + radius_m * angle.cos(),
                center[1] + radius_m * angle.sin(),
            ]);
        }
    }

    fn open_straight_lab_track() -> TrackAreaContractV1 {
        open_lab_track_from_center(
            "open_straight_lab_v1",
            (0..=30).map(|index| [index as f64 * 4.0, 0.0]).collect(),
            4.0,
        )
    }

    fn open_s_bend_lab_track() -> TrackAreaContractV1 {
        let sample_count = 34;
        let mut center = Vec::with_capacity(sample_count);
        for index in 0..sample_count {
            let t = index as f64 / (sample_count - 1) as f64;
            center.push([116.0 * t, 18.0 * (2.0 * std::f64::consts::PI * t).sin()]);
        }
        open_lab_track_from_center("open_s_bend_lab_v1", center, 4.5)
    }

    fn open_chicane_lab_track() -> TrackAreaContractV1 {
        let center = vec![
            [0.0, 0.0],
            [25.0, 0.0],
            [45.0, 18.0],
            [70.0, -18.0],
            [95.0, -18.0],
            [120.0, 0.0],
            [150.0, 0.0],
        ];
        open_lab_track_from_center("open_chicane_lab_v1", center, 4.2)
    }

    fn open_8gp_self_crossing_lab_track() -> TrackAreaContractV1 {
        let sample_count = 49;
        let start_t = -0.35 * std::f64::consts::PI;
        let end_t = 1.35 * std::f64::consts::PI;
        let mut center = Vec::with_capacity(sample_count);
        for index in 0..sample_count {
            let t = start_t + (end_t - start_t) * index as f64 / (sample_count - 1) as f64;
            center.push([60.0 * t.sin(), 32.0 * t.sin() * t.cos()]);
        }
        let mut track = open_lab_track_from_center("open_8gp_self_crossing_lab_v1", center, 4.0);
        track.metadata.push((
            "corridor_topology".to_owned(),
            "self_crossing_open_corridor".into(),
        ));
        track
    }

    fn closed_8gp_self_crossing_gp_track() -> TrackAreaContractV1 {
        let sample_count = 193;
        let start_t = 0.08;
        let half_width_m = 6.0;
        let x_radius_m = 1200.0;
        let y_radius_m = 700.0;
        let mut center = Vec::with_capacity(sample_count);
        for index in 0..sample_count {
            let t = start_t + 2.0 * std::f64::consts::PI * index as f64 / sample_count as f64;
            center.push([x_radius_m * t.sin(), y_radius_m * t.sin() * t.cos()]);
        }
        let (_, tangents) = right_normals_world(&center);
        let mut left = Vec::with_capacity(center.len());
        let mut right = Vec::with_capacity(center.len());
        for (point, tangent) in center.iter().zip(tangents) {
            let right_normal = [tangent[1], -tangent[0]];
            left.push(point_sub(*point, point_scale(right_normal, half_width_m)));
            right.push(point_add(*point, point_scale(right_normal, half_width_m)));
        }
        let mut track = TrackAreaContractV1::new("closed_8gp_self_crossing_gp_v1", left, right);
        track.trajectory_mode = "closed".to_owned();
        track.metadata = vec![
            (
                "fixture".to_owned(),
                "closed_8gp_self_crossing_gp_v1".into(),
            ),
            (
                "corridor_topology".to_owned(),
                "self_crossing_closed_corridor".into(),
            ),
            ("benchmark_scale".to_owned(), "gp_real_size".into()),
            ("nominal_width_m".to_owned(), 12.0.into()),
        ];
        track.start_finish_xy_m = Some(crate::contracts::StartFinish {
            p1_m: track.left_boundary_xy_m[0],
            p2_m: track.right_boundary_xy_m[0],
        });
        track
    }

    fn closed_8gp_self_crossing_gymkhana_track() -> TrackAreaContractV1 {
        let cone_spacing_m: f64 = 26.0;
        let cone_radius_m: f64 = 0.15;
        let cone_inner_clearance_m: f64 = 0.05;
        let cone_inner_radius_m: f64 = cone_radius_m + cone_inner_clearance_m;
        let half_width_m: f64 = 1.5;
        let cone_turn_radius_m: f64 = cone_inner_radius_m + half_width_m;
        let left_cone = [-0.5 * cone_spacing_m, 0.0];
        let right_cone = [0.5 * cone_spacing_m, 0.0];
        let tangent_slope = cone_turn_radius_m
            / ((0.25 * cone_spacing_m * cone_spacing_m - cone_turn_radius_m * cone_turn_radius_m)
                .sqrt());
        let tangent_angle = (std::f64::consts::FRAC_PI_2 - tangent_slope.atan()).abs();

        let mut center = Vec::with_capacity(128);
        append_arc_points(
            &mut center,
            left_cone,
            cone_turn_radius_m,
            tangent_angle,
            -tangent_angle,
            40,
            false,
            false,
        );
        append_arc_points(
            &mut center,
            right_cone,
            cone_turn_radius_m,
            std::f64::consts::PI - tangent_angle,
            std::f64::consts::PI + tangent_angle,
            40,
            true,
            false,
        );
        let (_, tangents) = right_normals_world(&center);
        let mut left = Vec::with_capacity(center.len());
        let mut right = Vec::with_capacity(center.len());
        for (point, tangent) in center.iter().zip(tangents) {
            let right_normal = [tangent[1], -tangent[0]];
            left.push(point_sub(*point, point_scale(right_normal, half_width_m)));
            right.push(point_add(*point, point_scale(right_normal, half_width_m)));
        }
        let mut track = TrackAreaContractV1::new("gymkhana_8gp_two_cone_closed_v1", left, right);
        track.trajectory_mode = "closed".to_owned();
        track.metadata = vec![
            (
                "fixture".to_owned(),
                "gymkhana_8gp_two_cone_closed_v1".into(),
            ),
            (
                "corridor_topology".to_owned(),
                "self_crossing_closed_corridor".into(),
            ),
            ("benchmark_scale".to_owned(), "gymkhana_gp8".into()),
            ("nominal_width_m".to_owned(), (2.0 * half_width_m).into()),
            ("cone_spacing_m".to_owned(), cone_spacing_m.into()),
            ("cone_radius_m".to_owned(), cone_radius_m.into()),
            (
                "cone_inner_clearance_m".to_owned(),
                cone_inner_clearance_m.into(),
            ),
            ("cone_inner_radius_m".to_owned(), cone_inner_radius_m.into()),
            ("cone_turn_radius_m".to_owned(), cone_turn_radius_m.into()),
        ];
        track.start_finish_xy_m = Some(crate::contracts::StartFinish {
            p1_m: track.left_boundary_xy_m[0],
            p2_m: track.right_boundary_xy_m[0],
        });
        track
    }

    fn rough_closed_8gp_user_drawn_track() -> TrackAreaContractV1 {
        let left = vec![
            [2.669239994441613, 4.0283707506539495],
            [2.0949041369959898, 5.58268173217919],
            [2.6016720254656716, 6.630438907775342],
            [4.815079984973313, 6.630438907775342],
            [6.065458728058601, 5.0761281518376045],
            [9.681079581182102, 3.589467704314498],
            [10.035633655353228, 7.019016540362901],
            [8.008214696719484, 8.201249810463905],
            [6.419996559929494, 6.511638212109402],
            [4.426370158108845, 4.096021284656084],
        ];
        let right = vec![
            [1.537282621635536, 3.6736186106549615],
            [1.30080307695736, 6.056235019019125],
            [2.0778264862368223, 7.069341954114794],
            [4.933501242458773, 7.610545774956863],
            [6.572209923053042, 6.089235086933273],
            [8.447778263268477, 5.143778685839738],
            [9.173948045657166, 6.368746801383],
            [8.126652647680185, 6.959451062477546],
            [5.169997255024686, 3.555642662900934],
            [2.871943901369439, 2.9979389208955425],
        ];
        let mut track = TrackAreaContractV1::new("rough_user_drawn_8gp_closed_v1", left, right);
        track.trajectory_mode = "closed".to_owned();
        track.metadata = vec![
            (
                "fixture".to_owned(),
                "rough_user_drawn_8gp_closed_v1".into(),
            ),
            (
                "corridor_topology".to_owned(),
                "self_crossing_closed_corridor".into(),
            ),
            ("benchmark_scale".to_owned(), "user_gymkhana_sketch".into()),
        ];
        track.start_finish_xy_m = Some(crate::contracts::StartFinish {
            p1_m: [3.5647012418880246, 5.261754633593767],
            p2_m: [2.365176012899757, 2.5755357957194134],
        });
        track
    }

    fn assert_open_station_contract(
        track: &TrackAreaContractV1,
        sample_count: usize,
        first_last_gap_min_m: f64,
    ) {
        let options = FixedCenterlineStationOptions {
            sample_count,
            dense_count: 480,
            density_smooth_window: 9,
            density_max_adjacent_ratio: 1.35,
            target_spacing_max_adjacent_ratio: 1.45,
            target_spacing_metric: "hybrid_area_centerline".to_owned(),
            ..FixedCenterlineStationOptions::default()
        };

        let view = build_production_sections_track_view(track, &options);
        assert_eq!(
            legacy_station_builder_for_track(track),
            ProductionStationBuilder::OpenAreaStation
        );
        assert_eq!(metadata_str(&view.metadata, "trajectory_mode"), "open");
        assert_eq!(
            metadata_str(&view.metadata, "station_geometry_source"),
            "universal_area_route_pair"
        );
        assert_eq!(
            metadata_str(&view.metadata, "open_repair_progress_source"),
            "shared_dtw_frame_progress"
        );
        assert_eq!(view.centerline_xy_m.len(), sample_count);
        assert_eq!(view.station_s_m.first().copied().unwrap_or(f64::NAN), 0.0);
        assert!(view.station_s_m.windows(2).all(|pair| pair[0] < pair[1]));
        assert!(
            metadata_f64(&view.metadata, "first_last_gap_m") > first_last_gap_min_m,
            "open stationing must preserve the fixture endpoint gap"
        );
        assert_eq!(
            metadata_i64(&view.metadata, "adjacent_section_crossing_count_horizon2"),
            0,
            "pairing mode={} roll={} paired_crossings={} final_pairs={:?}",
            metadata_str(&view.metadata, "dtw_pairing_mode"),
            metadata_i64(&view.metadata, "dtw_alignment_roll_bias"),
            metadata_i64(
                &view.metadata,
                "dtw_alignment_roll_bias_selected_crossing_count"
            ),
            station_horizon_crossing_pairs_closed(
                &view.left_boundary_xy_m,
                &view.right_boundary_xy_m,
                2
            )
        );
        assert_eq!(
            metadata_i64(&view.metadata, "station_raw_boundary_crossing_count"),
            0
        );
        assert!(
            metadata_i64(&view.metadata, "dtw_paired_right_plateau_run_max") <= 4,
            "open DTW pairing must not collapse into a fan-out plateau"
        );
        assert!(
            metadata_f64(&view.metadata, "station_spacing_adjacent_ratio_max")
                <= OPEN_STATION_SPACING_ADJACENT_RATIO_HARD_LIMIT
        );
        assert!(
            metadata_f64(&view.metadata, "cell_area_adjacent_ratio_max")
                <= OPEN_STATION_CELL_AREA_ADJACENT_RATIO_HARD_LIMIT
        );
        assert!(
            (view.station_s_m.last().copied().unwrap_or(0.0)
                - metadata_f64(&view.metadata, "total_length_m"))
            .abs()
                <= 1.0e-9
        );
    }

    fn open_concave_kink_crossing_lab_track() -> TrackAreaContractV1 {
        let scale = 4.0;
        let left = vec![
            [0.0, 0.0],
            [0.0, 38.0],
            [18.0, 62.0],
            [52.0, 66.0],
            [64.0, 48.0],
            [45.0, 39.0],
            [34.0, 15.0],
        ]
        .into_iter()
        .map(|point| [point[0] * scale, point[1] * scale])
        .collect::<Vec<_>>();
        let right = vec![
            [8.0, 0.0],
            [8.0, 34.0],
            [24.0, 53.0],
            [47.0, 56.0],
            [52.0, 49.0],
            [36.0, 34.0],
            [27.0, 9.0],
        ]
        .into_iter()
        .map(|point| [point[0] * scale, point[1] * scale])
        .collect::<Vec<_>>();
        let mut track = TrackAreaContractV1::new(
            "open_concave_kink_crossing_lab_v1",
            left.clone(),
            right.clone(),
        );
        track.trajectory_mode = "open".to_owned();
        track.start_finish_xy_m = Some(crate::contracts::StartFinish {
            p1_m: left[0],
            p2_m: right[0],
        });
        track.finish_line_xy_m = Some(crate::contracts::StartFinish {
            p1_m: *left.last().unwrap(),
            p2_m: *right.last().unwrap(),
        });
        track.metadata = vec![(
            "fixture".to_owned(),
            "open_concave_kink_crossing_lab_v1".into(),
        )];
        track
    }

    #[allow(dead_code)]
    #[derive(Debug)]
    struct StationParityMetrics {
        max_station_s_delta_m: f64,
        rms_centerline_delta_m: f64,
        max_centerline_delta_m: f64,
        max_width_left_delta_m: f64,
        max_width_right_delta_m: f64,
        max_section_dir_delta: f64,
        max_section_dir_derivative_delta: f64,
        max_kappa_delta_1pm: f64,
        production_clamped_count: i64,
        worst_centerline_index: usize,
        worst_centerline_production: Point2,
        worst_centerline_python_generated: Point2,
    }

    fn crate_path(relative_path: &str) -> std::path::PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR")).join(relative_path)
    }

    fn read_track_area_contract(path: &Path) -> TrackAreaContractV1 {
        let body = fs::read_to_string(path)
            .unwrap_or_else(|error| panic!("failed to read {}: {error}", path.display()));
        let value = parse_json_str(&body).unwrap();
        TrackAreaContractV1::from_json(&value).unwrap()
    }

    fn metadata_i64(metadata: &JsonObject, key: &str) -> i64 {
        metadata
            .iter()
            .find_map(|(entry_key, value)| {
                (entry_key == key).then_some(match value {
                    JsonValue::Integer(value) => *value,
                    JsonValue::Number(value) => *value as i64,
                    _ => 0,
                })
            })
            .unwrap_or(0)
    }

    fn metadata_f64(metadata: &JsonObject, key: &str) -> f64 {
        metadata
            .iter()
            .find_map(|(entry_key, value)| {
                (entry_key == key).then_some(match value {
                    JsonValue::Integer(value) => *value as f64,
                    JsonValue::Number(value) => *value,
                    _ => 0.0,
                })
            })
            .unwrap_or(0.0)
    }

    fn metadata_str<'a>(metadata: &'a JsonObject, key: &str) -> &'a str {
        metadata
            .iter()
            .find_map(|(entry_key, value)| {
                (entry_key == key).then_some(match value {
                    JsonValue::String(value) => value.as_str(),
                    _ => "",
                })
            })
            .unwrap_or("")
    }

    fn metadata_bool(metadata: &JsonObject, key: &str) -> bool {
        metadata
            .iter()
            .find_map(|(entry_key, value)| {
                (entry_key == key).then_some(match value {
                    JsonValue::Bool(value) => *value,
                    _ => false,
                })
            })
            .unwrap_or(false)
    }

    fn metadata_value<'a>(metadata: &'a JsonObject, key: &str) -> &'a JsonValue {
        metadata
            .iter()
            .find_map(|(entry_key, value)| (entry_key == key).then_some(value))
            .unwrap_or_else(|| panic!("missing metadata key {key}"))
    }

    fn view_kappa_1pm(view: &SectionsTrackViewV1) -> Vec<f64> {
        three_point_curvature(&view.centerline_xy_m)
    }

    fn station_parity_metrics(
        production: &SectionsTrackViewV1,
        python_generated: &SectionsTrackViewV1,
    ) -> StationParityMetrics {
        let count = production
            .centerline_xy_m
            .len()
            .min(python_generated.centerline_xy_m.len());
        assert!(count > 0, "station parity requires non-empty views");

        let production_kappa = view_kappa_1pm(production);
        let python_generated_kappa = view_kappa_1pm(python_generated);
        let (_, production_segment_lengths) =
            closed_polyline_arclength(&production.centerline_xy_m);
        let (_, python_generated_segment_lengths) =
            closed_polyline_arclength(&python_generated.centerline_xy_m);
        let mut squared_centerline_delta = 0.0;
        let mut max_station_s_delta_m: f64 = 0.0;
        let mut max_centerline_delta_m = 0.0;
        let mut max_width_left_delta_m: f64 = 0.0;
        let mut max_width_right_delta_m: f64 = 0.0;
        let mut max_section_dir_delta: f64 = 0.0;
        let mut max_section_dir_derivative_delta: f64 = 0.0;
        let mut max_kappa_delta_1pm: f64 = 0.0;
        let mut worst_centerline_index = 0;

        for index in 0..count {
            max_station_s_delta_m = max_station_s_delta_m
                .max((production.station_s_m[index] - python_generated.station_s_m[index]).abs());
            let centerline_delta = distance(
                production.centerline_xy_m[index],
                python_generated.centerline_xy_m[index],
            );
            squared_centerline_delta += centerline_delta * centerline_delta;
            if centerline_delta > max_centerline_delta_m {
                max_centerline_delta_m = centerline_delta;
                worst_centerline_index = index;
            }
            max_width_left_delta_m = max_width_left_delta_m
                .max((production.width_left_m[index] - python_generated.width_left_m[index]).abs());
            max_width_right_delta_m = max_width_right_delta_m.max(
                (production.width_right_m[index] - python_generated.width_right_m[index]).abs(),
            );
            max_section_dir_delta = max_section_dir_delta.max(distance(
                production.section_dirs_xy[index],
                python_generated.section_dirs_xy[index],
            ));
            let next = (index + 1) % count;
            let production_ds = production_segment_lengths[index];
            let python_generated_ds = python_generated_segment_lengths[index];
            if production_ds > 1e-9 && python_generated_ds > 1e-9 {
                let production_derivative = point_scale(
                    point_sub(
                        production.section_dirs_xy[next],
                        production.section_dirs_xy[index],
                    ),
                    1.0 / production_ds,
                );
                let python_generated_derivative = point_scale(
                    point_sub(
                        python_generated.section_dirs_xy[next],
                        python_generated.section_dirs_xy[index],
                    ),
                    1.0 / python_generated_ds,
                );
                max_section_dir_derivative_delta = max_section_dir_derivative_delta
                    .max(distance(production_derivative, python_generated_derivative));
            }
            max_kappa_delta_1pm = max_kappa_delta_1pm
                .max((production_kappa[index] - python_generated_kappa[index]).abs());
        }

        StationParityMetrics {
            max_station_s_delta_m,
            rms_centerline_delta_m: (squared_centerline_delta / count as f64).sqrt(),
            max_centerline_delta_m,
            max_width_left_delta_m,
            max_width_right_delta_m,
            max_section_dir_delta,
            max_section_dir_derivative_delta,
            max_kappa_delta_1pm,
            production_clamped_count: metadata_i64(
                &production.metadata,
                "centerline_projection_clamped_count",
            ),
            worst_centerline_index,
            worst_centerline_production: production.centerline_xy_m[worst_centerline_index],
            worst_centerline_python_generated: python_generated.centerline_xy_m
                [worst_centerline_index],
        }
    }

    #[derive(Debug)]
    struct SectionNormalAngleStats {
        p95_deg: f64,
    }

    fn open_section_normal_angle_stats(view: &SectionsTrackViewV1) -> SectionNormalAngleStats {
        let angles = view
            .centerline_xy_m
            .iter()
            .enumerate()
            .map(|(index, _)| {
                let tangent = if view.centerline_xy_m.len() <= 1 {
                    [1.0, 0.0]
                } else if index == 0 {
                    normalize(
                        point_sub(view.centerline_xy_m[1], view.centerline_xy_m[0]),
                        [1.0, 0.0],
                    )
                } else if index + 1 == view.centerline_xy_m.len() {
                    normalize(
                        point_sub(view.centerline_xy_m[index], view.centerline_xy_m[index - 1]),
                        [1.0, 0.0],
                    )
                } else {
                    normalize(
                        point_sub(
                            view.centerline_xy_m[index + 1],
                            view.centerline_xy_m[index - 1],
                        ),
                        [1.0, 0.0],
                    )
                };
                let normal = [tangent[1], -tangent[0]];
                let section_dir = normalize(view.section_dirs_xy[index], [1.0, 0.0]);
                dot(section_dir, normal)
                    .abs()
                    .clamp(0.0, 1.0)
                    .acos()
                    .to_degrees()
            })
            .collect::<Vec<_>>();
        SectionNormalAngleStats {
            p95_deg: percentile(angles, 95.0),
        }
    }

    #[test]
    fn builds_paired_sections_with_requested_station_count() {
        let options = StationBuilderOptions {
            sample_count: 32,
            ..StationBuilderOptions::default()
        };
        let view = build_sections_track_view(&oval_track(), &options);

        assert_eq!(view.left_boundary_xy_m.len(), 32);
        assert_eq!(view.right_boundary_xy_m.len(), 32);
        assert_eq!(view.centerline_xy_m.len(), 32);
        assert_eq!(view.normals_xy.len(), 32);
        assert!(view.station_s_m.windows(2).all(|pair| pair[0] <= pair[1]));
    }

    #[test]
    fn builds_canonical_area_station_view_with_python_contract_metadata() {
        let track = oval_track();
        let reference_seed = build_sections_track_view(
            &track,
            &StationBuilderOptions {
                sample_count: 32,
                ..StationBuilderOptions::default()
            },
        );
        let reference = ReferenceTrack {
            centerline_world: reference_seed.centerline_xy_m,
            width_right_m: reference_seed.width_right_m,
            width_left_m: reference_seed.width_left_m,
        };
        let options = FixedCenterlineStationOptions {
            sample_count: 32,
            dense_count: 320,
            ..FixedCenterlineStationOptions::default()
        };

        let view = build_area_station_sections_track_view(&track, &reference, &options);

        assert_eq!(
            view.view_id,
            "oval_canonical_area_station_sections_track_view_v1"
        );
        assert_eq!(
            metadata_str(&view.metadata, "source"),
            "canonical_area_station_generator"
        );
        assert_eq!(metadata_str(&view.metadata, "centerline_mode"), "fixed");
        assert_eq!(
            metadata_str(&view.metadata, "placement_mode"),
            "area_preserving_chords"
        );
        assert_eq!(
            metadata_str(&view.metadata, "station_builder"),
            "canonical_area_station_generator"
        );
        assert!(
            matches!(
                metadata_value(&view.metadata, "station_progress"),
                JsonValue::Array(values) if values.len() == 32
            ),
            "station_progress metadata must mirror Python canonical generator"
        );
        assert!(view
            .quality_metrics
            .iter()
            .any(|(key, _)| key == "area_preserving_repair_changed_count"));
        let JsonValue::Array(trace_rows) =
            metadata_value(&view.metadata, "area_repair_station_trace")
        else {
            panic!("area_repair_station_trace must be an array");
        };
        assert_eq!(trace_rows.len(), 32);
        let JsonValue::Object(first_trace_row) = &trace_rows[0] else {
            panic!("area_repair_station_trace row must be an object");
        };
        assert_eq!(
            metadata_f64(&view.metadata, "area_repair_chord_perp_epsilon_m"),
            AREA_REPAIR_CHORD_PERP_EPS_M
        );
        assert!(metadata_i64(&view.metadata, "area_repair_local_rejected_miss_count") >= 0);
        assert!(metadata_i64(&view.metadata, "area_repair_local_rejected_off_chord_count") >= 0);
        assert!(metadata_i64(&view.metadata, "area_repair_local_rejected_crossing_count") >= 0);
        assert!(
            metadata_i64(
                &view.metadata,
                "area_repair_lr_projection_rejected_topology_count"
            ) >= 0
        );
        assert!(metadata_i64(&view.metadata, "area_repair_local_reverted_off_chord_count") >= 0);
        for key in ["selected_miss", "score", "initial", "post_local", "final"] {
            assert!(
                first_trace_row
                    .iter()
                    .any(|(entry_key, _)| entry_key == key),
                "area_repair_station_trace row must include {key}"
            );
        }
    }

    #[test]
    fn production_station_builder_selects_asymmetric_loop_canonical_area_path() {
        let track = read_track_area_contract(&crate_path(
            "tests/public-fixtures/asymmetric-loop-track-area-v1.json",
        ));
        let options = FixedCenterlineStationOptions {
            sample_count: 48,
            dense_count: 384,
            ..FixedCenterlineStationOptions::default()
        };

        let view = build_production_sections_track_view(&track, &options);
        assert_eq!(
            legacy_station_builder_for_track(&track),
            ProductionStationBuilder::CanonicalAreaStation
        );
        assert_eq!(
            metadata_str(&view.metadata, "station_geometry_source"),
            "universal_area_route_pair"
        );
        assert_eq!(
            metadata_str(&view.metadata, "station_builder"),
            "universal_area_route_pair"
        );
        assert_eq!(metadata_i64(&view.metadata, "station_ray_miss_count"), 0);
        assert_eq!(
            metadata_i64(
                &view.metadata,
                "area_preserving_repair_horizon2_crossing_count"
            ),
            0
        );
    }

    #[test]
    fn production_station_builder_uses_area_by_default_for_compact_oval() {
        let track = read_track_area_contract(&crate_path(
            "tests/public-fixtures/compact-oval-track-area-v1.json",
        ));
        let options = FixedCenterlineStationOptions {
            sample_count: 48,
            dense_count: 384,
            ..FixedCenterlineStationOptions::default()
        };

        let view = build_production_sections_track_view(&track, &options);

        assert_eq!(
            legacy_station_builder_for_track(&track),
            ProductionStationBuilder::CanonicalAreaStation
        );
        assert_eq!(
            metadata_str(&view.metadata, "station_geometry_source"),
            "universal_area_route_pair"
        );
        assert_eq!(
            metadata_str(&view.metadata, "station_builder"),
            "universal_area_route_pair"
        );
        assert_eq!(metadata_i64(&view.metadata, "dtw_alignment_roll_bias"), 0);
        assert_eq!(
            metadata_i64(
                &view.metadata,
                "area_preserving_repair_horizon2_crossing_count"
            ),
            0
        );
        assert_eq!(metadata_i64(&view.metadata, "station_ray_miss_count"), 0);
    }

    #[test]
    fn open_track_uses_open_area_station_generator_by_default() {
        let track = open_s_bend_lab_track();
        let options = FixedCenterlineStationOptions {
            sample_count: 80,
            dense_count: 480,
            density_smooth_window: 9,
            density_max_adjacent_ratio: 1.35,
            target_spacing_max_adjacent_ratio: 1.45,
            target_spacing_metric: "hybrid_area_centerline".to_owned(),
            ..FixedCenterlineStationOptions::default()
        };

        let view = build_production_sections_track_view(&track, &options);
        let normal_stats = open_section_normal_angle_stats(&view);

        eprintln!("open S-bend station normal stats: {normal_stats:#?}");
        eprintln!(
            "open S-bend station quality metrics: {:#?}",
            view.quality_metrics
        );

        assert_eq!(
            legacy_station_builder_for_track(&track),
            ProductionStationBuilder::OpenAreaStation
        );
        assert_eq!(
            metadata_str(&view.metadata, "station_geometry_source"),
            "universal_area_route_pair"
        );
        assert_eq!(metadata_str(&view.metadata, "trajectory_mode"), "open");
        assert_eq!(
            metadata_str(&view.metadata, "open_repair_progress_source"),
            "shared_dtw_frame_progress"
        );
        assert!(
            metadata_i64(&view.metadata, "dtw_paired_right_plateau_run_max") <= 4,
            "open S-bend DTW pairing must not collapse into a fan-out plateau"
        );
        assert_eq!(view.centerline_xy_m.len(), 80);
        assert_eq!(view.station_s_m.first().copied().unwrap_or(f64::NAN), 0.0);
        assert!(view.station_s_m.windows(2).all(|pair| pair[0] < pair[1]));
        assert!(
            metadata_f64(&view.metadata, "first_last_gap_m") > 100.0,
            "open stationing must not add a closure segment"
        );
        assert!(
            (view.station_s_m.last().copied().unwrap_or(0.0)
                - metadata_f64(&view.metadata, "total_length_m"))
            .abs()
                <= 1.0e-9
        );
        assert!(
            normal_stats.p95_deg < 10.0,
            "open station normals must stay near centerline normals: {normal_stats:#?}"
        );
        assert!(
            metadata_f64(&view.metadata, "station_spacing_adjacent_ratio_max") <= 1.60,
            "open station spacing should be bounded"
        );
        assert_eq!(
            metadata_str(&view.metadata, "target_spacing_metric"),
            "hybrid_area_centerline"
        );
        assert!(
            metadata_f64(&view.metadata, "target_area_spacing_ratio_after") <= 1.46,
            "open station area spacing should be bounded by the topology adapter"
        );
        assert!(
            metadata_f64(&view.metadata, "cell_area_adjacent_ratio_max") <= 2.25,
            "open station cell areas should be bounded"
        );
        assert_eq!(
            metadata_i64(&view.metadata, "adjacent_section_crossing_count_horizon2"),
            0
        );
        assert_eq!(
            metadata_i64(&view.metadata, "open_repair_crossing_count_after"),
            0
        );
    }

    #[test]
    fn open_auto_and_exact_counts_sample_the_same_prepared_route() {
        let track = open_s_bend_lab_track();
        let auto_options = FixedCenterlineStationOptions {
            sample_count: 160,
            ..FixedCenterlineStationOptions::default()
        };
        let auto_plan = prepare_production_station_plan(&track, &auto_options);
        let resolved_count = auto_plan.complexity().recommended_station_count;
        let mut resolved_auto_options = auto_options.clone();
        resolved_auto_options.sample_count = resolved_count;
        resolved_auto_options.dense_count = resolved_auto_options
            .dense_count
            .max((resolved_count * 8).max(320));
        let auto_view = build_production_sections_track_view_from_plan(
            &track,
            &resolved_auto_options,
            auto_plan,
        );

        let exact_options = FixedCenterlineStationOptions {
            sample_count: resolved_count,
            ..FixedCenterlineStationOptions::default()
        };
        let exact_plan = prepare_production_station_plan(&track, &exact_options);
        let exact_view =
            build_production_sections_track_view_from_plan(&track, &exact_options, exact_plan);

        assert_eq!(auto_view, exact_view);
    }

    #[test]
    fn closed_auto_and_exact_counts_sample_the_same_prepared_route() {
        let track = read_track_area_contract(&crate_path(
            "tests/public-fixtures/asymmetric-loop-track-area-v1.json",
        ));
        let auto_options = FixedCenterlineStationOptions {
            sample_count: 160,
            ..FixedCenterlineStationOptions::default()
        };
        let auto_plan = prepare_production_station_plan(&track, &auto_options);
        let resolved_count = auto_plan.complexity().recommended_station_count;
        let mut resolved_auto_options = auto_options.clone();
        resolved_auto_options.sample_count = resolved_count;
        resolved_auto_options.dense_count = resolved_auto_options
            .dense_count
            .max((resolved_count * 8).max(320));
        let auto_view = build_production_sections_track_view_from_plan(
            &track,
            &resolved_auto_options,
            auto_plan,
        );

        let exact_options = FixedCenterlineStationOptions {
            sample_count: resolved_count,
            ..FixedCenterlineStationOptions::default()
        };
        let exact_plan = prepare_production_station_plan(&track, &exact_options);
        let exact_view =
            build_production_sections_track_view_from_plan(&track, &exact_options, exact_plan);

        assert_eq!(auto_view, exact_view);
    }

    #[test]
    fn open_station_contract_fixtures_preserve_open_topology() {
        assert_open_station_contract(&open_straight_lab_track(), 80, 100.0);
        assert_open_station_contract(&open_chicane_lab_track(), 80, 100.0);
    }

    #[test]
    fn open_8gp_self_crossing_fixture_is_valid_open_corridor() {
        let track = open_8gp_self_crossing_lab_track();
        let options = FixedCenterlineStationOptions {
            sample_count: 80,
            dense_count: 640,
            target_spacing_metric: "centerline".to_owned(),
            ..FixedCenterlineStationOptions::default()
        };

        let view = build_production_sections_track_view(&track, &options);

        assert_eq!(metadata_str(&view.metadata, "trajectory_mode"), "open");
        assert_eq!(
            metadata_str(&view.metadata, "station_builder"),
            "universal_area_route_pair"
        );
        assert_eq!(
            metadata_str(&view.metadata, "station_frame_source"),
            "dtw_pairs"
        );
        assert_eq!(
            metadata_str(&view.metadata, "chord_repair"),
            "open_normal_line_dtw_boundary_fallback"
        );
        assert!(!metadata_str(&view.metadata, "dtw_pairing_mode").is_empty());
        assert_eq!(
            metadata_i64(&view.metadata, "adjacent_section_crossing_count_horizon2"),
            0
        );
        assert_eq!(view.centerline_xy_m.len(), 80);
        assert_eq!(view.station_s_m.first().copied().unwrap_or(f64::NAN), 0.0);
        assert!(view.station_s_m.windows(2).all(|pair| pair[0] < pair[1]));
        assert!(
            metadata_f64(&view.metadata, "first_last_gap_m") > 20.0,
            "8-GP fixture must remain an open route with distinct start/finish"
        );
    }

    #[test]
    fn open_8gp_area_builder_handles_self_crossing_corridor() {
        let mut track = open_8gp_self_crossing_lab_track();
        track.metadata.clear();
        let options = FixedCenterlineStationOptions {
            sample_count: 80,
            dense_count: 640,
            target_spacing_metric: "centerline".to_owned(),
            ..FixedCenterlineStationOptions::default()
        };

        let view = build_open_area_station_sections_track_view(&track, &options);

        assert_eq!(
            metadata_str(&view.metadata, "station_builder"),
            "open_area_station_generator"
        );
        assert_eq!(view.centerline_xy_m.len(), 80);
        assert_eq!(
            metadata_i64(&view.metadata, "adjacent_section_crossing_count_horizon2"),
            0
        );
        let widths = view
            .left_boundary_xy_m
            .iter()
            .zip(&view.right_boundary_xy_m)
            .map(|(left, right)| distance(*left, *right))
            .collect::<Vec<_>>();
        let min_width = widths.iter().copied().fold(f64::INFINITY, f64::min);
        let max_width = widths.iter().copied().fold(0.0, f64::max);
        assert!(
            min_width > 5.0,
            "open 8GP area width collapsed: {min_width}"
        );
        assert!(
            max_width < 11.0,
            "open 8GP area width exploded: {max_width}"
        );
    }

    #[test]
    fn open_8gp_stations_preserve_route_order_through_crossing() {
        let track = open_8gp_self_crossing_lab_track();
        let options = FixedCenterlineStationOptions {
            sample_count: 80,
            dense_count: 640,
            target_spacing_metric: "centerline".to_owned(),
            ..FixedCenterlineStationOptions::default()
        };

        let view = build_production_sections_track_view(&track, &options);
        let mut best_nonlocal_distance = f64::INFINITY;
        let mut best_pair = (0_usize, 0_usize);
        for i in 0..view.centerline_xy_m.len() {
            for j in (i + 18)..view.centerline_xy_m.len() {
                let d = distance(view.centerline_xy_m[i], view.centerline_xy_m[j]);
                if d < best_nonlocal_distance {
                    best_nonlocal_distance = d;
                    best_pair = (i, j);
                }
            }
        }

        assert!(
            best_nonlocal_distance < 6.0,
            "8-GP should contain a route crossing: best_pair={best_pair:?} distance={best_nonlocal_distance}"
        );
        assert!(
            view.station_s_m[best_pair.1] - view.station_s_m[best_pair.0] > 60.0,
            "nearby XY crossing samples must stay far apart in route progress"
        );
    }

    #[test]
    fn open_8gp_nearest_xy_is_not_progress_truth() {
        let track = open_8gp_self_crossing_lab_track();
        let options = FixedCenterlineStationOptions {
            sample_count: 80,
            dense_count: 640,
            target_spacing_metric: "centerline".to_owned(),
            ..FixedCenterlineStationOptions::default()
        };

        let view = build_production_sections_track_view(&track, &options);
        let mut nearest_later_station = (0_usize, 0_usize, f64::INFINITY);
        for i in 0..view.centerline_xy_m.len() {
            for j in (i + 18)..view.centerline_xy_m.len() {
                let d = distance(view.centerline_xy_m[i], view.centerline_xy_m[j]);
                if d < nearest_later_station.2 {
                    nearest_later_station = (i, j, d);
                }
            }
        }

        let (i, j, xy_distance_m) = nearest_later_station;
        let route_delta_m = view.station_s_m[j] - view.station_s_m[i];
        assert!(
            xy_distance_m < 6.0,
            "fixture must contain a close XY crossing pair; got {nearest_later_station:?}"
        );
        assert!(
            route_delta_m > 60.0,
            "nearest XY crossing pair must not become adjacent route progress: pair={nearest_later_station:?} route_delta_m={route_delta_m}"
        );
        assert!(
            j - i > 18,
            "nearest XY crossing pair should remain non-local in station index: pair={nearest_later_station:?}"
        );
    }

    #[test]
    fn open_8gp_left_right_do_not_swap_at_crossing() {
        let track = open_8gp_self_crossing_lab_track();
        let options = FixedCenterlineStationOptions {
            sample_count: 80,
            dense_count: 640,
            target_spacing_metric: "centerline".to_owned(),
            ..FixedCenterlineStationOptions::default()
        };

        let view = build_production_sections_track_view(&track, &options);
        for index in 0..view.centerline_xy_m.len() {
            let chord = point_sub(
                view.right_boundary_xy_m[index],
                view.left_boundary_xy_m[index],
            );
            let chord_dir = normalize(chord, [1.0, 0.0]);
            assert!(
                dot(chord_dir, view.section_dirs_xy[index]) > 0.999,
                "left/right section direction swapped at station {index}"
            );
        }
    }

    #[test]
    fn open_8gp_fact_corridor_widths_are_stable() {
        let track = open_8gp_self_crossing_lab_track();
        let options = FixedCenterlineStationOptions {
            sample_count: 80,
            dense_count: 640,
            target_spacing_metric: "centerline".to_owned(),
            ..FixedCenterlineStationOptions::default()
        };

        let view = build_production_sections_track_view(&track, &options);
        let widths = view
            .left_boundary_xy_m
            .iter()
            .zip(&view.right_boundary_xy_m)
            .map(|(left, right)| distance(*left, *right))
            .collect::<Vec<_>>();
        let min_width = widths.iter().copied().fold(f64::INFINITY, f64::min);
        let max_width = widths.iter().copied().fold(0.0, f64::max);

        assert!(
            min_width > 5.0,
            "8-GP fact corridor width collapsed: {min_width}"
        );
        assert!(
            max_width < 11.0,
            "8-GP fact corridor width exploded: {max_width}"
        );
        assert_eq!(
            metadata_i64(&view.metadata, "adjacent_section_crossing_count_horizon2"),
            0
        );
    }

    #[test]
    fn closed_8gp_self_crossing_fixture_is_valid_gp_scale_closed_corridor() {
        let track = closed_8gp_self_crossing_gp_track();
        let options = FixedCenterlineStationOptions {
            sample_count: 160,
            dense_count: 1280,
            target_spacing_metric: "centerline".to_owned(),
            ..FixedCenterlineStationOptions::default()
        };

        let view = build_production_sections_track_view(&track, &options);

        assert_eq!(
            metadata_str(&view.metadata, "station_builder"),
            "universal_area_route_pair"
        );
        assert_eq!(
            metadata_str(&view.metadata, "closed_pair_normalizer"),
            "canonical_route_anchored_arclength_dtw"
        );
        assert_eq!(view.centerline_xy_m.len(), 160);
        assert_eq!(view.station_s_m.first().copied().unwrap_or(f64::NAN), 0.0);
        assert!(view.station_s_m.windows(2).all(|pair| pair[0] < pair[1]));
        assert!(
            (5400.0..6400.0).contains(&metadata_f64(&view.metadata, "total_length_m")),
            "closed 8-GP should be GP-scale; total_length_m={}",
            metadata_f64(&view.metadata, "total_length_m")
        );
        assert_eq!(
            metadata_i64(&view.metadata, "adjacent_section_crossing_count_horizon2"),
            0
        );
    }

    #[test]
    fn closed_8gp_nearest_xy_is_not_progress_truth() {
        let track = closed_8gp_self_crossing_gp_track();
        let options = FixedCenterlineStationOptions {
            sample_count: 160,
            dense_count: 1280,
            target_spacing_metric: "centerline".to_owned(),
            ..FixedCenterlineStationOptions::default()
        };

        let view = build_production_sections_track_view(&track, &options);
        let mut nearest_later_station = (0_usize, 0_usize, f64::INFINITY);
        for i in 0..view.centerline_xy_m.len() {
            for j in (i + 24)..view.centerline_xy_m.len() {
                let route_delta = (j - i).min(view.centerline_xy_m.len() + i - j);
                if route_delta < 24 {
                    continue;
                }
                let d = distance(view.centerline_xy_m[i], view.centerline_xy_m[j]);
                if d < nearest_later_station.2 {
                    nearest_later_station = (i, j, d);
                }
            }
        }

        let (i, j, xy_distance_m) = nearest_later_station;
        let route_delta_m = view.station_s_m[j] - view.station_s_m[i];
        assert!(
            xy_distance_m < 50.0,
            "closed 8-GP fixture must contain a close non-local XY crossing pair; got {nearest_later_station:?}"
        );
        assert!(
            route_delta_m > 2000.0,
            "nearest XY crossing pair must stay far apart in route progress: pair={nearest_later_station:?} route_delta_m={route_delta_m}"
        );
    }

    #[test]
    fn closed_8gp_left_right_do_not_swap_at_crossing() {
        let track = closed_8gp_self_crossing_gp_track();
        let options = FixedCenterlineStationOptions {
            sample_count: 160,
            dense_count: 1280,
            target_spacing_metric: "centerline".to_owned(),
            ..FixedCenterlineStationOptions::default()
        };

        let view = build_production_sections_track_view(&track, &options);
        let widths = view
            .left_boundary_xy_m
            .iter()
            .zip(&view.right_boundary_xy_m)
            .map(|(left, right)| distance(*left, *right))
            .collect::<Vec<_>>();
        let min_width = widths.iter().copied().fold(f64::INFINITY, f64::min);
        let max_width = widths.iter().copied().fold(0.0, f64::max);

        assert!(min_width > 11.0, "closed 8-GP width collapsed: {min_width}");
        assert!(max_width < 13.0, "closed 8-GP width exploded: {max_width}");
        for index in 0..view.centerline_xy_m.len() {
            let chord = point_sub(
                view.right_boundary_xy_m[index],
                view.left_boundary_xy_m[index],
            );
            let chord_dir = normalize(chord, [1.0, 0.0]);
            assert!(
                dot(chord_dir, view.section_dirs_xy[index]) > 0.999,
                "left/right section direction swapped at station {index}"
            );
        }
    }

    #[test]
    fn closed_8gp_gymkhana_two_cone_fixture_is_valid_closed_corridor() {
        let track = closed_8gp_self_crossing_gymkhana_track();
        let options = FixedCenterlineStationOptions {
            sample_count: 96,
            dense_count: 768,
            target_spacing_metric: "centerline".to_owned(),
            ..FixedCenterlineStationOptions::default()
        };

        let view = build_production_sections_track_view(&track, &options);

        assert_eq!(
            metadata_str(&view.metadata, "station_builder"),
            "universal_area_route_pair"
        );
        assert_eq!(
            metadata_str(&view.metadata, "closed_pair_normalizer"),
            "canonical_route_anchored_arclength_dtw"
        );
        assert_eq!(view.centerline_xy_m.len(), 96);
        assert!(
            (60.0..65.0).contains(&metadata_f64(&view.metadata, "total_length_m")),
            "gymkhana GP8 length should be compact two-cone loops plus crossing tangents; total_length_m={}",
            metadata_f64(&view.metadata, "total_length_m")
        );
        assert_eq!(
            metadata_i64(&view.metadata, "adjacent_section_crossing_count_horizon2"),
            0,
            "pairing mode={} roll={} paired_crossings={} final_pairs={:?}",
            metadata_str(&view.metadata, "dtw_pairing_mode"),
            metadata_i64(&view.metadata, "dtw_alignment_roll_bias"),
            metadata_i64(
                &view.metadata,
                "dtw_alignment_roll_bias_selected_crossing_count"
            ),
            station_horizon_crossing_pairs_closed(
                &view.left_boundary_xy_m,
                &view.right_boundary_xy_m,
                2
            )
        );
    }

    #[test]
    fn closed_8gp_gymkhana_area_builder_handles_self_crossing_corridor() {
        let track = closed_8gp_self_crossing_gymkhana_track();
        let options = FixedCenterlineStationOptions {
            sample_count: 96,
            dense_count: 768,
            target_spacing_metric: "centerline".to_owned(),
            ..FixedCenterlineStationOptions::default()
        };
        let reference = build_midref_reference_track_from_raw_boundaries(&track);
        let view = build_area_station_sections_track_view(&track, &reference, &options);

        assert_eq!(view.centerline_xy_m.len(), 96);
        assert_eq!(
            metadata_i64(&view.metadata, "adjacent_section_crossing_count_horizon2"),
            0
        );
        assert_eq!(
            metadata_i64(
                &view.metadata,
                "area_preserving_repair_horizon2_crossing_count"
            ),
            0
        );
        let widths = view
            .left_boundary_xy_m
            .iter()
            .zip(&view.right_boundary_xy_m)
            .map(|(left, right)| distance(*left, *right))
            .collect::<Vec<_>>();
        let min_width = widths.iter().copied().fold(f64::INFINITY, f64::min);
        let max_width = widths.iter().copied().fold(0.0, f64::max);
        assert!(min_width > 2.5, "8GP area width collapsed: {min_width}");
        assert!(max_width < 7.5, "8GP area width exploded: {max_width}");
    }

    #[test]
    fn closed_rough_user_8gp_is_rejected_when_section_frame_folds() {
        let track = rough_closed_8gp_user_drawn_track();
        let mut options = FixedCenterlineStationOptions {
            dense_count: 1280,
            target_spacing_metric: "centerline".to_owned(),
            normal_repair_max_angle_deg: 20.0,
            ..FixedCenterlineStationOptions::default()
        };
        options.sample_count =
            estimate_station_complexity(&track, &options).recommended_station_count;

        let view = build_production_sections_track_view(&track, &options);
        let error = crate::station_generation::validate_station_topology_for_point_mass(&view)
            .expect_err("a folded section frame must fail the strict station contract");
        assert_eq!(error.code, "station.invalidSectionFrame");
        assert!(error
            .diagnostics
            .iter()
            .any(|(key, value)| key == "section_frame_det_sign_flip_count"
                && matches!(value, JsonValue::Integer(count) if *count > 0)));
        let max_center_to_chord_perp_error = view
            .centerline_xy_m
            .iter()
            .zip(&view.left_boundary_xy_m)
            .zip(&view.right_boundary_xy_m)
            .map(|((center, left), right)| center_to_chord_perp_error_m(*center, *left, *right))
            .fold(0.0, f64::max);
        assert!(
            max_center_to_chord_perp_error <= metadata_f64(&view.metadata, "area_repair_chord_perp_limit_m") + 1.0e-9,
            "rough 8GP station centers exceeded the configured area chord tolerance; max error={max_center_to_chord_perp_error}"
        );
        let raw_left_self = closed_polyline_self_intersection_count(&track.left_boundary_xy_m);
        let raw_right_self = closed_polyline_self_intersection_count(&track.right_boundary_xy_m);
        let fact_left_self = closed_polyline_self_intersection_count(&view.left_boundary_xy_m);
        let fact_right_self = closed_polyline_self_intersection_count(&view.right_boundary_xy_m);
        assert!(
            fact_left_self <= raw_left_self,
            "rough 8GP fact left boundary introduced extra self-intersections: raw={raw_left_self} fact={fact_left_self}"
        );
        assert!(
            fact_right_self <= raw_right_self,
            "rough 8GP fact right boundary introduced extra self-intersections: raw={raw_right_self} fact={fact_right_self}"
        );
        let raw_left_right = closed_polyline_pair_intersection_count(
            &track.left_boundary_xy_m,
            &track.right_boundary_xy_m,
        );
        let fact_left_right = closed_polyline_pair_intersection_count(
            &view.left_boundary_xy_m,
            &view.right_boundary_xy_m,
        );
        assert!(
            fact_left_right <= raw_left_right,
            "rough 8GP fact boundaries introduced extra left/right intersections: raw={raw_left_right} fact={fact_left_right}"
        );
        assert!(
            metadata_i64(&view.metadata, "adjacent_section_crossing_count_horizon2") <= 2,
            "rough 8GP should keep residual strict station crossings localized"
        );
        assert!(
            metadata_i64(
                &view.metadata,
                "area_preserving_repair_horizon2_crossing_count"
            ) <= 2,
            "rough 8GP should keep residual strict repair crossings localized"
        );
        assert!(
            metadata_f64(&view.metadata, "area_preserving_repair_angle_abs_max_deg") <= 20.0,
            "rough 8GP repair must not hide topology issues with diagonal 60-degree sections"
        );
        assert!(
            metadata_f64(
                &view.metadata,
                "area_repair_left_endpoint_projection_spacing_min_m"
            ) > 1.0e-4,
            "rough 8GP left endpoint projection spacing collapsed"
        );
        assert!(
            metadata_f64(
                &view.metadata,
                "area_repair_right_endpoint_projection_spacing_min_m"
            ) > 1.0e-4,
            "rough 8GP right endpoint projection spacing collapsed"
        );
        assert!(
            metadata_f64(&view.metadata, "area_repair_lr_projection_ratio_p95") < 8.0,
            "rough 8GP left/right progress pairing has a large p95 gap ratio: {}",
            metadata_f64(&view.metadata, "area_repair_lr_projection_ratio_p95")
        );
    }

    #[test]
    fn cancellable_station_pipeline_preserves_outputs_byte_for_byte() {
        let fixtures = vec![
            read_track_area_contract(&crate_path(
                "tests/public-fixtures/asymmetric-loop-track-area-v1.json",
            )),
            read_track_area_contract(&crate_path(
                "tests/public-fixtures/compact-oval-track-area-v1.json",
            )),
            closed_8gp_self_crossing_gymkhana_track(),
            open_8gp_self_crossing_lab_track(),
        ];
        let options = FixedCenterlineStationOptions {
            sample_count: 40,
            dense_count: 320,
            ..FixedCenterlineStationOptions::default()
        };
        let never_cancel = || false;

        for track in fixtures {
            let expected = build_production_sections_track_view(&track, &options);
            let control = StationGenerationControl::cancellable(&never_cancel);
            let plan = prepare_production_station_plan_with_control(&track, &options, control)
                .expect("never-cancelled preparation");
            let actual = build_production_sections_track_view_from_plan_with_control(
                &track, &options, plan, control,
            )
            .expect("never-cancelled station build");

            assert_eq!(actual, expected, "track_id={}", track.track_id);
        }
    }

    #[test]
    fn closed_8gp_gymkhana_nearest_xy_is_not_progress_truth() {
        let track = closed_8gp_self_crossing_gymkhana_track();
        let options = FixedCenterlineStationOptions {
            sample_count: 96,
            dense_count: 768,
            target_spacing_metric: "centerline".to_owned(),
            ..FixedCenterlineStationOptions::default()
        };

        let view = build_production_sections_track_view(&track, &options);
        let mut nearest_later_station = (0_usize, 0_usize, f64::INFINITY);
        for i in 0..view.centerline_xy_m.len() {
            for j in (i + 18)..view.centerline_xy_m.len() {
                let route_delta = (j - i).min(view.centerline_xy_m.len() + i - j);
                if route_delta < 18 {
                    continue;
                }
                let d = distance(view.centerline_xy_m[i], view.centerline_xy_m[j]);
                if d < nearest_later_station.2 {
                    nearest_later_station = (i, j, d);
                }
            }
        }

        let (i, j, xy_distance_m) = nearest_later_station;
        let route_delta_m = view.station_s_m[j] - view.station_s_m[i];
        assert!(
            xy_distance_m < 2.0,
            "gymkhana GP8 should contain a close non-local crossing pair; got {nearest_later_station:?}"
        );
        assert!(
            route_delta_m > 30.0,
            "nearest XY crossing pair must stay far apart in route progress: pair={nearest_later_station:?} route_delta_m={route_delta_m}"
        );
    }

    #[test]
    fn adaptive_station_count_detects_public_self_crossing_gymkhana_complexity() {
        let track = closed_8gp_self_crossing_gymkhana_track();
        let report = estimate_station_complexity(&track, &FixedCenterlineStationOptions::default());

        assert!(
            (64..=96).contains(&report.recommended_station_count),
            "two-cone 8GP should use a moderate station count: {report:?}"
        );
        assert!(
            report.crossing_zone_count > 0,
            "8GP complexity must record its non-local route crossing"
        );
    }

    #[test]
    fn open_concave_kink_station_repair_removes_crossing_chords() {
        let track = open_concave_kink_crossing_lab_track();
        let options = FixedCenterlineStationOptions {
            sample_count: 48,
            dense_count: 384,
            density_smooth_window: 9,
            density_max_adjacent_ratio: 1.35,
            target_spacing_max_adjacent_ratio: 1.45,
            target_spacing_metric: "hybrid_area_centerline".to_owned(),
            ..FixedCenterlineStationOptions::default()
        };

        let view = build_production_sections_track_view(&track, &options);

        eprintln!(
            "open concave kink station quality metrics: {:#?}",
            view.quality_metrics
        );

        assert_eq!(metadata_str(&view.metadata, "trajectory_mode"), "open");
        assert_eq!(
            metadata_str(&view.metadata, "open_repair_progress_source"),
            "shared_dtw_frame_progress"
        );
        assert_eq!(
            metadata_str(&view.metadata, "station_geometry_source"),
            "universal_area_route_pair"
        );
        assert!(
            metadata_i64(&view.metadata, "dtw_paired_right_plateau_run_max") <= 4,
            "concave/kink DTW pairing must not collapse into a fan-out plateau"
        );
        assert!(
            metadata_f64(&view.metadata, "first_last_gap_m") > 100.0,
            "fixture must remain an open path, not an implicit loop"
        );
        assert!(
            metadata_i64(&view.metadata, "open_repair_crossing_count_before") > 0,
            "fixture should reproduce the old crossing-prone geometry before repair"
        );
        assert_eq!(
            metadata_i64(&view.metadata, "open_repair_crossing_count_after"),
            0
        );
        assert_eq!(
            metadata_i64(&view.metadata, "adjacent_section_crossing_count_horizon2"),
            0
        );
        assert_eq!(
            metadata_i64(&view.metadata, "station_raw_boundary_crossing_count"),
            0
        );
        assert!(
            distance(
                view.left_boundary_xy_m[0],
                track.left_boundary_xy_m.first().copied().unwrap()
            ) <= 1e-9
        );
        assert!(
            distance(
                view.right_boundary_xy_m[0],
                track.right_boundary_xy_m.first().copied().unwrap()
            ) <= 1e-9
        );
        assert!(
            distance(
                *view.left_boundary_xy_m.last().unwrap(),
                track.left_boundary_xy_m.last().copied().unwrap()
            ) <= 1e-9
        );
        assert!(
            distance(
                *view.right_boundary_xy_m.last().unwrap(),
                track.right_boundary_xy_m.last().copied().unwrap()
            ) <= 1e-9
        );
        assert!(
            metadata_f64(&view.metadata, "station_spacing_adjacent_ratio_max")
                <= OPEN_STATION_SPACING_ADJACENT_RATIO_HARD_LIMIT
        );
        assert!(
            metadata_f64(&view.metadata, "cell_area_adjacent_ratio_max")
                <= OPEN_STATION_CELL_AREA_ADJACENT_RATIO_HARD_LIMIT
        );
        assert!(
            metadata_f64(&view.metadata, "open_repair_frame_endpoint_delta_max_m") <= 12.0,
            "open station repair must stay close to the shared DTW frame"
        );
        assert!(
            metadata_f64(&view.metadata, "open_repair_frame_endpoint_delta_p95_m") <= 5.0,
            "most open station chords should remain close to the shared DTW frame"
        );
    }

    fn assert_explicit_production_station_builder_matches_default_selector(
        case_name: &str,
        track_area_path: &Path,
        builder: ProductionStationBuilder,
        station_geometry_source: &str,
    ) {
        let track = read_track_area_contract(track_area_path);
        let default_options = FixedCenterlineStationOptions {
            sample_count: 48,
            dense_count: 384,
            ..FixedCenterlineStationOptions::default()
        };
        let explicit_options = FixedCenterlineStationOptions {
            sample_count: 48,
            dense_count: 384,
            ..FixedCenterlineStationOptions::default()
        };

        let default_view = build_production_sections_track_view(&track, &default_options);
        let explicit_view = build_legacy_sections_track_view(&track, &explicit_options, builder);
        let metrics = station_parity_metrics(&default_view, &explicit_view);

        eprintln!("{case_name} explicit production builder parity: {metrics:#?}");

        assert_eq!(
            default_view.centerline_xy_m.len(),
            explicit_view.centerline_xy_m.len(),
            "{case_name} explicit builder must preserve station count"
        );
        assert_eq!(
            metadata_str(&explicit_view.metadata, "station_geometry_source"),
            station_geometry_source,
            "{case_name} explicit builder must publish the expected source"
        );
        assert!(
            metrics.max_station_s_delta_m <= 1e-12
                && metrics.max_centerline_delta_m <= 1e-12
                && metrics.max_width_left_delta_m <= 1e-12
                && metrics.max_width_right_delta_m <= 1e-12
                && metrics.max_section_dir_delta <= 1e-12
                && metrics.max_section_dir_derivative_delta <= 1e-12
                && metrics.max_kappa_delta_1pm <= 1e-12,
            "{case_name} explicit builder diverged from default selector: {metrics:#?}"
        );
    }

    #[test]
    fn explicit_asymmetric_loop_area_recipe_matches_default_production_selector() {
        assert_explicit_production_station_builder_matches_default_selector(
            "public_asymmetric_loop",
            &crate_path("tests/public-fixtures/asymmetric-loop-track-area-v1.json"),
            ProductionStationBuilder::CanonicalAreaStation,
            "canonical_area_station_generator",
        );
    }

    #[test]
    fn explicit_compact_oval_area_recipe_matches_default_production_selector() {
        assert_explicit_production_station_builder_matches_default_selector(
            "public_compact_oval",
            &crate_path("tests/public-fixtures/compact-oval-track-area-v1.json"),
            ProductionStationBuilder::CanonicalAreaStation,
            "canonical_area_station_generator",
        );
    }

    #[test]
    fn explicit_compact_oval_generated_boundary_recipe_remains_legacy_debug_path() {
        let track = read_track_area_contract(&crate_path(
            "tests/public-fixtures/compact-oval-track-area-v1.json",
        ));
        let explicit_options = FixedCenterlineStationOptions {
            sample_count: 48,
            dense_count: 384,
            ..FixedCenterlineStationOptions::default()
        };

        let view = build_legacy_sections_track_view(
            &track,
            &explicit_options,
            ProductionStationBuilder::GeneratedBoundaryPair,
        );

        assert_eq!(
            metadata_str(&view.metadata, "station_geometry_source"),
            "generated_boundary_pair"
        );
        assert_eq!(
            metadata_str(&view.metadata, "station_builder"),
            "generated_boundary_pair"
        );
        assert_eq!(metadata_i64(&view.metadata, "dtw_alignment_roll_bias"), 0);
    }

    #[test]
    fn resamples_closed_polyline_without_endpoint_duplicate() {
        let points = vec![[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]];
        let sampled = resample_closed_polyline(&points, 4);

        assert_eq!(sampled, points);
    }

    #[test]
    fn asymmetric_loop_production_station_metadata_declares_raw_boundary_reference_source() {
        let track = read_track_area_contract(&crate_path(
            "tests/public-fixtures/asymmetric-loop-track-area-v1.json",
        ));
        let view = build_production_sections_track_view(
            &track,
            &FixedCenterlineStationOptions {
                sample_count: 48,
                dense_count: 384,
                ..FixedCenterlineStationOptions::default()
            },
        );

        assert_eq!(
            metadata_str(&view.metadata, "reference_source"),
            "generated_from_raw_boundaries"
        );
        assert_ne!(
            metadata_str(&view.metadata, "reference_source"),
            "embedded_rice_reftrack_csv"
        );
    }

    #[test]
    fn asymmetric_loop_production_station_output_changes_when_raw_boundaries_change() {
        let track = read_track_area_contract(&crate_path(
            "tests/public-fixtures/asymmetric-loop-track-area-v1.json",
        ));
        let options = FixedCenterlineStationOptions {
            sample_count: 48,
            dense_count: 384,
            ..FixedCenterlineStationOptions::default()
        };
        let baseline = build_production_sections_track_view(&track, &options);

        let mut poisoned = track.clone();
        for point in poisoned.left_boundary_xy_m.iter_mut().take(7).skip(3) {
            point[0] += 0.75;
            point[1] -= 0.40;
        }
        let poisoned_view = build_production_sections_track_view(&poisoned, &options);
        let metrics = station_parity_metrics(&poisoned_view, &baseline);

        assert!(
            metrics.max_centerline_delta_m > 0.05,
            "Rice production did not respond to raw boundary perturbation: {metrics:#?}"
        );
    }
}
