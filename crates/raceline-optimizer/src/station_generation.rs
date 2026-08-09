use crate::contracts::{
    option_start_finish_to_json, prepared_station_bundle_hash_v3, sections_track_view_hash_v2,
    station_generation_request_key_v3, station_geometry_content_hash_v1,
    station_geometry_content_hash_v2, station_options_hash_v2, Point2, SectionsTrackViewV1,
    StationRecipeV1, StationSourceRefV1, TrackAreaContractV1, PREPARED_STATION_BUNDLE_HASH_V3,
    SECTIONS_TRACK_VIEW_HASH_V2,
};
use crate::json::{parse_json_str, JsonValue};
use crate::section_frame::section_frame_progress;
use crate::station::{
    build_production_sections_track_view_from_plan_with_control,
    prepare_production_station_plan_with_control, DtwAlignmentRollBias,
    FixedCenterlineStationOptions, StationComplexityReport, StationGenerationControl,
};
use crate::{JsonObject, ToJsonValue};

const STATION_SECTION_DET_MIN: f64 = 0.02;
const STATION_SECTION_FORWARD_PROGRESS_MIN: f64 = 0.02;
const STATION_SECTION_DET_WARN: f64 = 0.25;
const STATION_SECTION_FORWARD_PROGRESS_WARN: f64 = 0.6;
const STATION_SECTION_ROTATION_MAX_WARN_DEG: f64 = 45.0;
const STATION_SECTION_ROTATION_P95_WARN_DEG: f64 = 15.0;
const STATION_CURVATURE_WIDTH_RISK_WARN: f64 = 1.0;

pub const STATION_GENERATOR_CONTRACT: &str = "station_generation_contract.v9";
pub const STATION_GENERATOR_VERSION: &str = "0.6.9";
pub const STATION_VALIDATION_CONTRACT: &str = "station_validation_contract.v1";
pub const STATION_VALIDATION_VERSION: &str = "1.0.0";

#[derive(Clone, Debug, PartialEq)]
pub struct StationGenerationRequestV1 {
    pub request_id: String,
    pub request_key: String,
    pub project_id: String,
    pub station_count: usize,
    pub count_mode: StationCountMode,
    pub track_area: TrackAreaContractV1,
    pub station_options: FixedCenterlineStationOptions,
    pub station_options_hash: String,
    pub source_ref: StationSourceRefV1,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StationCountMode {
    Auto,
    Exact,
}

impl StationCountMode {
    fn as_str(self) -> &'static str {
        match self {
            Self::Auto => "auto",
            Self::Exact => "exact",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StationValidationMode {
    Strict,
    PointMass,
}

#[derive(Clone, Debug, PartialEq)]
pub struct StationGenerationResultV1 {
    pub request_key: String,
    pub sections_track_view: SectionsTrackViewV1,
    pub model_track_area: TrackAreaContractV1,
    pub diagnostics: JsonObject,
    pub requested_count_mode: StationCountMode,
    pub resolved_station_count: usize,
    pub complexity_report: Option<StationComplexityReport>,
    pub station_options_hash: String,
    pub source_ref: StationSourceRefV1,
}

#[derive(Clone, Debug, PartialEq)]
pub struct StationGenerationProgressEventV1 {
    pub run_id: String,
    pub phase: String,
    pub progress: Option<f64>,
    pub message: Option<String>,
    pub station_count: Option<usize>,
    pub metadata: JsonObject,
    pub model_track_area: Option<TrackAreaContractV1>,
    pub diagnostics: JsonObject,
}

pub type StationGenerationProgressCallback<'a> =
    &'a mut dyn FnMut(StationGenerationProgressEventV1);
pub type StationGenerationCancelCheck<'a> = &'a dyn Fn() -> bool;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum StationGenerationExecutionError {
    Cancelled,
    Invalid(String),
}

#[derive(Clone, Debug, PartialEq)]
pub struct StationSectionFrameAuditRow {
    pub station_index: usize,
    pub sample_label: String,
    pub station_s_m: f64,
    pub center_x_m: f64,
    pub center_y_m: f64,
    pub width_left_m: f64,
    pub width_right_m: f64,
    pub n_m: f64,
    pub tangent_x: f64,
    pub tangent_y: f64,
    pub section_dir_x: f64,
    pub section_dir_y: f64,
    pub section_dir_ds_x: f64,
    pub section_dir_ds_y: f64,
    pub section_rotation_prev_deg: f64,
    pub section_rotation_next_deg: f64,
    pub curvature_signed_1pm: f64,
    pub curvature_width_risk: f64,
    pub section_det: f64,
    pub forward_progress_per_speed: f64,
    pub sigma_dt_ds_at_1mps: f64,
    pub dn_ds_at_1mps: f64,
    pub pure_frenet_factor_debug: f64,
}

impl StationGenerationRequestV1 {
    pub fn parse_product(input_json: &str) -> Result<Self, String> {
        let value = parse_json_str(input_json).map_err(|error| format!("invalid json: {error}"))?;
        if optional_string(&value, "schema_version").as_deref()
            != Some("station_generation_request.v4")
        {
            return Err(
                "unsupported product station request; expected station_generation_request.v4"
                    .to_owned(),
            );
        }
        ensure_json_fields(
            &value,
            &[
                "schema_version",
                "request_id",
                "request_key",
                "project_id",
                "generator_contract",
                "generator_version",
                "validation_contract",
                "validation_version",
                "source_ref",
                "station_validation_mode",
                "count_mode",
                "station_count",
                "direction",
                "station_options",
                "station_options_hash",
                "track_area",
            ],
            "station_generation_request.v4",
        )?;
        let source_ref = required_field(&value, "source_ref")?;
        ensure_json_fields(
            source_ref,
            &[
                "schema_version",
                "project_id",
                "geometry_id",
                "geometry_content_hash",
                "route_id",
            ],
            "station_source_ref.v1",
        )?;
        if required_string(&value, "generator_contract")? != STATION_GENERATOR_CONTRACT
            || required_string(&value, "generator_version")? != STATION_GENERATOR_VERSION
            || required_string(&value, "validation_contract")? != STATION_VALIDATION_CONTRACT
            || required_string(&value, "validation_version")? != STATION_VALIDATION_VERSION
        {
            return Err("incompatible station generator or validation contract".to_owned());
        }
        Self::parse(input_json)
    }

    pub fn parse(input_json: &str) -> Result<Self, String> {
        let value = parse_json_str(input_json).map_err(|error| format!("invalid json: {error}"))?;
        let schema_version = optional_string(&value, "schema_version")
            .unwrap_or_else(|| "station_generation_request.v1".to_owned());
        if !matches!(
            schema_version.as_str(),
            "station_generation_request.v1"
                | "station_generation_request.v2"
                | "station_generation_request.v3"
                | "station_generation_request.v4"
        ) {
            return Err("unsupported station generation request schema".to_owned());
        }
        let track_area = TrackAreaContractV1::from_json(required_field(&value, "track_area")?)?;
        match track_area.trajectory_mode.as_str() {
            "closed" => match track_area.direction.as_deref() {
                Some("clockwise" | "counterclockwise") => {}
                _ => {
                    return Err(
                        "closed track_area direction must be clockwise or counterclockwise"
                            .to_owned(),
                    )
                }
            },
            "open" => {
                if !matches!(
                    track_area.direction.as_deref(),
                    None | Some("clockwise" | "counterclockwise")
                ) {
                    return Err(
                        "open track_area direction must be clockwise, counterclockwise or null"
                            .to_owned(),
                    );
                }
                if track_area.start_finish_xy_m.is_none() || track_area.finish_line_xy_m.is_none() {
                    return Err(
                        "open track_area requires start_finish_xy_m and finish_line_xy_m"
                            .to_owned(),
                    );
                }
            }
            _ => return Err("track_area trajectory_mode must be open or closed".to_owned()),
        }
        let count_mode = match optional_string(&value, "count_mode").as_deref() {
            None if matches!(
                schema_version.as_str(),
                "station_generation_request.v3" | "station_generation_request.v4"
            ) =>
            {
                return Err(
                    "count_mode is required for current station generation requests".to_owned(),
                )
            }
            None | Some("exact") => StationCountMode::Exact,
            Some("auto") => StationCountMode::Auto,
            Some(_) => return Err("count_mode must be auto or exact".to_owned()),
        };
        let station_count = match value.get("station_count") {
            Some(value) => value
                .as_u32()
                .ok_or_else(|| "station_count must be an integer".to_owned())?
                as usize,
            None if count_mode == StationCountMode::Auto => 64,
            None => return Err("station_count is required for exact count mode".to_owned()),
        };
        if station_count < 2 {
            return Err("station_count must be at least 2".to_owned());
        }
        let request_id = optional_string(&value, "request_id").unwrap_or_else(|| "unknown".into());
        let request_key = if matches!(
            schema_version.as_str(),
            "station_generation_request.v3" | "station_generation_request.v4"
        ) {
            required_string(&value, "request_key")?
        } else {
            "legacy_station_request".to_owned()
        };
        let project_id = optional_string(&value, "project_id").unwrap_or_else(|| "unknown".into());
        let station_options = parse_station_options(
            station_count,
            value
                .get("station_options")
                .or_else(|| value.get("solve_options")),
        )?;
        let source_ref = if matches!(
            schema_version.as_str(),
            "station_generation_request.v3" | "station_generation_request.v4"
        ) {
            let source = required_field(&value, "source_ref")?;
            if required_string(source, "schema_version")? != "station_source_ref.v1" {
                return Err("unsupported station source reference".to_owned());
            }
            StationSourceRefV1 {
                project_id: required_string(source, "project_id")?,
                geometry_id: required_string(source, "geometry_id")?,
                geometry_content_hash: required_string(source, "geometry_content_hash")?,
                route_id: required_string(source, "route_id")?,
            }
        } else {
            StationSourceRefV1 {
                project_id: project_id.clone(),
                geometry_id: track_area.track_id.clone(),
                geometry_content_hash: station_geometry_content_hash_v1(&track_area),
                route_id: track_area.track_id.clone(),
            }
        };
        if source_ref.project_id != project_id {
            return Err("source_ref.project_id must match project_id".to_owned());
        }
        if source_ref.route_id != track_area.track_id {
            return Err("source_ref.route_id must match track_area.track_id".to_owned());
        }
        if matches!(
            schema_version.as_str(),
            "station_generation_request.v3" | "station_generation_request.v4"
        ) {
            let direction = required_string(&value, "direction")?;
            if !matches!(direction.as_str(), "clockwise" | "counterclockwise") {
                return Err("direction must be clockwise or counterclockwise".to_owned());
            }
            if track_area.direction.as_deref() != Some(direction.as_str()) {
                return Err("direction must match track_area.direction".to_owned());
            }
        }
        let actual_geometry_hash = if schema_version == "station_generation_request.v4" {
            station_geometry_content_hash_v2(&track_area)
        } else {
            station_geometry_content_hash_v1(&track_area)
        };
        if source_ref.geometry_content_hash != actual_geometry_hash {
            return Err("source_ref.geometry_content_hash does not match track_area".to_owned());
        }
        let station_options_hash = if matches!(
            schema_version.as_str(),
            "station_generation_request.v3" | "station_generation_request.v4"
        ) {
            let supplied = required_string(&value, "station_options_hash")?;
            if schema_version == "station_generation_request.v4" {
                let empty_options = JsonValue::Object(Vec::new());
                let options = value.get("station_options").unwrap_or(&empty_options);
                if supplied != station_options_hash_v2(options) {
                    return Err("station_options_hash does not match station_options".to_owned());
                }
            }
            supplied
        } else {
            "legacy_station_options".to_owned()
        };
        if schema_version == "station_generation_request.v4" {
            let requested_count = (count_mode == StationCountMode::Exact).then_some(station_count);
            let direction = required_string(&value, "direction")?;
            let expected_key = station_generation_request_key_v3(
                &source_ref,
                count_mode.as_str(),
                requested_count,
                &direction,
                &station_options_hash,
                STATION_GENERATOR_CONTRACT,
                STATION_GENERATOR_VERSION,
                STATION_VALIDATION_CONTRACT,
                STATION_VALIDATION_VERSION,
            );
            if request_key != expected_key {
                return Err("request_key does not match station request".to_owned());
            }
        }
        Ok(Self {
            request_id,
            request_key,
            project_id,
            station_count,
            count_mode,
            track_area,
            station_options,
            station_options_hash,
            source_ref,
        })
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct StationTopologyValidationIssue {
    pub code: String,
    pub message: String,
    pub diagnostics: JsonObject,
}

#[derive(Clone, Debug, PartialEq)]
pub struct StationTopologyAuditReport {
    pub valid: bool,
    pub issues: Vec<StationTopologyValidationIssue>,
    pub diagnostics: JsonObject,
}

pub fn audit_station_topology(sections: &SectionsTrackViewV1) -> StationTopologyAuditReport {
    let count = sections.station_s_m.len();
    let mut diagnostics = vec![
        ("station_count".to_owned(), JsonValue::Integer(count as i64)),
        ("track_id".to_owned(), sections.track_id.clone().into()),
    ];
    let mut issue_specs = Vec::<(&str, &str, JsonObject)>::new();
    if count < 2 {
        issue_specs.push((
            "station.invalidTopology",
            "station topology requires at least two stations",
            Vec::new(),
        ));
    }
    let arrays_are_consistent = sections.centerline_xy_m.len() == count
        && sections.left_boundary_xy_m.len() == count
        && sections.right_boundary_xy_m.len() == count
        && sections.normals_xy.len() == count
        && sections.width_left_m.len() == count
        && sections.width_right_m.len() == count
        && sections.section_dirs_xy.len() == count;
    if !arrays_are_consistent {
        let detail = vec![
            (
                "centerline_count".to_owned(),
                JsonValue::Integer(sections.centerline_xy_m.len() as i64),
            ),
            (
                "left_boundary_count".to_owned(),
                JsonValue::Integer(sections.left_boundary_xy_m.len() as i64),
            ),
            (
                "right_boundary_count".to_owned(),
                JsonValue::Integer(sections.right_boundary_xy_m.len() as i64),
            ),
        ];
        diagnostics.extend(detail.clone());
        issue_specs.push((
            "station.invalidTopology",
            "station topology arrays have inconsistent lengths",
            detail,
        ));
    } else if count >= 2 {
        let trajectory_mode = sections
            .metadata
            .iter()
            .find(|(key, _)| key == "trajectory_mode")
            .and_then(|(_, value)| value.as_str());
        let requested_direction = sections
            .metadata
            .iter()
            .find(|(key, _)| key == "requested_direction")
            .and_then(|(_, value)| value.as_str());
        if !matches!(trajectory_mode, Some("open" | "closed"))
            || trajectory_mode == Some("closed")
                && !matches!(requested_direction, Some("clockwise" | "counterclockwise"))
            || requested_direction.is_some()
                && !matches!(requested_direction, Some("clockwise" | "counterclockwise"))
        {
            issue_specs.push((
                "station.invalidTopology",
                "station topology metadata is invalid",
                Vec::new(),
            ));
        }
        let station_s_is_strict = sections
            .station_s_m
            .windows(2)
            .all(|pair| pair[0].is_finite() && pair[1].is_finite() && pair[1] > pair[0]);
        if !station_s_is_strict {
            issue_specs.push((
                "station.invalidTopology",
                "station progress must be finite and strictly increasing",
                Vec::new(),
            ));
        }
        let invalid_width_indices = sections
            .left_boundary_xy_m
            .iter()
            .zip(&sections.right_boundary_xy_m)
            .zip(sections.width_left_m.iter().zip(&sections.width_right_m))
            .enumerate()
            .filter_map(|(index, ((left, right), (left_width, right_width)))| {
                let chord_width = point_distance(*left, *right);
                let center = sections.centerline_xy_m[index];
                let normal = sections.normals_xy[index];
                let section_dir = sections.section_dirs_xy[index];
                let left_vector = point_sub(*left, center);
                let right_vector = point_sub(*right, center);
                let tolerance = 1.0e-5 * left_width.max(*right_width).max(1.0);
                (!left_width.is_finite()
                    || !right_width.is_finite()
                    || *left_width <= 1.0e-6
                    || *right_width <= 1.0e-6
                    || !chord_width.is_finite()
                    || chord_width <= 1e-6
                    || point_norm(normal) <= 1.0e-6
                    || point_norm(section_dir) <= 1.0e-6
                    || (point_norm(left_vector) - *left_width).abs() > tolerance
                    || (point_norm(right_vector) - *right_width).abs() > tolerance)
                    .then_some(index)
            })
            .collect::<Vec<_>>();
        diagnostics.push((
            "invalid_width_indices".to_owned(),
            usize_array_to_json(&invalid_width_indices),
        ));
        if !invalid_width_indices.is_empty() {
            issue_specs.push((
                "station.invalidTopology",
                "station topology contains invalid station widths",
                vec![(
                    "invalid_width_indices".to_owned(),
                    usize_array_to_json(&invalid_width_indices),
                )],
            ));
        }

        let min_center_spacing = adjacent_min_distance(&sections.centerline_xy_m);
        let min_left_spacing = adjacent_min_distance(&sections.left_boundary_xy_m);
        let min_right_spacing = adjacent_min_distance(&sections.right_boundary_xy_m);
        diagnostics.extend([
            ("min_center_spacing_m".to_owned(), min_center_spacing.into()),
            (
                "min_left_endpoint_spacing_m".to_owned(),
                min_left_spacing.into(),
            ),
            (
                "min_right_endpoint_spacing_m".to_owned(),
                min_right_spacing.into(),
            ),
        ]);
        if min_center_spacing <= 1e-5 || min_left_spacing <= 1e-5 || min_right_spacing <= 1e-5 {
            issue_specs.push((
                "station.invalidTopology",
                "station topology contains near-zero adjacent station spacing",
                Vec::new(),
            ));
        }

        let crossing_pairs = station_horizon_crossing_pairs(
            &sections.left_boundary_xy_m,
            &sections.right_boundary_xy_m,
            2,
            trajectory_mode == Some("closed"),
        );
        diagnostics.push((
            "adjacent_section_crossing_count_horizon2".to_owned(),
            JsonValue::Integer(crossing_pairs.len() as i64),
        ));
        diagnostics.push((
            "adjacent_section_crossing_pairs_horizon2".to_owned(),
            index_pair_array_to_json(&crossing_pairs),
        ));
        if !crossing_pairs.is_empty() {
            issue_specs.push((
                "station.invalidTopology",
                "station topology contains crossing station chords",
                vec![(
                    "adjacent_section_crossing_pairs_horizon2".to_owned(),
                    index_pair_array_to_json(&crossing_pairs),
                )],
            ));
        }

        let regularity = station_section_frame_regularity(sections);
        diagnostics.extend(regularity.to_diagnostics());
        if regularity.min_abs_section_det <= STATION_SECTION_DET_MIN {
            issue_specs.push((
                "station.invalidSectionFrame",
                "station section-frame map determinant magnitude is too small",
                regularity.to_diagnostics(),
            ));
        }
        if regularity.section_det_sign_flip_count > 0 {
            issue_specs.push((
                "station.invalidSectionFrame",
                "station section-frame map determinant reverses orientation",
                regularity.to_diagnostics(),
            ));
        }
        if regularity.min_forward_progress <= STATION_SECTION_FORWARD_PROGRESS_MIN {
            issue_specs.push((
                "station.invalidSectionFrame",
                "station section-frame forward progress is too small or reversed",
                regularity.to_diagnostics(),
            ));
        }
    }

    let issues = issue_specs
        .into_iter()
        .map(|(code, message, mut detail)| {
            detail.push(("station_count".to_owned(), JsonValue::Integer(count as i64)));
            StationTopologyValidationIssue {
                code: code.to_owned(),
                message: message.to_owned(),
                diagnostics: detail,
            }
        })
        .collect::<Vec<_>>();
    StationTopologyAuditReport {
        valid: issues.is_empty(),
        issues,
        diagnostics,
    }
}

pub fn validate_station_topology(
    sections: &SectionsTrackViewV1,
) -> Result<(), StationTopologyValidationIssue> {
    let report = audit_station_topology(sections);
    report.issues.into_iter().next().map_or(Ok(()), Err)
}

pub fn validate_station_topology_for_point_mass(
    sections: &SectionsTrackViewV1,
) -> Result<(), StationTopologyValidationIssue> {
    validate_station_topology(sections)
}

fn validate_station_topology_for_mode(
    mode: StationValidationMode,
    sections: &SectionsTrackViewV1,
) -> Result<(), StationTopologyValidationIssue> {
    match mode {
        StationValidationMode::Strict => validate_station_topology(sections),
        StationValidationMode::PointMass => validate_station_topology_for_point_mass(sections),
    }
}

pub fn validate_station_section_frame_regular(
    sections: &SectionsTrackViewV1,
) -> Result<(), StationTopologyValidationIssue> {
    let regularity = station_section_frame_regularity(sections);
    let mut diagnostics = vec![
        (
            "station_count".to_owned(),
            JsonValue::Integer(sections.station_s_m.len() as i64),
        ),
        ("track_id".to_owned(), sections.track_id.clone().into()),
    ];
    diagnostics.extend(regularity.to_diagnostics());
    if regularity.min_abs_section_det <= STATION_SECTION_DET_MIN {
        return Err(StationTopologyValidationIssue {
            code: "station.invalidSectionFrame".to_owned(),
            message: "station section-frame map determinant magnitude is too small".to_owned(),
            diagnostics,
        });
    }
    if regularity.section_det_sign_flip_count > 0 {
        return Err(StationTopologyValidationIssue {
            code: "station.invalidSectionFrame".to_owned(),
            message: "station section-frame map determinant reverses orientation".to_owned(),
            diagnostics,
        });
    }
    if regularity.min_forward_progress <= STATION_SECTION_FORWARD_PROGRESS_MIN {
        return Err(StationTopologyValidationIssue {
            code: "station.invalidSectionFrame".to_owned(),
            message: "station section-frame forward progress is too small or reversed".to_owned(),
            diagnostics,
        });
    }
    Ok(())
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct StationSectionFrameRegularity {
    min_section_det: f64,
    min_section_det_station: usize,
    min_section_det_n_m: f64,
    min_abs_section_det: f64,
    min_abs_section_det_station: usize,
    min_abs_section_det_n_m: f64,
    section_det_reference_sign: f64,
    section_det_sign_flip_count: usize,
    min_forward_progress: f64,
    min_forward_progress_station: usize,
    max_section_rotation_deg: f64,
    section_rotation_p95_deg: f64,
    max_curvature_width_risk: f64,
    max_curvature_width_risk_station: usize,
}

impl StationSectionFrameRegularity {
    fn to_diagnostics(self) -> JsonObject {
        vec![
            (
                "section_frame_min_det".to_owned(),
                self.min_section_det.into(),
            ),
            (
                "section_frame_min_det_station".to_owned(),
                JsonValue::Integer(self.min_section_det_station as i64),
            ),
            (
                "section_frame_min_det_n_m".to_owned(),
                self.min_section_det_n_m.into(),
            ),
            (
                "section_frame_min_abs_det".to_owned(),
                self.min_abs_section_det.into(),
            ),
            (
                "section_frame_min_abs_det_station".to_owned(),
                JsonValue::Integer(self.min_abs_section_det_station as i64),
            ),
            (
                "section_frame_min_abs_det_n_m".to_owned(),
                self.min_abs_section_det_n_m.into(),
            ),
            (
                "section_frame_det_reference_sign".to_owned(),
                self.section_det_reference_sign.into(),
            ),
            (
                "section_frame_det_sign_flip_count".to_owned(),
                JsonValue::Integer(self.section_det_sign_flip_count as i64),
            ),
            (
                "section_frame_min_forward_progress".to_owned(),
                self.min_forward_progress.into(),
            ),
            (
                "section_frame_min_forward_progress_station".to_owned(),
                JsonValue::Integer(self.min_forward_progress_station as i64),
            ),
            (
                "section_dir_rotation_max_deg".to_owned(),
                self.max_section_rotation_deg.into(),
            ),
            (
                "section_dir_rotation_p95_deg".to_owned(),
                self.section_rotation_p95_deg.into(),
            ),
            (
                "curvature_width_risk_max".to_owned(),
                self.max_curvature_width_risk.into(),
            ),
            (
                "curvature_width_risk_max_station".to_owned(),
                JsonValue::Integer(self.max_curvature_width_risk_station as i64),
            ),
            (
                "section_frame_warn_min_det".to_owned(),
                JsonValue::Bool(self.min_abs_section_det < STATION_SECTION_DET_WARN),
            ),
            (
                "section_frame_warn_min_forward_progress".to_owned(),
                JsonValue::Bool(self.min_forward_progress < STATION_SECTION_FORWARD_PROGRESS_WARN),
            ),
            (
                "section_frame_warn_max_rotation".to_owned(),
                JsonValue::Bool(
                    self.max_section_rotation_deg > STATION_SECTION_ROTATION_MAX_WARN_DEG,
                ),
            ),
            (
                "section_frame_warn_p95_rotation".to_owned(),
                JsonValue::Bool(
                    self.section_rotation_p95_deg > STATION_SECTION_ROTATION_P95_WARN_DEG,
                ),
            ),
            (
                "section_frame_warn_curvature_width_risk".to_owned(),
                JsonValue::Bool(self.max_curvature_width_risk > STATION_CURVATURE_WIDTH_RISK_WARN),
            ),
        ]
    }
}

fn station_section_frame_regularity(
    sections: &SectionsTrackViewV1,
) -> StationSectionFrameRegularity {
    let mut min_section_det = f64::INFINITY;
    let mut min_section_det_station = 0_usize;
    let mut min_section_det_n_m = 0.0_f64;
    let mut min_abs_section_det = f64::INFINITY;
    let mut min_abs_section_det_station = 0_usize;
    let mut min_abs_section_det_n_m = 0.0_f64;
    let mut section_det_reference_sign = 0.0_f64;
    let mut section_det_sign_flip_count = 0_usize;
    let mut min_forward_progress = f64::INFINITY;
    let mut min_forward_progress_station = 0_usize;
    let mut max_curvature_width_risk = 0.0_f64;
    let mut max_curvature_width_risk_station = 0_usize;

    for row in station_section_frame_audit_rows(sections) {
        if row.section_det < min_section_det {
            min_section_det = row.section_det;
            min_section_det_station = row.station_index;
            min_section_det_n_m = row.n_m;
        }
        if row.section_det.abs() < min_abs_section_det {
            min_abs_section_det = row.section_det.abs();
            min_abs_section_det_station = row.station_index;
            min_abs_section_det_n_m = row.n_m;
        }
        if row.section_det.abs() > 1.0e-9 {
            let sign = row.section_det.signum();
            if section_det_reference_sign == 0.0 {
                section_det_reference_sign = sign;
            } else if sign != section_det_reference_sign {
                section_det_sign_flip_count += 1;
            }
        }
        if row.forward_progress_per_speed < min_forward_progress {
            min_forward_progress = row.forward_progress_per_speed;
            min_forward_progress_station = row.station_index;
        }
        let curvature_width_risk = row.curvature_width_risk;
        if curvature_width_risk > max_curvature_width_risk {
            max_curvature_width_risk = curvature_width_risk;
            max_curvature_width_risk_station = row.station_index;
        }
    }

    let closed = station_sections_are_closed(sections);
    let mut rotations = adjacent_section_rotations_deg(sections, closed);
    rotations.sort_by(f64::total_cmp);
    let max_section_rotation_deg = rotations.iter().copied().fold(0.0, f64::max);
    let section_rotation_p95_deg = percentile_sorted(&rotations, 0.95);

    StationSectionFrameRegularity {
        min_section_det,
        min_section_det_station,
        min_section_det_n_m,
        min_abs_section_det,
        min_abs_section_det_station,
        min_abs_section_det_n_m,
        section_det_reference_sign,
        section_det_sign_flip_count,
        min_forward_progress,
        min_forward_progress_station,
        max_section_rotation_deg,
        section_rotation_p95_deg,
        max_curvature_width_risk,
        max_curvature_width_risk_station,
    }
}

#[must_use]
pub fn station_section_frame_audit_rows(
    sections: &SectionsTrackViewV1,
) -> Vec<StationSectionFrameAuditRow> {
    let count = sections.station_s_m.len();
    let closed = station_sections_are_closed(sections);
    let tangents = station_centerline_tangents(sections, closed);
    let section_dir_ds = station_section_dir_derivatives(sections, closed);
    let mut rows = Vec::with_capacity(count * 20);

    for station in 0..count {
        let tangent = tangents[station];
        let left_normal = [-tangent[1], tangent[0]];
        let curvature_signed = station_curvature_signed(sections, &tangents, station, closed);
        let curvature_width_risk = curvature_signed.abs()
            * sections.width_left_m[station].max(sections.width_right_m[station]);
        let previous = previous_station_index(station, count, closed);
        let next = next_station_index(station, count, closed);
        let section_rotation_prev_deg = section_rotation_deg(
            sections.section_dirs_xy[previous],
            sections.section_dirs_xy[station],
        );
        let section_rotation_next_deg = section_rotation_deg(
            sections.section_dirs_xy[station],
            sections.section_dirs_xy[next],
        );
        let samples = [
            ("right_edge", -sections.width_right_m[station]),
            ("right_mid", -0.5 * sections.width_right_m[station]),
            ("center", 0.0),
            ("left_mid", 0.5 * sections.width_left_m[station]),
            ("left_edge", sections.width_left_m[station]),
        ];
        for (sample_label, n_m) in samples {
            rows.push(station_section_frame_audit_row(
                station,
                sample_label.to_owned(),
                sections.station_s_m[station],
                sections.centerline_xy_m[station],
                sections.width_left_m[station],
                sections.width_right_m[station],
                n_m,
                tangent,
                left_normal,
                sections.section_dirs_xy[station],
                section_dir_ds[station],
                section_rotation_prev_deg,
                section_rotation_next_deg,
                curvature_signed,
                curvature_width_risk,
            ));
        }

        let next = next_station_index(station, count, closed);
        if next != station {
            for tau in [0.25, 0.5, 0.75] {
                let center = lerp_point(
                    sections.centerline_xy_m[station],
                    sections.centerline_xy_m[next],
                    tau,
                );
                let tangent = normalize_point(
                    lerp_point(tangents[station], tangents[next], tau),
                    tangents[station],
                );
                let left_normal = [-tangent[1], tangent[0]];
                let section_dir = normalize_point(
                    lerp_point(
                        sections.section_dirs_xy[station],
                        sections.section_dirs_xy[next],
                        tau,
                    ),
                    sections.section_dirs_xy[station],
                );
                let section_dir_ds_sample =
                    lerp_point(section_dir_ds[station], section_dir_ds[next], tau);
                let width_left = lerp(
                    sections.width_left_m[station],
                    sections.width_left_m[next],
                    tau,
                );
                let width_right = lerp(
                    sections.width_right_m[station],
                    sections.width_right_m[next],
                    tau,
                );
                let station_s_m = lerp(
                    sections.station_s_m[station],
                    sections.station_s_m[next],
                    tau,
                );
                let curvature_signed = lerp(
                    station_curvature_signed(sections, &tangents, station, closed),
                    station_curvature_signed(sections, &tangents, next, closed),
                    tau,
                );
                let curvature_width_risk = curvature_signed.abs() * width_left.max(width_right);
                let samples = [
                    ("right_edge", -width_right),
                    ("right_mid", -0.5 * width_right),
                    ("center", 0.0),
                    ("left_mid", 0.5 * width_left),
                    ("left_edge", width_left),
                ];
                for (base_label, n_m) in samples {
                    rows.push(station_section_frame_audit_row(
                        station,
                        format!("interval_tau_{tau:.2}_{base_label}"),
                        station_s_m,
                        center,
                        width_left,
                        width_right,
                        n_m,
                        tangent,
                        left_normal,
                        section_dir,
                        section_dir_ds_sample,
                        section_rotation_prev_deg,
                        section_rotation_next_deg,
                        curvature_signed,
                        curvature_width_risk,
                    ));
                }
            }
        }
    }

    rows
}

fn station_section_frame_audit_row(
    station_index: usize,
    sample_label: String,
    station_s_m: f64,
    center_xy_m: Point2,
    width_left_m: f64,
    width_right_m: f64,
    n_m: f64,
    tangent: Point2,
    left_normal: Point2,
    section_dir: Point2,
    section_dir_ds: Point2,
    section_rotation_prev_deg: f64,
    section_rotation_next_deg: f64,
    curvature_signed_1pm: f64,
    curvature_width_risk: f64,
) -> StationSectionFrameAuditRow {
    let progress = section_frame_progress(
        n_m,
        1.0,
        0.0,
        0.0,
        tangent,
        left_normal,
        section_dir,
        section_dir_ds,
    );
    StationSectionFrameAuditRow {
        station_index,
        sample_label,
        station_s_m,
        center_x_m: center_xy_m[0],
        center_y_m: center_xy_m[1],
        width_left_m,
        width_right_m,
        n_m,
        tangent_x: tangent[0],
        tangent_y: tangent[1],
        section_dir_x: section_dir[0],
        section_dir_y: section_dir[1],
        section_dir_ds_x: section_dir_ds[0],
        section_dir_ds_y: section_dir_ds[1],
        section_rotation_prev_deg,
        section_rotation_next_deg,
        curvature_signed_1pm,
        curvature_width_risk,
        section_det: progress.det_geom,
        forward_progress_per_speed: progress.forward_progress_per_speed,
        sigma_dt_ds_at_1mps: progress.sigma_dt_ds,
        dn_ds_at_1mps: progress.dn_ds,
        pure_frenet_factor_debug: 1.0 - n_m * curvature_signed_1pm,
    }
}

fn adjacent_min_distance(points: &[Point2]) -> f64 {
    if points.len() < 2 {
        return f64::INFINITY;
    }
    points
        .windows(2)
        .map(|pair| point_distance(pair[0], pair[1]))
        .fold(f64::INFINITY, f64::min)
}

fn station_sections_are_closed(sections: &SectionsTrackViewV1) -> bool {
    metadata_value(&sections.metadata, "trajectory_mode")
        .and_then(JsonValue::as_str)
        .map(|mode| mode == "closed")
        .unwrap_or_else(|| {
            sections.centerline_xy_m.len() > 2
                && point_distance(
                    sections.centerline_xy_m[0],
                    *sections
                        .centerline_xy_m
                        .last()
                        .unwrap_or(&sections.centerline_xy_m[0]),
                ) < 1.0e-6
        })
}

fn station_centerline_tangents(sections: &SectionsTrackViewV1, closed: bool) -> Vec<Point2> {
    let count = sections.centerline_xy_m.len();
    (0..count)
        .map(|index| {
            let previous = previous_station_index(index, count, closed);
            let next = next_station_index(index, count, closed);
            normalize_point(
                point_sub(
                    sections.centerline_xy_m[next],
                    sections.centerline_xy_m[previous],
                ),
                sections
                    .normals_xy
                    .get(index)
                    .map(|normal| [normal[1], -normal[0]])
                    .unwrap_or([1.0, 0.0]),
            )
        })
        .collect()
}

fn station_section_dir_derivatives(sections: &SectionsTrackViewV1, closed: bool) -> Vec<Point2> {
    let count = sections.section_dirs_xy.len();
    (0..count)
        .map(|index| {
            let previous = previous_station_index(index, count, closed);
            let next = next_station_index(index, count, closed);
            let ds = station_delta_s(sections, previous, next, closed).max(1.0e-9);
            [
                (sections.section_dirs_xy[next][0] - sections.section_dirs_xy[previous][0]) / ds,
                (sections.section_dirs_xy[next][1] - sections.section_dirs_xy[previous][1]) / ds,
            ]
        })
        .collect()
}

fn station_curvature_signed(
    sections: &SectionsTrackViewV1,
    tangents: &[Point2],
    index: usize,
    closed: bool,
) -> f64 {
    let count = tangents.len();
    if count < 2 {
        return 0.0;
    }
    let previous = previous_station_index(index, count, closed);
    let next = next_station_index(index, count, closed);
    let ds = station_delta_s(sections, previous, next, closed).max(1.0e-9);
    let turn =
        cross(tangents[previous], tangents[next]).atan2(dot(tangents[previous], tangents[next]));
    turn / ds
}

fn adjacent_section_rotations_deg(sections: &SectionsTrackViewV1, closed: bool) -> Vec<f64> {
    let count = sections.section_dirs_xy.len();
    if count < 2 {
        return Vec::new();
    }
    let pair_count = if closed { count } else { count - 1 };
    (0..pair_count)
        .map(|index| {
            let next = next_station_index(index, count, closed);
            cross(
                sections.section_dirs_xy[index],
                sections.section_dirs_xy[next],
            )
            .atan2(dot(
                sections.section_dirs_xy[index],
                sections.section_dirs_xy[next],
            ))
            .abs()
            .to_degrees()
        })
        .collect()
}

fn section_rotation_deg(first: Point2, second: Point2) -> f64 {
    cross(first, second)
        .atan2(dot(first, second))
        .abs()
        .to_degrees()
}

fn previous_station_index(index: usize, count: usize, closed: bool) -> usize {
    if count == 0 {
        return 0;
    }
    if index == 0 {
        if closed {
            count - 1
        } else {
            0
        }
    } else {
        index - 1
    }
}

fn next_station_index(index: usize, count: usize, closed: bool) -> usize {
    if count == 0 {
        return 0;
    }
    if index + 1 >= count {
        if closed {
            0
        } else {
            count - 1
        }
    } else {
        index + 1
    }
}

fn station_delta_s(
    sections: &SectionsTrackViewV1,
    previous: usize,
    next: usize,
    closed: bool,
) -> f64 {
    if previous == next {
        return adjacent_min_distance(&sections.centerline_xy_m).max(1.0e-9);
    }
    if next > previous {
        return (sections.station_s_m[next] - sections.station_s_m[previous]).abs();
    }
    if closed {
        let total = sections.station_s_m.last().copied().unwrap_or(0.0)
            + point_distance(
                *sections
                    .centerline_xy_m
                    .last()
                    .unwrap_or(&sections.centerline_xy_m[0]),
                sections.centerline_xy_m[0],
            );
        (total - sections.station_s_m[previous] + sections.station_s_m[next]).abs()
    } else {
        (sections.station_s_m[previous] - sections.station_s_m[next]).abs()
    }
}

fn station_horizon_crossing_pairs(
    left: &[Point2],
    right: &[Point2],
    horizon: usize,
    closed: bool,
) -> Vec<(usize, usize)> {
    let count = left.len().min(right.len());
    let mut pairs = Vec::new();
    for index in 0..count {
        for offset in 1..=horizon.max(1) {
            let next = if closed {
                (index + offset) % count
            } else {
                index + offset
            };
            if next >= count || next == index {
                continue;
            }
            if segment_intersects(left[index], right[index], left[next], right[next]) {
                let pair = if index < next {
                    (index, next)
                } else {
                    (next, index)
                };
                if !pairs.contains(&pair) {
                    pairs.push(pair);
                }
            }
        }
    }
    pairs
}

fn segment_intersects(a: Point2, b: Point2, c: Point2, d: Point2) -> bool {
    const EPS: f64 = 1e-9;
    let o1 = cross(point_sub(b, a), point_sub(c, a));
    let o2 = cross(point_sub(b, a), point_sub(d, a));
    let o3 = cross(point_sub(d, c), point_sub(a, c));
    let o4 = cross(point_sub(d, c), point_sub(b, c));
    if o1 * o2 < -EPS && o3 * o4 < -EPS {
        return true;
    }
    point_on_segment(a, b, c)
        || point_on_segment(a, b, d)
        || point_on_segment(c, d, a)
        || point_on_segment(c, d, b)
}

fn point_on_segment(a: Point2, b: Point2, point: Point2) -> bool {
    const EPS: f64 = 1e-9;
    cross(point_sub(b, a), point_sub(point, a)).abs() <= EPS
        && ranges_overlap(a[0], b[0], point[0], point[0])
        && ranges_overlap(a[1], b[1], point[1], point[1])
}

fn ranges_overlap(a0: f64, a1: f64, b0: f64, b1: f64) -> bool {
    a0.min(a1) <= b0.max(b1) && b0.min(b1) <= a0.max(a1)
}

fn point_sub(a: Point2, b: Point2) -> Point2 {
    [a[0] - b[0], a[1] - b[1]]
}

fn lerp(left: f64, right: f64, tau: f64) -> f64 {
    left + (right - left) * tau
}

fn lerp_point(left: Point2, right: Point2, tau: f64) -> Point2 {
    [lerp(left[0], right[0], tau), lerp(left[1], right[1], tau)]
}

fn dot(a: Point2, b: Point2) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}

fn point_norm(value: Point2) -> f64 {
    value[0].hypot(value[1])
}

fn cross(a: Point2, b: Point2) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}

fn normalize_point(value: Point2, fallback: Point2) -> Point2 {
    let length = point_distance(value, [0.0, 0.0]);
    if length > 1.0e-9 && length.is_finite() {
        [value[0] / length, value[1] / length]
    } else {
        fallback
    }
}

fn point_distance(a: Point2, b: Point2) -> f64 {
    let delta = point_sub(a, b);
    (delta[0] * delta[0] + delta[1] * delta[1]).sqrt()
}

fn percentile_sorted(values: &[f64], fraction: f64) -> f64 {
    if values.is_empty() {
        return 0.0;
    }
    let index = ((values.len() - 1) as f64 * fraction.clamp(0.0, 1.0)).round() as usize;
    values[index.min(values.len() - 1)]
}

fn usize_array_to_json(values: &[usize]) -> JsonValue {
    JsonValue::Array(
        values
            .iter()
            .map(|value| JsonValue::Integer(*value as i64))
            .collect(),
    )
}

fn index_pair_array_to_json(values: &[(usize, usize)]) -> JsonValue {
    JsonValue::Array(
        values
            .iter()
            .map(|(first, second)| {
                JsonValue::Array(vec![
                    JsonValue::Integer(*first as i64),
                    JsonValue::Integer(*second as i64),
                ])
            })
            .collect(),
    )
}

#[must_use]
pub fn build_model_track_area_from_sections(
    raw_track_area: &TrackAreaContractV1,
    sections: &SectionsTrackViewV1,
    contract_role: &str,
) -> TrackAreaContractV1 {
    let mut model_track_area = raw_track_area.clone();
    model_track_area.schema_version = TrackAreaContractV1::SCHEMA_VERSION.to_owned();
    model_track_area.left_boundary_xy_m = sections.left_boundary_xy_m.clone();
    model_track_area.right_boundary_xy_m = sections.right_boundary_xy_m.clone();
    model_track_area.metadata.extend(sections.metadata.clone());
    upsert_metadata(
        &mut model_track_area.metadata,
        "contract_role",
        contract_role.into(),
    );
    upsert_metadata(
        &mut model_track_area.metadata,
        "fact_station_count",
        JsonValue::Integer(model_track_area.left_boundary_xy_m.len() as i64),
    );
    model_track_area
}

pub fn generate_station_geometry(
    request: &StationGenerationRequestV1,
    progress: Option<StationGenerationProgressCallback<'_>>,
) -> StationGenerationResultV1 {
    generate_station_geometry_cancellable(request, progress, None)
        .expect("station generation without a cancellation check cannot be cancelled")
}

pub fn generate_station_geometry_cancellable(
    request: &StationGenerationRequestV1,
    progress: Option<StationGenerationProgressCallback<'_>>,
    cancel_check: Option<StationGenerationCancelCheck<'_>>,
) -> Result<StationGenerationResultV1, StationGenerationExecutionError> {
    let control = cancel_check.map_or_else(
        StationGenerationControl::never_cancelled,
        StationGenerationControl::cancellable,
    );
    generate_station_geometry_with_control(request, progress, cancel_check, control)
}

fn generate_station_geometry_with_control(
    request: &StationGenerationRequestV1,
    mut progress: Option<StationGenerationProgressCallback<'_>>,
    cancel_check: Option<StationGenerationCancelCheck<'_>>,
    control: StationGenerationControl<'_>,
) -> Result<StationGenerationResultV1, StationGenerationExecutionError> {
    ensure_station_generation_active(cancel_check)?;
    let station_plan = prepare_production_station_plan_with_control(
        &request.track_area,
        &request.station_options,
        control,
    )
    .map_err(|_| StationGenerationExecutionError::Cancelled)?;
    ensure_station_generation_active(cancel_check)?;
    let complexity_report =
        (request.count_mode == StationCountMode::Auto).then(|| station_plan.complexity().clone());
    let resolved_station_count = complexity_report
        .as_ref()
        .map_or(request.station_count, |report| {
            report.recommended_station_count
        });
    let mut resolved_station_options = request.station_options.clone();
    resolved_station_options.sample_count = resolved_station_count;
    resolved_station_options.dense_count = resolved_station_options
        .dense_count
        .max((resolved_station_count * 8).max(320));

    emit_station_progress(
        &mut progress,
        StationGenerationProgressEventV1 {
            run_id: request.request_id.clone(),
            phase: "raw_boundaries_received".to_owned(),
            progress: Some(0.0),
            message: Some("station.phase.raw_boundaries_received".to_owned()),
            station_count: Some(resolved_station_count),
            metadata: vec![
                (
                    "track_id".to_owned(),
                    request.track_area.track_id.clone().into(),
                ),
                (
                    "raw_left_boundary_count".to_owned(),
                    JsonValue::Integer(request.track_area.left_boundary_xy_m.len() as i64),
                ),
                (
                    "raw_right_boundary_count".to_owned(),
                    JsonValue::Integer(request.track_area.right_boundary_xy_m.len() as i64),
                ),
            ],
            model_track_area: None,
            diagnostics: Vec::new(),
        },
    );
    ensure_station_generation_active(cancel_check)?;

    let mut sections = build_production_sections_track_view_from_plan_with_control(
        &request.track_area,
        &resolved_station_options,
        station_plan,
        control,
    )
    .map_err(|_| StationGenerationExecutionError::Cancelled)?;
    ensure_station_generation_active(cancel_check)?;
    upsert_metadata(
        &mut sections.metadata,
        "trajectory_mode",
        request.track_area.trajectory_mode.clone().into(),
    );
    let model_track_area = build_model_track_area_from_sections(
        &request.track_area,
        &sections,
        "fact_track_area_from_rust_station_generator",
    );
    let mut diagnostics = station_generation_diagnostics(&sections);
    let topology_audit = audit_station_topology(&sections);
    upsert_metadata(
        &mut diagnostics,
        "topology_audit",
        station_topology_audit_json(&topology_audit),
    );
    upsert_metadata(
        &mut diagnostics,
        "requested_count_mode",
        request.count_mode.as_str().into(),
    );
    upsert_metadata(
        &mut diagnostics,
        "resolved_station_count",
        JsonValue::Integer(resolved_station_count as i64),
    );
    if let Some(report) = &complexity_report {
        upsert_metadata(
            &mut diagnostics,
            "complexity_report",
            station_complexity_report_json(report),
        );
    }

    for (phase, progress_value, diagnostic_keys) in [
        (
            "dense_resample_ready",
            0.25,
            &[
                "dense_count",
                "dense_ray_miss_count",
                "adaptive_density_max_before_slew",
            ][..],
        ),
        (
            "seam_roll_selected",
            0.40,
            &[
                "dtw_alignment_roll_bias_mode",
                "dtw_alignment_roll_bias",
                "dtw_alignment_roll_bias_selected_score",
                "dtw_alignment_roll_bias_selected_crossing_count",
            ][..],
        ),
        (
            "dtw_pairs_ready",
            0.55,
            &[
                "dtw_band",
                "station_frame_source",
                "centerline_projection_clamped_count",
                "adjacent_section_crossing_count",
                "adjacent_section_crossing_count_horizon2",
            ][..],
        ),
        (
            "target_stations_ready",
            0.75,
            &[
                "station_count",
                "density_source",
                "adaptive_density_max",
                "density_max_adjacent_ratio_after",
                "station_ray_miss_count",
            ][..],
        ),
    ] {
        ensure_station_generation_active(cancel_check)?;
        emit_station_progress(
            &mut progress,
            StationGenerationProgressEventV1 {
                run_id: request.request_id.clone(),
                phase: phase.to_owned(),
                progress: Some(progress_value),
                message: Some(format!("station.phase.{phase}")),
                station_count: Some(sections.station_s_m.len()),
                metadata: sections.metadata.clone(),
                model_track_area: None,
                diagnostics: filtered_station_diagnostics(&sections, diagnostic_keys),
            },
        );
    }

    emit_station_progress(
        &mut progress,
        StationGenerationProgressEventV1 {
            run_id: request.request_id.clone(),
            phase: "model_track_area_ready".to_owned(),
            progress: Some(0.90),
            message: Some("station.phase.model_track_area_ready".to_owned()),
            station_count: Some(sections.station_s_m.len()),
            metadata: sections.metadata.clone(),
            model_track_area: Some(model_track_area.clone()),
            diagnostics: diagnostics.clone(),
        },
    );
    ensure_station_generation_active(cancel_check)?;

    Ok(StationGenerationResultV1 {
        request_key: request.request_key.clone(),
        sections_track_view: sections,
        model_track_area,
        diagnostics,
        requested_count_mode: request.count_mode,
        resolved_station_count,
        complexity_report,
        station_options_hash: request.station_options_hash.clone(),
        source_ref: request.source_ref.clone(),
    })
}

fn ensure_station_generation_active(
    cancel_check: Option<StationGenerationCancelCheck<'_>>,
) -> Result<(), StationGenerationExecutionError> {
    if cancel_check.is_some_and(|check| check()) {
        Err(StationGenerationExecutionError::Cancelled)
    } else {
        Ok(())
    }
}

fn station_complexity_report_json(report: &StationComplexityReport) -> JsonValue {
    JsonValue::Object(vec![
        (
            "recommended_station_count".to_owned(),
            JsonValue::Integer(report.recommended_station_count as i64),
        ),
        (
            "complexity_score".to_owned(),
            report.complexity_score.into(),
        ),
        ("route_length_m".to_owned(), report.route_length_m.into()),
        (
            "total_abs_heading_rad".to_owned(),
            report.total_abs_heading_rad.into(),
        ),
        ("width_p10_m".to_owned(), report.width_p10_m.into()),
        ("width_median_m".to_owned(), report.width_median_m.into()),
        (
            "max_segment_to_width_ratio".to_owned(),
            report.max_segment_to_width_ratio.into(),
        ),
        (
            "crossing_zone_count".to_owned(),
            JsonValue::Integer(report.crossing_zone_count as i64),
        ),
        (
            "raw_left_boundary_count".to_owned(),
            JsonValue::Integer(report.raw_left_boundary_count as i64),
        ),
        (
            "raw_right_boundary_count".to_owned(),
            JsonValue::Integer(report.raw_right_boundary_count as i64),
        ),
    ])
}

fn station_topology_audit_json(report: &StationTopologyAuditReport) -> JsonValue {
    JsonValue::Object(vec![
        ("valid".to_owned(), report.valid.into()),
        (
            "issue_count".to_owned(),
            JsonValue::Integer(report.issues.len() as i64),
        ),
        (
            "issues".to_owned(),
            JsonValue::Array(
                report
                    .issues
                    .iter()
                    .map(|issue| {
                        JsonValue::Object(vec![
                            ("code".to_owned(), issue.code.clone().into()),
                            ("message".to_owned(), issue.message.clone().into()),
                            (
                                "diagnostics".to_owned(),
                                JsonValue::Object(issue.diagnostics.clone()),
                            ),
                        ])
                    })
                    .collect(),
            ),
        ),
        (
            "diagnostics".to_owned(),
            JsonValue::Object(report.diagnostics.clone()),
        ),
    ])
}

pub fn generate_station_geometry_json_with_progress(
    input_json: &str,
    progress: Option<StationGenerationProgressCallback<'_>>,
) -> Result<String, String> {
    let request = StationGenerationRequestV1::parse_product(input_json)?;
    let validation_mode = parse_station_validation_mode_from_request_json(input_json)?;
    let mut progress = progress;
    let result = {
        let mut forward_progress = |event| emit_station_progress(&mut progress, event);
        generate_station_geometry(&request, Some(&mut forward_progress))
    };
    validate_station_topology_for_mode(validation_mode, &result.sections_track_view).map_err(
        |issue| {
            format!(
                "{}: {}; diagnostics={}",
                issue.code,
                issue.message,
                JsonValue::Object(issue.diagnostics).to_pretty_string()
            )
        },
    )?;
    let response = station_generation_response_json(&result).to_pretty_string();
    emit_station_validation_passed(&request, &result, &mut progress);
    Ok(response)
}

pub fn generate_station_geometry_json_with_progress_and_cancel(
    input_json: &str,
    progress: Option<StationGenerationProgressCallback<'_>>,
    cancel_check: StationGenerationCancelCheck<'_>,
) -> Result<String, StationGenerationExecutionError> {
    let request = StationGenerationRequestV1::parse_product(input_json)
        .map_err(StationGenerationExecutionError::Invalid)?;
    let validation_mode = parse_station_validation_mode_from_request_json(input_json)
        .map_err(StationGenerationExecutionError::Invalid)?;
    let mut progress = progress;
    let result = {
        let mut forward_progress = |event| emit_station_progress(&mut progress, event);
        generate_station_geometry_cancellable(
            &request,
            Some(&mut forward_progress),
            Some(cancel_check),
        )?
    };
    ensure_station_generation_active(Some(cancel_check))?;
    validate_station_topology_for_mode(validation_mode, &result.sections_track_view).map_err(
        |issue| {
            StationGenerationExecutionError::Invalid(format!(
                "{}: {}; diagnostics={}",
                issue.code,
                issue.message,
                JsonValue::Object(issue.diagnostics).to_pretty_string()
            ))
        },
    )?;
    ensure_station_generation_active(Some(cancel_check))?;
    let response = station_generation_response_json(&result).to_pretty_string();
    ensure_station_generation_active(Some(cancel_check))?;
    emit_station_validation_passed(&request, &result, &mut progress);
    Ok(response)
}

/// Offline compatibility entry point. Mobile/FFI must use the strict product
/// station request handled by `generate_station_geometry_json_with_progress`.
pub fn generate_station_geometry_legacy_json_with_progress(
    input_json: &str,
    progress: Option<StationGenerationProgressCallback<'_>>,
) -> Result<String, String> {
    let request = StationGenerationRequestV1::parse(input_json)?;
    let validation_mode = parse_station_validation_mode_from_request_json(input_json)?;
    let mut progress = progress;
    let result = {
        let mut forward_progress = |event| emit_station_progress(&mut progress, event);
        generate_station_geometry(&request, Some(&mut forward_progress))
    };
    validate_station_topology_for_mode(validation_mode, &result.sections_track_view).map_err(
        |issue| {
            format!(
                "{}: {}; diagnostics={}",
                issue.code,
                issue.message,
                JsonValue::Object(issue.diagnostics).to_pretty_string()
            )
        },
    )?;
    let response = station_generation_response_json(&result).to_pretty_string();
    emit_station_validation_passed(&request, &result, &mut progress);
    Ok(response)
}

fn emit_station_validation_passed(
    request: &StationGenerationRequestV1,
    result: &StationGenerationResultV1,
    progress: &mut Option<StationGenerationProgressCallback<'_>>,
) {
    emit_station_progress(
        progress,
        StationGenerationProgressEventV1 {
            run_id: request.request_id.clone(),
            phase: "station_validation_passed".to_owned(),
            progress: Some(1.0),
            message: Some("station.phase.station_validation_passed".to_owned()),
            station_count: Some(result.sections_track_view.station_s_m.len()),
            metadata: result.sections_track_view.metadata.clone(),
            model_track_area: Some(result.model_track_area.clone()),
            diagnostics: result.diagnostics.clone(),
        },
    );
}

pub fn station_generation_response_json(result: &StationGenerationResultV1) -> JsonValue {
    let route_identity = prepared_route_identity_json(&result.model_track_area);
    let sections_hash = sections_track_view_hash_v2(&result.sections_track_view);
    let recipe = StationRecipeV1 {
        direction: result
            .model_track_area
            .direction
            .clone()
            .unwrap_or_default(),
        station_options_hash: result.station_options_hash.clone(),
        resolved_station_count: result.resolved_station_count,
        generator_contract: STATION_GENERATOR_CONTRACT.to_owned(),
        generator_version: STATION_GENERATOR_VERSION.to_owned(),
        validation_contract: STATION_VALIDATION_CONTRACT.to_owned(),
        validation_version: STATION_VALIDATION_VERSION.to_owned(),
    };
    let prepared_bundle_hash = prepared_station_bundle_hash_v3(
        &result.source_ref,
        &recipe,
        &result.model_track_area.units,
        &result.model_track_area.trajectory_mode,
        result.model_track_area.direction.as_deref(),
        result.model_track_area.start_finish_xy_m.as_ref(),
        result.model_track_area.finish_line_xy_m.as_ref(),
        &sections_hash,
    );
    let source_ref = station_source_ref_json(&result.source_ref);
    let recipe_json = station_recipe_json(&recipe);
    let validation_summary = JsonValue::Object(vec![
        (
            "schema_version".to_owned(),
            "station_validation_summary.v1".into(),
        ),
        (
            "validation_contract".to_owned(),
            STATION_VALIDATION_CONTRACT.into(),
        ),
        (
            "validation_version".to_owned(),
            STATION_VALIDATION_VERSION.into(),
        ),
        ("status".to_owned(), "passed".into()),
        ("error_key".to_owned(), JsonValue::Null),
        (
            "diagnostics".to_owned(),
            JsonValue::Object(result.diagnostics.clone()),
        ),
    ]);
    let bundle = JsonValue::Object(vec![
        (
            "schema_version".to_owned(),
            "prepared_station_bundle.v3".into(),
        ),
        ("source_ref".to_owned(), source_ref.clone()),
        ("recipe".to_owned(), recipe_json),
        ("route_identity".to_owned(), route_identity),
        (
            "sections_hash_algorithm".to_owned(),
            SECTIONS_TRACK_VIEW_HASH_V2.into(),
        ),
        ("sections_track_view_hash".to_owned(), sections_hash.into()),
        (
            "sections_track_view".to_owned(),
            result.sections_track_view.to_json_value(),
        ),
        ("validation_summary".to_owned(), validation_summary),
        (
            "bundle_hash_algorithm".to_owned(),
            PREPARED_STATION_BUNDLE_HASH_V3.into(),
        ),
        ("bundle_hash".to_owned(), prepared_bundle_hash.into()),
    ]);
    JsonValue::Object(vec![
        (
            "schema_version".to_owned(),
            "station_generation_response.v5".into(),
        ),
        ("runtime".to_owned(), "rust_station_generator".into()),
        ("request_key".to_owned(), result.request_key.clone().into()),
        ("source_ref".to_owned(), source_ref),
        (
            "generator_contract".to_owned(),
            STATION_GENERATOR_CONTRACT.into(),
        ),
        (
            "generator_version".to_owned(),
            STATION_GENERATOR_VERSION.into(),
        ),
        (
            "requested_count_mode".to_owned(),
            result.requested_count_mode.as_str().into(),
        ),
        (
            "resolved_station_count".to_owned(),
            JsonValue::Integer(result.resolved_station_count as i64),
        ),
        (
            "complexity_report".to_owned(),
            result
                .complexity_report
                .as_ref()
                .map(station_complexity_report_json)
                .unwrap_or(JsonValue::Null),
        ),
        ("bundle".to_owned(), bundle),
        (
            "diagnostics".to_owned(),
            JsonValue::Object(result.diagnostics.clone()),
        ),
    ])
}

fn station_source_ref_json(source: &StationSourceRefV1) -> JsonValue {
    JsonValue::Object(vec![
        ("schema_version".to_owned(), "station_source_ref.v1".into()),
        ("project_id".to_owned(), source.project_id.clone().into()),
        ("geometry_id".to_owned(), source.geometry_id.clone().into()),
        (
            "geometry_content_hash".to_owned(),
            source.geometry_content_hash.clone().into(),
        ),
        ("route_id".to_owned(), source.route_id.clone().into()),
    ])
}

fn station_recipe_json(recipe: &StationRecipeV1) -> JsonValue {
    JsonValue::Object(vec![
        ("schema_version".to_owned(), "station_recipe.v1".into()),
        ("direction".to_owned(), recipe.direction.clone().into()),
        (
            "station_options_hash".to_owned(),
            recipe.station_options_hash.clone().into(),
        ),
        (
            "resolved_station_count".to_owned(),
            JsonValue::Integer(recipe.resolved_station_count as i64),
        ),
        (
            "generator_contract".to_owned(),
            recipe.generator_contract.clone().into(),
        ),
        (
            "generator_version".to_owned(),
            recipe.generator_version.clone().into(),
        ),
        (
            "validation_contract".to_owned(),
            recipe.validation_contract.clone().into(),
        ),
        (
            "validation_version".to_owned(),
            recipe.validation_version.clone().into(),
        ),
    ])
}

fn prepared_route_identity_json(area: &TrackAreaContractV1) -> JsonValue {
    JsonValue::Object(vec![
        (
            "schema_version".to_owned(),
            "prepared_route_identity.v1".into(),
        ),
        ("track_id".to_owned(), area.track_id.clone().into()),
        ("units".to_owned(), area.units.clone().into()),
        (
            "trajectory_mode".to_owned(),
            area.trajectory_mode.clone().into(),
        ),
        (
            "direction".to_owned(),
            area.direction
                .clone()
                .map(Into::into)
                .unwrap_or(JsonValue::Null),
        ),
        (
            "start_finish_xy_m".to_owned(),
            option_start_finish_to_json(&area.start_finish_xy_m),
        ),
        (
            "finish_line_xy_m".to_owned(),
            option_start_finish_to_json(&area.finish_line_xy_m),
        ),
    ])
}

pub fn station_generation_progress_event_to_json(
    event: &StationGenerationProgressEventV1,
) -> JsonValue {
    JsonValue::Object(vec![
        (
            "schema_version".to_owned(),
            "station_generation_progress_event.v1".into(),
        ),
        ("run_id".to_owned(), event.run_id.clone().into()),
        ("phase".to_owned(), event.phase.clone().into()),
        (
            "progress".to_owned(),
            event.progress.map(Into::into).unwrap_or(JsonValue::Null),
        ),
        (
            "message".to_owned(),
            event
                .message
                .clone()
                .map(Into::into)
                .unwrap_or(JsonValue::Null),
        ),
        (
            "station_count".to_owned(),
            event
                .station_count
                .map(|value| JsonValue::Integer(value as i64))
                .unwrap_or(JsonValue::Null),
        ),
        (
            "metadata".to_owned(),
            JsonValue::Object(event.metadata.clone()),
        ),
        (
            "model_track_area".to_owned(),
            event
                .model_track_area
                .as_ref()
                .map(ToJsonValue::to_json_value)
                .unwrap_or(JsonValue::Null),
        ),
        (
            "diagnostics".to_owned(),
            JsonValue::Object(event.diagnostics.clone()),
        ),
    ])
}

fn emit_station_progress(
    progress: &mut Option<StationGenerationProgressCallback<'_>>,
    event: StationGenerationProgressEventV1,
) {
    if let Some(callback) = progress.as_mut() {
        callback(event);
    }
}

fn station_generation_diagnostics(sections: &SectionsTrackViewV1) -> JsonObject {
    let mut diagnostics = vec![
        (
            "station_count".to_owned(),
            JsonValue::Integer(sections.station_s_m.len() as i64),
        ),
        (
            "quality_metrics".to_owned(),
            JsonValue::Object(sections.quality_metrics.clone()),
        ),
    ];

    for key in [
        "station_geometry_source",
        "trajectory_mode",
        "station_builder",
        "production_station_builder",
        "dense_count",
        "dtw_band",
        "station_frame_source",
        "density_source",
        "dtw_alignment_roll_bias_mode",
        "dtw_alignment_roll_bias",
        "dtw_alignment_roll_bias_selected_score",
        "dtw_alignment_roll_bias_selected_crossing_count",
        "adaptive_density_max",
        "adaptive_density_max_before_slew",
        "density_max_adjacent_ratio_after",
        "centerline_projection_clamped_count",
        "station_ray_miss_count",
        "dense_ray_miss_count",
        "area_preserving_repair_horizon2_crossing_count",
        "area_preserving_repair_all_crossing_count",
        "first_last_gap_m",
        "station_spacing_adjacent_ratio_max",
        "cell_area_adjacent_ratio_max",
    ] {
        if let Some(value) = metadata_value(&sections.metadata, key) {
            diagnostics.push((key.to_owned(), value.clone()));
        }
    }

    diagnostics.extend(station_section_frame_regularity(sections).to_diagnostics());

    diagnostics
}

fn filtered_station_diagnostics(sections: &SectionsTrackViewV1, keys: &[&str]) -> JsonObject {
    let mut diagnostics = Vec::new();

    for key in keys {
        if let Some(value) = metadata_value(&sections.metadata, key) {
            diagnostics.push(((*key).to_owned(), value.clone()));
        }
    }

    diagnostics
}

pub(crate) fn parse_station_options(
    station_count: usize,
    value: Option<&JsonValue>,
) -> Result<FixedCenterlineStationOptions, String> {
    let mut options = FixedCenterlineStationOptions {
        sample_count: station_count,
        ..Default::default()
    };
    let Some(value) = value else {
        return Ok(options);
    };

    if let Some(sample_count) = optional_usize(value, "sample_count")
        .or_else(|| optional_usize(value, "target_station_count"))
        .or_else(|| optional_usize(value, "station_count"))
    {
        options.sample_count = sample_count;
    }
    if let Some(dense_count) = optional_usize(value, "dense_count") {
        options.dense_count = dense_count;
    }
    if value.get("production_station_builder").is_some()
        || value.get("station_builder").is_some()
        || value.get("station_geometry_source").is_some()
    {
        return Err(
            "legacy station builder selection is not supported by the product station contract"
                .to_owned(),
        );
    }
    if let Some(value) = optional_usize(value, "dtw_frame_smoothing_window") {
        options.dtw_frame_smoothing_window = value;
    }
    if let Some(value) = optional_f64(value, "dtw_frame_turn_density_gain") {
        options.dtw_frame_turn_density_gain = value;
    }
    if let Some(value) = optional_f64(value, "dtw_frame_band_ratio") {
        options.dtw_frame_band_ratio = value;
    }
    if let Some(value) = parse_roll_bias(value.get("dtw_frame_alignment_roll_bias"))? {
        options.dtw_frame_alignment_roll_bias = value;
    }
    if let Some(value) = optional_f64(value, "dtw_frame_centerline_normal_cost_weight") {
        options.dtw_frame_centerline_normal_cost_weight = value;
    }
    if let Some(value) = optional_f64(value, "dtw_frame_slide_cost_weight") {
        options.dtw_frame_slide_cost_weight = value;
    }
    if let Some(value) = optional_f64(value, "dtw_frame_slide_step_penalty") {
        options.dtw_frame_slide_step_penalty = value;
    }
    if let Some(value) = optional_f64(value, "dtw_frame_slide_repeat_penalty") {
        options.dtw_frame_slide_repeat_penalty = value;
    }
    if let Some(value) = optional_f64(value, "turn_density_gain") {
        options.dtw_frame_turn_density_gain = value;
    }
    if let Some(value) = optional_usize(value, "turn_analysis_smoothing_window") {
        options.turn_analysis_smoothing_window = value;
    }
    if let Some(value) = optional_string(value, "turn_density_source") {
        options.turn_density_source = value;
    }
    if let Some(value) = optional_usize(value, "density_smooth_window") {
        options.density_smooth_window = value;
    }
    if let Some(value) = optional_f64(value, "density_max_adjacent_ratio") {
        options.density_max_adjacent_ratio = value;
    }
    if let Some(value) = optional_string(value, "density_slew_mode") {
        options.density_slew_mode = value;
    }
    if let Some(value) = optional_f64(value, "target_spacing_max_adjacent_ratio") {
        options.target_spacing_max_adjacent_ratio = value;
    }
    if let Some(value) = optional_string(value, "target_spacing_metric") {
        options.target_spacing_metric = value;
    }
    if let Some(value) = optional_f64(value, "curvature_low_percentile") {
        options.curvature_low_percentile = value.clamp(0.0, 100.0);
    }
    if let Some(value) = optional_f64(value, "curvature_high_percentile") {
        options.curvature_high_percentile = value.clamp(0.0, 100.0);
    }
    if let Some(value) = optional_f64(value, "density_area_length_cap_multiplier") {
        options.density_area_length_cap_multiplier = value.max(0.0);
    }
    if let Some(value) = optional_f64(value, "straight_weight") {
        options.straight_weight = value.max(0.0);
    }
    if let Some(value) = optional_f64(value, "curved_weight") {
        options.curved_weight = value.max(0.0);
    }

    Ok(options)
}

fn parse_station_validation_mode(
    value: Option<&JsonValue>,
) -> Result<StationValidationMode, String> {
    let Some(value) = value else {
        return Ok(StationValidationMode::Strict);
    };
    let Some(raw) = value.as_str() else {
        return Err("station_validation_mode must be a string".to_owned());
    };
    let normalized = raw.trim().to_ascii_lowercase();
    match normalized.as_str() {
        "" | "strict" | "section_frame" | "mintime" | "car_bike" => {
            Ok(StationValidationMode::Strict)
        }
        "point" | "point_mass" | "point_based" | "point-based" => {
            Ok(StationValidationMode::PointMass)
        }
        _ => Err(format!("unsupported station_validation_mode: {raw}")),
    }
}

fn parse_station_validation_mode_from_request_json(
    input_json: &str,
) -> Result<StationValidationMode, String> {
    let value = parse_json_str(input_json).map_err(|error| format!("invalid json: {error}"))?;
    parse_station_validation_mode(
        value
            .get("station_validation_mode")
            .or_else(|| value.get("validation_mode")),
    )
}

fn parse_roll_bias(value: Option<&JsonValue>) -> Result<Option<DtwAlignmentRollBias>, String> {
    let Some(value) = value else {
        return Ok(None);
    };
    if let Some(raw) = value.as_str() {
        return if raw.eq_ignore_ascii_case("auto") {
            Ok(Some(DtwAlignmentRollBias::Auto))
        } else {
            raw.parse::<isize>()
                .map(|number| Some(DtwAlignmentRollBias::Explicit(number)))
                .map_err(|error| format!("invalid dtw_frame_alignment_roll_bias: {error}"))
        };
    }
    if let Some(number) = value.as_f64() {
        if number.is_finite() && number.fract() == 0.0 {
            return Ok(Some(DtwAlignmentRollBias::Explicit(number as isize)));
        }
    }
    Err("dtw_frame_alignment_roll_bias must be `auto` or an integer".to_owned())
}

fn required_field<'a>(value: &'a JsonValue, key: &str) -> Result<&'a JsonValue, String> {
    value
        .get(key)
        .ok_or_else(|| format!("missing required field: {key}"))
}

fn ensure_json_fields(value: &JsonValue, allowed: &[&str], context: &str) -> Result<(), String> {
    let JsonValue::Object(entries) = value else {
        return Err(format!("{context} must be an object"));
    };
    if let Some((key, _)) = entries
        .iter()
        .find(|(key, _)| !allowed.contains(&key.as_str()))
    {
        return Err(format!("unexpected field in {context}: {key}"));
    }
    Ok(())
}

fn required_string(value: &JsonValue, key: &str) -> Result<String, String> {
    value
        .get(key)
        .and_then(JsonValue::as_str)
        .map(str::to_owned)
        .ok_or_else(|| format!("missing string field: {key}"))
}

fn optional_string(value: &JsonValue, key: &str) -> Option<String> {
    value
        .get(key)
        .and_then(JsonValue::as_str)
        .map(str::to_owned)
}

fn optional_f64(value: &JsonValue, key: &str) -> Option<f64> {
    value.get(key).and_then(JsonValue::as_f64)
}

fn optional_usize(value: &JsonValue, key: &str) -> Option<usize> {
    value
        .get(key)
        .and_then(JsonValue::as_u32)
        .map(|value| value as usize)
}

fn metadata_value<'a>(metadata: &'a JsonObject, key: &str) -> Option<&'a JsonValue> {
    metadata
        .iter()
        .find(|(entry_key, _)| entry_key == key)
        .map(|(_, value)| value)
}

fn upsert_metadata(metadata: &mut JsonObject, key: &str, value: JsonValue) {
    if let Some((_, existing)) = metadata.iter_mut().find(|(entry_key, _)| entry_key == key) {
        *existing = value;
    } else {
        metadata.push((key.to_owned(), value));
    }
}

#[cfg(test)]
mod tests {
    use std::cell::Cell;
    use std::sync::atomic::{AtomicBool, Ordering};

    use crate::contracts::{
        station_generation_request_key_v3, station_geometry_content_hash_v1,
        station_geometry_content_hash_v2, station_options_hash_v2, SectionsTrackViewV1,
        StationSourceRefV1, TrackAreaContractV1,
    };
    use crate::json::{parse_json_str, JsonValue};
    use crate::station::{
        build_production_sections_track_view, FixedCenterlineStationOptions,
        StationGenerationControl,
    };
    use crate::ToJsonValue;

    use super::{
        audit_station_topology, generate_station_geometry,
        generate_station_geometry_json_with_progress_and_cancel,
        generate_station_geometry_legacy_json_with_progress,
        generate_station_geometry_with_control, station_generation_response_json, StationCountMode,
        StationGenerationExecutionError, StationGenerationProgressEventV1,
        StationGenerationRequestV1, STATION_GENERATOR_CONTRACT, STATION_GENERATOR_VERSION,
    };

    fn product_station_request_json() -> String {
        let mut track = TrackAreaContractV1::new(
            "product-track",
            vec![[0.0, 0.0], [20.0, 0.0], [20.0, 8.0], [0.0, 8.0]],
            vec![[2.0, 2.0], [18.0, 2.0], [18.0, 6.0], [2.0, 6.0]],
        );
        track.direction = Some("clockwise".to_owned());
        let source_ref = StationSourceRefV1 {
            project_id: "product-project".to_owned(),
            geometry_id: "product-geometry".to_owned(),
            geometry_content_hash: station_geometry_content_hash_v2(&track),
            route_id: track.track_id.clone(),
        };
        let station_options = JsonValue::Object(Vec::new());
        let station_options_hash = station_options_hash_v2(&station_options);
        let request_key = station_generation_request_key_v3(
            &source_ref,
            "exact",
            Some(24),
            "clockwise",
            &station_options_hash,
            STATION_GENERATOR_CONTRACT,
            STATION_GENERATOR_VERSION,
            super::STATION_VALIDATION_CONTRACT,
            super::STATION_VALIDATION_VERSION,
        );
        JsonValue::Object(vec![
            (
                "schema_version".to_owned(),
                "station_generation_request.v4".into(),
            ),
            ("request_id".to_owned(), "product-request".into()),
            ("request_key".to_owned(), request_key.into()),
            ("project_id".to_owned(), "product-project".into()),
            (
                "generator_contract".to_owned(),
                STATION_GENERATOR_CONTRACT.into(),
            ),
            (
                "generator_version".to_owned(),
                STATION_GENERATOR_VERSION.into(),
            ),
            (
                "validation_contract".to_owned(),
                super::STATION_VALIDATION_CONTRACT.into(),
            ),
            (
                "validation_version".to_owned(),
                super::STATION_VALIDATION_VERSION.into(),
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
            ("station_validation_mode".to_owned(), "strict".into()),
            ("count_mode".to_owned(), "exact".into()),
            ("station_count".to_owned(), JsonValue::Integer(24)),
            ("direction".to_owned(), "clockwise".into()),
            ("station_options".to_owned(), station_options),
            (
                "station_options_hash".to_owned(),
                station_options_hash.into(),
            ),
            ("track_area".to_owned(), track.to_json_value()),
        ])
        .to_pretty_string()
    }

    #[test]
    fn product_station_request_rejects_unknown_root_field() {
        let input = product_station_request_json().replacen(
            "\"request_id\": \"product-request\"",
            "\"request_id\": \"product-request\",\n  \"legacy_geometry\": true",
            1,
        );
        let error = StationGenerationRequestV1::parse_product(&input).unwrap_err();
        assert!(error.contains("unexpected field in station_generation_request.v4"));
    }

    #[test]
    fn product_station_request_rejects_unknown_source_field() {
        let input = product_station_request_json().replacen(
            "\"route_id\": \"product-track\"",
            "\"route_id\": \"product-track\",\n    \"legacy_hash\": \"ignored\"",
            1,
        );
        let error = StationGenerationRequestV1::parse_product(&input).unwrap_err();
        assert!(error.contains("unexpected field in station_source_ref.v1"));
    }

    #[test]
    fn product_station_request_rejects_incompatible_generator_identity() {
        let input = product_station_request_json().replacen(
            STATION_GENERATOR_VERSION,
            "obsolete-generator-version",
            1,
        );
        let error = StationGenerationRequestV1::parse_product(&input).unwrap_err();
        assert!(error.contains("incompatible station generator or validation contract"));
    }

    #[test]
    fn cancellation_after_first_progress_emits_no_terminal_generation_stage() {
        let cancelled = AtomicBool::new(false);
        let mut phases = Vec::new();
        let mut progress = |event: StationGenerationProgressEventV1| {
            phases.push(event.phase);
            cancelled.store(true, Ordering::Relaxed);
        };

        let result = generate_station_geometry_json_with_progress_and_cancel(
            &product_station_request_json(),
            Some(&mut progress),
            &|| cancelled.load(Ordering::Relaxed),
        );

        assert_eq!(result, Err(StationGenerationExecutionError::Cancelled));
        assert_eq!(phases, vec!["raw_boundaries_received"]);
    }

    #[test]
    fn cancellation_after_candidate_emits_no_terminal_success() {
        let cancelled = AtomicBool::new(false);
        let mut phases = Vec::new();
        let mut progress = |event: StationGenerationProgressEventV1| {
            if event.phase == "model_track_area_ready" {
                cancelled.store(true, Ordering::Relaxed);
            }
            phases.push(event.phase);
        };

        let result = generate_station_geometry_json_with_progress_and_cancel(
            &product_station_request_json(),
            Some(&mut progress),
            &|| cancelled.load(Ordering::Relaxed),
        );

        assert_eq!(result, Err(StationGenerationExecutionError::Cancelled));
        assert_eq!(
            phases.last().map(String::as_str),
            Some("model_track_area_ready")
        );
        assert!(!phases
            .iter()
            .any(|phase| phase == "station_validation_passed"));
    }

    fn assert_cancellation_at_internal_phase(
        mut request: StationGenerationRequestV1,
        phase: &'static str,
        open: bool,
    ) {
        if open {
            request.track_area.trajectory_mode = "open".to_owned();
        }
        let cancelled = Cell::new(false);
        let phase_hits = Cell::new(0_usize);
        let cancel_check = || cancelled.get();
        let phase_observer = |observed: &'static str| {
            if observed == phase {
                phase_hits.set(phase_hits.get() + 1);
                cancelled.set(true);
            }
        };
        let control = StationGenerationControl::testable(&cancel_check, &phase_observer);
        let mut progress_phases = Vec::new();
        let mut progress = |event: StationGenerationProgressEventV1| {
            progress_phases.push(event.phase);
        };

        let cancelled_result = generate_station_geometry_with_control(
            &request,
            Some(&mut progress),
            Some(&cancel_check),
            control,
        );

        assert_eq!(
            cancelled_result,
            Err(StationGenerationExecutionError::Cancelled),
            "phase={phase}"
        );
        assert_eq!(phase_hits.get(), 1, "phase={phase}");
        assert!(!progress_phases
            .iter()
            .any(|observed| observed == "station_validation_passed"));
    }

    #[test]
    fn cancellation_is_observed_inside_each_heavy_station_phase() {
        let closed = StationGenerationRequestV1::parse_product(&product_station_request_json())
            .expect("closed station request");
        for phase in [
            "closed_dtw",
            "adaptive_resampling",
            "closed_refinement_pass",
            "complexity_preparation",
        ] {
            assert_cancellation_at_internal_phase(closed.clone(), phase, false);
        }
        let mut open = closed.clone();
        assert_cancellation_at_internal_phase(open.clone(), "open_dtw", true);

        let closed_retry = generate_station_geometry(&closed, None);
        assert_eq!(closed_retry.sections_track_view.centerline_xy_m.len(), 24);
        open.track_area.trajectory_mode = "open".to_owned();
        let open_retry = generate_station_geometry(&open, None);
        assert_eq!(open_retry.sections_track_view.centerline_xy_m.len(), 24);
    }

    #[test]
    fn station_generation_progress_emits_ordered_preview_phases() {
        let mut events: Vec<StationGenerationProgressEventV1> = Vec::new();
        let input = r#"{
          "schema_version": "station_generation_request.v1",
          "request_id": "station-progress-test",
          "project_id": "project-1",
          "station_count": 24,
          "track_area": {
            "schema_version": "TrackAreaContractV1",
            "track_id": "oval-fixture",
            "units": "m",
            "left_boundary_xy_m": [[0,0], [20,0], [20,8], [0,8]],
            "right_boundary_xy_m": [[2,2], [18,2], [18,6], [2,6]],
            "trajectory_mode": "closed",
            "direction": "clockwise",
            "metadata": {}
          },
          "station_options": {
            "dense_count": 320,
            "dtw_frame_alignment_roll_bias": "auto"
          }
        }"#;
        let mut callback = |event: StationGenerationProgressEventV1| events.push(event);
        let response =
            generate_station_geometry_legacy_json_with_progress(input, Some(&mut callback))
                .unwrap();

        assert!(response.contains("station_generation_response.v5"));
        assert!(response.contains(STATION_GENERATOR_CONTRACT));
        assert!(response.contains(STATION_GENERATOR_VERSION));
        assert_eq!(
            events
                .iter()
                .map(|event| event.phase.as_str())
                .collect::<Vec<_>>(),
            vec![
                "raw_boundaries_received",
                "dense_resample_ready",
                "seam_roll_selected",
                "dtw_pairs_ready",
                "target_stations_ready",
                "model_track_area_ready",
                "station_validation_passed"
            ]
        );

        let final_event = events.last().expect("final station event must exist");
        assert_eq!(final_event.phase, "station_validation_passed");
        assert_eq!(final_event.progress, Some(1.0));
        assert!(final_event.model_track_area.is_some());
        assert_eq!(final_event.station_count, Some(24));
        assert!(events
            .iter()
            .find(|event| event.phase == "seam_roll_selected")
            .is_some_and(|event| event
                .diagnostics
                .iter()
                .any(|(key, _)| key == "dtw_alignment_roll_bias")));
    }

    #[test]
    fn station_generation_handles_sparse_synthetic_open_corridor() {
        let input = r#"{
          "schema_version": "station_generation_request.v1",
          "request_id": "synthetic-open-corridor-regression",
          "project_id": "synthetic-open-corridor-project",
          "station_count": 160,
          "track_area": {
            "schema_version": "TrackAreaContractV1",
            "track_id": "synthetic-open-corridor-track",
            "units": "m",
            "left_boundary_xy_m": [
              [0.0, 0.0],
              [1.0, 7.0],
              [3.0, 14.0]
            ],
            "right_boundary_xy_m": [
              [5.0, 0.0],
              [6.0, 7.0],
              [8.0, 14.0]
            ],
            "start_finish_xy_m": {
              "p1_m": [0.0, 1.0],
              "p2_m": [5.0, 1.0]
            },
            "finish_line_xy_m": {
              "p1_m": [2.0, 13.0],
              "p2_m": [7.0, 13.0]
            },
            "trajectory_mode": "open",
            "metadata": {}
          },
          "station_options": {
            "straight_weight": 0.5,
            "curved_weight": 1.5,
            "turn_density_source": "centerline",
            "curvature_low_percentile": 35,
            "curvature_high_percentile": 85
          }
        }"#;

        let response = generate_station_geometry_legacy_json_with_progress(input, None)
            .expect("a sparse but valid open corridor must generate stations");
        let value = parse_json_str(&response).expect("station response must be valid JSON");
        let metadata = value
            .get("bundle")
            .and_then(|bundle| bundle.get("sections_track_view"))
            .and_then(|sections| sections.get("metadata"))
            .expect("station response must include section metadata");

        assert!(response.contains("station_generation_response.v5"));
        assert!(response.contains("open_area_station_generator"));
        assert_eq!(
            metadata
                .get("open_repair_synchronized_progress_fallback_count")
                .and_then(JsonValue::as_u32),
            Some(0),
            "the synthetic corridor should be handled without a synchronized-progress fallback"
        );
        assert_eq!(
            metadata
                .get("adjacent_section_crossing_count_horizon2")
                .and_then(JsonValue::as_u32),
            Some(0)
        );
    }

    #[test]
    fn station_generation_rejects_legacy_builder_selector() {
        let input = r#"{
          "schema_version": "station_generation_request.v1",
          "request_id": "station-explicit-builder-test",
          "project_id": "project-1",
          "station_count": 24,
          "track_area": {
            "schema_version": "TrackAreaContractV1",
            "track_id": "rice_manual",
            "units": "m",
            "left_boundary_xy_m": [[0,0], [20,0], [20,8], [0,8]],
            "right_boundary_xy_m": [[2,2], [18,2], [18,6], [2,6]],
            "trajectory_mode": "closed",
            "direction": "clockwise",
            "metadata": {}
          },
          "station_options": {
            "dense_count": 320,
            "production_station_builder": "generated_boundary_pair"
          }
        }"#;

        let error = generate_station_geometry_legacy_json_with_progress(input, None).unwrap_err();
        assert!(error.contains("legacy station builder selection is not supported"));
    }

    #[test]
    fn station_generation_auto_mode_resolves_and_reports_station_count() {
        let track_json = include_str!("../tests/public-fixtures/compact-oval-track-area-v1.json");
        let track_value = parse_json_str(track_json).unwrap();
        let track = TrackAreaContractV1::from_json(&track_value).unwrap();
        let source_ref = StationSourceRefV1 {
            project_id: "project-1".to_owned(),
            geometry_id: "geometry-1".to_owned(),
            geometry_content_hash: station_geometry_content_hash_v1(&track),
            route_id: track.track_id.clone(),
        };
        let request = StationGenerationRequestV1 {
            request_key: "test_station_request".to_owned(),
            request_id: "auto-count-public-oval".to_owned(),
            project_id: "project-1".to_owned(),
            station_count: 160,
            count_mode: StationCountMode::Auto,
            track_area: track,
            station_options: FixedCenterlineStationOptions::default(),
            station_options_hash: "fnv1a_optionstest".to_owned(),
            source_ref,
        };

        let result = generate_station_geometry(&request, None);
        let report = result
            .complexity_report
            .as_ref()
            .expect("auto station generation must include complexity report");

        assert_eq!(result.requested_count_mode, StationCountMode::Auto);
        assert_eq!(
            result.sections_track_view.station_s_m.len(),
            result.resolved_station_count
        );
        assert_eq!(
            result.resolved_station_count,
            report.recommended_station_count
        );
        assert!((64..=80).contains(&result.resolved_station_count));

        let response = station_generation_response_json(&result);
        assert_eq!(
            response
                .get("requested_count_mode")
                .and_then(JsonValue::as_str),
            Some("auto")
        );
        assert_eq!(
            response
                .get("resolved_station_count")
                .and_then(JsonValue::as_u32),
            Some(result.resolved_station_count as u32)
        );
    }

    #[test]
    fn station_topology_audit_reports_all_detected_failures() {
        let sections = SectionsTrackViewV1 {
            schema_version: SectionsTrackViewV1::SCHEMA_VERSION.to_owned(),
            view_id: "invalid-multi-issue".to_owned(),
            track_id: "invalid-multi-issue".to_owned(),
            station_s_m: vec![0.0, 1.0, 2.0, 3.0],
            centerline_xy_m: vec![[1.0, 1.0], [1.0, 1.0], [1.0, 1.0], [1.0, 1.0]],
            left_boundary_xy_m: vec![[0.0, 0.0], [0.0, 0.0], [2.0, 2.0], [0.0, 2.0]],
            right_boundary_xy_m: vec![[2.0, 2.0], [2.0, 2.0], [0.0, 0.0], [2.0, 0.0]],
            normals_xy: vec![[0.0, 1.0]; 4],
            width_left_m: vec![1.0; 4],
            width_right_m: vec![1.0; 4],
            section_dirs_xy: vec![[0.0, -1.0]; 4],
            quality_metrics: Vec::new(),
            metadata: Vec::new(),
        };

        let report = audit_station_topology(&sections);

        assert!(!report.valid);
        assert!(
            report.issues.len() >= 2,
            "audit must preserve multiple simultaneous failures: {report:?}"
        );
        assert!(report
            .issues
            .iter()
            .any(|issue| issue.message.contains("near-zero")));
        assert!(report
            .issues
            .iter()
            .any(|issue| issue.message.contains("crossing")));
    }

    #[test]
    fn station_options_parse_curvature_density_controls() {
        let input = parse_json_str(
            r#"{
              "dtw_frame_turn_density_gain": 2.25,
              "turn_density_source": "boundary_curvature",
              "turn_analysis_smoothing_window": 5,
              "density_smooth_window": 7,
              "density_max_adjacent_ratio": 1.5,
              "density_slew_mode": "peak_preserve",
              "target_spacing_max_adjacent_ratio": 1.25,
              "target_spacing_metric": "section_area",
              "curvature_low_percentile": 30,
              "curvature_high_percentile": 90,
              "density_area_length_cap_multiplier": 2.5,
              "straight_weight": 0.4,
              "curved_weight": 2.2
            }"#,
        )
        .unwrap();

        let options = super::parse_station_options(64, Some(&input)).unwrap();

        assert_eq!(options.sample_count, 64);
        assert_eq!(options.dtw_frame_turn_density_gain, 2.25);
        assert_eq!(options.turn_density_source, "boundary_curvature");
        assert_eq!(options.turn_analysis_smoothing_window, 5);
        assert_eq!(options.density_smooth_window, 7);
        assert_eq!(options.density_max_adjacent_ratio, 1.5);
        assert_eq!(options.density_slew_mode, "peak_preserve");
        assert_eq!(options.target_spacing_max_adjacent_ratio, 1.25);
        assert_eq!(options.target_spacing_metric, "section_area");
        assert_eq!(options.curvature_low_percentile, 30.0);
        assert_eq!(options.curvature_high_percentile, 90.0);
        assert_eq!(options.density_area_length_cap_multiplier, 2.5);
        assert_eq!(options.straight_weight, 0.4);
        assert_eq!(options.curved_weight, 2.2);
    }

    #[test]
    fn station_generation_applies_curvature_density_controls_to_layout() {
        let track_json =
            include_str!("../tests/public-fixtures/asymmetric-loop-track-area-v1.json");
        let track_value = parse_json_str(track_json).unwrap();
        let track = TrackAreaContractV1::from_json(&track_value).unwrap();
        let default_options = FixedCenterlineStationOptions {
            sample_count: 80,
            dense_count: 1200,
            ..FixedCenterlineStationOptions::default()
        };
        let tuned_options = FixedCenterlineStationOptions {
            sample_count: 80,
            dense_count: 1200,
            dtw_frame_turn_density_gain: 5.0,
            turn_density_source: "boundary_curvature".to_owned(),
            density_smooth_window: 1,
            density_max_adjacent_ratio: 0.0,
            target_spacing_max_adjacent_ratio: 0.0,
            ..FixedCenterlineStationOptions::default()
        };

        let default_view = build_production_sections_track_view(&track, &default_options);
        let tuned_view = build_production_sections_track_view(&track, &tuned_options);
        let max_centerline_delta = default_view
            .centerline_xy_m
            .iter()
            .zip(&tuned_view.centerline_xy_m)
            .map(|(left, right)| {
                let dx = left[0] - right[0];
                let dy = left[1] - right[1];
                (dx * dx + dy * dy).sqrt()
            })
            .fold(0.0_f64, f64::max);

        assert_eq!(default_view.centerline_xy_m.len(), 80);
        assert_eq!(tuned_view.centerline_xy_m.len(), 80);
        assert!(
            max_centerline_delta > 1e-3,
            "extreme station density controls should move station placement"
        );
    }

    #[test]
    fn station_generation_rejects_open_track_closed_builders() {
        let input = r#"{
          "schema_version": "station_generation_request.v1",
          "request_id": "open-builder-guard-test",
          "project_id": "project-1",
          "station_count": 24,
          "track_area": {
            "schema_version": "TrackAreaContractV1",
            "track_id": "open-fixture",
            "units": "m",
            "left_boundary_xy_m": [[0,0], [10,2], [20,0]],
            "right_boundary_xy_m": [[0,4], [10,6], [20,4]],
            "start_finish_xy_m": {"p1_m": [0,0], "p2_m": [0,4]},
            "finish_line_xy_m": {"p1_m": [20,0], "p2_m": [20,4]},
            "trajectory_mode": "open",
            "metadata": {}
          },
          "station_options": {
            "production_station_builder": "generated_boundary_pair"
          }
        }"#;

        let error = super::StationGenerationRequestV1::parse(input).unwrap_err();
        assert!(error.contains("legacy station builder selection is not supported"));
    }

    #[test]
    fn station_generation_rejects_open_builder_for_closed_track() {
        let input = r#"{
          "schema_version": "station_generation_request.v1",
          "request_id": "closed-builder-guard-test",
          "project_id": "project-1",
          "station_count": 24,
          "track_area": {
            "schema_version": "TrackAreaContractV1",
            "track_id": "closed-fixture",
            "units": "m",
            "left_boundary_xy_m": [[0,0], [20,0], [20,8], [0,8]],
            "right_boundary_xy_m": [[2,2], [18,2], [18,6], [2,6]],
            "trajectory_mode": "closed",
            "direction": "clockwise",
            "metadata": {}
          },
          "station_options": {
            "production_station_builder": "open_area_station_generator"
          }
        }"#;

        let error = super::StationGenerationRequestV1::parse(input).unwrap_err();
        assert!(error.contains("legacy station builder selection is not supported"));
    }

    #[test]
    fn station_topology_validation_rejects_crossing_station_chords() {
        let sections = SectionsTrackViewV1 {
            schema_version: SectionsTrackViewV1::SCHEMA_VERSION.to_owned(),
            view_id: "crossing-fixture".to_owned(),
            track_id: "crossing-fixture".to_owned(),
            station_s_m: vec![0.0, 1.0],
            centerline_xy_m: vec![[0.4, 0.4], [0.6, 0.4]],
            left_boundary_xy_m: vec![[0.0, 0.0], [1.0, 0.0]],
            right_boundary_xy_m: vec![[1.0, 1.0], [0.0, 1.0]],
            normals_xy: vec![
                [-std::f64::consts::FRAC_1_SQRT_2; 2],
                [
                    std::f64::consts::FRAC_1_SQRT_2,
                    -std::f64::consts::FRAC_1_SQRT_2,
                ],
            ],
            width_left_m: vec![0.4 * std::f64::consts::SQRT_2; 2],
            width_right_m: vec![0.6 * std::f64::consts::SQRT_2; 2],
            section_dirs_xy: vec![
                [std::f64::consts::FRAC_1_SQRT_2; 2],
                [
                    -std::f64::consts::FRAC_1_SQRT_2,
                    std::f64::consts::FRAC_1_SQRT_2,
                ],
            ],
            quality_metrics: Vec::new(),
            metadata: vec![("trajectory_mode".to_owned(), "open".into())],
        };

        let error = super::validate_station_topology(&sections).unwrap_err();

        assert_eq!(error.code, "station.invalidTopology");
        assert!(error.message.contains("crossing station chords"));
    }

    #[test]
    fn point_mass_station_validation_rejects_crossing_station_chords() {
        let sections = SectionsTrackViewV1 {
            schema_version: SectionsTrackViewV1::SCHEMA_VERSION.to_owned(),
            view_id: "point-crossing-fixture".to_owned(),
            track_id: "point-crossing-fixture".to_owned(),
            station_s_m: vec![0.0, 1.0],
            centerline_xy_m: vec![[0.4, 0.4], [0.6, 0.4]],
            left_boundary_xy_m: vec![[0.0, 0.0], [1.0, 0.0]],
            right_boundary_xy_m: vec![[1.0, 1.0], [0.0, 1.0]],
            normals_xy: vec![
                [-std::f64::consts::FRAC_1_SQRT_2; 2],
                [
                    std::f64::consts::FRAC_1_SQRT_2,
                    -std::f64::consts::FRAC_1_SQRT_2,
                ],
            ],
            width_left_m: vec![0.4 * std::f64::consts::SQRT_2; 2],
            width_right_m: vec![0.6 * std::f64::consts::SQRT_2; 2],
            section_dirs_xy: vec![
                [std::f64::consts::FRAC_1_SQRT_2; 2],
                [
                    -std::f64::consts::FRAC_1_SQRT_2,
                    std::f64::consts::FRAC_1_SQRT_2,
                ],
            ],
            quality_metrics: Vec::new(),
            metadata: vec![("trajectory_mode".to_owned(), "open".into())],
        };

        let error = super::validate_station_topology_for_point_mass(&sections).unwrap_err();

        assert_eq!(error.code, "station.invalidTopology");
        assert!(error.message.contains("crossing station chords"));
        assert!(error
            .diagnostics
            .iter()
            .any(|(key, _)| key == "adjacent_section_crossing_pairs_horizon2"));
    }

    #[test]
    fn point_mass_station_validation_rejects_boundary_endpoint_plateau() {
        let sections = SectionsTrackViewV1 {
            schema_version: SectionsTrackViewV1::SCHEMA_VERSION.to_owned(),
            view_id: "point-boundary-plateau-fixture".to_owned(),
            track_id: "point-boundary-plateau-fixture".to_owned(),
            station_s_m: vec![0.0, 1.0, 2.0],
            centerline_xy_m: vec![[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]],
            left_boundary_xy_m: vec![[0.0, 1.0], [1.0, 1.0], [1.0, 1.0]],
            right_boundary_xy_m: vec![[0.0, -1.0], [1.0, -1.0], [2.0, -1.0]],
            normals_xy: vec![[0.0, 1.0]; 3],
            width_left_m: vec![1.0, 1.0, std::f64::consts::SQRT_2],
            width_right_m: vec![1.0; 3],
            section_dirs_xy: vec![[0.0, -1.0]; 3],
            quality_metrics: Vec::new(),
            metadata: vec![("trajectory_mode".to_owned(), "open".into())],
        };

        let strict_error = super::validate_station_topology(&sections).unwrap_err();

        assert_eq!(strict_error.code, "station.invalidTopology");
        assert!(strict_error
            .message
            .contains("near-zero adjacent station spacing"));
        let point_error = super::validate_station_topology_for_point_mass(&sections).unwrap_err();
        assert_eq!(point_error.code, strict_error.code);
        assert_eq!(point_error.message, strict_error.message);
    }

    #[test]
    fn point_mass_station_validation_rejects_centerline_plateau() {
        let sections = SectionsTrackViewV1 {
            schema_version: SectionsTrackViewV1::SCHEMA_VERSION.to_owned(),
            view_id: "point-centerline-plateau-fixture".to_owned(),
            track_id: "point-centerline-plateau-fixture".to_owned(),
            station_s_m: vec![0.0, 1.0, 2.0],
            centerline_xy_m: vec![[0.0, 0.0], [0.0, 0.0], [2.0, 0.0]],
            left_boundary_xy_m: vec![[0.0, 1.0], [1.0, 1.0], [2.0, 1.0]],
            right_boundary_xy_m: vec![[0.0, -1.0], [1.0, -1.0], [2.0, -1.0]],
            normals_xy: vec![[0.0, 1.0]; 3],
            width_left_m: vec![1.0, std::f64::consts::SQRT_2, 1.0],
            width_right_m: vec![1.0, std::f64::consts::SQRT_2, 1.0],
            section_dirs_xy: vec![[0.0, -1.0]; 3],
            quality_metrics: Vec::new(),
            metadata: vec![("trajectory_mode".to_owned(), "open".into())],
        };

        let error = super::validate_station_topology_for_point_mass(&sections).unwrap_err();

        assert_eq!(error.code, "station.invalidTopology");
        assert!(error.message.contains("near-zero adjacent station spacing"));
    }

    #[test]
    fn station_topology_validation_reports_section_frame_regular_track() {
        let sections = SectionsTrackViewV1 {
            schema_version: SectionsTrackViewV1::SCHEMA_VERSION.to_owned(),
            view_id: "regular-section-fixture".to_owned(),
            track_id: "regular-section-fixture".to_owned(),
            station_s_m: vec![0.0, 1.0, 2.0],
            centerline_xy_m: vec![[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]],
            left_boundary_xy_m: vec![[0.0, 1.0], [1.0, 1.0], [2.0, 1.0]],
            right_boundary_xy_m: vec![[0.0, -1.0], [1.0, -1.0], [2.0, -1.0]],
            normals_xy: vec![[0.0, 1.0]; 3],
            width_left_m: vec![1.0; 3],
            width_right_m: vec![1.0; 3],
            section_dirs_xy: vec![[0.0, -1.0]; 3],
            quality_metrics: Vec::new(),
            metadata: vec![("trajectory_mode".to_owned(), "open".into())],
        };

        super::validate_station_topology(&sections).unwrap();
        super::validate_station_section_frame_regular(&sections).unwrap();
        let diagnostics = super::station_generation_diagnostics(&sections);

        assert!(diagnostics
            .iter()
            .any(|(key, _)| key == "section_frame_min_det"));
        assert!(diagnostics
            .iter()
            .any(|(key, _)| key == "section_frame_min_forward_progress"));
        let audit_rows = super::station_section_frame_audit_rows(&sections);
        assert!(audit_rows.len() > sections.station_s_m.len() * 5);
        assert!(audit_rows.iter().any(|row| {
            row.station_index == 1
                && row.sample_label == "center"
                && (row.section_det - 1.0).abs() < 1.0e-12
                && (row.forward_progress_per_speed - 1.0).abs() < 1.0e-12
        }));
        assert!(audit_rows.iter().any(|row| {
            row.station_index == 1
                && row.sample_label == "interval_tau_0.50_center"
                && (row.section_det - 1.0).abs() < 1.0e-12
                && (row.forward_progress_per_speed - 1.0).abs() < 1.0e-12
        }));
    }

    #[test]
    fn station_topology_validation_accepts_consistently_reversed_section_orientation() {
        let sections = SectionsTrackViewV1 {
            schema_version: SectionsTrackViewV1::SCHEMA_VERSION.to_owned(),
            view_id: "reversed-regular-section-fixture".to_owned(),
            track_id: "reversed-regular-section-fixture".to_owned(),
            station_s_m: vec![0.0, 1.0, 2.0],
            centerline_xy_m: vec![[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]],
            left_boundary_xy_m: vec![[0.0, 1.0], [1.0, 1.0], [2.0, 1.0]],
            right_boundary_xy_m: vec![[0.0, -1.0], [1.0, -1.0], [2.0, -1.0]],
            normals_xy: vec![[0.0, -1.0]; 3],
            width_left_m: vec![1.0; 3],
            width_right_m: vec![1.0; 3],
            section_dirs_xy: vec![[0.0, 1.0]; 3],
            quality_metrics: Vec::new(),
            metadata: vec![("trajectory_mode".to_owned(), "open".into())],
        };

        super::validate_station_topology(&sections).unwrap();
        let audit_rows = super::station_section_frame_audit_rows(&sections);
        assert!(audit_rows.iter().all(|row| row.section_det < 0.0));
    }

    #[test]
    fn station_topology_validation_rejects_folded_section_frame_corridor() {
        let sections = SectionsTrackViewV1 {
            schema_version: SectionsTrackViewV1::SCHEMA_VERSION.to_owned(),
            view_id: "folded-section-fixture".to_owned(),
            track_id: "folded-section-fixture".to_owned(),
            station_s_m: vec![0.0, 1.0, 2.0],
            centerline_xy_m: vec![[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]],
            left_boundary_xy_m: vec![[0.0, 2.0], [1.0, 2.0], [2.0, 2.0]],
            right_boundary_xy_m: vec![[0.0, -2.0], [1.0, -2.0], [2.0, -2.0]],
            normals_xy: vec![[0.0, 1.0]; 3],
            width_left_m: vec![2.0; 3],
            width_right_m: vec![2.0; 3],
            section_dirs_xy: vec![[-1.0, 0.0], [0.0, -1.0], [1.0, 0.0]],
            quality_metrics: Vec::new(),
            metadata: vec![("trajectory_mode".to_owned(), "open".into())],
        };

        let error = super::validate_station_topology(&sections).unwrap_err();
        assert_eq!(error.code, "station.invalidSectionFrame");

        let error = super::validate_station_section_frame_regular(&sections).unwrap_err();

        assert_eq!(error.code, "station.invalidSectionFrame");
        assert!(error.message.contains("determinant"));
        assert!(error
            .diagnostics
            .iter()
            .any(|(key, _)| key == "section_frame_min_det_station"));
    }

    #[test]
    fn station_validation_matches_shared_cross_runtime_corpus() {
        let corpus = parse_json_str(include_str!(
            "../tests/public-fixtures/station-validation-contract-v1.json"
        ))
        .unwrap();
        let cases = corpus
            .get("cases")
            .and_then(JsonValue::as_array)
            .expect("validation corpus must contain cases");

        for test_case in cases {
            let id = test_case
                .get("id")
                .and_then(JsonValue::as_str)
                .expect("case must have id");
            let expected = test_case.get("expected").expect("case must have expected");
            let expected_accepted = match expected.get("accepted") {
                Some(JsonValue::Bool(value)) => *value,
                _ => panic!("case {id} must have boolean expected.accepted"),
            };
            let expected_error_key = expected.get("error_key").and_then(JsonValue::as_str);
            let sections = SectionsTrackViewV1::from_json(
                test_case
                    .get("sections_track_view")
                    .expect("case must have sections_track_view"),
            )
            .unwrap_or_else(|error| panic!("case {id} is malformed: {error}"));
            let report = audit_station_topology(&sections);
            let actual_error_key = report.issues.first().map(|issue| issue.code.as_str());

            assert_eq!(report.valid, expected_accepted, "case {id}");
            assert_eq!(actual_error_key, expected_error_key, "case {id}");
        }
    }
}
