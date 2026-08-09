use crate::contracts::{
    station_options_hash_v2, AccelerationEnvelopeV1, PointMassProfileV1, TrackAreaContractV1,
    TrajectoryResultSeriesV1,
};
use crate::json::{parse_json_str, JsonValue};
use crate::mintime::{solve_result_visualization_json, PreparedStationGeometryV3};
use crate::point_mass::{
    solve_point_mass_velocity_vector_ocp_with_progress, EnvelopeCheckPoints,
    PointMassProgressUpdate, PointMassSolveOptions, PointMassSolveResult, PublishGeometryMode,
};
use crate::station_generation::{
    generate_station_geometry, generate_station_geometry_json_with_progress,
    generate_station_geometry_json_with_progress_and_cancel,
    generate_station_geometry_legacy_json_with_progress, station_generation_response_json,
    validate_station_topology_for_point_mass, StationGenerationExecutionError,
    StationGenerationRequestV1,
};
use crate::trajectory_quality::with_unified_trajectory_quality;
use crate::ToJsonValue;

pub use crate::mintime::{
    mintime_progress_event_to_json, MintimeProgressCallback, MintimeProgressEvent,
};
pub use crate::station_generation::{
    station_generation_progress_event_to_json, StationGenerationProgressCallback,
    StationGenerationProgressEventV1,
};

#[derive(Clone, Debug, PartialEq)]
pub struct SolverApiError {
    pub code: String,
    pub message: String,
    pub details: Option<JsonValue>,
}

impl SolverApiError {
    pub fn new(code: impl Into<String>, message: impl Into<String>) -> Self {
        Self {
            code: code.into(),
            message: message.into(),
            details: None,
        }
    }

    pub fn with_details(mut self, details: JsonValue) -> Self {
        self.details = Some(details);
        self
    }

    pub fn to_json_string(&self) -> String {
        let mut entries = vec![
            ("schema_version".to_owned(), "rust_solver_error.v1".into()),
            ("code".to_owned(), self.code.clone().into()),
            ("error".to_owned(), self.message.clone().into()),
        ];
        if let Some(details) = &self.details {
            entries.push(("details".to_owned(), details.clone()));
        }

        JsonValue::Object(entries).to_pretty_string()
    }
}

impl std::fmt::Display for SolverApiError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(formatter, "{}: {}", self.code, self.message)
    }
}

impl std::error::Error for SolverApiError {}

#[derive(Clone, Debug, PartialEq)]
pub struct PointMassProgressEvent {
    pub phase: String,
    pub iteration: Option<u32>,
    pub progress: Option<f64>,
    pub message: Option<String>,
    pub preview_trajectory_result: Option<TrajectoryResultSeriesV1>,
    pub best_lap_time_s: Option<f64>,
    pub model_track_area: Option<TrackAreaContractV1>,
}

pub trait SolverCancelToken {
    fn is_cancelled(&self) -> bool;
}

pub type PointMassProgressCallback<'a> = &'a mut dyn FnMut(PointMassProgressEvent);

pub fn solve_point_mass_json(input_json: &str) -> Result<String, SolverApiError> {
    solve_point_mass_json_with_progress(input_json, None, None)
}

pub fn solve_car_mintime_json(input_json: &str) -> Result<String, SolverApiError> {
    crate::car_mintime::solve_car_mintime_json(input_json)
}

pub fn solve_car_mintime_json_with_progress<'a>(
    input_json: &str,
    progress: Option<MintimeProgressCallback<'a>>,
    cancel_token: Option<&'a dyn SolverCancelToken>,
) -> Result<String, SolverApiError> {
    crate::car_mintime::solve_car_mintime_json_with_progress(input_json, progress, cancel_token)
}

pub fn solve_bike_mintime_json(input_json: &str) -> Result<String, SolverApiError> {
    crate::bike_mintime::solve_bike_mintime_json(input_json)
}

pub fn solve_bike_mintime_json_with_progress<'a>(
    input_json: &str,
    progress: Option<MintimeProgressCallback<'a>>,
    cancel_token: Option<&'a dyn SolverCancelToken>,
) -> Result<String, SolverApiError> {
    crate::bike_mintime::solve_bike_mintime_json_with_progress(input_json, progress, cancel_token)
}

pub fn build_point_mass_track_area_json(input_json: &str) -> Result<String, SolverApiError> {
    build_station_geometry_json(input_json)
}

pub fn build_station_geometry_json(input_json: &str) -> Result<String, SolverApiError> {
    generate_station_geometry_json_with_progress(input_json, None)
        .map_err(|message| SolverApiError::new("solve.invalidRequest", message))
}

pub fn build_station_geometry_json_with_progress(
    input_json: &str,
    progress: Option<crate::station_generation::StationGenerationProgressCallback<'_>>,
) -> Result<String, SolverApiError> {
    generate_station_geometry_json_with_progress(input_json, progress)
        .map_err(|message| SolverApiError::new("solve.invalidRequest", message))
}

pub fn build_station_geometry_json_with_progress_and_cancel(
    input_json: &str,
    progress: Option<crate::station_generation::StationGenerationProgressCallback<'_>>,
    cancel_token: &dyn SolverCancelToken,
) -> Result<String, SolverApiError> {
    generate_station_geometry_json_with_progress_and_cancel(input_json, progress, &|| {
        cancel_token.is_cancelled()
    })
    .map_err(|error| match error {
        StationGenerationExecutionError::Cancelled => {
            SolverApiError::new("solve.cancelled", "station generation cancelled")
        }
        StationGenerationExecutionError::Invalid(message) => {
            SolverApiError::new("solve.invalidRequest", message)
        }
    })
}

pub fn build_station_geometry_legacy_json(input_json: &str) -> Result<String, SolverApiError> {
    generate_station_geometry_legacy_json_with_progress(input_json, None)
        .map_err(|message| SolverApiError::new("solve.invalidRequest", message))
}

pub fn build_station_geometry_response_json(
    request: &StationGenerationRequestV1,
) -> crate::station_generation::StationGenerationResultV1 {
    generate_station_geometry(request, None)
}

pub fn station_generation_result_to_json(
    result: &crate::station_generation::StationGenerationResultV1,
) -> JsonValue {
    station_generation_response_json(result)
}

pub fn solve_point_mass_json_with_progress(
    input_json: &str,
    mut progress: Option<PointMassProgressCallback<'_>>,
    cancel_token: Option<&dyn SolverCancelToken>,
) -> Result<String, SolverApiError> {
    emit_progress(
        &mut progress,
        PointMassProgressEvent {
            phase: "preprocessing".to_owned(),
            iteration: None,
            progress: Some(0.0),
            message: Some("solve.phase.preprocessing".to_owned()),
            preview_trajectory_result: None,
            best_lap_time_s: None,
            model_track_area: None,
        },
    );
    if cancel_token.is_some_and(SolverCancelToken::is_cancelled) {
        return Err(SolverApiError::new(
            "solve.cancelled",
            "solve cancelled before preprocessing",
        ));
    }

    let request = PointMassJsonRequest::parse_product(input_json)?;
    let (sections, model_track_area) = match &request.geometry_input {
        PointMassGeometryInput::PreparedStationGeometry(prepared) => (
            prepared.sections_track_view.clone(),
            prepared.model_track_area(),
        ),
        // v1 stays available for offline fixtures. Mobile sends v2 and never
        // reaches this path.
        PointMassGeometryInput::LegacyRawGeometry(_) => {
            let mut station_request = StationGenerationRequestV1::parse(input_json)
                .map_err(|message| SolverApiError::new("solve.invalidRequest", message))?;
            station_request.station_options.sample_count = request.station_count;
            let station_result = generate_station_geometry(&station_request, None);
            (
                station_result.sections_track_view,
                station_result.model_track_area,
            )
        }
    };
    validate_station_topology_for_point_mass(&sections).map_err(|issue| {
        SolverApiError::new(issue.code, issue.message)
            .with_details(JsonValue::Object(issue.diagnostics))
    })?;

    emit_progress(
        &mut progress,
        PointMassProgressEvent {
            phase: "preprocessing".to_owned(),
            iteration: None,
            progress: Some(1.0),
            message: Some("station.phase.model_track_area_ready".to_owned()),
            preview_trajectory_result: None,
            best_lap_time_s: None,
            model_track_area: Some(model_track_area.clone()),
        },
    );

    emit_progress(
        &mut progress,
        PointMassProgressEvent {
            phase: "running".to_owned(),
            iteration: None,
            progress: None,
            message: Some("solve.phase.running".to_owned()),
            preview_trajectory_result: None,
            best_lap_time_s: None,
            model_track_area: None,
        },
    );
    if cancel_token.is_some_and(SolverCancelToken::is_cancelled) {
        return Err(SolverApiError::new(
            "solve.cancelled",
            "solve cancelled before optimizer start",
        ));
    }

    let mut latest_optimizer_iteration = None;
    let result = {
        let mut solve_progress = |update: PointMassProgressUpdate| match update {
            PointMassProgressUpdate::OptimizerIteration {
                iteration,
                objective_value,
            } => {
                latest_optimizer_iteration = Some(iteration);
                emit_progress(
                    &mut progress,
                    PointMassProgressEvent {
                        phase: "running".to_owned(),
                        iteration: Some(iteration),
                        progress: None,
                        message: Some(format!(
                            "solve.phase.running.objective={objective_value:.6}"
                        )),
                        preview_trajectory_result: None,
                        best_lap_time_s: None,
                        model_track_area: None,
                    },
                );
            }
            PointMassProgressUpdate::Preview(preview) => {
                emit_progress(
                    &mut progress,
                    PointMassProgressEvent {
                        phase: "running".to_owned(),
                        iteration: preview.iteration.or(latest_optimizer_iteration),
                        progress: None,
                        message: Some("solve.phase.running".to_owned()),
                        preview_trajectory_result: Some(preview.series),
                        best_lap_time_s: Some(preview.lap_time_s),
                        model_track_area: None,
                    },
                );
            }
        };

        solve_point_mass_velocity_vector_ocp_with_progress(
            &sections,
            &request.profile,
            &request.envelope,
            request.solve_options.clone(),
            Some(&mut solve_progress),
        )
    }
    .map_err(map_solver_error)?;
    let completed_lap_time_s = result.lap_time_s;
    emit_progress(
        &mut progress,
        PointMassProgressEvent {
            phase: "postprocessing".to_owned(),
            iteration: latest_optimizer_iteration,
            progress: Some(0.0),
            message: Some("solve.phase.postprocessing".to_owned()),
            preview_trajectory_result: None,
            best_lap_time_s: Some(completed_lap_time_s),
            model_track_area: None,
        },
    );
    let response = point_mass_response_json(request, result, &sections);

    emit_progress(
        &mut progress,
        PointMassProgressEvent {
            phase: "completed".to_owned(),
            iteration: latest_optimizer_iteration,
            progress: Some(1.0),
            message: Some("solve.phase.completed".to_owned()),
            preview_trajectory_result: None,
            best_lap_time_s: Some(completed_lap_time_s),
            model_track_area: None,
        },
    );

    Ok(response.to_pretty_string())
}

fn emit_progress(
    progress: &mut Option<PointMassProgressCallback<'_>>,
    event: PointMassProgressEvent,
) {
    if let Some(callback) = progress.as_mut() {
        callback(event);
    }
}

pub fn point_mass_progress_event_to_json(event: &PointMassProgressEvent) -> JsonValue {
    JsonValue::Object(vec![
        (
            "schema_version".to_owned(),
            "native_solver_progress.v1".into(),
        ),
        ("phase".to_owned(), event.phase.clone().into()),
        (
            "iteration".to_owned(),
            event
                .iteration
                .map(|value| JsonValue::Integer(value as i64))
                .unwrap_or(JsonValue::Null),
        ),
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
            "best_lap_time_s".to_owned(),
            event
                .best_lap_time_s
                .map(Into::into)
                .unwrap_or(JsonValue::Null),
        ),
        (
            "preview_trajectory_result".to_owned(),
            event
                .preview_trajectory_result
                .as_ref()
                .map(ToJsonValue::to_json_value)
                .unwrap_or(JsonValue::Null),
        ),
        (
            "model_track_area".to_owned(),
            event
                .model_track_area
                .as_ref()
                .map(ToJsonValue::to_json_value)
                .unwrap_or(JsonValue::Null),
        ),
    ])
}

fn map_solver_error(error: String) -> SolverApiError {
    if error.contains("Ipopt dynamic backend")
        || error.contains("failed to load Ipopt")
        || error.contains("missing Ipopt symbol")
    {
        return SolverApiError::new("solve.nativeBackendUnavailable", error);
    }

    SolverApiError::new("solve.runtimeFailed", error)
}

struct PointMassJsonRequest {
    station_count: usize,
    geometry_input: PointMassGeometryInput,
    profile: PointMassProfileV1,
    envelope: AccelerationEnvelopeV1,
    solve_options: PointMassSolveOptions,
}

#[allow(clippy::large_enum_variant)]
enum PointMassGeometryInput {
    LegacyRawGeometry(TrackAreaContractV1),
    PreparedStationGeometry(PreparedStationGeometryV3),
}

impl PointMassJsonRequest {
    fn track_area(&self) -> TrackAreaContractV1 {
        match &self.geometry_input {
            PointMassGeometryInput::LegacyRawGeometry(area) => area.clone(),
            PointMassGeometryInput::PreparedStationGeometry(prepared) => {
                prepared.model_track_area()
            }
        }
    }

    fn prepared_station_geometry(&self) -> Option<&PreparedStationGeometryV3> {
        match &self.geometry_input {
            PointMassGeometryInput::PreparedStationGeometry(prepared) => Some(prepared),
            PointMassGeometryInput::LegacyRawGeometry(_) => None,
        }
    }
}

impl PointMassJsonRequest {
    fn parse_product(input_json: &str) -> Result<Self, SolverApiError> {
        let value = parse_json_str(input_json).map_err(|error| {
            SolverApiError::new(
                "solve.invalidRequest",
                format!("invalid json request: {error}"),
            )
        })?;
        if optional_string(&value, "schema_version").as_deref()
            != Some("rust_solver_http_request.v5")
        {
            return Err(SolverApiError::new(
                "solve.invalidRequest",
                "unsupported product request version; expected rust_solver_http_request.v5",
            ));
        }
        Self::parse(input_json)
    }

    fn parse(input_json: &str) -> Result<Self, SolverApiError> {
        let value = parse_json_str(input_json).map_err(|error| {
            SolverApiError::new(
                "solve.invalidRequest",
                format!("invalid json request: {error}"),
            )
        })?;
        let schema_version = optional_string(&value, "schema_version");
        if matches!(
            schema_version.as_deref(),
            Some("rust_solver_http_request.v2" | "rust_solver_http_request.v3")
        ) {
            return Err(SolverApiError::new(
                "solve.invalidRequest",
                "obsolete product request version; expected rust_solver_http_request.v5",
            ));
        }
        let prepared_station_geometry = value
            .get("prepared_station_geometry")
            .map(PreparedStationGeometryV3::parse)
            .transpose()?;
        if schema_version.as_deref() == Some("rust_solver_http_request.v5") {
            ensure_point_request_fields(
                &value,
                &[
                    "schema_version",
                    "request_id",
                    "project_id",
                    "source_ref",
                    "station_count",
                    "station_options",
                    "solve_options",
                    "prepared_station_geometry",
                    "point_mass_profile",
                    "acceleration_envelope",
                ],
                "v5 point solver request",
            )?;
            let outer_source = required_field(&value, "source_ref")?;
            ensure_point_request_fields(
                outer_source,
                &[
                    "schema_version",
                    "project_id",
                    "geometry_id",
                    "geometry_content_hash",
                    "route_id",
                ],
                "request source_ref",
            )?;
            if outer_source
                .get("schema_version")
                .and_then(JsonValue::as_str)
                != Some("station_source_ref.v1")
            {
                return Err(SolverApiError::new(
                    "solve.invalidRequest",
                    "unsupported request source_ref schema",
                ));
            }
            let prepared = prepared_station_geometry.as_ref().ok_or_else(|| {
                SolverApiError::new(
                    "solve.invalidRequest",
                    "v5 request requires prepared station geometry",
                )
            })?;
            let source_matches = [
                ("project_id", prepared.source_ref.project_id.as_str()),
                ("geometry_id", prepared.source_ref.geometry_id.as_str()),
                (
                    "geometry_content_hash",
                    prepared.source_ref.geometry_content_hash.as_str(),
                ),
                ("route_id", prepared.source_ref.route_id.as_str()),
            ]
            .iter()
            .all(|(key, expected)| {
                outer_source.get(key).and_then(JsonValue::as_str) == Some(*expected)
            });
            if !source_matches
                || optional_string(&value, "project_id").as_deref()
                    != Some(prepared.source_ref.project_id.as_str())
            {
                return Err(SolverApiError::new(
                    "solve.invalidRequest",
                    "request source_ref does not match prepared station bundle",
                ));
            }
        }
        let geometry_input = match prepared_station_geometry {
            Some(prepared) => PointMassGeometryInput::PreparedStationGeometry(prepared),
            None if schema_version.as_deref() != Some("rust_solver_http_request.v5") => {
                PointMassGeometryInput::LegacyRawGeometry(
                    TrackAreaContractV1::from_json(required_field(&value, "track_area")?)
                        .map_err(|message| SolverApiError::new("solve.invalidRequest", message))?,
                )
            }
            None => unreachable!("v5 prepared geometry requirement checked above"),
        };
        let profile = PointMassProfileV1::from_json(required_field(&value, "point_mass_profile")?)
            .map_err(|message| SolverApiError::new("solve.invalidRequest", message))?;
        let envelope =
            AccelerationEnvelopeV1::from_json(required_field(&value, "acceleration_envelope")?)
                .map_err(|message| SolverApiError::new("solve.invalidRequest", message))?;
        let station_count = required_field(&value, "station_count")?
            .as_u32()
            .ok_or_else(|| {
                SolverApiError::new("solve.invalidRequest", "station_count must be an integer")
            })? as usize;
        if station_count < 20 {
            return Err(SolverApiError::new(
                "solve.invalidRequest",
                "station_count must be at least 20",
            ));
        }
        if let PointMassGeometryInput::PreparedStationGeometry(prepared) = &geometry_input {
            if prepared.sections_track_view.station_s_m.len() != station_count {
                return Err(SolverApiError::new(
                    "solve.invalidRequest",
                    "prepared station geometry station count does not match request",
                ));
            }
            let station_options = value
                .get("station_options")
                .cloned()
                .unwrap_or_else(|| JsonValue::Object(Vec::new()));
            if !matches!(station_options, JsonValue::Object(_))
                || station_options_hash_v2(&station_options) != prepared.station_options_hash
            {
                return Err(SolverApiError::new(
                    "solve.invalidRequest",
                    "solve options do not match prepared station recipe",
                ));
            }
        }
        let solve_options = parse_point_mass_solve_options(value.get("solve_options"))?;

        Ok(Self {
            station_count,
            geometry_input,
            profile,
            envelope,
            solve_options,
        })
    }
}

fn ensure_point_request_fields(
    value: &JsonValue,
    allowed: &[&str],
    context: &str,
) -> Result<(), SolverApiError> {
    let JsonValue::Object(entries) = value else {
        return Err(SolverApiError::new(
            "solve.invalidRequest",
            format!("{context} must be an object"),
        ));
    };
    if let Some((key, _)) = entries
        .iter()
        .find(|(key, _)| !allowed.contains(&key.as_str()))
    {
        return Err(SolverApiError::new(
            "solve.invalidRequest",
            format!("{context} contains unsupported field {key}"),
        ));
    }
    Ok(())
}

fn parse_point_mass_solve_options(
    value: Option<&JsonValue>,
) -> Result<PointMassSolveOptions, SolverApiError> {
    let mut options = PointMassSolveOptions::default();
    let Some(JsonValue::Object(_)) = value else {
        return Ok(options);
    };
    let value = value.expect("checked as object");

    options.n_second_diff_weight =
        optional_f64(value, "n_second_diff_weight").unwrap_or(options.n_second_diff_weight);
    options.velocity_second_diff_weight = optional_f64(value, "q_second_diff_weight")
        .or_else(|| optional_f64(value, "velocity_second_diff_weight"))
        .unwrap_or(options.velocity_second_diff_weight);
    options.control_slew_weight =
        optional_f64(value, "control_slew_weight").unwrap_or(options.control_slew_weight);
    options.g_mps2 = optional_f64(value, "g_mps2").unwrap_or(options.g_mps2);
    options.envelope_safety =
        optional_f64(value, "envelope_safety").unwrap_or(options.envelope_safety);
    options.smooth_abs_eps =
        optional_f64(value, "smooth_abs_eps").unwrap_or(options.smooth_abs_eps);
    options.accel_component_bound_mps2 =
        optional_f64(value, "accel_component_bound_mps2").or(options.accel_component_bound_mps2);
    options.min_segment_time_s =
        optional_f64(value, "min_segment_time_s").unwrap_or(options.min_segment_time_s);
    options.max_segment_time_s =
        optional_f64(value, "max_segment_time_s").unwrap_or(options.max_segment_time_s);
    options.max_iter = optional_u32(value, "max_iter")
        .map(|number| number as i32)
        .unwrap_or(options.max_iter);
    options.tol = optional_f64(value, "tol").unwrap_or(options.tol);
    options.acceptable_tol =
        optional_f64(value, "acceptable_tol").unwrap_or(options.acceptable_tol);
    options.acceptable_iter = optional_u32(value, "acceptable_iter")
        .map(|number| number as i32)
        .unwrap_or(options.acceptable_iter);
    options.ipopt_print_level = optional_u32(value, "ipopt_print_level")
        .map(|number| number as i32)
        .unwrap_or(options.ipopt_print_level);
    if let Some(raw) = value.get("ipopt_linear_solver").and_then(JsonValue::as_str) {
        options.ipopt_linear_solver = Some(raw.to_owned());
    }
    if let Some(raw) = value.get("ipopt_dll_path").and_then(JsonValue::as_str) {
        options.ipopt_dll_path = Some(raw.into());
    }

    if let Some(raw) = value
        .get("envelope_check_points")
        .and_then(JsonValue::as_str)
    {
        options.envelope_check_points = EnvelopeCheckPoints::parse(raw)
            .map_err(|message| SolverApiError::new("solve.invalidRequest", message))?;
    }
    if let Some(raw) = value
        .get("publish_geometry_mode")
        .and_then(JsonValue::as_str)
    {
        options.publish_geometry_mode = PublishGeometryMode::parse(raw)
            .map_err(|message| SolverApiError::new("solve.invalidRequest", message))?;
    }
    options.output_sample_count =
        optional_u32(value, "output_sample_count").map(|number| number as usize);
    options.width_opt_m = optional_f64(value, "width_opt_m")
        .or_else(|| optional_f64(value, "width_opt"))
        .unwrap_or(options.width_opt_m);

    Ok(options)
}

fn point_mass_response_json(
    request: PointMassJsonRequest,
    result: PointMassSolveResult,
    sections: &crate::contracts::SectionsTrackViewV1,
) -> JsonValue {
    let closed = request.track_area().trajectory_mode != "open";
    let mut model_track_area = request.track_area();
    model_track_area.left_boundary_xy_m = sections.left_boundary_xy_m.clone();
    model_track_area.right_boundary_xy_m = sections.right_boundary_xy_m.clone();
    model_track_area.metadata.extend(sections.metadata.clone());
    model_track_area.metadata.push((
        "contract_role".to_owned(),
        "fact_track_area_from_rust_point_mass".into(),
    ));
    model_track_area.metadata.push((
        "fact_station_count".to_owned(),
        JsonValue::Integer(result.series.station_index.as_ref().map_or(0, Vec::len) as i64),
    ));
    if let Some(prepared) = request.prepared_station_geometry() {
        model_track_area.metadata.push((
            "station_geometry_source".to_owned(),
            "prepared_station_geometry".into(),
        ));
        model_track_area.metadata.push((
            "station_geometry_artifact_key".to_owned(),
            prepared.prepared_bundle_hash.clone().into(),
        ));
        model_track_area.metadata.push((
            "sections_track_view_hash".to_owned(),
            prepared.sections_track_view_hash.clone().into(),
        ));
    }

    let trajectory_result = result.series.to_json_value();
    let diagnostics = with_unified_trajectory_quality(
        JsonValue::Object(vec![(
            "schema_version".to_owned(),
            "point_mass_diagnostics.v1".into(),
        )]),
        Some(result.lap_time_s),
        &trajectory_result,
        None,
        closed,
    );

    let mut response_fields = vec![
        (
            "schema_version".to_owned(),
            "rust_solver_response.v1".into(),
        ),
        (
            "runtime".to_owned(),
            "rust_point_mass_envelope_sections".into(),
        ),
        ("status".to_owned(), result.status.into()),
        ("lap_time_estimate_s".to_owned(), result.lap_time_s.into()),
        ("objective_value".to_owned(), result.objective_value.into()),
        ("trajectory_result".to_owned(), trajectory_result),
        (
            "model_track_area".to_owned(),
            model_track_area.to_json_value(),
        ),
        (
            "visualization".to_owned(),
            solve_result_visualization_json(&result.series, closed),
        ),
        ("diagnostics".to_owned(), diagnostics),
        ("warnings".to_owned(), JsonValue::Array(Vec::new())),
    ];
    if !closed {
        response_fields.insert(4, ("open_run_time_s".to_owned(), result.lap_time_s.into()));
    }
    JsonValue::Object(response_fields)
}

fn required_field<'a>(value: &'a JsonValue, key: &str) -> Result<&'a JsonValue, SolverApiError> {
    value.get(key).ok_or_else(|| {
        SolverApiError::new(
            "solve.invalidRequest",
            format!("missing required field: {key}"),
        )
    })
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

fn optional_u32(value: &JsonValue, key: &str) -> Option<u32> {
    value.get(key).and_then(JsonValue::as_u32)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::contracts::{station_geometry_content_hash_v2, StationSourceRefV1};
    use crate::point_mass::PointMassSolveResult;
    use crate::station::FixedCenterlineStationOptions;
    use crate::station_generation::{StationCountMode, StationGenerationRequestV1};
    use std::fs;
    use std::path::Path;

    fn crate_path(relative_path: &str) -> std::path::PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR")).join(relative_path)
    }

    fn metadata_str<'a>(metadata: &'a JsonValue, key: &str) -> &'a str {
        let JsonValue::Object(entries) = metadata else {
            panic!("metadata must be an object");
        };

        entries
            .iter()
            .find(|(entry_key, _)| entry_key == key)
            .and_then(|(_, value)| value.as_str())
            .unwrap_or_else(|| panic!("missing metadata string key {key}"))
    }

    fn point_product_v5_request() -> JsonValue {
        let track_json = fs::read_to_string(crate_path(
            "tests/public-fixtures/compact-oval-track-area-v1.json",
        ))
        .expect("public compact oval fixture must be readable");
        let track = TrackAreaContractV1::from_json(
            &parse_json_str(&track_json).expect("public compact oval fixture must parse"),
        )
        .expect("public compact oval fixture must be a track-area contract");
        let source_ref = StationSourceRefV1 {
            project_id: "10000000-0000-4000-8000-000000000001".to_owned(),
            geometry_id: "10000000-0000-4000-8000-000000000002".to_owned(),
            geometry_content_hash: station_geometry_content_hash_v2(&track),
            route_id: track.track_id.clone(),
        };
        let station_options = JsonValue::Object(Vec::new());
        let station_request = StationGenerationRequestV1 {
            request_id: "synthetic-point-stations".to_owned(),
            request_key: "synthetic-point-stations-exact-24".to_owned(),
            project_id: source_ref.project_id.clone(),
            station_count: 24,
            count_mode: StationCountMode::Exact,
            track_area: track,
            station_options: FixedCenterlineStationOptions::default(),
            station_options_hash: station_options_hash_v2(&station_options),
            source_ref: source_ref.clone(),
        };
        let station_response =
            station_generation_response_json(&generate_station_geometry(&station_request, None));
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
        let profile = PointMassProfileV1 {
            schema_version: PointMassProfileV1::SCHEMA_VERSION.to_owned(),
            profile_id: "synthetic-point-profile".to_owned(),
            model_kind: PointMassProfileV1::MODEL_KIND.to_owned(),
            params: vec![
                ("v_max_mps".to_owned(), 50.0.into()),
                ("ax_forward_max_g".to_owned(), 0.5.into()),
                ("ax_brake_max_g".to_owned(), 1.0.into()),
                ("ay_left_max_g".to_owned(), 1.2.into()),
                ("ay_right_max_g".to_owned(), 1.2.into()),
                ("coupling_exponent".to_owned(), 2.0.into()),
            ],
            metadata: Vec::new(),
        };
        let envelope = profile
            .to_acceleration_envelope(9.81)
            .expect("synthetic point profile must define an envelope");

        JsonValue::Object(vec![
            (
                "schema_version".to_owned(),
                "rust_solver_http_request.v5".into(),
            ),
            ("request_id".to_owned(), "synthetic-point-solve".into()),
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
            ("station_options".to_owned(), station_options),
            ("solve_options".to_owned(), JsonValue::Object(Vec::new())),
            ("prepared_station_geometry".to_owned(), prepared),
            ("point_mass_profile".to_owned(), profile.to_json_value()),
            ("acceleration_envelope".to_owned(), envelope.to_json_value()),
        ])
    }

    fn point_mass_track_area_preview_request(track_path: &Path) -> String {
        let track_json = fs::read_to_string(track_path)
            .unwrap_or_else(|error| panic!("failed to read {}: {error}", track_path.display()));
        let mut track_value = parse_json_str(&track_json).expect("fixture track JSON must parse");
        if let JsonValue::Object(entries) = &mut track_value {
            if !entries.iter().any(|(key, _)| key == "direction") {
                entries.push(("direction".to_owned(), "clockwise".into()));
            }
        }
        let track_json = track_value.to_pretty_string();

        format!(
            r#"{{
              "track_area": {track_json},
              "station_count": 160,
              "point_mass_profile": {{
                "schema_version": "PointMassProfileV1",
                "profile_id": "test_point",
                "model_kind": "point_mass_envelope",
                "params": {{
                  "v_max_mps": 50,
                  "ax_forward_max_g": 0.5,
                  "ax_brake_max_g": 1.0,
                  "ay_left_max_g": 1.2,
                  "ay_right_max_g": 1.2,
                  "coupling_exponent": 2
                }},
                "metadata": {{}}
              }},
              "acceleration_envelope": {{
                "schema_version": "AccelerationEnvelopeV1",
                "envelope_id": "test_envelope",
                "speed_mps": [0, 50],
                "ax_drive_max_mps2": [5, 5],
                "ax_brake_max_mps2": [10, 10],
                "ay_left_max_mps2": [12, 12],
                "ay_right_max_mps2": [12, 12],
                "coupling_exponent": 2,
                "metadata": {{}}
              }}
            }}"#
        )
    }

    #[test]
    fn rejects_invalid_json_with_typed_error() {
        let error = solve_point_mass_json("{").expect_err("invalid json must fail");

        assert_eq!(error.code, "solve.invalidRequest");
        assert!(error.to_json_string().contains("rust_solver_error.v1"));
    }

    #[test]
    fn rejects_missing_station_count_with_typed_error() {
        let error = solve_point_mass_json(
            r#"{
              "track_area": {},
              "point_mass_profile": {},
              "acceleration_envelope": {}
            }"#,
        )
        .expect_err("missing fields must fail");

        assert_eq!(error.code, "solve.invalidRequest");
    }

    #[test]
    fn point_mass_product_requests_require_prepared_station_geometry() {
        for version in [
            "rust_solver_http_request.v2",
            "rust_solver_http_request.v3",
            "rust_solver_http_request.v6",
        ] {
            let result = PointMassJsonRequest::parse_product(&format!(
                r#"{{"schema_version":"{version}"}}"#
            ));
            let Err(error) = result else {
                panic!("{version} must not regenerate stations from raw geometry");
            };

            assert_eq!(error.code, "solve.invalidRequest");
            assert!(error.message.contains("rust_solver_http_request.v5"));
        }
        assert!(PointMassJsonRequest::parse_product("{}").is_err());
    }

    #[test]
    fn point_mass_product_v5_golden_passes_product_parser() {
        PointMassJsonRequest::parse_product(&point_product_v5_request().to_pretty_string())
            .expect("app v5 point request must pass the product parser");
    }

    #[test]
    fn point_mass_product_v5_rejects_station_options_mismatch() {
        let mut request = point_product_v5_request();
        let JsonValue::Object(entries) = &mut request else {
            unreachable!();
        };
        let (_, station_options) = entries
            .iter_mut()
            .find(|(key, _)| key == "station_options")
            .expect("point fixture must contain station options");
        *station_options = JsonValue::Object(vec![(
            "straight_weight".to_owned(),
            JsonValue::Number(99.0),
        )]);

        let Err(error) = PointMassJsonRequest::parse_product(&request.to_pretty_string()) else {
            panic!("tampered station options must not reach the point solver");
        };
        assert_eq!(error.code, "solve.invalidRequest");
        assert!(error.message.contains("prepared station recipe"));
    }

    #[test]
    fn point_mass_product_v5_rejects_unknown_root_field() {
        let mut request = point_product_v5_request();
        let JsonValue::Object(entries) = &mut request else {
            unreachable!();
        };
        entries.push(("raw_track_area".to_owned(), JsonValue::Null));

        let Err(error) = PointMassJsonRequest::parse(&request.to_pretty_string()) else {
            panic!("v5 product request must reject unknown root fields");
        };
        assert_eq!(error.code, "solve.invalidRequest");
        assert!(error.message.contains("raw_track_area"));
    }

    #[test]
    fn point_mass_product_v5_rejects_unknown_source_field() {
        let mut request = point_product_v5_request();
        let JsonValue::Object(root_entries) = &mut request else {
            unreachable!();
        };
        let source = root_entries
            .iter_mut()
            .find(|(key, _)| key == "source_ref")
            .map(|(_, value)| value)
            .expect("fixture must contain source_ref");
        let JsonValue::Object(entries) = source else {
            unreachable!();
        };
        entries.push(("legacy_track_id".to_owned(), "obsolete".into()));

        let Err(error) = PointMassJsonRequest::parse(&request.to_pretty_string()) else {
            panic!("v5 product request must reject unknown source fields");
        };
        assert_eq!(error.code, "solve.invalidRequest");
        assert!(error.message.contains("legacy_track_id"));
    }

    #[test]
    fn point_mass_preview_uses_production_asymmetric_loop_station_selector() {
        let request = point_mass_track_area_preview_request(&crate_path(
            "tests/public-fixtures/asymmetric-loop-track-area-v1.json",
        ));

        let response = build_station_geometry_legacy_json(&request).unwrap();
        let value = parse_json_str(&response).unwrap();
        let sections = value
            .get("bundle")
            .and_then(|bundle| bundle.get("sections_track_view"))
            .unwrap();
        let metadata = sections.get("metadata").unwrap();

        assert_eq!(
            metadata_str(metadata, "station_geometry_source"),
            "universal_area_route_pair"
        );
        assert_eq!(
            metadata_str(metadata, "station_builder"),
            "universal_area_route_pair"
        );
    }

    #[test]
    fn station_geometry_endpoint_response_contains_hashed_sections_and_route_identity() {
        let request = point_mass_track_area_preview_request(&crate_path(
            "tests/public-fixtures/compact-oval-track-area-v1.json",
        ));

        let response = build_station_geometry_legacy_json(&request).unwrap();
        let value = parse_json_str(&response).unwrap();

        assert_eq!(
            value.get("schema_version").and_then(JsonValue::as_str),
            Some("station_generation_response.v5")
        );
        let bundle = value.get("bundle").expect("response must include bundle");
        assert!(bundle.get("sections_track_view").is_some());
        assert_eq!(
            bundle
                .get("sections_hash_algorithm")
                .and_then(JsonValue::as_str),
            Some(crate::contracts::SECTIONS_TRACK_VIEW_HASH_V2)
        );
        assert!(bundle
            .get("sections_track_view_hash")
            .and_then(JsonValue::as_str)
            .is_some());
        assert!(bundle.get("route_identity").is_some());
        assert!(bundle.get("bundle_hash").is_some());
        assert!(value.get("model_track_area").is_none());
        assert!(value.get("diagnostics").is_some());
    }

    #[test]
    fn point_mass_preview_uses_production_compact_oval_station_selector() {
        let request = point_mass_track_area_preview_request(&crate_path(
            "tests/public-fixtures/compact-oval-track-area-v1.json",
        ));

        let response = build_station_geometry_legacy_json(&request).unwrap();
        let value = parse_json_str(&response).unwrap();
        let sections = value
            .get("bundle")
            .and_then(|bundle| bundle.get("sections_track_view"))
            .unwrap();
        let metadata = sections.get("metadata").unwrap();

        assert_eq!(
            metadata_str(metadata, "station_geometry_source"),
            "universal_area_route_pair"
        );
        assert_eq!(
            metadata_str(metadata, "station_builder"),
            "universal_area_route_pair"
        );
    }

    #[test]
    fn open_point_mass_response_exposes_open_runtime_and_unclosed_display() {
        let request = PointMassJsonRequest::parse(
            r#"{
              "track_area": {
                "schema_version": "TrackAreaContractV1",
                "track_id": "open_response_contract",
                "units": "m",
                "trajectory_mode": "open",
                "left_boundary_xy_m": [[0, 2], [10, 2], [20, 2]],
                "right_boundary_xy_m": [[0, -2], [10, -2], [20, -2]],
                "metadata": {}
              },
              "station_count": 20,
              "point_mass_profile": {
                "schema_version": "PointMassProfileV1",
                "profile_id": "test_point",
                "model_kind": "point_mass_envelope",
                "params": {"v_max_mps": 50},
                "metadata": {}
              },
              "acceleration_envelope": {
                "schema_version": "AccelerationEnvelopeV1",
                "envelope_id": "test_envelope",
                "speed_mps": [0, 50],
                "ax_drive_max_mps2": [5, 5],
                "ax_brake_max_mps2": [10, 10],
                "ay_left_max_mps2": [12, 12],
                "ay_right_max_mps2": [12, 12],
                "coupling_exponent": 2,
                "metadata": {}
              }
            }"#,
        )
        .unwrap();
        let sections = crate::contracts::SectionsTrackViewV1 {
            schema_version: crate::contracts::SectionsTrackViewV1::SCHEMA_VERSION.to_owned(),
            view_id: "open_response_sections".to_owned(),
            track_id: "open_response_contract".to_owned(),
            station_s_m: (0..20).map(|index| index as f64).collect(),
            centerline_xy_m: (0..20).map(|index| [index as f64, 0.0]).collect(),
            left_boundary_xy_m: (0..20).map(|index| [index as f64, 2.0]).collect(),
            right_boundary_xy_m: (0..20).map(|index| [index as f64, -2.0]).collect(),
            normals_xy: vec![[0.0, 1.0]; 20],
            width_left_m: vec![2.0; 20],
            width_right_m: vec![2.0; 20],
            section_dirs_xy: vec![[0.0, 1.0]; 20],
            quality_metrics: Vec::new(),
            metadata: vec![("trajectory_mode".to_owned(), "open".into())],
        };
        let series = crate::contracts::TrajectoryResultSeriesV1 {
            s_m: (0..20).map(|index| index as f64).collect(),
            x_m: (0..20).map(|index| index as f64).collect(),
            y_m: vec![0.0; 20],
            heading_rad: vec![0.0; 20],
            kappa_1pm: vec![0.0; 20],
            v_mps: vec![5.0; 20],
            ax_mps2: vec![0.0; 20],
            ay_mps2: vec![0.0; 20],
            utilization_cornering: vec![0.0; 20],
            utilization_longitudinal: vec![0.0; 20],
            utilization_combined: vec![0.0; 20],
            station_index: Some((0..20).collect()),
        };
        let response = point_mass_response_json(
            request,
            PointMassSolveResult {
                series,
                lap_time_s: 4.2,
                status: "Solve_Succeeded".to_owned(),
                objective_value: 4.2,
            },
            &sections,
        );

        assert_eq!(
            response.get("open_run_time_s").and_then(JsonValue::as_f64),
            Some(4.2)
        );
        assert_eq!(
            response
                .get("visualization")
                .and_then(|value| value.get("display_trajectory"))
                .and_then(|value| value.get("closed")),
            Some(&JsonValue::Bool(false))
        );
        assert!(
            response
                .get("diagnostics")
                .and_then(|value| value.get("unified_trajectory_quality"))
                .is_some(),
            "point responses should include unified quality diagnostics"
        );
    }
}
