use crate::contracts::{
    option_start_finish_to_json, optional_start_finish, prepared_station_bundle_hash_v3,
    sections_track_view_hash_v2, station_options_hash_v2, SectionsTrackViewV1, StartFinish,
    StationRecipeV1, StationSourceRefV1, TrackAreaContractV1, TrajectoryResultSeriesV1,
    PREPARED_STATION_BUNDLE_HASH_V3, SECTIONS_TRACK_VIEW_HASH_V2,
};
use crate::json::{parse_json_str, JsonValue};
use crate::solver_api::SolverApiError;
use crate::station_generation::{
    validate_station_topology, STATION_GENERATOR_CONTRACT, STATION_GENERATOR_VERSION,
    STATION_VALIDATION_CONTRACT, STATION_VALIDATION_VERSION,
};
use crate::trajectory_quality::with_unified_trajectory_quality;
use crate::vehicle_dynamics::{VehicleDynamicsModelFamily, VehicleDynamicsProfileV1};
use crate::ToJsonValue;

const BRAKING_POINT_MIN_DECEL_MPS2: f64 = 0.25;
const SPEED_EXTREMA_BASE_SMOOTH_RADIUS_SAMPLES: usize = 3;
const SPEED_EXTREMA_BASE_SAMPLE_COUNT: usize = 160;
const SPEED_EXTREMA_MAX_SMOOTH_RADIUS_SAMPLES: usize = 24;
const DISPLAY_TRAJECTORY_SAMPLES_PER_STATION: usize = 5;
const DISPLAY_TRAJECTORY_GENERATOR_VERSION: &str = "7";

pub const MINTIME_AXIS_COLUMNS: &[&str] = &["s_m", "t_s"];
pub const CAR_DOUBLE_TRACK_STATE_COLUMNS: &[&str] =
    &["v_mps", "beta_rad", "omega_z_radps", "n_m", "xi_rad"];
pub const CAR_DOUBLE_TRACK_CONTROL_COLUMNS: &[&str] =
    &["delta_rad", "f_drive_N", "f_brake_N", "gamma_y_N"];
pub const BIKE_SINGLE_TRACK_LEAN_STATE_COLUMNS: &[&str] = &[
    "v_mps",
    "beta_rad",
    "omega_z_radps",
    "n_m",
    "xi_rad",
    "phi_rad",
];
pub const BIKE_SINGLE_TRACK_LEAN_V2_STATE_COLUMNS: &[&str] = &[
    "v_mps",
    "beta_rad",
    "omega_z_radps",
    "n_m",
    "xi_rad",
    "phi_rad",
    "phi_dot_radps",
];
pub const BIKE_SINGLE_TRACK_LEAN_CONTROL_COLUMNS: &[&str] =
    &["delta_rad", "f_drive_N", "f_brake_N", "phi_dot_radps"];
pub const BIKE_COUNTERSTEER_LEAN_V1_STATE_COLUMNS: &[&str] = &[
    "v_mps",
    "beta_rad",
    "omega_z_radps",
    "n_m",
    "xi_rad",
    "phi_rad",
    "phi_dot_radps",
    "delta_rad",
    "delta_dot_radps",
];
pub const BIKE_COUNTERSTEER_LEAN_V1_CONTROL_COLUMNS: &[&str] =
    &["steering_torque_Nm", "f_drive_N", "f_brake_N"];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MintimeNlpLayout {
    pub model_family: VehicleDynamicsModelFamily,
    pub state_columns: &'static [&'static str],
    pub control_columns: &'static [&'static str],
}

impl MintimeNlpLayout {
    #[must_use]
    pub fn for_family(model_family: VehicleDynamicsModelFamily) -> Self {
        match model_family {
            VehicleDynamicsModelFamily::CarDynamics => Self {
                model_family,
                state_columns: CAR_DOUBLE_TRACK_STATE_COLUMNS,
                control_columns: CAR_DOUBLE_TRACK_CONTROL_COLUMNS,
            },
            VehicleDynamicsModelFamily::BikeDynamics => Self {
                model_family,
                state_columns: BIKE_SINGLE_TRACK_LEAN_V2_STATE_COLUMNS,
                control_columns: BIKE_SINGLE_TRACK_LEAN_CONTROL_COLUMNS,
            },
        }
    }

    pub fn for_python_model_family(value: &str) -> Result<Self, String> {
        match value {
            "car_double_track" => Ok(Self::for_family(VehicleDynamicsModelFamily::CarDynamics)),
            "bike_single_track_lean_v2" => {
                Ok(Self::for_family(VehicleDynamicsModelFamily::BikeDynamics))
            }
            "bike_countersteer_lean_v1" => Ok(Self {
                model_family: VehicleDynamicsModelFamily::BikeDynamics,
                state_columns: BIKE_COUNTERSTEER_LEAN_V1_STATE_COLUMNS,
                control_columns: BIKE_COUNTERSTEER_LEAN_V1_CONTROL_COLUMNS,
            }),
            "bike_single_track_lean" => Ok(Self {
                model_family: VehicleDynamicsModelFamily::BikeDynamics,
                state_columns: BIKE_SINGLE_TRACK_LEAN_STATE_COLUMNS,
                control_columns: BIKE_SINGLE_TRACK_LEAN_CONTROL_COLUMNS,
            }),
            _ => Err(format!("unsupported Python mintime model family: {value}")),
        }
    }

    #[must_use]
    pub fn expected_state_csv_columns(self) -> Vec<&'static str> {
        MINTIME_AXIS_COLUMNS
            .iter()
            .chain(self.state_columns.iter())
            .copied()
            .collect()
    }

    #[must_use]
    pub fn expected_control_csv_columns(self) -> Vec<&'static str> {
        MINTIME_AXIS_COLUMNS
            .iter()
            .chain(self.control_columns.iter())
            .copied()
            .collect()
    }

    #[must_use]
    pub fn dimensions_for_station_count(
        self,
        station_count: usize,
        closed: bool,
    ) -> MintimeNlpDimensions {
        let interval_count = if closed {
            station_count
        } else {
            station_count.saturating_sub(1)
        };

        MintimeNlpDimensions {
            station_count,
            interval_count,
            state_variable_count: station_count * self.state_columns.len(),
            control_variable_count: interval_count * self.control_columns.len(),
            collocation_state_variable_count: 0,
        }
    }

    pub fn validate_state_csv_columns(self, actual: &[String]) -> Result<(), String> {
        validate_columns("states.csv", &self.expected_state_csv_columns(), actual)
    }

    pub fn validate_control_csv_columns(self, actual: &[String]) -> Result<(), String> {
        validate_columns("controls.csv", &self.expected_control_csv_columns(), actual)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MintimeNlpDimensions {
    pub station_count: usize,
    pub interval_count: usize,
    pub state_variable_count: usize,
    pub control_variable_count: usize,
    pub collocation_state_variable_count: usize,
}

impl MintimeNlpDimensions {
    #[must_use]
    pub fn decision_variable_count(self) -> usize {
        self.state_variable_count
            + self.control_variable_count
            + self.collocation_state_variable_count
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct MintimeSolveRequestV1 {
    pub request_id: String,
    pub project_id: String,
    pub station_count: usize,
    pub geometry_input: MintimeGeometryInput,
    pub vehicle_dynamics_profile: VehicleDynamicsProfileV1,
    pub solve_options: Vec<(String, JsonValue)>,
}

#[derive(Clone, Debug, PartialEq)]
#[allow(clippy::large_enum_variant)]
pub enum MintimeGeometryInput {
    LegacyRawGeometry(TrackAreaContractV1),
    PreparedStationGeometry(PreparedStationGeometryV3),
}

impl MintimeSolveRequestV1 {
    #[must_use]
    pub fn track_area(&self) -> TrackAreaContractV1 {
        match &self.geometry_input {
            MintimeGeometryInput::LegacyRawGeometry(area) => area.clone(),
            MintimeGeometryInput::PreparedStationGeometry(prepared) => prepared.model_track_area(),
        }
    }

    #[must_use]
    pub fn prepared_station_geometry(&self) -> Option<&PreparedStationGeometryV3> {
        match &self.geometry_input {
            MintimeGeometryInput::PreparedStationGeometry(prepared) => Some(prepared),
            MintimeGeometryInput::LegacyRawGeometry(_) => None,
        }
    }
}

/// Exact station geometry prepared before a production solve.  The solver may
/// validate this object, but must not rebuild it from raw boundary polylines.
#[derive(Clone, Debug, PartialEq)]
pub struct PreparedStationGeometryV3 {
    pub source_ref: StationSourceRefV1,
    pub prepared_bundle_hash: String,
    pub prepared_bundle_hash_algorithm: String,
    pub sections_track_view_hash: String,
    pub sections_hash_algorithm: String,
    pub station_options_hash: String,
    pub direction: String,
    pub generator_contract: String,
    pub generator_version: String,
    pub validation_contract: String,
    pub validation_version: String,
    pub resolved_station_count: usize,
    pub route_identity: PreparedRouteIdentityV1,
    pub sections_track_view: SectionsTrackViewV1,
}

#[derive(Clone, Debug, PartialEq)]
pub struct PreparedRouteIdentityV1 {
    pub track_id: String,
    pub units: String,
    pub trajectory_mode: String,
    pub direction: Option<String>,
    pub start_finish_xy_m: Option<StartFinish>,
    pub finish_line_xy_m: Option<StartFinish>,
}

impl PreparedStationGeometryV3 {
    pub fn parse(value: &JsonValue) -> Result<Self, SolverApiError> {
        ensure_prepared_fields(
            value,
            &[
                "schema_version",
                "requested_count_mode",
                "resolved_station_count",
                "complexity_report",
                "bundle",
                "diagnostics",
            ],
            "prepared station geometry",
        )?;
        if required_string(value, "schema_version")? != "prepared_station_geometry.v4" {
            return Err(SolverApiError::new(
                "solve.invalidPreparedStationGeometry",
                "unsupported prepared station geometry schema",
            ));
        }
        let bundle = required_field(value, "bundle")?;
        ensure_prepared_fields(
            bundle,
            &[
                "schema_version",
                "source_ref",
                "recipe",
                "route_identity",
                "sections_hash_algorithm",
                "sections_track_view_hash",
                "sections_track_view",
                "validation_summary",
                "bundle_hash_algorithm",
                "bundle_hash",
            ],
            "prepared station bundle",
        )?;
        if required_string(bundle, "schema_version")? != "prepared_station_bundle.v3" {
            return Err(SolverApiError::new(
                "solve.invalidPreparedStationGeometry",
                "unsupported prepared station bundle schema",
            ));
        }
        let source_value = required_field(bundle, "source_ref")?;
        ensure_prepared_fields(
            source_value,
            &[
                "schema_version",
                "project_id",
                "geometry_id",
                "geometry_content_hash",
                "route_id",
            ],
            "station source reference",
        )?;
        if required_string(source_value, "schema_version")? != "station_source_ref.v1" {
            return Err(SolverApiError::new(
                "solve.invalidPreparedStationGeometry",
                "unsupported station source reference",
            ));
        }
        let source_ref = StationSourceRefV1 {
            project_id: required_string(source_value, "project_id")?,
            geometry_id: required_string(source_value, "geometry_id")?,
            geometry_content_hash: required_string(source_value, "geometry_content_hash")?,
            route_id: required_string(source_value, "route_id")?,
        };
        if !is_contract_uuid(&source_ref.project_id) || !is_contract_uuid(&source_ref.geometry_id) {
            return Err(SolverApiError::new(
                "solve.invalidPreparedStationGeometry",
                "station source project_id and geometry_id must be UUIDs",
            ));
        }
        let recipe_value = required_field(bundle, "recipe")?;
        ensure_prepared_fields(
            recipe_value,
            &[
                "schema_version",
                "direction",
                "station_options_hash",
                "resolved_station_count",
                "generator_contract",
                "generator_version",
                "validation_contract",
                "validation_version",
            ],
            "station recipe",
        )?;
        if required_string(recipe_value, "schema_version")? != "station_recipe.v1" {
            return Err(SolverApiError::new(
                "solve.invalidPreparedStationGeometry",
                "unsupported station recipe",
            ));
        }
        let resolved_station_count = required_usize(recipe_value, "resolved_station_count")?;
        let recipe = StationRecipeV1 {
            direction: required_string(recipe_value, "direction")?,
            station_options_hash: required_string(recipe_value, "station_options_hash")?,
            resolved_station_count,
            generator_contract: required_string(recipe_value, "generator_contract")?,
            generator_version: required_string(recipe_value, "generator_version")?,
            validation_contract: required_string(recipe_value, "validation_contract")?,
            validation_version: required_string(recipe_value, "validation_version")?,
        };
        let sections_value = required_field(bundle, "sections_track_view")?;
        ensure_prepared_fields(
            sections_value,
            &[
                "schema_version",
                "view_id",
                "track_id",
                "station_s_m",
                "centerline_xy_m",
                "left_boundary_xy_m",
                "right_boundary_xy_m",
                "normals_xy",
                "width_left_m",
                "width_right_m",
                "section_dirs_xy",
                "quality_metrics",
                "metadata",
            ],
            "sections track view",
        )?;
        let sections_track_view =
            SectionsTrackViewV1::from_json(sections_value).map_err(|message| {
                SolverApiError::new("solve.invalidPreparedStationGeometry", message)
            })?;
        let route_identity =
            PreparedRouteIdentityV1::parse(required_field(bundle, "route_identity")?)?;

        let prepared_bundle_hash = required_string(bundle, "bundle_hash")?;
        let prepared_bundle_hash_algorithm = required_string(bundle, "bundle_hash_algorithm")?;
        let sections_track_view_hash = required_string(bundle, "sections_track_view_hash")?;
        let sections_hash_algorithm = required_string(bundle, "sections_hash_algorithm")?;
        let station_options_hash = recipe.station_options_hash.clone();
        let generator_contract = recipe.generator_contract.clone();
        let generator_version = recipe.generator_version.clone();
        let validation_contract = recipe.validation_contract.clone();
        let validation_version = recipe.validation_version.clone();

        if sections_hash_algorithm != SECTIONS_TRACK_VIEW_HASH_V2 {
            return Err(SolverApiError::new(
                "solve.invalidPreparedStationGeometry",
                "unsupported prepared sections hash algorithm",
            ));
        }
        if prepared_bundle_hash_algorithm != PREPARED_STATION_BUNDLE_HASH_V3 {
            return Err(SolverApiError::new(
                "solve.invalidPreparedStationGeometry",
                "unsupported prepared station bundle hash algorithm",
            ));
        }
        if generator_contract != STATION_GENERATOR_CONTRACT {
            return Err(SolverApiError::new(
                "solve.invalidPreparedStationGeometry",
                "unsupported prepared station generator contract",
            ));
        }
        if generator_version != STATION_GENERATOR_VERSION {
            return Err(SolverApiError::new(
                "solve.invalidPreparedStationGeometry",
                "unsupported prepared station generator version",
            ));
        }
        if validation_contract != STATION_VALIDATION_CONTRACT
            || validation_version != STATION_VALIDATION_VERSION
        {
            return Err(SolverApiError::new(
                "solve.invalidPreparedStationGeometry",
                "unsupported station validation contract",
            ));
        }
        let validation_summary = required_field(bundle, "validation_summary")?;
        ensure_prepared_fields(
            validation_summary,
            &[
                "schema_version",
                "validation_contract",
                "validation_version",
                "status",
                "error_key",
                "diagnostics",
            ],
            "station validation summary",
        )?;
        if required_string(validation_summary, "schema_version")? != "station_validation_summary.v1"
            || required_string(validation_summary, "validation_contract")? != validation_contract
            || required_string(validation_summary, "validation_version")? != validation_version
            || required_string(validation_summary, "status")? != "passed"
            || !matches!(validation_summary.get("error_key"), Some(JsonValue::Null))
        {
            return Err(SolverApiError::new(
                "solve.invalidPreparedStationGeometry",
                "invalid station validation summary",
            ));
        }

        if sections_track_view.station_s_m.len() < 2 {
            return Err(SolverApiError::new(
                "solve.invalidPreparedStationGeometry",
                "prepared station geometry requires at least two stations",
            ));
        }
        if sections_track_view.track_id != route_identity.track_id {
            return Err(SolverApiError::new(
                "solve.invalidPreparedStationGeometry",
                "prepared route and sections have different track ids",
            ));
        }
        if source_ref.route_id != route_identity.track_id
            || source_ref.route_id != sections_track_view.track_id
        {
            return Err(SolverApiError::new(
                "solve.invalidPreparedStationGeometry",
                "station source route does not match prepared geometry",
            ));
        }
        if sections_track_view.station_s_m.len() != resolved_station_count
            || required_usize(value, "resolved_station_count")? != resolved_station_count
        {
            return Err(SolverApiError::new(
                "solve.invalidPreparedStationGeometry",
                "resolved station count does not match prepared sections",
            ));
        }
        let sections_topology = sections_track_view
            .metadata
            .iter()
            .find(|(key, _)| key == "trajectory_mode")
            .and_then(|(_, value)| value.as_str())
            .ok_or_else(|| {
                SolverApiError::new(
                    "solve.invalidPreparedStationGeometry",
                    "prepared sections require trajectory_mode metadata",
                )
            })?;
        if !matches!(sections_topology, "open" | "closed") {
            return Err(SolverApiError::new(
                "solve.invalidPreparedStationGeometry",
                "prepared sections contain unsupported trajectory_mode",
            ));
        }
        if sections_topology != route_identity.trajectory_mode {
            return Err(SolverApiError::new(
                "solve.invalidPreparedStationGeometry",
                "prepared topology does not match sections_track_view",
            ));
        }
        let requested_direction = sections_track_view
            .metadata
            .iter()
            .find(|(key, _)| key == "requested_direction")
            .and_then(|(_, value)| value.as_str());
        if !matches!(recipe.direction.as_str(), "clockwise" | "counterclockwise") {
            return Err(SolverApiError::new(
                "solve.invalidPreparedStationGeometry",
                "prepared recipe contains unsupported direction",
            ));
        }
        if sections_topology == "closed" && requested_direction.is_none() {
            return Err(SolverApiError::new(
                "solve.invalidPreparedStationGeometry",
                "closed prepared sections require requested_direction metadata",
            ));
        }
        if let Some(requested_direction) = requested_direction {
            if !matches!(requested_direction, "clockwise" | "counterclockwise") {
                return Err(SolverApiError::new(
                    "solve.invalidPreparedStationGeometry",
                    "prepared sections contain unsupported requested_direction",
                ));
            }
            if route_identity.direction.as_deref() != Some(requested_direction) {
                return Err(SolverApiError::new(
                    "solve.invalidPreparedStationGeometry",
                    "prepared direction does not match sections_track_view",
                ));
            }
            if recipe.direction != requested_direction {
                return Err(SolverApiError::new(
                    "solve.invalidPreparedStationGeometry",
                    "prepared recipe direction does not match sections_track_view",
                ));
            }
        }
        if route_identity
            .direction
            .as_deref()
            .is_some_and(|direction| direction != recipe.direction)
        {
            return Err(SolverApiError::new(
                "solve.invalidPreparedStationGeometry",
                "prepared recipe direction does not match route identity",
            ));
        }
        let computed_sections_hash = sections_track_view_hash_v2(&sections_track_view);
        if computed_sections_hash != sections_track_view_hash {
            return Err(SolverApiError::new(
                "solve.invalidPreparedStationGeometry",
                "prepared sections hash does not match sections_track_view",
            ));
        }
        let computed_bundle_hash = prepared_station_bundle_hash_v3(
            &source_ref,
            &recipe,
            &route_identity.units,
            &route_identity.trajectory_mode,
            route_identity.direction.as_deref(),
            route_identity.start_finish_xy_m.as_ref(),
            route_identity.finish_line_xy_m.as_ref(),
            &sections_track_view_hash,
        );
        if computed_bundle_hash != prepared_bundle_hash {
            return Err(SolverApiError::new(
                "solve.invalidPreparedStationGeometry",
                "prepared bundle hash does not match route and sections",
            ));
        }
        validate_station_topology(&sections_track_view).map_err(|issue| {
            SolverApiError::new(issue.code, issue.message)
                .with_details(JsonValue::Object(issue.diagnostics))
        })?;

        Ok(Self {
            source_ref,
            prepared_bundle_hash,
            prepared_bundle_hash_algorithm,
            sections_track_view_hash,
            sections_hash_algorithm,
            station_options_hash,
            direction: recipe.direction,
            generator_contract,
            generator_version,
            validation_contract,
            validation_version,
            resolved_station_count,
            route_identity,
            sections_track_view,
        })
    }

    #[must_use]
    pub fn model_track_area(&self) -> TrackAreaContractV1 {
        TrackAreaContractV1 {
            schema_version: TrackAreaContractV1::SCHEMA_VERSION.to_owned(),
            track_id: self.route_identity.track_id.clone(),
            units: self.route_identity.units.clone(),
            left_boundary_xy_m: self.sections_track_view.left_boundary_xy_m.clone(),
            right_boundary_xy_m: self.sections_track_view.right_boundary_xy_m.clone(),
            start_finish_xy_m: self.route_identity.start_finish_xy_m.clone(),
            finish_line_xy_m: self.route_identity.finish_line_xy_m.clone(),
            trajectory_mode: self.route_identity.trajectory_mode.clone(),
            direction: self.route_identity.direction.clone(),
            metadata: self.sections_track_view.metadata.clone(),
            image_path: None,
            image_width_px: None,
            image_height_px: None,
            meters_per_pixel: None,
        }
    }
}

impl PreparedRouteIdentityV1 {
    fn parse(value: &JsonValue) -> Result<Self, SolverApiError> {
        ensure_prepared_fields(
            value,
            &[
                "schema_version",
                "track_id",
                "units",
                "trajectory_mode",
                "direction",
                "start_finish_xy_m",
                "finish_line_xy_m",
            ],
            "prepared route identity",
        )?;
        if required_string(value, "schema_version")? != "prepared_route_identity.v1" {
            return Err(SolverApiError::new(
                "solve.invalidPreparedStationGeometry",
                "unsupported prepared route identity schema",
            ));
        }
        let trajectory_mode = required_string(value, "trajectory_mode")?;
        if !matches!(trajectory_mode.as_str(), "open" | "closed") {
            return Err(SolverApiError::new(
                "solve.invalidPreparedStationGeometry",
                "unsupported prepared trajectory_mode",
            ));
        }
        let units = required_string(value, "units")?;
        if units != "m" {
            return Err(SolverApiError::new(
                "solve.invalidPreparedStationGeometry",
                "unsupported prepared route units",
            ));
        }
        let direction = optional_string(value, "direction");
        if direction
            .as_deref()
            .is_some_and(|value| !matches!(value, "clockwise" | "counterclockwise"))
        {
            return Err(SolverApiError::new(
                "solve.invalidPreparedStationGeometry",
                "unsupported prepared route direction",
            ));
        }
        if trajectory_mode == "closed" && direction.is_none() {
            return Err(SolverApiError::new(
                "solve.invalidPreparedStationGeometry",
                "closed prepared route requires direction",
            ));
        }
        let start_finish_xy_m =
            optional_start_finish(value, "start_finish_xy_m").map_err(|message| {
                SolverApiError::new("solve.invalidPreparedStationGeometry", message)
            })?;
        let finish_line_xy_m =
            optional_start_finish(value, "finish_line_xy_m").map_err(|message| {
                SolverApiError::new("solve.invalidPreparedStationGeometry", message)
            })?;
        if trajectory_mode == "open" && (start_finish_xy_m.is_none() || finish_line_xy_m.is_none())
        {
            return Err(SolverApiError::new(
                "solve.invalidPreparedStationGeometry",
                "open prepared route requires start and finish lines",
            ));
        }
        Ok(Self {
            track_id: required_string(value, "track_id")?,
            units,
            trajectory_mode,
            direction,
            start_finish_xy_m,
            finish_line_xy_m,
        })
    }
}

impl ToJsonValue for PreparedStationGeometryV3 {
    fn to_json_value(&self) -> JsonValue {
        JsonValue::Object(vec![
            (
                "schema_version".to_owned(),
                "prepared_station_geometry.v4".into(),
            ),
            ("requested_count_mode".to_owned(), "exact".into()),
            (
                "resolved_station_count".to_owned(),
                JsonValue::Integer(self.resolved_station_count as i64),
            ),
            (
                "bundle".to_owned(),
                JsonValue::Object(vec![
                    (
                        "schema_version".to_owned(),
                        "prepared_station_bundle.v3".into(),
                    ),
                    (
                        "source_ref".to_owned(),
                        station_source_ref_to_json(&self.source_ref),
                    ),
                    (
                        "recipe".to_owned(),
                        JsonValue::Object(vec![
                            ("schema_version".to_owned(), "station_recipe.v1".into()),
                            ("direction".to_owned(), self.direction.clone().into()),
                            (
                                "station_options_hash".to_owned(),
                                self.station_options_hash.clone().into(),
                            ),
                            (
                                "resolved_station_count".to_owned(),
                                JsonValue::Integer(self.resolved_station_count as i64),
                            ),
                            (
                                "generator_contract".to_owned(),
                                self.generator_contract.clone().into(),
                            ),
                            (
                                "generator_version".to_owned(),
                                self.generator_version.clone().into(),
                            ),
                            (
                                "validation_contract".to_owned(),
                                self.validation_contract.clone().into(),
                            ),
                            (
                                "validation_version".to_owned(),
                                self.validation_version.clone().into(),
                            ),
                        ]),
                    ),
                    (
                        "route_identity".to_owned(),
                        self.route_identity.to_json_value(),
                    ),
                    (
                        "sections_hash_algorithm".to_owned(),
                        self.sections_hash_algorithm.clone().into(),
                    ),
                    (
                        "sections_track_view_hash".to_owned(),
                        self.sections_track_view_hash.clone().into(),
                    ),
                    (
                        "sections_track_view".to_owned(),
                        self.sections_track_view.to_json_value(),
                    ),
                    (
                        "validation_summary".to_owned(),
                        JsonValue::Object(vec![
                            (
                                "schema_version".to_owned(),
                                "station_validation_summary.v1".into(),
                            ),
                            (
                                "validation_contract".to_owned(),
                                self.validation_contract.clone().into(),
                            ),
                            (
                                "validation_version".to_owned(),
                                self.validation_version.clone().into(),
                            ),
                            ("status".to_owned(), "passed".into()),
                            ("error_key".to_owned(), JsonValue::Null),
                            ("diagnostics".to_owned(), JsonValue::Object(Vec::new())),
                        ]),
                    ),
                    (
                        "bundle_hash_algorithm".to_owned(),
                        self.prepared_bundle_hash_algorithm.clone().into(),
                    ),
                    (
                        "bundle_hash".to_owned(),
                        self.prepared_bundle_hash.clone().into(),
                    ),
                ]),
            ),
            ("diagnostics".to_owned(), JsonValue::Object(Vec::new())),
        ])
    }
}

fn station_source_ref_to_json(source: &StationSourceRefV1) -> JsonValue {
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

impl ToJsonValue for PreparedRouteIdentityV1 {
    fn to_json_value(&self) -> JsonValue {
        JsonValue::Object(vec![
            (
                "schema_version".to_owned(),
                "prepared_route_identity.v1".into(),
            ),
            ("track_id".to_owned(), self.track_id.clone().into()),
            ("units".to_owned(), self.units.clone().into()),
            (
                "trajectory_mode".to_owned(),
                self.trajectory_mode.clone().into(),
            ),
            (
                "direction".to_owned(),
                self.direction
                    .clone()
                    .map_or(JsonValue::Null, JsonValue::from),
            ),
            (
                "start_finish_xy_m".to_owned(),
                option_start_finish_to_json(&self.start_finish_xy_m),
            ),
            (
                "finish_line_xy_m".to_owned(),
                option_start_finish_to_json(&self.finish_line_xy_m),
            ),
        ])
    }
}

impl MintimeSolveRequestV1 {
    pub fn parse_product(
        input_json: &str,
        expected_family: VehicleDynamicsModelFamily,
    ) -> Result<Self, SolverApiError> {
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
        Self::parse(input_json, expected_family)
    }

    pub fn parse(
        input_json: &str,
        expected_family: VehicleDynamicsModelFamily,
    ) -> Result<Self, SolverApiError> {
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
        let project_id =
            optional_string(&value, "project_id").unwrap_or_else(|| "unknown".to_owned());
        if schema_version.as_deref() == Some("rust_solver_http_request.v5") {
            ensure_request_fields(
                &value,
                &[
                    "schema_version",
                    "request_id",
                    "project_id",
                    "source_ref",
                    "station_count",
                    "solve_options",
                    "prepared_station_geometry",
                    "vehicle_dynamics_profile",
                ],
                "v5 solver request",
            )?;
            let outer_source = required_field(&value, "source_ref")?;
            ensure_request_fields(
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
            if required_string(outer_source, "schema_version")? != "station_source_ref.v1" {
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
            if required_string(outer_source, "project_id")? != project_id
                || required_string(outer_source, "project_id")? != prepared.source_ref.project_id
                || required_string(outer_source, "geometry_id")? != prepared.source_ref.geometry_id
                || required_string(outer_source, "geometry_content_hash")?
                    != prepared.source_ref.geometry_content_hash
                || required_string(outer_source, "route_id")? != prepared.source_ref.route_id
            {
                return Err(SolverApiError::new(
                    "solve.invalidRequest",
                    "request source_ref does not match prepared station bundle",
                ));
            }
        }
        let geometry_input = match prepared_station_geometry {
            Some(prepared) => MintimeGeometryInput::PreparedStationGeometry(prepared),
            None if schema_version.as_deref() != Some("rust_solver_http_request.v5") => {
                MintimeGeometryInput::LegacyRawGeometry(
                    TrackAreaContractV1::from_json(required_field(&value, "track_area")?)
                        .map_err(|message| SolverApiError::new("solve.invalidRequest", message))?,
                )
            }
            None => unreachable!("v4 prepared geometry requirement checked above"),
        };
        let vehicle_dynamics_profile = VehicleDynamicsProfileV1::from_json(required_field(
            &value,
            "vehicle_dynamics_profile",
        )?)
        .map_err(|message| SolverApiError::new("solve.invalidRequest", message))?;

        if vehicle_dynamics_profile.model_family != expected_family {
            return Err(SolverApiError::new(
                "solve.invalidRequest",
                format!(
                    "vehicle_dynamics_profile.model_family must be {}",
                    expected_family.as_str()
                ),
            ));
        }

        let station_count = required_field(&value, "station_count")?
            .as_u32()
            .ok_or_else(|| {
                SolverApiError::new("solve.invalidRequest", "station_count must be an integer")
            })? as usize;
        if let MintimeGeometryInput::PreparedStationGeometry(prepared) = &geometry_input {
            if station_count != prepared.resolved_station_count {
                return Err(SolverApiError::new(
                    "solve.invalidRequest",
                    "station_count must match prepared station geometry",
                ));
            }
        }
        let solve_options = optional_object(&value, "solve_options");
        if let MintimeGeometryInput::PreparedStationGeometry(prepared) = &geometry_input {
            let solve_direction = solve_options
                .iter()
                .find(|(key, _)| key == "direction")
                .and_then(|(_, value)| value.as_str())
                .ok_or_else(|| {
                    SolverApiError::new(
                        "solve.invalidRequest",
                        "solve_options.direction is required",
                    )
                })?;
            let solve_station_count = solve_options
                .iter()
                .find(|(key, _)| key == "station_count")
                .and_then(|(_, value)| value.as_u32())
                .map(|value| value as usize)
                .ok_or_else(|| {
                    SolverApiError::new(
                        "solve.invalidRequest",
                        "solve_options.station_count is required",
                    )
                })?;
            let station_options = solve_options
                .iter()
                .find(|(key, _)| key == "station_options")
                .map_or_else(|| JsonValue::Object(Vec::new()), |(_, value)| value.clone());
            if !matches!(station_options, JsonValue::Object(_))
                || solve_direction != prepared.direction
                || solve_station_count != station_count
                || station_options_hash_v2(&station_options) != prepared.station_options_hash
            {
                return Err(SolverApiError::new(
                    "solve.invalidRequest",
                    "solve options do not match prepared station recipe",
                ));
            }
        }
        validate_native_initialization_contract(&solve_options)?;

        Ok(Self {
            request_id: optional_string(&value, "request_id")
                .unwrap_or_else(|| "unknown".to_owned()),
            project_id,
            station_count,
            geometry_input,
            vehicle_dynamics_profile,
            solve_options,
        })
    }
}

fn validate_native_initialization_contract(
    solve_options: &[(String, JsonValue)],
) -> Result<(), SolverApiError> {
    const REMOVED_POINT_WARMUP_FIELDS: [&str; 3] = [
        "point_warmup_ax_forward_g",
        "point_warmup_ax_brake_g",
        "point_warmup_ay_g",
    ];

    let removed_field = solve_options.iter().find_map(|(key, value)| {
        if REMOVED_POINT_WARMUP_FIELDS.contains(&key.as_str())
            || ((key == "seed_source" || key == "v1_seed_source")
                && value.as_str() == Some("auto_point_warmup"))
        {
            Some(key.as_str())
        } else {
            None
        }
    });
    if let Some(field) = removed_field {
        return Err(SolverApiError::new(
            "solve.invalidRequest",
            format!(
                "unsupported legacy point warm-up option '{field}'; car and bike models require native initialization"
            ),
        ));
    }

    Ok(())
}

#[derive(Clone, Debug, PartialEq)]
pub struct MintimeProgressEvent {
    pub phase: String,
    pub iteration: Option<u32>,
    pub progress: Option<f64>,
    pub stage: Option<String>,
    pub stage_index: Option<u32>,
    pub stage_count: Option<u32>,
    pub stage_progress: Option<f64>,
    pub overall_progress: Option<f64>,
    pub preview_source: Option<String>,
    pub message: Option<String>,
    pub preview_trajectory_result: Option<TrajectoryResultSeriesV1>,
    pub best_lap_time_s: Option<f64>,
    pub model_track_area: Option<TrackAreaContractV1>,
}

pub type MintimeProgressCallback<'a> = &'a mut dyn FnMut(MintimeProgressEvent);

#[derive(Clone, Debug, PartialEq)]
pub struct MintimeSolveResult {
    pub runtime: String,
    pub status: String,
    pub lap_time_estimate_s: Option<f64>,
    pub trajectory_result: TrajectoryResultSeriesV1,
    pub trajectory_dense: Option<JsonValue>,
    pub trajectory_contract: Option<JsonValue>,
    pub model_track_area: TrackAreaContractV1,
    pub visualization: JsonValue,
    pub diagnostics: JsonValue,
    pub warnings: Vec<String>,
}

pub trait MintimeBackend {
    fn solver_id(&self) -> &'static str;

    fn solve(
        &self,
        request: MintimeSolveRequestV1,
        progress: Option<MintimeProgressCallback<'_>>,
    ) -> Result<MintimeSolveResult, SolverApiError>;
}

pub fn backend_unavailable_error(solver_id: &str) -> SolverApiError {
    SolverApiError::new(
        "solve.nativeBackendUnavailable",
        format!("{solver_id} mintime backend is not implemented in Rust yet"),
    )
}

pub fn mintime_progress_event_to_json(event: &MintimeProgressEvent) -> JsonValue {
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
            event
                .progress
                .map(JsonValue::from)
                .unwrap_or(JsonValue::Null),
        ),
        (
            "stage".to_owned(),
            event
                .stage
                .clone()
                .map(JsonValue::from)
                .unwrap_or(JsonValue::Null),
        ),
        (
            "stage_index".to_owned(),
            event
                .stage_index
                .map(|value| JsonValue::Integer(i64::from(value)))
                .unwrap_or(JsonValue::Null),
        ),
        (
            "stage_count".to_owned(),
            event
                .stage_count
                .map(|value| JsonValue::Integer(i64::from(value)))
                .unwrap_or(JsonValue::Null),
        ),
        (
            "stage_progress".to_owned(),
            event
                .stage_progress
                .map(JsonValue::from)
                .unwrap_or(JsonValue::Null),
        ),
        (
            "overall_progress".to_owned(),
            event
                .overall_progress
                .map(JsonValue::from)
                .unwrap_or(JsonValue::Null),
        ),
        (
            "preview_source".to_owned(),
            event
                .preview_source
                .clone()
                .map(JsonValue::from)
                .unwrap_or(JsonValue::Null),
        ),
        (
            "message".to_owned(),
            event
                .message
                .clone()
                .map(JsonValue::from)
                .unwrap_or(JsonValue::Null),
        ),
        (
            "best_lap_time_s".to_owned(),
            event
                .best_lap_time_s
                .map(JsonValue::from)
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

pub fn mintime_result_to_json(result: &MintimeSolveResult) -> JsonValue {
    let open = result.model_track_area.trajectory_mode == "open";
    let time_value = result
        .lap_time_estimate_s
        .map(JsonValue::from)
        .unwrap_or(JsonValue::Null);
    let trajectory_result = result.trajectory_result.to_json_value();
    let diagnostics = with_unified_trajectory_quality(
        result.diagnostics.clone(),
        result.lap_time_estimate_s,
        &trajectory_result,
        result.trajectory_dense.as_ref(),
        !open,
    );
    let visualization = result
        .trajectory_dense
        .as_ref()
        .and_then(|dense| solve_result_visualization_json_from_dense(dense, !open))
        .unwrap_or_else(|| result.visualization.clone());

    let mut entries = vec![
        (
            "schema_version".to_owned(),
            "rust_solver_response.v1".into(),
        ),
        ("runtime".to_owned(), result.runtime.clone().into()),
        ("status".to_owned(), result.status.clone().into()),
        ("lap_time_estimate_s".to_owned(), time_value),
        ("trajectory_result".to_owned(), trajectory_result),
        (
            "model_track_area".to_owned(),
            result.model_track_area.to_json_value(),
        ),
        ("visualization".to_owned(), visualization),
        ("diagnostics".to_owned(), diagnostics),
        (
            "warnings".to_owned(),
            JsonValue::Array(
                result
                    .warnings
                    .iter()
                    .cloned()
                    .map(JsonValue::from)
                    .collect(),
            ),
        ),
    ];
    if let Some(trajectory_dense) = &result.trajectory_dense {
        entries.push(("trajectory_dense".to_owned(), trajectory_dense.clone()));
    }
    if let Some(trajectory_contract) = &result.trajectory_contract {
        entries.push((
            "trajectory_contract".to_owned(),
            trajectory_contract.clone(),
        ));
    }
    if open {
        entries.insert(
            4,
            (
                "open_run_time_s".to_owned(),
                result
                    .lap_time_estimate_s
                    .map(JsonValue::from)
                    .unwrap_or(JsonValue::Null),
            ),
        );
    }

    JsonValue::Object(entries)
}

pub fn solve_result_visualization_json(
    series: &TrajectoryResultSeriesV1,
    closed: bool,
) -> JsonValue {
    let braking_point_indices = braking_point_indices(series, closed, 1, 0.0);
    let speed_extrema = speed_extrema(series, closed, usize::MAX);
    let display_trajectory =
        display_trajectory_json(series, closed, &braking_point_indices, &speed_extrema);

    JsonValue::Object(vec![
        (
            "schema_version".to_owned(),
            "solve_result_visualization.v1".into(),
        ),
        (
            "braking_points".to_owned(),
            JsonValue::Array(
                braking_point_indices
                    .iter()
                    .copied()
                    .map(|index| speed_mark_json(series, index, "metric-brakingPoints"))
                    .collect(),
            ),
        ),
        (
            "speed_peaks".to_owned(),
            JsonValue::Array(
                speed_extrema
                    .iter()
                    .map(|extremum| {
                        speed_extremum_mark_json(series, *extremum, "metric-speedPeaks")
                    })
                    .collect(),
            ),
        ),
        (
            "longitudinal_accel_trace".to_owned(),
            accel_trace_json(series, &series.ax_mps2),
        ),
        (
            "lateral_accel_trace".to_owned(),
            accel_trace_json(series, &series.ay_mps2),
        ),
        ("station_labels".to_owned(), station_labels_json(series, 32)),
        (
            "display_trajectory".to_owned(),
            display_trajectory.unwrap_or(JsonValue::Null),
        ),
    ])
}

fn solve_result_visualization_json_from_dense(
    dense: &JsonValue,
    closed: bool,
) -> Option<JsonValue> {
    let s_m = finite_json_number_array(dense, "s_m")?;
    let x_m = finite_json_number_array(dense, "x_m")?;
    let y_m = finite_json_number_array(dense, "y_m")?;
    let v_mps = finite_json_number_array(dense, "v_mps")?;
    let ax_mps2 = finite_json_number_array(dense, "ax_model_mps2")
        .or_else(|| finite_json_number_array(dense, "ax_mps2"))?;
    let ay_mps2 = finite_json_number_array(dense, "ay_model_mps2")
        .or_else(|| finite_json_number_array(dense, "ay_mps2"))?;
    let heading_rad = finite_json_number_array(dense, "heading_geo_rad")?;
    let kappa_1pm = finite_json_number_array(dense, "kappa_geo_1pm")?;
    let count = s_m.len();

    if count < 2
        || [
            x_m.len(),
            y_m.len(),
            v_mps.len(),
            ax_mps2.len(),
            ay_mps2.len(),
            heading_rad.len(),
            kappa_1pm.len(),
        ]
        .into_iter()
        .any(|length| length != count)
    {
        return None;
    }

    let zeros = vec![0.0; count];
    let dense_series = TrajectoryResultSeriesV1 {
        s_m,
        x_m,
        y_m,
        heading_rad,
        kappa_1pm,
        v_mps,
        ax_mps2,
        ay_mps2,
        utilization_cornering: zeros.clone(),
        utilization_longitudinal: zeros.clone(),
        utilization_combined: zeros,
        station_index: None,
    };
    let mut visualization = solve_result_visualization_json(&dense_series, closed);
    let display = json_object_field_mut(&mut visualization, "display_trajectory")?;
    let source = json_object_field_mut(display, "source")?;
    set_json_object_field(
        source,
        "geometry_source",
        JsonValue::from("trajectory_dense"),
    );

    if let Some(lean_rad) = finite_json_number_array(dense, "phi_rad") {
        if lean_rad.len() == count {
            append_display_sample_series(display, "lean_rad", &lean_rad, closed);
        }
    }

    Some(visualization)
}

fn finite_json_number_array(root: &JsonValue, key: &str) -> Option<Vec<f64>> {
    root.get(key)?
        .as_array()?
        .iter()
        .map(|value| value.as_f64().filter(|number| number.is_finite()))
        .collect()
}

fn json_object_field_mut<'a>(root: &'a mut JsonValue, key: &str) -> Option<&'a mut JsonValue> {
    let JsonValue::Object(entries) = root else {
        return None;
    };
    entries
        .iter_mut()
        .find_map(|(entry_key, value)| (entry_key == key).then_some(value))
}

fn set_json_object_field(root: &mut JsonValue, key: &str, value: JsonValue) {
    let JsonValue::Object(entries) = root else {
        return;
    };
    if let Some((_, current)) = entries.iter_mut().find(|(entry_key, _)| entry_key == key) {
        *current = value;
    } else {
        entries.push((key.to_owned(), value));
    }
}

fn append_display_sample_series(display: &mut JsonValue, key: &str, values: &[f64], closed: bool) {
    let Some(JsonValue::Array(samples)) = json_object_field_mut(display, "samples") else {
        return;
    };

    for (index, sample) in samples.iter_mut().enumerate() {
        let source_index = if closed && index == values.len() {
            0
        } else {
            index
        };
        let Some(value) = values.get(source_index).copied() else {
            continue;
        };
        set_json_object_field(sample, key, JsonValue::from(value));
    }
}

fn display_trajectory_json(
    series: &TrajectoryResultSeriesV1,
    closed: bool,
    braking_point_indices: &[usize],
    speed_extrema: &[SpeedExtremum],
) -> Option<JsonValue> {
    let count = trajectory_finite_count(series);
    if count < 2 {
        return None;
    }

    let control_indices = display_control_indices(series, count);
    let control_count = control_indices.len();
    let segment_count = if closed {
        control_count
    } else {
        control_count.saturating_sub(1)
    };
    if segment_count == 0 {
        return None;
    }

    let has_station_indices = series
        .station_index
        .as_ref()
        .is_some_and(|station_index| station_index.len() >= count);
    let samples_per_segment = if has_station_indices {
        DISPLAY_TRAJECTORY_SAMPLES_PER_STATION
    } else {
        1
    };
    let segment_sample_budget = segment_count * samples_per_segment;
    let total_length = display_total_length(series, count, closed);
    let mut samples = Vec::new();
    let mut sample_s_values = Vec::new();
    let mut source_index_to_sample_index = vec![0_usize; count];

    for control_segment in 0..segment_count {
        let segment = control_indices[control_segment];
        let next_control = if control_segment + 1 == control_count {
            0
        } else {
            control_segment + 1
        };
        let next = control_indices[next_control];
        let samples_per_segment =
            display_segment_sample_count(control_segment, segment_count, segment_sample_budget);

        for step in 0..samples_per_segment {
            let local_t = step as f64 / samples_per_segment as f64;
            sample_s_values.push(display_segment_s(
                series,
                segment,
                next,
                local_t,
                closed,
                total_length,
            ));
            samples.push(display_trajectory_sample_json(
                series,
                count,
                segment,
                next,
                local_t,
                samples.len(),
                closed,
                total_length,
            ));
        }
    }

    if !closed {
        let last_control = *control_indices.last().unwrap_or(&(count - 1));
        sample_s_values.push(display_segment_s(
            series,
            last_control,
            last_control,
            1.0,
            false,
            total_length,
        ));
        samples.push(display_trajectory_sample_json(
            series,
            count,
            last_control,
            last_control,
            1.0,
            samples.len(),
            false,
            total_length,
        ));
    } else if let Some(first) = samples.first().cloned() {
        samples.push(display_trajectory_closing_sample_json(
            first,
            samples.len(),
            total_length,
        ));
        sample_s_values.push(total_length);
    }

    populate_source_sample_index_map(series, &sample_s_values, &mut source_index_to_sample_index);
    let markers = display_trajectory_markers_json(
        series,
        braking_point_indices,
        speed_extrema,
        &source_index_to_sample_index,
        &samples,
    );

    Some(JsonValue::Object(vec![
        ("schema_version".to_owned(), "display_trajectory.v1".into()),
        ("coordinate_space".to_owned(), "track_m".into()),
        ("closed".to_owned(), JsonValue::Bool(closed)),
        (
            "source".to_owned(),
            JsonValue::Object(vec![
                ("generator".to_owned(), "rust_mintime_display_trace".into()),
                (
                    "generator_version".to_owned(),
                    DISPLAY_TRAJECTORY_GENERATOR_VERSION.into(),
                ),
                ("geometry_source".to_owned(), "trajectory_result".into()),
                (
                    "acceleration_frame".to_owned(),
                    "velocity_tangent_normal".into(),
                ),
            ]),
        ),
        ("samples".to_owned(), JsonValue::Array(samples)),
        ("markers".to_owned(), JsonValue::Array(markers)),
    ]))
}

fn display_control_indices(series: &TrajectoryResultSeriesV1, count: usize) -> Vec<usize> {
    let Some(station_index) = series.station_index.as_ref() else {
        return (0..count).collect();
    };
    if station_index.len() < count {
        return (0..count).collect();
    }

    let mut controls = Vec::new();
    let mut group_start = 0_usize;

    for index in 1..=count {
        let group_finished =
            index == count || station_index.get(index) != station_index.get(group_start);
        if group_finished {
            controls.push((group_start + index - 1) / 2);
            group_start = index;
        }
    }

    if controls.len() >= 2 && controls.len() < count {
        controls
    } else {
        (0..count).collect()
    }
}

fn populate_source_sample_index_map(
    series: &TrajectoryResultSeriesV1,
    sample_s_values: &[f64],
    source_index_to_sample_index: &mut [usize],
) {
    if sample_s_values.is_empty() {
        return;
    }

    for source_index in 0..source_index_to_sample_index.len() {
        let source_s = finite_at(&series.s_m, source_index, source_index as f64);
        let partition = sample_s_values.partition_point(|sample_s| *sample_s < source_s);
        let previous = partition.saturating_sub(1);
        let next = partition.min(sample_s_values.len() - 1);
        let previous_delta = (sample_s_values[previous] - source_s).abs();
        let next_delta = (sample_s_values[next] - source_s).abs();

        source_index_to_sample_index[source_index] = if next_delta < previous_delta {
            next
        } else {
            previous
        };
    }
}

fn display_segment_sample_count(
    segment_index: usize,
    segment_count: usize,
    segment_sample_budget: usize,
) -> usize {
    let base = segment_sample_budget / segment_count;
    let remainder = segment_sample_budget % segment_count;
    base + usize::from(segment_index < remainder)
}

fn trajectory_finite_count(series: &TrajectoryResultSeriesV1) -> usize {
    [
        series.s_m.len(),
        series.x_m.len(),
        series.y_m.len(),
        series.v_mps.len(),
        series.ax_mps2.len(),
        series.ay_mps2.len(),
    ]
    .into_iter()
    .min()
    .unwrap_or(0)
}

fn display_total_length(series: &TrajectoryResultSeriesV1, count: usize, closed: bool) -> f64 {
    if count < 2 {
        return 0.0;
    }

    let last_s = finite_at(&series.s_m, count - 1, (count - 1) as f64).max(0.0);
    if !closed {
        return last_s;
    }

    let mut deltas = Vec::new();
    for index in 1..count {
        let previous = finite_at(&series.s_m, index - 1, (index - 1) as f64);
        let current = finite_at(&series.s_m, index, index as f64);
        let delta = current - previous;
        if delta.is_finite() && delta > 1e-9 {
            deltas.push(delta);
        }
    }
    let closing_delta = if deltas.is_empty() {
        1.0
    } else {
        deltas.sort_by(f64::total_cmp);
        deltas[deltas.len() / 2]
    };

    (last_s + closing_delta).max(last_s + 1e-9)
}

fn display_segment_s(
    series: &TrajectoryResultSeriesV1,
    segment: usize,
    next: usize,
    local_t: f64,
    closed: bool,
    total_length: f64,
) -> f64 {
    let start = finite_at(&series.s_m, segment, segment as f64);
    let end = if closed && next == 0 {
        total_length
    } else {
        finite_at(&series.s_m, next, next as f64)
    };

    start + (end - start).max(1e-9) * local_t
}

fn display_trajectory_sample_json(
    series: &TrajectoryResultSeriesV1,
    count: usize,
    segment: usize,
    next: usize,
    local_t: f64,
    sample_index: usize,
    closed: bool,
    total_length: f64,
) -> JsonValue {
    let point = display_interpolate_point(series, count, segment, next, local_t, closed);
    let station = station_index(series, segment).max(0) as usize;
    let next_station = station_index(series, next).max(0) as usize;
    let mut entries = vec![
        (
            "sample_index".to_owned(),
            JsonValue::Integer(sample_index as i64),
        ),
        (
            "s_m".to_owned(),
            JsonValue::from(display_segment_s(
                series,
                segment,
                next,
                local_t,
                closed,
                total_length,
            )),
        ),
        ("x_m".to_owned(), JsonValue::from(point.0)),
        ("y_m".to_owned(), JsonValue::from(point.1)),
        (
            "station_index".to_owned(),
            JsonValue::Integer(station as i64),
        ),
        (
            "next_station_index".to_owned(),
            JsonValue::Integer(next_station as i64),
        ),
        (
            "local_t".to_owned(),
            JsonValue::from(local_t.clamp(0.0, 1.0)),
        ),
        (
            "speed_mps".to_owned(),
            JsonValue::from(
                interpolate_series_value(&series.v_mps, segment, next, local_t).max(0.0),
            ),
        ),
        (
            "ax_mps2".to_owned(),
            JsonValue::from(interpolate_series_value(
                &series.ax_mps2,
                segment,
                next,
                local_t,
            )),
        ),
        (
            "ay_mps2".to_owned(),
            JsonValue::from(interpolate_series_value(
                &series.ay_mps2,
                segment,
                next,
                local_t,
            )),
        ),
    ];

    if !series.heading_rad.is_empty() {
        entries.push((
            "heading_rad".to_owned(),
            JsonValue::from(interpolate_angle_value(
                &series.heading_rad,
                segment,
                next,
                local_t,
            )),
        ));
    }
    if !series.kappa_1pm.is_empty() {
        entries.push((
            "kappa_1pm".to_owned(),
            JsonValue::from(interpolate_series_value(
                &series.kappa_1pm,
                segment,
                next,
                local_t,
            )),
        ));
    }

    JsonValue::Object(entries)
}

fn display_trajectory_closing_sample_json(
    first: JsonValue,
    sample_index: usize,
    total_length: f64,
) -> JsonValue {
    let JsonValue::Object(entries) = first else {
        return JsonValue::Null;
    };
    JsonValue::Object(
        entries
            .into_iter()
            .map(|(key, value)| {
                if key == "sample_index" {
                    (key, JsonValue::Integer(sample_index as i64))
                } else if key == "s_m" {
                    (key, JsonValue::from(total_length))
                } else if key == "local_t" {
                    (key, JsonValue::from(1.0))
                } else {
                    (key, value)
                }
            })
            .collect(),
    )
}

fn display_trajectory_markers_json(
    series: &TrajectoryResultSeriesV1,
    braking_point_indices: &[usize],
    speed_extrema: &[SpeedExtremum],
    source_index_to_sample_index: &[usize],
    samples: &[JsonValue],
) -> Vec<JsonValue> {
    let mut markers = Vec::new();

    for index in braking_point_indices.iter().copied() {
        if let Some(marker) = display_marker_json(
            series,
            index,
            "brake_point",
            "display-brake-point",
            None,
            source_index_to_sample_index,
            samples,
        ) {
            markers.push(marker);
        }
    }

    for extremum in speed_extrema.iter().copied() {
        let (kind, prefix, label_prefix) = match extremum.kind {
            SpeedExtremumKind::Minimum => ("speed_min", "display-speed-min", Some("min")),
            SpeedExtremumKind::Maximum => ("speed_max", "display-speed-max", Some("max")),
        };
        if let Some(marker) = display_marker_json(
            series,
            extremum.index,
            kind,
            prefix,
            label_prefix,
            source_index_to_sample_index,
            samples,
        ) {
            markers.push(marker);
        }
    }

    markers
}

fn display_marker_json(
    series: &TrajectoryResultSeriesV1,
    source_index: usize,
    kind: &str,
    id_prefix: &str,
    label_prefix: Option<&str>,
    source_index_to_sample_index: &[usize],
    samples: &[JsonValue],
) -> Option<JsonValue> {
    let sample_index = *source_index_to_sample_index.get(source_index)?;
    let sample = samples.get(sample_index)?;
    let speed_mps = finite_at(&series.v_mps, source_index, 0.0).max(0.0);
    let speed_label = format!("{}m/s", speed_mps.round() as i64);
    let label = match label_prefix {
        Some(prefix) => format!("{prefix} {speed_label}"),
        None => speed_label,
    };

    Some(JsonValue::Object(vec![
        (
            "id".to_owned(),
            format!("{id_prefix}-{source_index}").into(),
        ),
        ("kind".to_owned(), kind.into()),
        (
            "sample_index".to_owned(),
            JsonValue::Integer(sample_index as i64),
        ),
        (
            "s_m".to_owned(),
            JsonValue::from(
                sample
                    .get("s_m")
                    .and_then(JsonValue::as_f64)
                    .unwrap_or_default(),
            ),
        ),
        (
            "x_m".to_owned(),
            JsonValue::from(
                sample
                    .get("x_m")
                    .and_then(JsonValue::as_f64)
                    .unwrap_or_default(),
            ),
        ),
        (
            "y_m".to_owned(),
            JsonValue::from(
                sample
                    .get("y_m")
                    .and_then(JsonValue::as_f64)
                    .unwrap_or_default(),
            ),
        ),
        ("speed_mps".to_owned(), JsonValue::from(speed_mps)),
        ("label".to_owned(), label.into()),
        (
            "diagnostics".to_owned(),
            JsonValue::Object(vec![(
                "source_index".to_owned(),
                JsonValue::Integer(source_index as i64),
            )]),
        ),
    ]))
}

fn display_interpolate_point(
    series: &TrajectoryResultSeriesV1,
    count: usize,
    segment: usize,
    next: usize,
    local_t: f64,
    closed: bool,
) -> (f64, f64) {
    let p1 = series_point(series, segment);
    let p2 = series_point(series, next);
    let chord = distance2(p1, p2);

    if chord > 1e-9 {
        if let (Some(start_heading), Some(end_heading)) = (
            finite_heading_at(&series.heading_rad, segment),
            finite_heading_at(&series.heading_rad, next),
        ) {
            let chord_direction = (p2.1 - p1.1).atan2(p2.0 - p1.0);
            let start_delta = angle_delta(start_heading, chord_direction).abs();
            let end_delta = angle_delta(end_heading, chord_direction).abs();

            if start_delta <= std::f64::consts::FRAC_PI_2
                && end_delta <= std::f64::consts::FRAC_PI_2
            {
                let m1 = (start_heading.cos() * chord, start_heading.sin() * chord);
                let m2 = (end_heading.cos() * chord, end_heading.sin() * chord);
                return cubic_hermite_point(p1, p2, m1, m2, local_t.clamp(0.0, 1.0));
            }
        }
    }

    let p0_index = neighbor_index(segment, -1, count, closed);
    let p3_index = neighbor_index(segment, 2, count, closed);
    let p0 = series_point(series, p0_index);
    let p3 = series_point(series, p3_index);

    centripetal_catmull_rom_point(p0, p1, p2, p3, local_t)
}

fn finite_heading_at(values: &[f64], index: usize) -> Option<f64> {
    values.get(index).copied().filter(|value| value.is_finite())
}

fn angle_delta(left: f64, right: f64) -> f64 {
    (left - right + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}

fn neighbor_index(index: usize, offset: isize, count: usize, closed: bool) -> usize {
    if closed {
        (index as isize + offset).rem_euclid(count as isize) as usize
    } else {
        (index as isize + offset).clamp(0, count.saturating_sub(1) as isize) as usize
    }
}

fn series_point(series: &TrajectoryResultSeriesV1, index: usize) -> (f64, f64) {
    (
        finite_at(&series.x_m, index, 0.0),
        finite_at(&series.y_m, index, 0.0),
    )
}

fn centripetal_catmull_rom_point(
    p0: (f64, f64),
    p1: (f64, f64),
    p2: (f64, f64),
    p3: (f64, f64),
    local_t: f64,
) -> (f64, f64) {
    let chord = distance2(p1, p2);
    if chord <= 1e-9 {
        return p1;
    }

    let mut m1 = ((p2.0 - p0.0) * 0.5, (p2.1 - p0.1) * 0.5);
    let mut m2 = ((p3.0 - p1.0) * 0.5, (p3.1 - p1.1) * 0.5);
    m1 = clamp_vector_length(m1, chord);
    m2 = clamp_vector_length(m2, chord);
    cubic_hermite_point(p1, p2, m1, m2, local_t.clamp(0.0, 1.0))
}

fn cubic_hermite_point(
    p1: (f64, f64),
    p2: (f64, f64),
    m1: (f64, f64),
    m2: (f64, f64),
    t: f64,
) -> (f64, f64) {
    let t2 = t * t;
    let t3 = t2 * t;
    let h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
    let h10 = t3 - 2.0 * t2 + t;
    let h01 = -2.0 * t3 + 3.0 * t2;
    let h11 = t3 - t2;

    (
        h00 * p1.0 + h10 * m1.0 + h01 * p2.0 + h11 * m2.0,
        h00 * p1.1 + h10 * m1.1 + h01 * p2.1 + h11 * m2.1,
    )
}

fn distance2(left: (f64, f64), right: (f64, f64)) -> f64 {
    ((right.0 - left.0).powi(2) + (right.1 - left.1).powi(2)).sqrt()
}

fn clamp_vector_length(vector: (f64, f64), max_length: f64) -> (f64, f64) {
    let length = (vector.0.powi(2) + vector.1.powi(2)).sqrt();
    if length <= max_length || length <= 1e-9 {
        vector
    } else {
        let scale = max_length / length;
        (vector.0 * scale, vector.1 * scale)
    }
}

fn interpolate_series_value(values: &[f64], segment: usize, next: usize, local_t: f64) -> f64 {
    let start = finite_at(values, segment, 0.0);
    let end = finite_at(values, next, start);
    start + (end - start) * local_t.clamp(0.0, 1.0)
}

fn interpolate_angle_value(values: &[f64], segment: usize, next: usize, local_t: f64) -> f64 {
    let start = finite_at(values, segment, 0.0);
    let end = finite_at(values, next, start);
    let delta = (end - start + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU)
        - std::f64::consts::PI;
    start + delta * local_t.clamp(0.0, 1.0)
}

fn finite_at(values: &[f64], index: usize, fallback: f64) -> f64 {
    values
        .get(index)
        .copied()
        .filter(|value| value.is_finite())
        .unwrap_or(fallback)
}

fn speed_mark_json(series: &TrajectoryResultSeriesV1, index: usize, id_prefix: &str) -> JsonValue {
    speed_mark_json_with_label(series, index, id_prefix, None)
}

fn speed_extremum_mark_json(
    series: &TrajectoryResultSeriesV1,
    extremum: SpeedExtremum,
    id_prefix: &str,
) -> JsonValue {
    speed_mark_json_with_label(
        series,
        extremum.index,
        &format!("{id_prefix}-{}", extremum.kind.label()),
        Some(extremum.kind.label()),
    )
}

fn speed_mark_json_with_label(
    series: &TrajectoryResultSeriesV1,
    index: usize,
    id_prefix: &str,
    label_prefix: Option<&str>,
) -> JsonValue {
    let speed_mps = series.v_mps.get(index).copied().unwrap_or_default();
    let speed_label = format!("{}m/s", speed_mps.max(0.0).round() as i64);
    let label = match label_prefix {
        Some(prefix) => format!("{prefix} {speed_label}"),
        None => speed_label,
    };

    JsonValue::Object(vec![
        ("id".to_owned(), format!("{id_prefix}-{index}").into()),
        (
            "station_index".to_owned(),
            JsonValue::Integer(station_index(series, index)),
        ),
        (
            "s_m".to_owned(),
            JsonValue::from(series.s_m.get(index).copied().unwrap_or_default()),
        ),
        (
            "x_m".to_owned(),
            JsonValue::from(series.x_m.get(index).copied().unwrap_or_default()),
        ),
        (
            "y_m".to_owned(),
            JsonValue::from(series.y_m.get(index).copied().unwrap_or_default()),
        ),
        ("speed_mps".to_owned(), JsonValue::from(speed_mps.max(0.0))),
        ("label".to_owned(), label.into()),
    ])
}

fn accel_trace_json(series: &TrajectoryResultSeriesV1, values: &[f64]) -> JsonValue {
    let scale = max_abs(values).unwrap_or(1.0).max(1e-9);

    JsonValue::Array(
        values
            .iter()
            .enumerate()
            .filter_map(|(index, value)| {
                if !value.is_finite() {
                    return None;
                }

                Some(JsonValue::Object(vec![
                    (
                        "station_index".to_owned(),
                        JsonValue::Integer(station_index(series, index)),
                    ),
                    (
                        "s_m".to_owned(),
                        JsonValue::from(series.s_m.get(index).copied().unwrap_or(index as f64)),
                    ),
                    (
                        "x_m".to_owned(),
                        JsonValue::from(series.x_m.get(index).copied().unwrap_or_default()),
                    ),
                    (
                        "y_m".to_owned(),
                        JsonValue::from(series.y_m.get(index).copied().unwrap_or_default()),
                    ),
                    ("value_mps2".to_owned(), JsonValue::from(*value)),
                    (
                        "normalized_value".to_owned(),
                        JsonValue::from((value / scale).clamp(-1.0, 1.0)),
                    ),
                    ("label".to_owned(), JsonValue::Null),
                ]))
            })
            .collect(),
    )
}

fn station_labels_json(series: &TrajectoryResultSeriesV1, max_labels: usize) -> JsonValue {
    if max_labels == 0 || series.s_m.is_empty() {
        return JsonValue::Array(Vec::new());
    }

    let stride = (series.s_m.len() / max_labels).max(1);
    JsonValue::Array(
        (0..series.s_m.len())
            .step_by(stride)
            .map(|index| {
                let station = station_index(series, index);
                JsonValue::Object(vec![
                    ("station_index".to_owned(), JsonValue::Integer(station)),
                    (
                        "s_m".to_owned(),
                        JsonValue::from(series.s_m.get(index).copied().unwrap_or(index as f64)),
                    ),
                    (
                        "x_m".to_owned(),
                        JsonValue::from(series.x_m.get(index).copied().unwrap_or_default()),
                    ),
                    (
                        "y_m".to_owned(),
                        JsonValue::from(series.y_m.get(index).copied().unwrap_or_default()),
                    ),
                    ("label".to_owned(), format!("S{station}").into()),
                ])
            })
            .collect(),
    )
}

pub fn speed_peak_indices(
    series: &TrajectoryResultSeriesV1,
    closed: bool,
    count: usize,
    _min_separation_m: f64,
) -> Vec<usize> {
    speed_extrema(series, closed, count)
        .into_iter()
        .map(|extremum| extremum.index)
        .collect()
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct SpeedExtremum {
    index: usize,
    kind: SpeedExtremumKind,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum SpeedExtremumKind {
    Minimum,
    Maximum,
}

impl SpeedExtremumKind {
    fn label(self) -> &'static str {
        match self {
            Self::Minimum => "min",
            Self::Maximum => "max",
        }
    }
}

fn speed_extrema(
    series: &TrajectoryResultSeriesV1,
    closed: bool,
    count: usize,
) -> Vec<SpeedExtremum> {
    let values = smoothed_speed_mps(&series.v_mps, closed);
    let len = values.len();

    if count == 0 || len < 3 {
        return Vec::new();
    }

    let indices: Box<dyn Iterator<Item = usize>> = if closed {
        Box::new(0..len)
    } else {
        Box::new(1..(len - 1))
    };
    let mut extrema = Vec::new();

    for index in indices {
        let previous_index = if index == 0 { len - 1 } else { index - 1 };
        let next_index = if index + 1 == len { 0 } else { index + 1 };
        let Some(previous) = values
            .get(previous_index)
            .copied()
            .filter(|value| value.is_finite())
        else {
            continue;
        };
        let Some(current) = values.get(index).copied().filter(|value| value.is_finite()) else {
            continue;
        };
        let Some(next) = values
            .get(next_index)
            .copied()
            .filter(|value| value.is_finite())
        else {
            continue;
        };

        if current > previous && current > next {
            extrema.push(SpeedExtremum {
                index,
                kind: SpeedExtremumKind::Maximum,
            });
        } else if current < previous && current < next {
            extrema.push(SpeedExtremum {
                index,
                kind: SpeedExtremumKind::Minimum,
            });
        }
    }

    extrema.truncate(count);
    extrema
}

fn smoothed_speed_mps(values: &[f64], closed: bool) -> Vec<f64> {
    let len = values.len();
    if len == 0 {
        return Vec::new();
    }

    let radius = speed_extrema_smooth_radius(len);
    (0..len)
        .map(|index| {
            let mut weighted_sum = 0.0;
            let mut weight_sum = 0.0;

            for offset in -(radius as isize)..=(radius as isize) {
                let Some(sample_index) = smooth_sample_index(index, offset, len, closed) else {
                    continue;
                };
                let value = values[sample_index];
                if !value.is_finite() {
                    continue;
                }

                let weight = (radius + 1).saturating_sub(offset.unsigned_abs()) as f64;
                weighted_sum += value * weight;
                weight_sum += weight;
            }

            if weight_sum > 0.0 {
                weighted_sum / weight_sum
            } else {
                values[index]
            }
        })
        .collect()
}

fn speed_extrema_smooth_radius(count: usize) -> usize {
    ((count * SPEED_EXTREMA_BASE_SMOOTH_RADIUS_SAMPLES + (SPEED_EXTREMA_BASE_SAMPLE_COUNT / 2))
        / SPEED_EXTREMA_BASE_SAMPLE_COUNT)
        .clamp(
            SPEED_EXTREMA_BASE_SMOOTH_RADIUS_SAMPLES,
            SPEED_EXTREMA_MAX_SMOOTH_RADIUS_SAMPLES,
        )
}

fn smooth_sample_index(index: usize, offset: isize, len: usize, closed: bool) -> Option<usize> {
    let candidate = index as isize + offset;

    if closed {
        Some(candidate.rem_euclid(len as isize) as usize)
    } else if candidate >= 0 && candidate < len as isize {
        Some(candidate as usize)
    } else {
        None
    }
}

pub fn braking_point_indices(
    series: &TrajectoryResultSeriesV1,
    _closed: bool,
    count: usize,
    _min_separation_m: f64,
) -> Vec<usize> {
    if count == 0 || series.ax_mps2.is_empty() {
        return Vec::new();
    }

    let Some(index) =
        finite_extreme_index(&series.ax_mps2, |candidate, current| candidate < current)
    else {
        return Vec::new();
    };

    let accel = series.ax_mps2[index];
    if accel <= -BRAKING_POINT_MIN_DECEL_MPS2 {
        vec![index]
    } else {
        Vec::new()
    }
}

fn finite_extreme_index(values: &[f64], is_better: impl Fn(f64, f64) -> bool) -> Option<usize> {
    let mut selected: Option<(usize, f64)> = None;

    for (index, value) in values.iter().copied().enumerate() {
        if !value.is_finite() {
            continue;
        }

        match selected {
            Some((_, current)) if !is_better(value, current) => {}
            _ => selected = Some((index, value)),
        }
    }

    selected.map(|(index, _)| index)
}

fn max_abs(values: &[f64]) -> Option<f64> {
    values
        .iter()
        .copied()
        .filter(|value| value.is_finite())
        .map(f64::abs)
        .reduce(f64::max)
}

fn station_index(series: &TrajectoryResultSeriesV1, index: usize) -> i64 {
    series
        .station_index
        .as_ref()
        .and_then(|values| values.get(index))
        .copied()
        .unwrap_or(index as i64)
        .max(0)
}

fn validate_columns(
    artifact_name: &str,
    expected: &[&str],
    actual: &[String],
) -> Result<(), String> {
    let actual_refs = actual.iter().map(String::as_str).collect::<Vec<_>>();

    if actual_refs == expected {
        Ok(())
    } else {
        Err(format!(
            "{artifact_name} columns mismatch: expected [{}], got [{}]",
            expected.join("; "),
            actual_refs.join("; ")
        ))
    }
}

fn ensure_prepared_fields(
    value: &JsonValue,
    allowed: &[&str],
    context: &str,
) -> Result<(), SolverApiError> {
    let JsonValue::Object(entries) = value else {
        return Err(SolverApiError::new(
            "solve.invalidPreparedStationGeometry",
            format!("{context} must be an object"),
        ));
    };
    if let Some((key, _)) = entries
        .iter()
        .find(|(key, _)| !allowed.contains(&key.as_str()))
    {
        return Err(SolverApiError::new(
            "solve.invalidPreparedStationGeometry",
            format!("{context} contains unsupported field {key}"),
        ));
    }
    Ok(())
}

fn ensure_request_fields(
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

fn is_contract_uuid(value: &str) -> bool {
    if value == "00000000-0000-0000-0000-000000000000"
        || value == "ffffffff-ffff-ffff-ffff-ffffffffffff"
    {
        return true;
    }
    let bytes = value.as_bytes();
    bytes.len() == 36
        && [8, 13, 18, 23].iter().all(|index| bytes[*index] == b'-')
        && bytes
            .iter()
            .enumerate()
            .all(|(index, byte)| [8, 13, 18, 23].contains(&index) || byte.is_ascii_hexdigit())
        && matches!(bytes[14], b'1'..=b'8')
        && matches!(bytes[19].to_ascii_lowercase(), b'8' | b'9' | b'a' | b'b')
}

fn required_field<'a>(value: &'a JsonValue, key: &str) -> Result<&'a JsonValue, SolverApiError> {
    value
        .get(key)
        .ok_or_else(|| SolverApiError::new("solve.invalidRequest", format!("missing {key}")))
}

fn required_string(value: &JsonValue, key: &str) -> Result<String, SolverApiError> {
    required_field(value, key)?
        .as_str()
        .filter(|candidate| !candidate.is_empty())
        .map(str::to_owned)
        .ok_or_else(|| {
            SolverApiError::new(
                "solve.invalidRequest",
                format!("{key} must be a non-empty string"),
            )
        })
}

fn required_usize(value: &JsonValue, key: &str) -> Result<usize, SolverApiError> {
    value
        .get(key)
        .and_then(JsonValue::as_u32)
        .map(|value| value as usize)
        .filter(|value| *value > 0)
        .ok_or_else(|| {
            SolverApiError::new(
                "solve.invalidPreparedStationGeometry",
                format!("missing positive integer field: {key}"),
            )
        })
}

fn optional_string(value: &JsonValue, key: &str) -> Option<String> {
    value
        .get(key)
        .and_then(JsonValue::as_str)
        .map(str::to_owned)
}

fn optional_object(value: &JsonValue, key: &str) -> Vec<(String, JsonValue)> {
    match value.get(key) {
        Some(JsonValue::Object(entries)) => entries.clone(),
        _ => Vec::new(),
    }
}

#[cfg(test)]
mod tests {
    use super::{
        braking_point_indices, display_control_indices, display_interpolate_point,
        mintime_progress_event_to_json, mintime_result_to_json, solve_result_visualization_json,
        solve_result_visualization_json_from_dense, speed_peak_indices,
        validate_native_initialization_contract, MintimeNlpLayout, MintimeProgressEvent,
        MintimeSolveRequestV1, MintimeSolveResult, PreparedRouteIdentityV1,
        PreparedStationGeometryV3, DISPLAY_TRAJECTORY_SAMPLES_PER_STATION,
    };
    use crate::contracts::{
        prepared_station_bundle_hash_v3, sections_track_view_hash_v2, station_options_hash_v2,
        StartFinish, StationRecipeV1, StationSourceRefV1, TrackAreaContractV1,
        TrajectoryResultSeriesV1, PREPARED_STATION_BUNDLE_HASH_V3, SECTIONS_TRACK_VIEW_HASH_V2,
    };
    use crate::json::JsonValue;
    use crate::station::{build_production_sections_track_view, FixedCenterlineStationOptions};
    use crate::station_generation::{
        STATION_GENERATOR_CONTRACT, STATION_GENERATOR_VERSION, STATION_VALIDATION_CONTRACT,
        STATION_VALIDATION_VERSION,
    };
    use crate::vehicle_dynamics::VehicleDynamicsModelFamily;
    use crate::ToJsonValue;

    fn prepared_station_test_artifact() -> PreparedStationGeometryV3 {
        let track_value = crate::json::parse_json_str(include_str!(
            "../tests/public-fixtures/compact-oval-track-area-v1.json"
        ))
        .expect("public compact oval fixture must parse");
        let track = TrackAreaContractV1::from_json(&track_value)
            .expect("public compact oval fixture must be a track-area contract");
        let sections = build_production_sections_track_view(
            &track,
            &FixedCenterlineStationOptions {
                sample_count: 24,
                dense_count: 320,
                ..FixedCenterlineStationOptions::default()
            },
        );
        let route_identity = PreparedRouteIdentityV1 {
            track_id: track.track_id.clone(),
            units: "m".to_owned(),
            trajectory_mode: "closed".to_owned(),
            direction: Some("clockwise".to_owned()),
            start_finish_xy_m: None,
            finish_line_xy_m: None,
        };
        let source_ref = StationSourceRefV1 {
            project_id: "00000000-0000-4000-8000-000000000001".to_owned(),
            geometry_id: "00000000-0000-4000-8000-000000000002".to_owned(),
            geometry_content_hash: "fnv1a_geometrytest".to_owned(),
            route_id: route_identity.track_id.clone(),
        };
        let recipe = StationRecipeV1 {
            direction: "clockwise".to_owned(),
            station_options_hash: station_options_hash_v2(&JsonValue::Object(Vec::new())),
            resolved_station_count: sections.station_s_m.len(),
            generator_contract: STATION_GENERATOR_CONTRACT.to_owned(),
            generator_version: STATION_GENERATOR_VERSION.to_owned(),
            validation_contract: STATION_VALIDATION_CONTRACT.to_owned(),
            validation_version: STATION_VALIDATION_VERSION.to_owned(),
        };
        let sections_hash = sections_track_view_hash_v2(&sections);
        PreparedStationGeometryV3 {
            prepared_bundle_hash: prepared_station_bundle_hash_v3(
                &source_ref,
                &recipe,
                &route_identity.units,
                &route_identity.trajectory_mode,
                route_identity.direction.as_deref(),
                route_identity.start_finish_xy_m.as_ref(),
                route_identity.finish_line_xy_m.as_ref(),
                &sections_hash,
            ),
            source_ref,
            prepared_bundle_hash_algorithm: PREPARED_STATION_BUNDLE_HASH_V3.to_owned(),
            sections_track_view_hash: sections_hash,
            sections_hash_algorithm: SECTIONS_TRACK_VIEW_HASH_V2.to_owned(),
            station_options_hash: recipe.station_options_hash,
            direction: recipe.direction,
            generator_contract: recipe.generator_contract,
            generator_version: recipe.generator_version,
            validation_contract: recipe.validation_contract,
            validation_version: recipe.validation_version,
            resolved_station_count: recipe.resolved_station_count,
            route_identity,
            sections_track_view: sections,
        }
    }

    fn prepared_car_product_request_json() -> JsonValue {
        let prepared = prepared_station_test_artifact();
        JsonValue::Object(vec![
            (
                "schema_version".to_owned(),
                "rust_solver_http_request.v5".into(),
            ),
            ("request_id".to_owned(), "request-test-v5".into()),
            (
                "project_id".to_owned(),
                prepared.source_ref.project_id.clone().into(),
            ),
            (
                "source_ref".to_owned(),
                super::station_source_ref_to_json(&prepared.source_ref),
            ),
            (
                "station_count".to_owned(),
                JsonValue::Integer(prepared.resolved_station_count as i64),
            ),
            (
                "solve_options".to_owned(),
                JsonValue::Object(vec![
                    ("direction".to_owned(), prepared.direction.clone().into()),
                    (
                        "station_count".to_owned(),
                        JsonValue::Integer(prepared.resolved_station_count as i64),
                    ),
                    ("station_options".to_owned(), JsonValue::Object(Vec::new())),
                ]),
            ),
            (
                "prepared_station_geometry".to_owned(),
                prepared.to_json_value(),
            ),
            (
                "vehicle_dynamics_profile".to_owned(),
                JsonValue::Object(vec![
                    (
                        "schema_version".to_owned(),
                        "vehicle_dynamics_profile.v1".into(),
                    ),
                    ("profile_id".to_owned(), "car_dynamics:test".into()),
                    ("model_family".to_owned(), "car_dynamics".into()),
                    ("preset_id".to_owned(), "kart_125cc".into()),
                    ("solver_id".to_owned(), "old_car_mintime".into()),
                    (
                        "parameters".to_owned(),
                        JsonValue::Object(vec![("mass_kg".to_owned(), 165.0.into())]),
                    ),
                    (
                        "native_parameters".to_owned(),
                        JsonValue::Object(Vec::new()),
                    ),
                    ("metadata".to_owned(), JsonValue::Object(Vec::new())),
                ]),
            ),
        ])
    }

    #[test]
    fn parses_common_car_mintime_request_contract() {
        let request = MintimeSolveRequestV1::parse(
            r#"{
              "schema_version": "rust_solver_http_request.v1",
              "request_id": "req-1",
              "project_id": "project-1",
              "station_count": 80,
              "track_area": {
                "schema_version": "TrackAreaContractV1",
                "track_id": "track-1",
                "units": "m",
                "left_boundary_xy_m": [[0,0], [0,10], [10,10], [10,0]],
                "right_boundary_xy_m": [[2,2], [2,8], [8,8], [8,2]],
                "trajectory_mode": "closed",
                "metadata": {}
              },
              "vehicle_dynamics_profile": {
                "schema_version": "vehicle_dynamics_profile.v1",
                "profile_id": "car_dynamics:kart_125cc",
                "model_family": "car_dynamics",
                "preset_id": "kart_125cc",
                "solver_id": "old_car_mintime",
                "parameters": {"mass_kg": 165}
              },
              "solve_options": {"objective": "mintime"}
            }"#,
            VehicleDynamicsModelFamily::CarDynamics,
        )
        .unwrap();

        assert_eq!(request.request_id, "req-1");
        assert_eq!(request.station_count, 80);
        assert_eq!(request.track_area().track_id, "track-1");
        assert_eq!(
            request.vehicle_dynamics_profile.profile_id,
            "car_dynamics:kart_125cc"
        );
    }

    #[test]
    fn product_vehicle_request_rejects_unknown_or_missing_version() {
        for request in [
            r#"{"schema_version":"rust_solver_http_request.v6"}"#,
            r#"{}"#,
        ] {
            let error = MintimeSolveRequestV1::parse_product(
                request,
                VehicleDynamicsModelFamily::CarDynamics,
            )
            .expect_err("unknown product request versions must fail closed");
            assert_eq!(error.code, "solve.invalidRequest");
        }
    }

    #[test]
    fn product_vehicle_request_rejects_solve_direction_mismatch() {
        let mut tampered = prepared_car_product_request_json();
        let JsonValue::Object(root) = &mut tampered else {
            unreachable!();
        };
        let JsonValue::Object(solve_options) = root
            .iter_mut()
            .find(|(key, _)| key == "solve_options")
            .map(|(_, value)| value)
            .expect("synthetic request must contain solve_options")
        else {
            unreachable!();
        };
        let direction = solve_options
            .iter_mut()
            .find(|(key, _)| key == "direction")
            .map(|(_, value)| value)
            .expect("synthetic request must contain direction");
        *direction = "counterclockwise".into();
        let error = MintimeSolveRequestV1::parse_product(
            &tampered.to_pretty_string(),
            VehicleDynamicsModelFamily::CarDynamics,
        )
        .expect_err("solve direction must remain bound to the prepared recipe");
        assert_eq!(error.code, "solve.invalidRequest");
        assert!(error.message.contains("prepared station recipe"));
    }

    #[test]
    fn rejects_prepared_route_identity_that_differs_from_hashed_sections() {
        let mut artifact = prepared_station_test_artifact();
        artifact.route_identity.track_id = "different-track".to_owned();

        let error = PreparedStationGeometryV3::parse(&artifact.to_json_value()).unwrap_err();

        assert_eq!(error.code, "solve.invalidPreparedStationGeometry");
        assert!(error.message.contains("different track ids"));
    }

    #[test]
    fn rejects_prepared_route_topology_that_differs_from_hashed_sections() {
        let mut artifact = prepared_station_test_artifact();
        artifact.route_identity.trajectory_mode = "open".to_owned();
        artifact.route_identity.start_finish_xy_m = Some(StartFinish {
            p1_m: [0.0, -1.0],
            p2_m: [0.0, 1.0],
        });
        artifact.route_identity.finish_line_xy_m = Some(StartFinish {
            p1_m: [20.0, -1.0],
            p2_m: [20.0, 1.0],
        });

        let error = PreparedStationGeometryV3::parse(&artifact.to_json_value()).unwrap_err();

        assert_eq!(error.code, "solve.invalidPreparedStationGeometry");
        assert!(error.message.contains("topology"));
    }

    #[test]
    fn rejects_prepared_route_direction_that_differs_from_hashed_sections() {
        let mut artifact = prepared_station_test_artifact();
        artifact
            .sections_track_view
            .metadata
            .push(("requested_direction".to_owned(), "clockwise".into()));
        artifact.sections_track_view_hash =
            sections_track_view_hash_v2(&artifact.sections_track_view);
        artifact.route_identity.direction = Some("counterclockwise".to_owned());

        let error = PreparedStationGeometryV3::parse(&artifact.to_json_value()).unwrap_err();

        assert_eq!(error.code, "solve.invalidPreparedStationGeometry");
        assert!(error.message.contains("direction"));
    }

    #[test]
    fn rejects_unknown_or_missing_prepared_topology_metadata() {
        let mut unknown_route = prepared_station_test_artifact();
        unknown_route.route_identity.trajectory_mode = "loop".to_owned();
        let error = PreparedStationGeometryV3::parse(&unknown_route.to_json_value()).unwrap_err();
        assert_eq!(error.code, "solve.invalidPreparedStationGeometry");
        assert!(error.message.contains("trajectory_mode"));

        let mut missing_sections = prepared_station_test_artifact();
        missing_sections
            .sections_track_view
            .metadata
            .retain(|(key, _)| key != "trajectory_mode");
        let error =
            PreparedStationGeometryV3::parse(&missing_sections.to_json_value()).unwrap_err();
        assert_eq!(error.code, "solve.invalidPreparedStationGeometry");
        assert!(error.message.contains("trajectory_mode"));
    }

    #[test]
    fn rejects_unknown_direction_and_incomplete_open_route() {
        let mut unknown_direction = prepared_station_test_artifact();
        unknown_direction.route_identity.direction = Some("forward".to_owned());
        let error =
            PreparedStationGeometryV3::parse(&unknown_direction.to_json_value()).unwrap_err();
        assert_eq!(error.code, "solve.invalidPreparedStationGeometry");
        assert!(error.message.contains("direction"));

        let mut incomplete_open = prepared_station_test_artifact();
        incomplete_open.route_identity.trajectory_mode = "open".to_owned();
        incomplete_open
            .sections_track_view
            .metadata
            .retain(|(key, _)| key != "requested_direction");
        if let Some((_, topology)) = incomplete_open
            .sections_track_view
            .metadata
            .iter_mut()
            .find(|(key, _)| key == "trajectory_mode")
        {
            *topology = "open".into();
        }
        let error = PreparedStationGeometryV3::parse(&incomplete_open.to_json_value()).unwrap_err();
        assert_eq!(error.code, "solve.invalidPreparedStationGeometry");
        assert!(error.message.contains("start and finish"));
    }

    #[test]
    fn rejects_prepared_sections_mutation_with_stale_hash() {
        let mut artifact = prepared_station_test_artifact();
        artifact.sections_track_view.left_boundary_xy_m[1][0] += 100.0;

        let error = PreparedStationGeometryV3::parse(&artifact.to_json_value()).unwrap_err();

        assert_eq!(error.code, "solve.invalidPreparedStationGeometry");
        assert!(error.message.contains("sections hash"));
    }

    #[test]
    fn rejects_unsupported_prepared_generator_contract_and_version() {
        for (contract, version) in [
            ("station_generation_contract.v5", STATION_GENERATOR_VERSION),
            (STATION_GENERATOR_CONTRACT, "0.6.5"),
        ] {
            let mut artifact = prepared_station_test_artifact();
            artifact.generator_contract = contract.to_owned();
            artifact.generator_version = version.to_owned();

            let error = PreparedStationGeometryV3::parse(&artifact.to_json_value()).unwrap_err();

            assert_eq!(error.code, "solve.invalidPreparedStationGeometry");
        }
    }

    #[test]
    fn rejects_removed_point_warmup_contract_for_all_vehicle_models() {
        for solve_options in [
            vec![(
                "seed_source".to_owned(),
                JsonValue::String("auto_point_warmup".to_owned()),
            )],
            vec![(
                "v1_seed_source".to_owned(),
                JsonValue::String("auto_point_warmup".to_owned()),
            )],
            vec![(
                "point_warmup_ax_forward_g".to_owned(),
                JsonValue::Number(1.0),
            )],
        ] {
            let error = validate_native_initialization_contract(&solve_options).unwrap_err();
            assert_eq!(error.code, "solve.invalidRequest");
            assert!(error.message.contains("native initialization"));
        }
    }

    #[test]
    fn open_mintime_result_json_exposes_open_run_time() {
        let result = MintimeSolveResult {
            runtime: "test".to_owned(),
            status: "Solve_Succeeded".to_owned(),
            lap_time_estimate_s: Some(12.5),
            trajectory_result: test_series(vec![1.0, 2.0, 3.0]),
            trajectory_dense: None,
            trajectory_contract: None,
            model_track_area: TrackAreaContractV1 {
                schema_version: "TrackAreaContractV1".to_owned(),
                track_id: "open-track".to_owned(),
                units: "m".to_owned(),
                left_boundary_xy_m: Vec::new(),
                right_boundary_xy_m: Vec::new(),
                start_finish_xy_m: None,
                finish_line_xy_m: None,
                trajectory_mode: "open".to_owned(),
                direction: None,
                metadata: Vec::new(),
                image_path: None,
                image_width_px: None,
                image_height_px: None,
                meters_per_pixel: None,
            },
            visualization: JsonValue::Null,
            diagnostics: JsonValue::Null,
            warnings: Vec::new(),
        };
        let json = mintime_result_to_json(&result);

        assert_eq!(
            json.get("open_run_time_s").and_then(JsonValue::as_f64),
            Some(12.5)
        );
        assert_eq!(
            json.get("lap_time_estimate_s").and_then(JsonValue::as_f64),
            Some(12.5)
        );
        assert!(
            json.get("diagnostics")
                .and_then(|value| value.get("unified_trajectory_quality"))
                .is_some(),
            "mintime responses should include unified quality diagnostics"
        );
    }

    #[test]
    fn mintime_progress_json_exposes_staged_progress_fields() {
        let event = MintimeProgressEvent {
            phase: "running".to_owned(),
            iteration: Some(7),
            progress: Some(0.25),
            stage: Some("full_model".to_owned()),
            stage_index: Some(1),
            stage_count: Some(2),
            stage_progress: Some(0.5),
            overall_progress: Some(0.25),
            preview_source: Some("full_model".to_owned()),
            message: None,
            preview_trajectory_result: None,
            best_lap_time_s: None,
            model_track_area: None,
        };

        let json = mintime_progress_event_to_json(&event);

        assert_eq!(
            json.get("stage").and_then(JsonValue::as_str),
            Some("full_model")
        );
        assert_eq!(json.get("stage_index").and_then(JsonValue::as_u32), Some(1));
        assert_eq!(json.get("stage_count").and_then(JsonValue::as_u32), Some(2));
        assert_eq!(
            json.get("stage_progress").and_then(JsonValue::as_f64),
            Some(0.5)
        );
        assert_eq!(
            json.get("overall_progress").and_then(JsonValue::as_f64),
            Some(0.25)
        );
        assert_eq!(
            json.get("preview_source").and_then(JsonValue::as_str),
            Some("full_model")
        );
    }

    #[test]
    fn optimizer_progress_json_does_not_claim_completion_percentage() {
        let event = MintimeProgressEvent {
            phase: "running".to_owned(),
            iteration: Some(12),
            progress: Some(0.75),
            stage: Some("full_model".to_owned()),
            stage_index: Some(1),
            stage_count: Some(1),
            stage_progress: None,
            overall_progress: None,
            preview_source: None,
            message: Some("solve.phase.running".to_owned()),
            preview_trajectory_result: None,
            best_lap_time_s: None,
            model_track_area: None,
        };

        let json = mintime_progress_event_to_json(&event);

        assert_eq!(json.get("iteration").and_then(JsonValue::as_u32), Some(12));
        assert_eq!(json.get("progress").and_then(JsonValue::as_f64), Some(0.75));
        assert_eq!(json.get("stage_progress"), Some(&JsonValue::Null));
        assert_eq!(json.get("overall_progress"), Some(&JsonValue::Null));
    }

    #[test]
    fn speed_peak_selection_uses_smoothed_speed_extrema() {
        let series = test_series(vec![10.0, 30.0, 12.0, 28.0, 11.0, 31.0, 9.0]);
        let peaks = speed_peak_indices(&series, true, 2, 25.0);

        assert_eq!(peaks, vec![3, 6]);
    }

    #[test]
    fn braking_point_selection_uses_deceleration_minima() {
        let series = test_series(vec![10.0, 30.0, 12.0, 28.0, 11.0, 31.0, 9.0]);
        let points = braking_point_indices(&series, true, 4, 25.0);

        assert_eq!(points, vec![2]);
    }

    #[test]
    fn car_mintime_layout_matches_python_optimizer_csv_contract() {
        let layout = MintimeNlpLayout::for_family(VehicleDynamicsModelFamily::CarDynamics);
        let state_columns = [
            "s_m",
            "t_s",
            "v_mps",
            "beta_rad",
            "omega_z_radps",
            "n_m",
            "xi_rad",
        ]
        .into_iter()
        .map(str::to_owned)
        .collect::<Vec<_>>();
        let control_columns = [
            "s_m",
            "t_s",
            "delta_rad",
            "f_drive_N",
            "f_brake_N",
            "gamma_y_N",
        ]
        .into_iter()
        .map(str::to_owned)
        .collect::<Vec<_>>();

        layout.validate_state_csv_columns(&state_columns).unwrap();
        layout
            .validate_control_csv_columns(&control_columns)
            .unwrap();
        assert_eq!(
            layout
                .dimensions_for_station_count(159, true)
                .decision_variable_count(),
            159 * 5 + 159 * 4
        );
    }

    #[test]
    fn bike_mintime_layout_matches_python_optimizer_csv_contract() {
        let layout = MintimeNlpLayout::for_family(VehicleDynamicsModelFamily::BikeDynamics);
        let state_columns = [
            "s_m",
            "t_s",
            "v_mps",
            "beta_rad",
            "omega_z_radps",
            "n_m",
            "xi_rad",
            "phi_rad",
            "phi_dot_radps",
        ]
        .into_iter()
        .map(str::to_owned)
        .collect::<Vec<_>>();
        let control_columns = [
            "s_m",
            "t_s",
            "delta_rad",
            "f_drive_N",
            "f_brake_N",
            "phi_dot_radps",
        ]
        .into_iter()
        .map(str::to_owned)
        .collect::<Vec<_>>();

        layout.validate_state_csv_columns(&state_columns).unwrap();
        layout
            .validate_control_csv_columns(&control_columns)
            .unwrap();
        assert_eq!(
            layout
                .dimensions_for_station_count(159, true)
                .decision_variable_count(),
            159 * 7 + 159 * 4
        );
    }

    #[test]
    fn legacy_bike_layout_keeps_python_baseline_state_vector() {
        let layout = MintimeNlpLayout::for_python_model_family("bike_single_track_lean").unwrap();
        let state_columns = [
            "s_m",
            "t_s",
            "v_mps",
            "beta_rad",
            "omega_z_radps",
            "n_m",
            "xi_rad",
            "phi_rad",
        ]
        .into_iter()
        .map(str::to_owned)
        .collect::<Vec<_>>();

        layout.validate_state_csv_columns(&state_columns).unwrap();
        assert_eq!(
            layout
                .dimensions_for_station_count(159, true)
                .decision_variable_count(),
            159 * 6 + 159 * 4
        );
    }

    #[test]
    fn visualization_contract_contains_solver_owned_overlay_marks() {
        let series = test_series(vec![10.0, 30.0, 12.0, 28.0, 11.0, 31.0, 9.0]);
        let visualization = solve_result_visualization_json(&series, true);

        assert_eq!(
            visualization
                .get("schema_version")
                .and_then(crate::json::JsonValue::as_str),
            Some("solve_result_visualization.v1")
        );
        assert_eq!(
            visualization
                .get("braking_points")
                .and_then(crate::json::JsonValue::as_array)
                .map(|values| values.len()),
            Some(1)
        );
        assert_eq!(
            visualization
                .get("speed_peaks")
                .and_then(crate::json::JsonValue::as_array)
                .map(|values| values.len()),
            Some(2)
        );
        assert_eq!(
            visualization
                .get("speed_peaks")
                .and_then(crate::json::JsonValue::as_array)
                .and_then(|values| values.first())
                .and_then(|value| value.get("label"))
                .and_then(crate::json::JsonValue::as_str),
            Some("max 28m/s")
        );
        assert_eq!(
            visualization
                .get("speed_peaks")
                .and_then(crate::json::JsonValue::as_array)
                .and_then(|values| values.get(1))
                .and_then(|value| value.get("label"))
                .and_then(crate::json::JsonValue::as_str),
            Some("min 9m/s")
        );
        assert_eq!(
            visualization
                .get("longitudinal_accel_trace")
                .and_then(crate::json::JsonValue::as_array)
                .map(|values| values.len()),
            Some(7)
        );
        let display_trajectory = visualization
            .get("display_trajectory")
            .expect("visualization should include display_trajectory");
        assert_eq!(
            display_trajectory
                .get("schema_version")
                .and_then(crate::json::JsonValue::as_str),
            Some("display_trajectory.v1")
        );
        assert!(
            display_trajectory
                .get("samples")
                .and_then(crate::json::JsonValue::as_array)
                .map(|values| values.len())
                .unwrap_or_default()
                > series.s_m.len()
        );
        let markers = display_trajectory
            .get("markers")
            .and_then(crate::json::JsonValue::as_array)
            .expect("display trajectory markers");
        let first_marker = markers.first().expect("at least one marker");
        let marker_sample_index = first_marker
            .get("sample_index")
            .and_then(crate::json::JsonValue::as_f64)
            .unwrap() as usize;
        let sample = display_trajectory
            .get("samples")
            .and_then(crate::json::JsonValue::as_array)
            .and_then(|samples| samples.get(marker_sample_index))
            .expect("marker sample");
        assert_eq!(
            first_marker
                .get("x_m")
                .and_then(crate::json::JsonValue::as_f64),
            sample.get("x_m").and_then(crate::json::JsonValue::as_f64)
        );
    }

    #[test]
    fn dense_product_trajectory_owns_display_geometry_acceleration_and_lean() {
        let numbers = |values: &[f64]| {
            JsonValue::Array(values.iter().copied().map(JsonValue::from).collect())
        };
        let dense = JsonValue::Object(vec![
            ("s_m".to_owned(), numbers(&[0.0, 1.0, 2.0])),
            ("x_m".to_owned(), numbers(&[0.0, 11.0, 20.0])),
            ("y_m".to_owned(), numbers(&[0.0, 3.0, 0.0])),
            ("v_mps".to_owned(), numbers(&[10.0, 12.0, 11.0])),
            ("ax_model_mps2".to_owned(), numbers(&[-4.0, 0.0, 3.0])),
            ("ay_model_mps2".to_owned(), numbers(&[-2.0, 5.0, 1.0])),
            ("heading_geo_rad".to_owned(), numbers(&[0.0, 0.2, 0.0])),
            ("kappa_geo_1pm".to_owned(), numbers(&[0.0, 0.1, 0.0])),
            ("phi_rad".to_owned(), numbers(&[-0.2, 0.6, 0.1])),
        ]);

        let visualization =
            solve_result_visualization_json_from_dense(&dense, true).expect("dense display");
        let display = visualization
            .get("display_trajectory")
            .expect("display trajectory");
        let samples = display
            .get("samples")
            .and_then(JsonValue::as_array)
            .expect("display samples");

        assert_eq!(samples.len(), 4);
        assert_eq!(
            samples[1].get("x_m").and_then(JsonValue::as_f64),
            Some(11.0)
        );
        assert_eq!(
            samples[1].get("ay_mps2").and_then(JsonValue::as_f64),
            Some(5.0)
        );
        assert_eq!(
            samples[1].get("lean_rad").and_then(JsonValue::as_f64),
            Some(0.6)
        );
        assert_eq!(
            samples[3].get("lean_rad").and_then(JsonValue::as_f64),
            Some(-0.2)
        );
        assert_eq!(
            display
                .get("source")
                .and_then(|source| source.get("geometry_source"))
                .and_then(JsonValue::as_str),
            Some("trajectory_dense")
        );
        assert_eq!(
            display
                .get("source")
                .and_then(|source| source.get("acceleration_frame"))
                .and_then(JsonValue::as_str),
            Some("velocity_tangent_normal")
        );
    }

    #[test]
    fn display_trajectory_keeps_open_samples_monotonic_and_unwrapped() {
        let mut series = test_series(vec![10.0, 12.0, 14.0, 13.0]);
        series.s_m = vec![0.0, 4.0, 10.0, 20.0];
        series.x_m = vec![0.0, 4.0, 10.0, 20.0];
        series.y_m = vec![0.0; 4];
        series.ax_mps2 = vec![0.0; 4];
        series.ay_mps2 = vec![0.0; 4];
        series.heading_rad = vec![0.0; 4];
        series.kappa_1pm = vec![0.0; 4];
        series.utilization_cornering = vec![0.0; 4];
        series.utilization_longitudinal = vec![0.0; 4];
        series.utilization_combined = vec![0.0; 4];
        series.station_index = Some(vec![0, 1, 2, 3]);
        let visualization = solve_result_visualization_json(&series, false);
        let samples = visualization
            .get("display_trajectory")
            .and_then(|value| value.get("samples"))
            .and_then(crate::json::JsonValue::as_array)
            .expect("display samples");

        assert_eq!(
            samples
                .first()
                .and_then(|sample| sample.get("x_m"))
                .and_then(crate::json::JsonValue::as_f64),
            Some(0.0)
        );
        assert_eq!(
            samples
                .last()
                .and_then(|sample| sample.get("x_m"))
                .and_then(crate::json::JsonValue::as_f64),
            Some(20.0)
        );

        let mut previous_s = f64::NEG_INFINITY;
        for sample in samples {
            let s = sample
                .get("s_m")
                .and_then(crate::json::JsonValue::as_f64)
                .expect("sample s");
            assert!(s > previous_s);
            previous_s = s;
        }
    }

    #[test]
    fn display_trajectory_collapses_repeated_station_groups_to_control_knots() {
        let mut series = test_series(vec![10.0; 10]);
        series.station_index = Some(vec![0, 0, 0, 1, 1, 2, 2, 2, 2, 3]);

        assert_eq!(display_control_indices(&series, 10), vec![1, 3, 6, 9]);
    }

    #[test]
    fn display_trajectory_oversamples_dense_point_mass_series() {
        let collapsed_controls = 160;
        let count = collapsed_controls * DISPLAY_TRAJECTORY_SAMPLES_PER_STATION;
        let mut series = TrajectoryResultSeriesV1 {
            s_m: (0..count).map(|index| index as f64).collect(),
            x_m: (0..count).map(|index| index as f64 * 0.5).collect(),
            y_m: (0..count)
                .map(|index| ((index as f64) * 0.05).sin())
                .collect(),
            heading_rad: vec![0.0; count],
            kappa_1pm: vec![0.0; count],
            v_mps: vec![20.0; count],
            ax_mps2: vec![0.0; count],
            ay_mps2: vec![0.0; count],
            utilization_cornering: vec![0.0; count],
            utilization_longitudinal: vec![0.0; count],
            utilization_combined: vec![0.0; count],
            station_index: Some((0..count).map(|index| (index / 5) as i64).collect()),
        };
        series.heading_rad = (0..count)
            .map(|index| {
                let next = (index + 1).min(count - 1);
                (series.y_m[next] - series.y_m[index]).atan2(series.x_m[next] - series.x_m[index])
            })
            .collect();

        let visualization = solve_result_visualization_json(&series, true);
        let samples = visualization
            .get("display_trajectory")
            .and_then(|value| value.get("samples"))
            .and_then(crate::json::JsonValue::as_array)
            .expect("display samples");

        assert!(
            samples.len() > collapsed_controls,
            "dense point-mass display should oversample collapsed station controls, got {} samples for {} controls",
            samples.len(),
            collapsed_controls
        );
        let expected_display_samples =
            collapsed_controls * DISPLAY_TRAJECTORY_SAMPLES_PER_STATION + 1;
        assert!(
            samples.len() == expected_display_samples,
            "display trajectory should be station-count relative, got {} samples for {} controls",
            samples.len(),
            collapsed_controls
        );
        assert!(
            samples.iter().any(|sample| {
                sample
                    .get("local_t")
                    .and_then(crate::json::JsonValue::as_f64)
                    .is_some_and(|value| value > 0.0 && value < 1.0)
            }),
            "display trajectory must include interior samples so spline interpolation is visible"
        );
    }

    #[test]
    fn display_trajectory_uses_heading_tangents_when_available() {
        let mut series = test_series(vec![10.0, 10.0, 10.0, 10.0]);
        series.x_m = vec![0.0, 1.0, 2.0, 3.0];
        series.y_m = vec![0.0, 0.0, 1.0, 1.0];
        series.heading_rad = vec![0.0, 0.0, std::f64::consts::FRAC_PI_2, 0.0];

        let midpoint = display_interpolate_point(&series, 4, 1, 2, 0.5, false);

        assert!(
            (midpoint.0 - 1.5).abs() > 1e-6 || (midpoint.1 - 0.5).abs() > 1e-6,
            "heading-aware display interpolation should not collapse to linear midpoint"
        );
    }

    fn test_series(v_mps: Vec<f64>) -> TrajectoryResultSeriesV1 {
        let count = v_mps.len();

        TrajectoryResultSeriesV1 {
            s_m: (0..count).map(|index| index as f64 * 20.0).collect(),
            x_m: (0..count).map(|index| index as f64).collect(),
            y_m: vec![0.0; count],
            heading_rad: vec![0.0; count],
            kappa_1pm: vec![0.0; count],
            v_mps,
            ax_mps2: vec![0.0, 1.0, -1.0, 0.5, -0.5, 0.0, 0.0],
            ay_mps2: vec![0.0; count],
            utilization_cornering: vec![0.0; count],
            utilization_longitudinal: vec![0.0; count],
            utilization_combined: vec![0.0; count],
            station_index: Some((0..count).map(|index| index as i64).collect()),
        }
    }
}
