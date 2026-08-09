use raceline_optimizer::contracts::{
    station_geometry_content_hash_v2, station_options_hash_v2, AccelerationEnvelopeV1,
    PointMassProfileV1, StationSourceRefV1, TrackAreaContractV1,
};
use raceline_optimizer::json::{parse_json_str, JsonValue};
use raceline_optimizer::solver_api::{
    solve_bike_mintime_json, solve_car_mintime_json, solve_point_mass_json, SolverApiError,
};
use raceline_optimizer::station::FixedCenterlineStationOptions;
use raceline_optimizer::station_generation::{
    generate_station_geometry, station_generation_response_json, StationCountMode,
    StationGenerationRequestV1,
};
use raceline_optimizer::vehicle_dynamics::{VehicleDynamicsModelFamily, VehicleDynamicsProfileV1};
use std::ffi::OsString;
use std::fs;
use std::path::{Path, PathBuf};

const CLI_SCHEMA_VERSION: &str = "raceline_optimizer_vehicle.v1";
const CLI_PROJECT_ID: &str = "7c1eaeb2-42bb-4d2c-a3c2-aabbccddeeff";
const DEFAULT_STATION_COUNT: usize = 160;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ModelKind {
    PointMass,
    Car,
    Bike,
}

impl ModelKind {
    fn parse(value: &str) -> Result<Self, CliError> {
        match value {
            "point_mass" => Ok(Self::PointMass),
            "car" => Ok(Self::Car),
            "bike" => Ok(Self::Bike),
            _ => Err(CliError::input(format!(
                "vehicle.model must be point_mass, car, or bike; got {value:?}"
            ))),
        }
    }
}

#[derive(Debug)]
pub struct CliError {
    kind: CliErrorKind,
    message: String,
    solver: Option<SolverApiError>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CliErrorKind {
    Usage,
    Input,
    Solver,
    Io,
}

impl CliError {
    fn usage(message: impl Into<String>) -> Self {
        Self {
            kind: CliErrorKind::Usage,
            message: message.into(),
            solver: None,
        }
    }

    fn input(message: impl Into<String>) -> Self {
        Self {
            kind: CliErrorKind::Input,
            message: message.into(),
            solver: None,
        }
    }

    fn io(message: impl Into<String>) -> Self {
        Self {
            kind: CliErrorKind::Io,
            message: message.into(),
            solver: None,
        }
    }

    fn solver(error: SolverApiError) -> Self {
        Self {
            kind: CliErrorKind::Solver,
            message: error.to_string(),
            solver: Some(error),
        }
    }

    #[must_use]
    pub fn exit_code(&self) -> u8 {
        match self.kind {
            CliErrorKind::Usage | CliErrorKind::Input => 2,
            CliErrorKind::Solver => 3,
            CliErrorKind::Io => 4,
        }
    }

    #[must_use]
    pub fn render(&self) -> String {
        if let Some(error) = &self.solver {
            let mut rendered = error.to_json_string();
            if error.code == "solve.nativeBackendUnavailable" {
                rendered.push_str(
                    "\nIPOPT is unavailable. Pass --ipopt-library PATH or set RLC_IPOPT_LIBRARY.",
                );
            }
            rendered
        } else {
            self.message.clone()
        }
    }
}

impl std::fmt::Display for CliError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter.write_str(&self.message)
    }
}

impl std::error::Error for CliError {}

#[derive(Debug)]
struct OptimizeArgs {
    track: PathBuf,
    vehicle: PathBuf,
    output: PathBuf,
    stations: usize,
    ipopt_library: Option<PathBuf>,
}

#[derive(Debug)]
struct VehicleInput {
    model: ModelKind,
    profile: JsonValue,
    acceleration_envelope: Option<JsonValue>,
    solve_options: JsonValue,
}

pub fn run<I>(args: I) -> Result<Option<String>, CliError>
where
    I: IntoIterator<Item = OsString>,
{
    let args = args.into_iter().collect::<Vec<_>>();
    let Some(command) = args.first().and_then(|value| value.to_str()) else {
        return Err(CliError::usage(usage()));
    };

    match command {
        "optimize" => {
            let options = parse_optimize_args(&args[1..])?;
            optimize(&options)?;
            Ok(Some(format!("wrote {}", options.output.display())))
        }
        "inspect" => {
            if args.len() != 2 {
                return Err(CliError::usage(format!(
                    "inspect expects exactly one trajectory file\n\n{}",
                    usage()
                )));
            }
            let path = PathBuf::from(&args[1]);
            Ok(Some(inspect_file(&path)?.to_pretty_string()))
        }
        "help" | "--help" | "-h" => Ok(Some(usage())),
        _ => Err(CliError::usage(format!(
            "unknown command {command:?}\n\n{}",
            usage()
        ))),
    }
}

fn usage() -> String {
    [
        "raceline-optimize — racing line optimization from track boundaries",
        "",
        "USAGE:",
        "  raceline-optimize optimize --track TRACK.json --vehicle VEHICLE.json --output RESULT.json [--stations N] [--ipopt-library PATH]",
        "  raceline-optimize inspect RESULT.json",
        "",
        "The track must use TrackAreaContractV1. The vehicle must use",
        "raceline_optimizer_vehicle.v1 with model point_mass, car, or bike.",
    ]
    .join("\n")
}

fn parse_optimize_args(args: &[OsString]) -> Result<OptimizeArgs, CliError> {
    let mut track = None;
    let mut vehicle = None;
    let mut output = None;
    let mut stations = DEFAULT_STATION_COUNT;
    let mut ipopt_library = None;
    let mut index = 0;

    while index < args.len() {
        let flag = args[index]
            .to_str()
            .ok_or_else(|| CliError::usage("option names must be valid UTF-8"))?;
        let value = args
            .get(index + 1)
            .ok_or_else(|| CliError::usage(format!("missing value for {flag}\n\n{}", usage())))?;
        match flag {
            "--track" => track = Some(PathBuf::from(value)),
            "--vehicle" => vehicle = Some(PathBuf::from(value)),
            "--output" => output = Some(PathBuf::from(value)),
            "--stations" => {
                let raw = value
                    .to_str()
                    .ok_or_else(|| CliError::usage("--stations must be valid UTF-8"))?;
                stations = raw.parse::<usize>().map_err(|_| {
                    CliError::usage(format!(
                        "--stations must be a positive integer; got {raw:?}"
                    ))
                })?;
            }
            "--ipopt-library" => ipopt_library = Some(PathBuf::from(value)),
            _ => {
                return Err(CliError::usage(format!(
                    "unknown optimize option {flag:?}\n\n{}",
                    usage()
                )))
            }
        }
        index += 2;
    }

    if stations < 20 {
        return Err(CliError::input("--stations must be at least 20"));
    }
    Ok(OptimizeArgs {
        track: track.ok_or_else(|| CliError::usage("missing required --track"))?,
        vehicle: vehicle.ok_or_else(|| CliError::usage("missing required --vehicle"))?,
        output: output.ok_or_else(|| CliError::usage("missing required --output"))?,
        stations,
        ipopt_library,
    })
}

fn optimize(options: &OptimizeArgs) -> Result<(), CliError> {
    let track_json = read_text(&options.track)?;
    let vehicle_json = read_text(&options.vehicle)?;
    let (model, request) = build_solver_request(
        &track_json,
        &vehicle_json,
        options.stations,
        options.ipopt_library.as_deref(),
    )?;
    let request_json = request.to_pretty_string();
    let response = match model {
        ModelKind::PointMass => solve_point_mass_json(&request_json),
        ModelKind::Car => solve_car_mintime_json(&request_json),
        ModelKind::Bike => solve_bike_mintime_json(&request_json),
    }
    .map_err(CliError::solver)?;

    write_result(&options.output, &response)
}

fn write_result(output: &Path, response: &str) -> Result<(), CliError> {
    if let Some(parent) = output
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
    {
        fs::create_dir_all(parent).map_err(|error| {
            CliError::io(format!(
                "failed to create output directory {}: {error}",
                parent.display()
            ))
        })?;
    }
    let mut response = response.to_owned();
    response.push('\n');
    fs::write(output, response)
        .map_err(|error| CliError::io(format!("failed to write {}: {error}", output.display())))
}

fn read_text(path: &Path) -> Result<String, CliError> {
    fs::read_to_string(path)
        .map_err(|error| CliError::io(format!("failed to read {}: {error}", path.display())))
}

fn build_solver_request(
    track_json: &str,
    vehicle_json: &str,
    station_count: usize,
    ipopt_library: Option<&Path>,
) -> Result<(ModelKind, JsonValue), CliError> {
    let track_value = parse_json_str(track_json)
        .map_err(|error| CliError::input(format!("invalid track JSON: {error}")))?;
    let track = TrackAreaContractV1::from_json(&track_value)
        .map_err(|message| CliError::input(format!("invalid track contract: {message}")))?;
    validate_track(&track)?;
    let vehicle = parse_vehicle(vehicle_json)?;

    let empty_station_options = JsonValue::Object(Vec::new());
    let geometry_content_hash = station_geometry_content_hash_v2(&track);
    let geometry_id = uuid_from_hash(&geometry_content_hash);
    let source_ref = StationSourceRefV1 {
        project_id: CLI_PROJECT_ID.to_owned(),
        geometry_id,
        geometry_content_hash,
        route_id: track.track_id.clone(),
    };
    let station_options = FixedCenterlineStationOptions {
        sample_count: station_count,
        ..FixedCenterlineStationOptions::default()
    };
    let station_request = StationGenerationRequestV1 {
        request_id: uuid_from_hash(&source_ref.geometry_content_hash),
        request_key: format!("cli:{}:{station_count}", source_ref.geometry_content_hash),
        project_id: source_ref.project_id.clone(),
        station_count,
        count_mode: StationCountMode::Exact,
        track_area: track.clone(),
        station_options,
        station_options_hash: station_options_hash_v2(&empty_station_options),
        source_ref: source_ref.clone(),
    };
    let station_result = generate_station_geometry(&station_request, None);
    let station_response = station_generation_response_json(&station_result);
    let prepared = prepared_station_geometry(&station_response)?;
    let source_json = source_ref_json(&source_ref);
    let request_id = uuid_from_hash(&format!(
        "{}{:?}",
        source_ref.geometry_content_hash, vehicle.model
    ));

    let mut solve_options = match vehicle.solve_options {
        JsonValue::Object(entries) => entries,
        _ => unreachable!("vehicle parser guarantees object solve_options"),
    };
    if let Some(path) = ipopt_library {
        upsert(
            &mut solve_options,
            "ipopt_dll_path",
            path.to_string_lossy().into_owned().into(),
        );
    }

    let mut fields = vec![
        (
            "schema_version".to_owned(),
            "rust_solver_http_request.v5".into(),
        ),
        ("request_id".to_owned(), request_id.into()),
        ("project_id".to_owned(), CLI_PROJECT_ID.into()),
        ("source_ref".to_owned(), source_json),
        (
            "station_count".to_owned(),
            JsonValue::Integer(station_result.resolved_station_count as i64),
        ),
        ("prepared_station_geometry".to_owned(), prepared),
    ];

    match vehicle.model {
        ModelKind::PointMass => {
            fields.push(("station_options".to_owned(), empty_station_options));
            fields.push(("solve_options".to_owned(), JsonValue::Object(solve_options)));
            fields.push(("point_mass_profile".to_owned(), vehicle.profile));
            fields.push((
                "acceleration_envelope".to_owned(),
                vehicle
                    .acceleration_envelope
                    .expect("point-mass vehicle requires an acceleration envelope"),
            ));
        }
        ModelKind::Car | ModelKind::Bike => {
            upsert(
                &mut solve_options,
                "direction",
                track.direction.clone().unwrap_or_default().into(),
            );
            upsert(
                &mut solve_options,
                "station_count",
                JsonValue::Integer(station_result.resolved_station_count as i64),
            );
            upsert(&mut solve_options, "station_options", empty_station_options);
            fields.push(("solve_options".to_owned(), JsonValue::Object(solve_options)));
            fields.push(("vehicle_dynamics_profile".to_owned(), vehicle.profile));
        }
    }

    Ok((vehicle.model, JsonValue::Object(fields)))
}

fn validate_track(track: &TrackAreaContractV1) -> Result<(), CliError> {
    if track.schema_version != TrackAreaContractV1::SCHEMA_VERSION {
        return Err(CliError::input(format!(
            "track.schema_version must be {}",
            TrackAreaContractV1::SCHEMA_VERSION
        )));
    }
    if track.units != "m" {
        return Err(CliError::input("track.units must be m"));
    }
    if track.left_boundary_xy_m.len() < 3 || track.right_boundary_xy_m.len() < 3 {
        return Err(CliError::input(
            "track boundaries must each contain at least three points",
        ));
    }
    match track.trajectory_mode.as_str() {
        "closed" => {
            if !matches!(
                track.direction.as_deref(),
                Some("clockwise" | "counterclockwise")
            ) {
                return Err(CliError::input(
                    "closed tracks require direction clockwise or counterclockwise",
                ));
            }
        }
        "open" => {
            if track.start_finish_xy_m.is_none() || track.finish_line_xy_m.is_none() {
                return Err(CliError::input(
                    "open tracks require start_finish_xy_m and finish_line_xy_m",
                ));
            }
        }
        _ => {
            return Err(CliError::input(
                "track.trajectory_mode must be open or closed",
            ))
        }
    }
    Ok(())
}

fn parse_vehicle(input: &str) -> Result<VehicleInput, CliError> {
    let value = parse_json_str(input)
        .map_err(|error| CliError::input(format!("invalid vehicle JSON: {error}")))?;
    ensure_fields(
        &value,
        &[
            "schema_version",
            "model",
            "profile",
            "acceleration_envelope",
            "solve_options",
        ],
        "vehicle",
    )?;
    if string_field(&value, "schema_version")? != CLI_SCHEMA_VERSION {
        return Err(CliError::input(format!(
            "vehicle.schema_version must be {CLI_SCHEMA_VERSION}"
        )));
    }
    let model = ModelKind::parse(&string_field(&value, "model")?)?;
    let profile = field(&value, "profile")?.clone();
    let solve_options = value
        .get("solve_options")
        .cloned()
        .unwrap_or_else(|| JsonValue::Object(Vec::new()));
    if !matches!(solve_options, JsonValue::Object(_)) {
        return Err(CliError::input("vehicle.solve_options must be an object"));
    }

    let acceleration_envelope = match model {
        ModelKind::PointMass => {
            let profile_contract = PointMassProfileV1::from_json(&profile)
                .map_err(|message| CliError::input(format!("invalid point profile: {message}")))?;
            if profile_contract.schema_version != PointMassProfileV1::SCHEMA_VERSION
                || profile_contract.model_kind != PointMassProfileV1::MODEL_KIND
            {
                return Err(CliError::input(
                    "point profile must use PointMassProfileV1 / point_mass_envelope",
                ));
            }
            let envelope = field(&value, "acceleration_envelope")?.clone();
            let envelope_contract =
                AccelerationEnvelopeV1::from_json(&envelope).map_err(|message| {
                    CliError::input(format!("invalid acceleration envelope: {message}"))
                })?;
            if envelope_contract.schema_version != AccelerationEnvelopeV1::SCHEMA_VERSION {
                return Err(CliError::input(
                    "acceleration envelope must use AccelerationEnvelopeV1",
                ));
            }
            Some(envelope)
        }
        ModelKind::Car | ModelKind::Bike => {
            let profile_contract =
                VehicleDynamicsProfileV1::from_json(&profile).map_err(|message| {
                    CliError::input(format!("invalid vehicle dynamics profile: {message}"))
                })?;
            let expected = match model {
                ModelKind::Car => VehicleDynamicsModelFamily::CarDynamics,
                ModelKind::Bike => VehicleDynamicsModelFamily::BikeDynamics,
                ModelKind::PointMass => unreachable!(),
            };
            if profile_contract.model_family != expected {
                return Err(CliError::input(format!(
                    "vehicle profile family must be {}",
                    expected.as_str()
                )));
            }
            None
        }
    };

    Ok(VehicleInput {
        model,
        profile,
        acceleration_envelope,
        solve_options,
    })
}

fn prepared_station_geometry(response: &JsonValue) -> Result<JsonValue, CliError> {
    Ok(JsonValue::Object(vec![
        (
            "schema_version".to_owned(),
            "prepared_station_geometry.v4".into(),
        ),
        (
            "requested_count_mode".to_owned(),
            field(response, "requested_count_mode")?.clone(),
        ),
        (
            "resolved_station_count".to_owned(),
            field(response, "resolved_station_count")?.clone(),
        ),
        (
            "complexity_report".to_owned(),
            field(response, "complexity_report")?.clone(),
        ),
        ("bundle".to_owned(), field(response, "bundle")?.clone()),
        (
            "diagnostics".to_owned(),
            field(response, "diagnostics")?.clone(),
        ),
    ]))
}

fn source_ref_json(source: &StationSourceRefV1) -> JsonValue {
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

fn uuid_from_hash(hash: &str) -> String {
    let mut bytes = hash
        .bytes()
        .filter(u8::is_ascii_hexdigit)
        .take(32)
        .collect::<Vec<_>>();
    while bytes.len() < 32 {
        bytes.push(b'0');
    }
    bytes[12] = b'4';
    bytes[16] = b'8';
    let raw = String::from_utf8(bytes).expect("hexadecimal bytes are UTF-8");
    format!(
        "{}-{}-{}-{}-{}",
        &raw[0..8],
        &raw[8..12],
        &raw[12..16],
        &raw[16..20],
        &raw[20..32]
    )
}

fn upsert(entries: &mut Vec<(String, JsonValue)>, key: &str, value: JsonValue) {
    if let Some((_, current)) = entries.iter_mut().find(|(candidate, _)| candidate == key) {
        *current = value;
    } else {
        entries.push((key.to_owned(), value));
    }
}

fn ensure_fields(value: &JsonValue, allowed: &[&str], context: &str) -> Result<(), CliError> {
    let JsonValue::Object(entries) = value else {
        return Err(CliError::input(format!("{context} must be an object")));
    };
    if let Some((key, _)) = entries
        .iter()
        .find(|(key, _)| !allowed.contains(&key.as_str()))
    {
        return Err(CliError::input(format!(
            "{context} contains unsupported field {key}"
        )));
    }
    Ok(())
}

fn field<'a>(value: &'a JsonValue, key: &str) -> Result<&'a JsonValue, CliError> {
    value
        .get(key)
        .ok_or_else(|| CliError::input(format!("missing required field {key}")))
}

fn string_field(value: &JsonValue, key: &str) -> Result<String, CliError> {
    field(value, key)?
        .as_str()
        .map(str::to_owned)
        .ok_or_else(|| CliError::input(format!("{key} must be a string")))
}

pub fn inspect_file(path: &Path) -> Result<JsonValue, CliError> {
    let input = read_text(path)?;
    inspect_json(&input)
}

pub fn inspect_json(input: &str) -> Result<JsonValue, CliError> {
    let value = parse_json_str(input)
        .map_err(|error| CliError::input(format!("invalid result JSON: {error}")))?;
    let schema = string_field(&value, "schema_version")?;
    if schema == "rust_solver_error.v1" {
        return Err(CliError::solver(SolverApiError::new(
            string_field(&value, "code")?,
            string_field(&value, "error")?,
        )));
    }
    if schema != "rust_solver_response.v1" {
        return Err(CliError::input(format!(
            "unsupported result schema {schema:?}"
        )));
    }

    let trajectory = field(&value, "trajectory_result")?;
    let columns = [
        "s_m",
        "x_m",
        "y_m",
        "heading_rad",
        "kappa_1pm",
        "v_mps",
        "ax_mps2",
        "ay_mps2",
        "utilization_cornering",
        "utilization_longitudinal",
        "utilization_combined",
    ];
    let sample_count = numeric_array(trajectory, columns[0])?.len();
    if sample_count == 0 {
        return Err(CliError::input(
            "trajectory_result must contain at least one sample",
        ));
    }
    for column in &columns[1..] {
        let values = numeric_array(trajectory, column)?;
        if values.len() != sample_count {
            return Err(CliError::input(format!(
                "trajectory column {column} has {} rows; expected {sample_count}",
                values.len()
            )));
        }
    }
    let speeds = numeric_array(trajectory, "v_mps")?;
    let utilization = numeric_array(trajectory, "utilization_combined")?;
    let speed_min = speeds.iter().copied().fold(f64::INFINITY, f64::min);
    let speed_max = speeds.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let utilization_max = utilization
        .iter()
        .copied()
        .fold(f64::NEG_INFINITY, f64::max);
    let track = field(&value, "model_track_area")?;
    let quality = value
        .get("diagnostics")
        .and_then(|diagnostics| diagnostics.get("unified_trajectory_quality"))
        .and_then(|quality| quality.get("hard_gate"));

    Ok(JsonValue::Object(vec![
        (
            "schema_version".to_owned(),
            "raceline_optimizer_inspection.v1".into(),
        ),
        ("runtime".to_owned(), field(&value, "runtime")?.clone()),
        ("status".to_owned(), field(&value, "status")?.clone()),
        ("track_id".to_owned(), field(track, "track_id")?.clone()),
        (
            "trajectory_mode".to_owned(),
            field(track, "trajectory_mode")?.clone(),
        ),
        (
            "lap_time_estimate_s".to_owned(),
            value
                .get("lap_time_estimate_s")
                .cloned()
                .unwrap_or(JsonValue::Null),
        ),
        (
            "open_run_time_s".to_owned(),
            value
                .get("open_run_time_s")
                .cloned()
                .unwrap_or(JsonValue::Null),
        ),
        (
            "objective_value".to_owned(),
            value
                .get("objective_value")
                .cloned()
                .unwrap_or(JsonValue::Null),
        ),
        (
            "sample_count".to_owned(),
            JsonValue::Integer(sample_count as i64),
        ),
        ("speed_min_mps".to_owned(), speed_min.into()),
        ("speed_max_mps".to_owned(), speed_max.into()),
        (
            "utilization_combined_max".to_owned(),
            utilization_max.into(),
        ),
        (
            "hard_gate".to_owned(),
            quality.cloned().unwrap_or(JsonValue::Null),
        ),
        (
            "warnings".to_owned(),
            value
                .get("warnings")
                .cloned()
                .unwrap_or_else(|| JsonValue::Array(Vec::new())),
        ),
    ]))
}

fn numeric_array(value: &JsonValue, key: &str) -> Result<Vec<f64>, CliError> {
    let values = field(value, key)?
        .as_array()
        .ok_or_else(|| CliError::input(format!("trajectory.{key} must be an array")))?;
    values
        .iter()
        .enumerate()
        .map(|(index, value)| {
            value
                .as_f64()
                .filter(|number| number.is_finite())
                .ok_or_else(|| {
                    CliError::input(format!("trajectory.{key}[{index}] must be a finite number"))
                })
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    const TRACK: &str = include_str!("../examples/compact-oval-track.json");

    const POINT_VEHICLE: &str = include_str!("../examples/point-mass-vehicle.json");
    const CAR_VEHICLE: &str = include_str!("../examples/kart-vehicle.json");
    const BIKE_VEHICLE: &str = include_str!("../examples/motorcycle-vehicle.json");

    #[test]
    fn prepares_valid_point_mass_product_request() {
        let (model, request) = build_solver_request(TRACK, POINT_VEHICLE, 48, None).unwrap();
        assert_eq!(model, ModelKind::PointMass);
        assert_eq!(
            request.get("schema_version").and_then(JsonValue::as_str),
            Some("rust_solver_http_request.v5")
        );
        assert_eq!(
            request.get("station_count").and_then(JsonValue::as_u32),
            Some(48)
        );
        assert!(request.get("prepared_station_geometry").is_some());
    }

    #[test]
    fn prepares_valid_car_and_bike_product_requests() {
        for (vehicle, expected) in [
            (CAR_VEHICLE, ModelKind::Car),
            (BIKE_VEHICLE, ModelKind::Bike),
        ] {
            let (model, request) =
                build_solver_request(TRACK, vehicle, 40, Some(Path::new("custom-ipopt.dll")))
                    .unwrap();
            assert_eq!(model, expected);
            let solve_options = request.get("solve_options").unwrap();
            assert_eq!(
                solve_options
                    .get("ipopt_dll_path")
                    .and_then(JsonValue::as_str),
                Some("custom-ipopt.dll")
            );
            assert_eq!(
                solve_options
                    .get("station_count")
                    .and_then(JsonValue::as_u32),
                Some(40)
            );
        }
    }

    #[test]
    fn rejects_mismatched_vehicle_family() {
        let invalid = CAR_VEHICLE.replace("car_dynamics", "bike_dynamics");
        let error = build_solver_request(TRACK, &invalid, 40, None).unwrap_err();
        assert!(error.to_string().contains("profile family"));
    }

    #[test]
    fn inspect_reports_compact_trajectory_summary() {
        let result = synthetic_result_json();
        let summary = inspect_json(&result.to_pretty_string()).unwrap();
        assert_eq!(
            summary.get("sample_count").and_then(JsonValue::as_u32),
            Some(2)
        );
        assert_eq!(
            summary.get("speed_max_mps").and_then(JsonValue::as_f64),
            Some(20.0)
        );
        assert_eq!(
            summary
                .get("utilization_combined_max")
                .and_then(JsonValue::as_f64),
            Some(0.9)
        );
    }

    #[test]
    fn inspect_rejects_mismatched_columns() {
        let mut result = synthetic_result_json();
        let JsonValue::Object(entries) = result.get_mut_for_test("trajectory_result") else {
            unreachable!()
        };
        let (_, values) = entries.iter_mut().find(|(key, _)| key == "x_m").unwrap();
        *values = JsonValue::Array(vec![0.0.into()]);
        let error = inspect_json(&result.to_pretty_string()).unwrap_err();
        assert!(error.to_string().contains("expected 2"));
    }

    #[test]
    fn write_result_accepts_basename_only_output_path() {
        let path = PathBuf::from(format!(
            ".raceline-optimizer-cli-output-test-{}.json",
            std::process::id()
        ));
        let _ = fs::remove_file(&path);
        write_result(&path, "{\"ok\":true}").unwrap();
        assert_eq!(fs::read_to_string(&path).unwrap(), "{\"ok\":true}\n");
        fs::remove_file(path).unwrap();
    }

    #[test]
    fn inspect_rejects_empty_trajectory() {
        let mut result = synthetic_result_json();
        let JsonValue::Object(entries) = result.get_mut_for_test("trajectory_result") else {
            unreachable!()
        };
        for (_, values) in entries {
            *values = JsonValue::Array(Vec::new());
        }
        let error = inspect_json(&result.to_pretty_string()).unwrap_err();
        assert!(error
            .to_string()
            .contains("must contain at least one sample"));
    }

    fn synthetic_result_json() -> JsonValue {
        let array = |left: f64, right: f64| JsonValue::Array(vec![left.into(), right.into()]);
        JsonValue::Object(vec![
            (
                "schema_version".to_owned(),
                "rust_solver_response.v1".into(),
            ),
            ("runtime".to_owned(), "test".into()),
            ("status".to_owned(), "success".into()),
            ("lap_time_estimate_s".to_owned(), 12.5.into()),
            (
                "trajectory_result".to_owned(),
                JsonValue::Object(vec![
                    ("s_m".to_owned(), array(0.0, 1.0)),
                    ("x_m".to_owned(), array(0.0, 1.0)),
                    ("y_m".to_owned(), array(0.0, 0.0)),
                    ("heading_rad".to_owned(), array(0.0, 0.0)),
                    ("kappa_1pm".to_owned(), array(0.0, 0.0)),
                    ("v_mps".to_owned(), array(10.0, 20.0)),
                    ("ax_mps2".to_owned(), array(0.0, 0.0)),
                    ("ay_mps2".to_owned(), array(0.0, 0.0)),
                    ("utilization_cornering".to_owned(), array(0.5, 0.6)),
                    ("utilization_longitudinal".to_owned(), array(0.2, 0.3)),
                    ("utilization_combined".to_owned(), array(0.7, 0.9)),
                ]),
            ),
            (
                "model_track_area".to_owned(),
                JsonValue::Object(vec![
                    ("track_id".to_owned(), "synthetic".into()),
                    ("trajectory_mode".to_owned(), "closed".into()),
                ]),
            ),
            ("diagnostics".to_owned(), JsonValue::Object(Vec::new())),
            ("warnings".to_owned(), JsonValue::Array(Vec::new())),
        ])
    }

    trait JsonValueTestMut {
        fn get_mut_for_test(&mut self, key: &str) -> &mut JsonValue;
    }

    impl JsonValueTestMut for JsonValue {
        fn get_mut_for_test(&mut self, key: &str) -> &mut JsonValue {
            let JsonValue::Object(entries) = self else {
                panic!("expected object")
            };
            entries
                .iter_mut()
                .find(|(candidate, _)| candidate == key)
                .map(|(_, value)| value)
                .expect("missing test field")
        }
    }
}
