use raceline_optimizer::contracts::{
    station_geometry_content_hash_v2, station_options_hash_v2, AccelerationEnvelopeV1,
    PointMassProfileV1, StationSourceRefV1, TrackAreaContractV1,
};
use raceline_optimizer::json::{parse_json_str, JsonValue};
use raceline_optimizer::models::stock_catalogue::{
    resolve_embedded_stock_profile, ResolvedStockProfile, StockModelId, STOCK_CATALOG_VERSION,
};
use raceline_optimizer::solver_api::{
    solve_bike_mintime_json, solve_car_mintime_json, solve_point_mass_json, SolverApiError,
};
use raceline_optimizer::station::FixedCenterlineStationOptions;
use raceline_optimizer::station_generation::{
    generate_station_geometry, station_generation_response_json, StationCountMode,
    StationGenerationRequestV1,
};
use raceline_optimizer::vehicle_dynamics::{VehicleDynamicsModelFamily, VehicleDynamicsProfileV1};
use raceline_optimizer::ToJsonValue;
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
struct PrepareArgs {
    track: PathBuf,
    output: PathBuf,
    stations: usize,
}

#[derive(Debug)]
struct SolveRequestArgs {
    model: ModelKind,
    request: PathBuf,
    output: PathBuf,
    ipopt_library: Option<PathBuf>,
}

#[derive(Debug)]
struct StockPresetRef {
    catalog_version: String,
    model_id: StockModelId,
    preset_id: String,
}

#[derive(Debug)]
struct PreparedTrackInput {
    track: TrackAreaContractV1,
    source_ref: StationSourceRefV1,
    resolved_station_count: usize,
    prepared_station_geometry: JsonValue,
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
        "prepare" => {
            let options = parse_prepare_args(&args[1..])?;
            prepare(&options)?;
            Ok(Some(format!("wrote {}", options.output.display())))
        }
        "solve" => {
            let options = parse_solve_request_args(&args[1..])?;
            solve_request(&options)?;
            Ok(Some(format!("wrote {}", options.output.display())))
        }
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
        "  raceline-optimize prepare --track TRACK.json --output PREPARED.json [--stations N]",
        "  raceline-optimize solve --model MODEL --request REQUEST.json --output RESULT.json [--ipopt-library PATH]",
        "  raceline-optimize optimize --track TRACK.json --vehicle VEHICLE.json --output RESULT.json [--stations N] [--ipopt-library PATH]",
        "  raceline-optimize inspect RESULT.json",
        "",
        "The track must use TrackAreaContractV1. The vehicle must use",
        "raceline_optimizer_vehicle.v1 with model point_mass, car, or bike.",
    ]
    .join("\n")
}

fn parse_solve_request_args(args: &[OsString]) -> Result<SolveRequestArgs, CliError> {
    let mut model = None;
    let mut request = None;
    let mut output = None;
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
            "--model" => {
                let raw = value
                    .to_str()
                    .ok_or_else(|| CliError::usage("--model must be valid UTF-8"))?;
                model = Some(ModelKind::parse(raw)?);
            }
            "--request" => request = Some(PathBuf::from(value)),
            "--output" => output = Some(PathBuf::from(value)),
            "--ipopt-library" => ipopt_library = Some(PathBuf::from(value)),
            _ => {
                return Err(CliError::usage(format!(
                    "unknown solve option {flag:?}\n\n{}",
                    usage()
                )))
            }
        }
        index += 2;
    }

    Ok(SolveRequestArgs {
        model: model.ok_or_else(|| CliError::usage("missing required --model"))?,
        request: request.ok_or_else(|| CliError::usage("missing required --request"))?,
        output: output.ok_or_else(|| CliError::usage("missing required --output"))?,
        ipopt_library,
    })
}

fn parse_prepare_args(args: &[OsString]) -> Result<PrepareArgs, CliError> {
    let mut track = None;
    let mut output = None;
    let mut stations = DEFAULT_STATION_COUNT;
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
            "--output" => output = Some(PathBuf::from(value)),
            "--stations" => stations = parse_station_count(value, flag)?,
            _ => {
                return Err(CliError::usage(format!(
                    "unknown prepare option {flag:?}\n\n{}",
                    usage()
                )))
            }
        }
        index += 2;
    }

    validate_station_count(stations)?;
    Ok(PrepareArgs {
        track: track.ok_or_else(|| CliError::usage("missing required --track"))?,
        output: output.ok_or_else(|| CliError::usage("missing required --output"))?,
        stations,
    })
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
                stations = parse_station_count(value, flag)?;
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

    validate_station_count(stations)?;
    Ok(OptimizeArgs {
        track: track.ok_or_else(|| CliError::usage("missing required --track"))?,
        vehicle: vehicle.ok_or_else(|| CliError::usage("missing required --vehicle"))?,
        output: output.ok_or_else(|| CliError::usage("missing required --output"))?,
        stations,
        ipopt_library,
    })
}

fn parse_station_count(value: &OsString, flag: &str) -> Result<usize, CliError> {
    let raw = value
        .to_str()
        .ok_or_else(|| CliError::usage(format!("{flag} must be valid UTF-8")))?;
    raw.parse::<usize>()
        .map_err(|_| CliError::usage(format!("{flag} must be a positive integer; got {raw:?}")))
}

fn validate_station_count(stations: usize) -> Result<(), CliError> {
    if stations < 20 {
        return Err(CliError::input("--stations must be at least 20"));
    }
    Ok(())
}

fn prepare(options: &PrepareArgs) -> Result<(), CliError> {
    let track_json = read_text(&options.track)?;
    let prepared = build_prepared_station_geometry(&track_json, options.stations)?;
    write_result(&options.output, &prepared.to_pretty_string())
}

fn solve_request(options: &SolveRequestArgs) -> Result<(), CliError> {
    let request_json = read_text(&options.request)?;
    let mut request = parse_json_str(&request_json)
        .map_err(|error| CliError::input(format!("invalid solver request JSON: {error}")))?;
    if let Some(path) = &options.ipopt_library {
        inject_ipopt_library(&mut request, path)?;
    }
    let request_json = request.to_pretty_string();
    let response = match options.model {
        ModelKind::PointMass => solve_point_mass_json(&request_json),
        ModelKind::Car => solve_car_mintime_json(&request_json),
        ModelKind::Bike => solve_bike_mintime_json(&request_json),
    }
    .map_err(CliError::solver)?;
    write_result(&options.output, &response)
}

fn inject_ipopt_library(request: &mut JsonValue, path: &Path) -> Result<(), CliError> {
    let JsonValue::Object(request_fields) = request else {
        return Err(CliError::input("solver request must be an object"));
    };
    let solve_options = request_fields
        .iter_mut()
        .find(|(key, _)| key == "solve_options")
        .map(|(_, value)| value)
        .ok_or_else(|| CliError::input("solver request requires solve_options"))?;
    let JsonValue::Object(solve_fields) = solve_options else {
        return Err(CliError::input("solve_options must be an object"));
    };
    upsert(
        solve_fields,
        "ipopt_dll_path",
        path.to_string_lossy().into_owned().into(),
    );
    Ok(())
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
    let vehicle = parse_vehicle(vehicle_json)?;
    let prepared_track = prepare_track_input(track_json, station_count)?;
    let track = prepared_track.track;
    let source_ref = prepared_track.source_ref;
    let resolved_station_count = prepared_track.resolved_station_count;
    let prepared = prepared_track.prepared_station_geometry;
    let empty_station_options = JsonValue::Object(Vec::new());
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
            JsonValue::Integer(resolved_station_count as i64),
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
                JsonValue::Integer(resolved_station_count as i64),
            );
            upsert(&mut solve_options, "station_options", empty_station_options);
            fields.push(("solve_options".to_owned(), JsonValue::Object(solve_options)));
            fields.push(("vehicle_dynamics_profile".to_owned(), vehicle.profile));
        }
    }

    Ok((vehicle.model, JsonValue::Object(fields)))
}

fn build_prepared_station_geometry(
    track_json: &str,
    station_count: usize,
) -> Result<JsonValue, CliError> {
    Ok(prepare_track_input(track_json, station_count)?.prepared_station_geometry)
}

fn prepare_track_input(
    track_json: &str,
    station_count: usize,
) -> Result<PreparedTrackInput, CliError> {
    let track_value = parse_json_str(track_json)
        .map_err(|error| CliError::input(format!("invalid track JSON: {error}")))?;
    let track = TrackAreaContractV1::from_json(&track_value)
        .map_err(|message| CliError::input(format!("invalid track contract: {message}")))?;
    validate_track(&track)?;

    let empty_station_options = JsonValue::Object(Vec::new());
    let geometry_content_hash = station_geometry_content_hash_v2(&track);
    let source_ref = StationSourceRefV1 {
        project_id: CLI_PROJECT_ID.to_owned(),
        geometry_id: uuid_from_hash(&geometry_content_hash),
        geometry_content_hash,
        route_id: track.track_id.clone(),
    };
    let station_request = StationGenerationRequestV1 {
        request_id: uuid_from_hash(&source_ref.geometry_content_hash),
        request_key: format!("cli:{}:{station_count}", source_ref.geometry_content_hash),
        project_id: source_ref.project_id.clone(),
        station_count,
        count_mode: StationCountMode::Exact,
        track_area: track.clone(),
        station_options: FixedCenterlineStationOptions {
            sample_count: station_count,
            ..FixedCenterlineStationOptions::default()
        },
        station_options_hash: station_options_hash_v2(&empty_station_options),
        source_ref: source_ref.clone(),
    };
    let station_result = generate_station_geometry(&station_request, None);
    let station_response = station_generation_response_json(&station_result);
    let prepared_station_geometry = prepared_station_geometry(&station_response)?;

    Ok(PreparedTrackInput {
        track,
        source_ref,
        resolved_station_count: station_result.resolved_station_count,
        prepared_station_geometry,
    })
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
            "preset_ref",
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
    let solve_options = value
        .get("solve_options")
        .cloned()
        .unwrap_or_else(|| JsonValue::Object(Vec::new()));
    if !matches!(solve_options, JsonValue::Object(_)) {
        return Err(CliError::input("vehicle.solve_options must be an object"));
    }

    let has_profile = value.get("profile").is_some();
    let has_preset_ref = value.get("preset_ref").is_some();
    if has_profile == has_preset_ref {
        return Err(CliError::input(
            "vehicle requires exactly one of profile or preset_ref",
        ));
    }
    if has_preset_ref {
        ensure_unique_fields(&value, "vehicle")?;
        return parse_stock_vehicle(&value, model, solve_options);
    }

    let profile = field(&value, "profile")?.clone();

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

fn parse_stock_vehicle(
    vehicle: &JsonValue,
    model: ModelKind,
    solve_options: JsonValue,
) -> Result<VehicleInput, CliError> {
    if vehicle.get("acceleration_envelope").is_some() {
        return Err(CliError::input(
            "vehicle.acceleration_envelope cannot accompany preset_ref",
        ));
    }
    let preset_ref = parse_stock_preset_ref(field(vehicle, "preset_ref")?)?;
    validate_stock_model_family(model, preset_ref.model_id)?;
    let mut solve_options = match solve_options {
        JsonValue::Object(entries) => entries,
        _ => unreachable!("vehicle parser guarantees object solve_options"),
    };
    ensure_unique_object_entries(&solve_options, "vehicle.solve_options")?;
    if solve_options
        .iter()
        .any(|(key, _)| matches!(key.as_str(), "width_opt" | "width_opt_m"))
    {
        return Err(CliError::input(
            "stock preset_ref does not accept width overrides in solve_options",
        ));
    }
    validate_stock_execution_policy(preset_ref.model_id, &solve_options)?;

    let resolved = resolve_embedded_stock_profile(
        &preset_ref.catalog_version,
        preset_ref.model_id.as_str(),
        &preset_ref.preset_id,
    )
    .map_err(|error| CliError::input(format!("invalid stock preset_ref: {error}")))?;

    let (profile, acceleration_envelope) = match resolved {
        ResolvedStockProfile::PointMass {
            profile,
            acceleration_envelope,
            width_opt_m,
            ..
        } => {
            upsert(&mut solve_options, "width_opt_m", width_opt_m.into());
            (
                profile.to_json_value(),
                Some(acceleration_envelope.to_json_value()),
            )
        }
        ResolvedStockProfile::VehicleDynamics { profile, .. } => (profile.to_json_value(), None),
    };

    Ok(VehicleInput {
        model,
        profile,
        acceleration_envelope,
        solve_options: JsonValue::Object(solve_options),
    })
}

fn parse_stock_preset_ref(value: &JsonValue) -> Result<StockPresetRef, CliError> {
    ensure_fields(
        value,
        &["schema_version", "catalog_version", "model_id", "preset_id"],
        "vehicle.preset_ref",
    )?;
    ensure_unique_fields(value, "vehicle.preset_ref")?;
    if string_field(value, "schema_version")? != "stock_preset_ref.v1" {
        return Err(CliError::input(
            "vehicle.preset_ref.schema_version must be stock_preset_ref.v1",
        ));
    }
    let catalog_version = string_field(value, "catalog_version")?;
    if catalog_version != STOCK_CATALOG_VERSION {
        return Err(CliError::input(format!(
            "unknown public stock catalog version: {catalog_version}"
        )));
    }
    let model_id =
        StockModelId::parse(&string_field(value, "model_id")?).map_err(CliError::input)?;
    Ok(StockPresetRef {
        catalog_version,
        model_id,
        preset_id: string_field(value, "preset_id")?,
    })
}

fn validate_stock_model_family(model: ModelKind, model_id: StockModelId) -> Result<(), CliError> {
    let matches = matches!(
        (model, model_id),
        (ModelKind::PointMass, StockModelId::PointMass)
            | (ModelKind::Car, StockModelId::CarV1)
            | (ModelKind::Bike, StockModelId::MotoV1)
    );
    if matches {
        Ok(())
    } else {
        Err(CliError::input(format!(
            "vehicle.model does not match stock model_id {}",
            model_id.as_str()
        )))
    }
}

fn validate_stock_execution_policy(
    model_id: StockModelId,
    solve_options: &[(String, JsonValue)],
) -> Result<(), CliError> {
    match model_id {
        StockModelId::PointMass => reject_policy_fields(
            solve_options,
            &[
                "car_model_version",
                "bike_model_version",
                "moto_v1_formulation_mode",
            ],
            model_id,
        ),
        StockModelId::CarV1 => {
            require_option_string(solve_options, "car_model_version", "v1", model_id)?;
            reject_policy_fields(
                solve_options,
                &["bike_model_version", "moto_v1_formulation_mode"],
                model_id,
            )
        }
        StockModelId::MotoV1 => {
            require_option_string(
                solve_options,
                "bike_model_version",
                "v1_experimental",
                model_id,
            )?;
            require_option_string(
                solve_options,
                "moto_v1_formulation_mode",
                "t1n_preproduct_v1",
                model_id,
            )?;
            reject_policy_fields(solve_options, &["car_model_version"], model_id)
        }
    }
}

fn require_option_string(
    entries: &[(String, JsonValue)],
    key: &str,
    expected: &str,
    model_id: StockModelId,
) -> Result<(), CliError> {
    let actual = entries
        .iter()
        .find(|(candidate, _)| candidate == key)
        .and_then(|(_, value)| value.as_str());
    if actual == Some(expected) {
        Ok(())
    } else {
        Err(CliError::input(format!(
            "stock preset_ref {} requires solve_options.{key}={expected}",
            model_id.as_str()
        )))
    }
}

fn reject_policy_fields(
    entries: &[(String, JsonValue)],
    forbidden: &[&str],
    model_id: StockModelId,
) -> Result<(), CliError> {
    if let Some((key, _)) = entries
        .iter()
        .find(|(key, _)| forbidden.contains(&key.as_str()))
    {
        Err(CliError::input(format!(
            "stock preset_ref {} does not accept solve_options.{key}",
            model_id.as_str()
        )))
    } else {
        Ok(())
    }
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

fn ensure_unique_fields(value: &JsonValue, context: &str) -> Result<(), CliError> {
    let JsonValue::Object(entries) = value else {
        return Err(CliError::input(format!("{context} must be an object")));
    };
    ensure_unique_object_entries(entries, context)
}

fn ensure_unique_object_entries(
    entries: &[(String, JsonValue)],
    context: &str,
) -> Result<(), CliError> {
    for (index, (key, _)) in entries.iter().enumerate() {
        if entries[..index]
            .iter()
            .any(|(candidate, _)| candidate == key)
        {
            return Err(CliError::input(format!(
                "{context} contains duplicate field {key}"
            )));
        }
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
    const CAR_VEHICLE: &str = include_str!("../examples/car-vehicle.json");
    const BIKE_VEHICLE: &str = include_str!("../examples/motorcycle-vehicle.json");
    const STOCK_POINT_VEHICLE: &str = include_str!("../examples/stock-point-mass-vehicle.json");
    const STOCK_CAR_VEHICLE: &str = include_str!("../examples/stock-kart-car-v1-vehicle.json");
    const STOCK_BIKE_VEHICLE: &str = include_str!("../examples/stock-scooter-moto-v1-vehicle.json");
    const MOTO_V1_POLICY: &str = r#"{"bike_model_version":"v1_experimental","moto_v1_formulation_mode":"t1n_preproduct_v1"}"#;

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
    fn prepares_station_geometry_without_running_a_vehicle_solve() {
        let prepared = build_prepared_station_geometry(TRACK, 48).unwrap();
        assert_eq!(
            prepared.get("schema_version").and_then(JsonValue::as_str),
            Some("prepared_station_geometry.v4")
        );
        assert_eq!(
            prepared
                .get("resolved_station_count")
                .and_then(JsonValue::as_u32),
            Some(48)
        );
        assert!(prepared
            .get("bundle")
            .and_then(|bundle| bundle.get("sections_track_view_hash"))
            .and_then(JsonValue::as_str)
            .is_some());
    }

    #[test]
    fn parses_direct_solve_request_arguments() {
        let args = [
            OsString::from("--model"),
            OsString::from("car"),
            OsString::from("--request"),
            OsString::from("request.json"),
            OsString::from("--output"),
            OsString::from("result.json"),
            OsString::from("--ipopt-library"),
            OsString::from("ipopt.dll"),
        ];
        let parsed = parse_solve_request_args(&args).unwrap();
        assert_eq!(parsed.model, ModelKind::Car);
        assert_eq!(parsed.request, PathBuf::from("request.json"));
        assert_eq!(parsed.output, PathBuf::from("result.json"));
        assert_eq!(parsed.ipopt_library, Some(PathBuf::from("ipopt.dll")));
    }

    #[test]
    fn prepare_and_solve_reject_incomplete_or_unknown_options() {
        for args in [
            vec!["--track"],
            vec!["--unknown", "x"],
            vec![
                "--track",
                "track.json",
                "--output",
                "out.json",
                "--stations",
                "19",
            ],
        ] {
            let args: Vec<OsString> = args.into_iter().map(OsString::from).collect();
            assert_eq!(parse_prepare_args(&args).unwrap_err().exit_code(), 2);
        }
        for args in [
            vec!["--request"],
            vec!["--unknown", "x"],
            vec![
                "--model",
                "unsupported",
                "--request",
                "request.json",
                "--output",
                "out.json",
            ],
        ] {
            let args: Vec<OsString> = args.into_iter().map(OsString::from).collect();
            assert_eq!(parse_solve_request_args(&args).unwrap_err().exit_code(), 2);
        }
    }

    #[test]
    fn replay_library_override_preserves_the_request_and_rejects_wrong_shapes() {
        let mut request = parse_json_str(
            r#"{"request_id":"test","solve_options":{"tol":0.001,"ipopt_dll_path":"old"}}"#,
        )
        .unwrap();
        inject_ipopt_library(&mut request, Path::new("new.dll")).unwrap();
        assert_eq!(
            request.get("request_id").and_then(JsonValue::as_str),
            Some("test")
        );
        let options = request.get("solve_options").unwrap();
        assert_eq!(options.get("tol").and_then(JsonValue::as_f64), Some(0.001));
        assert_eq!(
            options.get("ipopt_dll_path").and_then(JsonValue::as_str),
            Some("new.dll")
        );
        for raw in ["[]", "{}", r#"{"solve_options":null}"#] {
            let mut request = parse_json_str(raw).unwrap();
            assert_eq!(
                inject_ipopt_library(&mut request, Path::new("new.dll"))
                    .unwrap_err()
                    .exit_code(),
                2
            );
        }
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
    fn parses_all_public_stock_preset_refs_with_explicit_execution_policy() {
        let cases = [
            ("point_mass", "point_mass", "point_reference", "{}"),
            (
                "car",
                "car_v1",
                "kart_125cc",
                r#"{"car_model_version":"v1"}"#,
            ),
            (
                "car",
                "car_v1",
                "micro_city_oka_matiz",
                r#"{"car_model_version":"v1"}"#,
            ),
            (
                "car",
                "car_v1",
                "generic_civilian",
                r#"{"car_model_version":"v1"}"#,
            ),
            (
                "car",
                "car_v1",
                "mx5_light_sport",
                r#"{"car_model_version":"v1"}"#,
            ),
            (
                "car",
                "car_v1",
                "gt3_track_car",
                r#"{"car_model_version":"v1"}"#,
            ),
            (
                "car",
                "car_v1",
                "formula_f1_2026",
                r#"{"car_model_version":"v1"}"#,
            ),
            ("bike", "moto_v1", "moto_125_scooter", MOTO_V1_POLICY),
            ("bike", "moto_v1", "moto_300_commuter", MOTO_V1_POLICY),
            ("bike", "moto_v1", "moto_450_motard", MOTO_V1_POLICY),
            ("bike", "moto_v1", "moto_700_naked", MOTO_V1_POLICY),
            ("bike", "moto_v1", "moto_600_supersport", MOTO_V1_POLICY),
            ("bike", "moto_v1", "moto_1000_superbike", MOTO_V1_POLICY),
            ("bike", "moto_v1", "moto_gp_prototype", MOTO_V1_POLICY),
        ];
        assert_eq!(cases.len(), 14);
        for (model, model_id, preset_id, policy) in cases {
            let parsed = parse_vehicle(&stock_vehicle_json(model, model_id, preset_id, policy))
                .unwrap_or_else(|error| panic!("{model_id}/{preset_id}: {error}"));
            assert!(matches!(parsed.profile, JsonValue::Object(_)));
            assert_eq!(
                parsed.acceleration_envelope.is_some(),
                model_id == "point_mass"
            );
        }
    }

    #[test]
    fn stock_examples_parse_and_build_exact_resolved_profiles() {
        let cases = [
            (STOCK_POINT_VEHICLE, "point_mass", "point_reference"),
            (STOCK_CAR_VEHICLE, "car_v1", "kart_125cc"),
            (STOCK_BIKE_VEHICLE, "moto_v1", "moto_125_scooter"),
        ];
        for (vehicle, model_id, preset_id) in cases {
            let source = parse_json_str(vehicle).unwrap();
            assert_eq!(
                source
                    .get("preset_ref")
                    .and_then(|value| value.get("catalog_version"))
                    .and_then(JsonValue::as_str),
                Some(STOCK_CATALOG_VERSION)
            );
            parse_vehicle(vehicle).unwrap();
            let (_, request) = build_solver_request(TRACK, vehicle, 4, None).unwrap();
            let resolved =
                resolve_embedded_stock_profile(STOCK_CATALOG_VERSION, model_id, preset_id).unwrap();
            match resolved {
                ResolvedStockProfile::PointMass {
                    profile,
                    acceleration_envelope,
                    width_opt_m,
                    ..
                } => {
                    assert_eq!(
                        request.get("point_mass_profile"),
                        Some(&profile.to_json_value())
                    );
                    assert_eq!(
                        request.get("acceleration_envelope"),
                        Some(&acceleration_envelope.to_json_value())
                    );
                    assert_eq!(
                        request
                            .get("solve_options")
                            .and_then(|value| value.get("width_opt_m"))
                            .and_then(JsonValue::as_f64),
                        Some(width_opt_m)
                    );
                }
                ResolvedStockProfile::VehicleDynamics { profile, .. } => assert_eq!(
                    request.get("vehicle_dynamics_profile"),
                    Some(&profile.to_json_value())
                ),
            }
        }
    }

    #[test]
    fn legacy_full_profile_loader_preserves_fixture_values_exactly() {
        for fixture in [POINT_VEHICLE, CAR_VEHICLE, BIKE_VEHICLE] {
            let source = parse_json_str(fixture).unwrap();
            let parsed = parse_vehicle(fixture).unwrap();
            assert_eq!(parsed.profile, field(&source, "profile").unwrap().clone());
            assert_eq!(
                parsed.solve_options,
                source
                    .get("solve_options")
                    .cloned()
                    .unwrap_or_else(|| JsonValue::Object(Vec::new()))
            );
            assert_eq!(
                parsed.acceleration_envelope,
                source.get("acceleration_envelope").cloned()
            );
        }
    }

    #[test]
    fn vehicle_requires_exactly_one_profile_source() {
        let neither = format!(
            r#"{{"schema_version":"{CLI_SCHEMA_VERSION}","model":"car","solve_options":{{}}}}"#
        );
        assert_vehicle_error(&neither, "exactly one of profile or preset_ref");
        let both = stock_vehicle_json(
            "car",
            "car_v1",
            "kart_125cc",
            r#"{"car_model_version":"v1"}"#,
        )
        .replace(r#""preset_ref":"#, r#""profile":{},"preset_ref":"#);
        assert_vehicle_error(&both, "exactly one of profile or preset_ref");
    }

    #[test]
    fn stock_ref_rejects_duplicates_schema_overrides_and_conflicts() {
        let vehicle = stock_vehicle_json(
            "car",
            "car_v1",
            "kart_125cc",
            r#"{"car_model_version":"v1"}"#,
        );
        assert_vehicle_error(
            &vehicle.replace(r#""model":"car""#, r#""model":"car","model":"car""#),
            "vehicle contains duplicate field model",
        );
        assert_vehicle_error(
            &vehicle.replace(
                r#""preset_id":"kart_125cc""#,
                r#""preset_id":"kart_125cc","preset_id":"kart_125cc""#,
            ),
            "vehicle.preset_ref contains duplicate field preset_id",
        );
        assert_vehicle_error(
            &vehicle.replace(
                r#""car_model_version":"v1""#,
                r#""car_model_version":"v1","car_model_version":"v1""#,
            ),
            "vehicle.solve_options contains duplicate field car_model_version",
        );
        assert_vehicle_error(
            &vehicle.replace("stock_preset_ref.v1", "stock_preset_ref.v2"),
            "schema_version must be stock_preset_ref.v1",
        );
        assert_vehicle_error(
            &stock_vehicle_json_with_ref_extra(
                "car",
                "car_v1",
                "kart_125cc",
                r#""parameter_overrides":{},"#,
                r#"{"car_model_version":"v1"}"#,
            ),
            "unsupported field parameter_overrides",
        );
        assert_vehicle_error(
            &stock_vehicle_json(
                "car",
                "car_v1",
                "kart_125cc",
                r#"{"car_model_version":"v1","width_opt_m":1.5}"#,
            ),
            "does not accept width overrides",
        );
        assert_vehicle_error(
            &stock_vehicle_json("point_mass", "point_mass", "point_reference", "{}").replace(
                r#""solve_options":{}"#,
                r#""acceleration_envelope":{},"solve_options":{}"#,
            ),
            "cannot accompany preset_ref",
        );
    }

    #[test]
    fn stock_ref_rejects_unknown_versions_presets_models_and_v2() {
        let car = stock_vehicle_json(
            "car",
            "car_v1",
            "kart_125cc",
            r#"{"car_model_version":"v1"}"#,
        );
        assert_vehicle_error(
            &car.replace(STOCK_CATALOG_VERSION, "unknown.v1"),
            "unknown public stock catalog version",
        );
        assert_vehicle_error(
            &stock_vehicle_json("car", "car_v1", "unknown", r#"{"car_model_version":"v1"}"#),
            "unknown public stock preset",
        );
        assert_vehicle_error(
            &stock_vehicle_json(
                "car",
                "car_v2",
                "kart_125cc",
                r#"{"car_model_version":"v2"}"#,
            ),
            "unknown public stock model_id: car_v2",
        );
        assert_vehicle_error(
            &stock_vehicle_json(
                "bike",
                "moto_v2",
                "moto_125_scooter",
                r#"{"bike_model_version":"v2"}"#,
            ),
            "unknown public stock model_id: moto_v2",
        );
        assert_vehicle_error(
            &stock_vehicle_json(
                "bike",
                "car_v1",
                "kart_125cc",
                r#"{"car_model_version":"v1"}"#,
            ),
            "does not match stock model_id",
        );
    }

    #[test]
    fn stock_ref_requires_exact_v1_execution_policy() {
        assert_vehicle_error(
            &stock_vehicle_json("car", "car_v1", "kart_125cc", "{}"),
            "requires solve_options.car_model_version=v1",
        );
        assert_vehicle_error(
            &stock_vehicle_json(
                "bike",
                "moto_v1",
                "moto_125_scooter",
                r#"{"bike_model_version":"v1_experimental","moto_v1_formulation_mode":"legacy"}"#,
            ),
            "requires solve_options.moto_v1_formulation_mode=t1n_preproduct_v1",
        );
        assert_vehicle_error(
            &stock_vehicle_json(
                "point_mass",
                "point_mass",
                "point_reference",
                r#"{"car_model_version":"v1"}"#,
            ),
            "does not accept solve_options.car_model_version",
        );
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

    fn stock_vehicle_json(model: &str, model_id: &str, preset_id: &str, policy: &str) -> String {
        stock_vehicle_json_with_ref_extra(model, model_id, preset_id, "", policy)
    }

    fn stock_vehicle_json_with_ref_extra(
        model: &str,
        model_id: &str,
        preset_id: &str,
        ref_extra: &str,
        policy: &str,
    ) -> String {
        format!(
            r#"{{"schema_version":"{CLI_SCHEMA_VERSION}","model":"{model}","preset_ref":{{"schema_version":"stock_preset_ref.v1","catalog_version":"{STOCK_CATALOG_VERSION}","model_id":"{model_id}",{ref_extra}"preset_id":"{preset_id}"}},"solve_options":{policy}}}"#
        )
    }

    fn assert_vehicle_error(input: &str, expected: &str) {
        let error = parse_vehicle(input).unwrap_err();
        assert!(
            error.to_string().contains(expected),
            "expected {expected:?}, got {error}"
        );
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
