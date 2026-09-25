use crate::contracts::{sha256_hex, AccelerationEnvelopeV1, PointMassProfileV1};
use crate::json::{parse_json_str, JsonValue};
use crate::vehicle_dynamics::{VehicleDynamicsModelFamily, VehicleDynamicsProfileV1};
use crate::JsonObject;
use std::collections::BTreeSet;

pub const STOCK_CATALOG_VERSION: &str = "generalized_classes_20260925.v1";
pub const STOCK_READING_SEMANTICS: &str = "stock_snapshot.v1";
pub const STOCK_CATALOG_FILE_SHA256: &str =
    "778e131b2f646e9a589e763543edf89e051b2c4efdd07635c24793e604d8fd12";

const STOCK_CATALOG_JSON: &str = include_str!("../../catalogs/vehicle-presets.v1.json");

const POINT_PRESETS: &[&str] = &["point_reference"];
const CAR_PRESETS: &[&str] = &[
    "kart_125cc",
    "micro_city_oka_matiz",
    "generic_civilian",
    "mx5_light_sport",
    "gt3_track_car",
    "formula_f1_2026",
];
const MOTO_PRESETS: &[&str] = &[
    "moto_125_scooter",
    "moto_300_commuter",
    "moto_450_motard",
    "moto_700_naked",
    "moto_600_supersport",
    "moto_1000_superbike",
    "moto_gp_prototype",
];

const CLASS_DATA_FIELDS: &[&str] = &[
    "operator_mass_kg",
    "vehicle_length_m",
    "vehicle_mass_kg",
    "vehicle_width_m",
];
const POINT_FIELDS: &[&str] = &[
    "ax_brake_max_g",
    "ax_forward_max_g",
    "ay_left_max_g",
    "ay_right_max_g",
    "coupling_exponent",
    "v_max_mps",
    "width_opt_m",
];
const CAR_FIELDS: &[&str] = &[
    "brake_bias_front",
    "brake_grip_level",
    "brake_response_s",
    "brake_strength",
    "cg_height_m",
    "delta_max_rad",
    "dn",
    "dragcoeff",
    "drive_front_fraction",
    "drive_grip_level",
    "drive_layout",
    "eps_kappa",
    "f_brake_max_n",
    "f_drive_max_n",
    "grip_level",
    "lateral_grip_level",
    "liftcoeff_front",
    "liftcoeff_rear",
    "mass_kg",
    "max_speed_mps",
    "n_gauss",
    "penalty_delta",
    "penalty_force",
    "power_kw",
    "power_max_w",
    "roll_stiffness_distribution",
    "rolling_resistance",
    "steering_response_s",
    "throttle_response_s",
    "tire_b_front",
    "tire_b_rear",
    "tire_c_front",
    "tire_c_rear",
    "tire_e_front",
    "tire_e_rear",
    "tire_eps_front",
    "tire_eps_rear",
    "tire_fz0_n",
    "track_width_front_m",
    "track_width_rear_m",
    "w_add_spl_regr",
    "w_tr_reopt",
    "w_veh_reopt",
    "wheelbase_front",
    "wheelbase_rear",
    "width_opt_m",
    "yaw_inertia_kgm2",
];
const MOTO_FIELDS: &[&str] = &[
    "brake_grip_level",
    "brake_response_s",
    "brake_strength",
    "cg_height_m",
    "dragcoeff",
    "drive_grip_level",
    "f_brake_max_n",
    "f_drive_max_n",
    "front_brake_bias",
    "front_weight_bias",
    "lateral_grip_level",
    "lean_angle_max_rad",
    "lean_rate_max_radps",
    "lean_response_s",
    "liftcoeff_front",
    "liftcoeff_rear",
    "max_speed_mps",
    "power_kw",
    "power_max_w",
    "rider_bike_mass_kg",
    "roll_damping",
    "roll_inertia_kgm2",
    "roll_tau_max_nm",
    "rolling_resistance",
    "stability_aggressiveness",
    "steering_angle_max_rad",
    "steering_response_s",
    "throttle_response_s",
    "tire_b_front",
    "tire_b_rear",
    "tire_c_front",
    "tire_c_rear",
    "tire_e_front",
    "tire_e_rear",
    "tire_eps_front",
    "tire_eps_rear",
    "tire_fz0_n",
    "tire_model",
    "wheelbase_m",
    "width_opt_m",
    "yaw_inertia_kgm2",
];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StockModelId {
    PointMass,
    CarV1,
    MotoV1,
}

impl StockModelId {
    pub fn parse(value: &str) -> Result<Self, String> {
        match value {
            "point_mass" => Ok(Self::PointMass),
            "car_v1" => Ok(Self::CarV1),
            "moto_v1" => Ok(Self::MotoV1),
            _ => Err(format!("unknown public stock model_id: {value}")),
        }
    }

    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::PointMass => "point_mass",
            Self::CarV1 => "car_v1",
            Self::MotoV1 => "moto_v1",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum VehicleClass {
    Point,
    Car,
    Moto,
}

impl VehicleClass {
    fn parse(value: &str) -> Result<Self, String> {
        match value {
            "point" => Ok(Self::Point),
            "car" => Ok(Self::Car),
            "moto" => Ok(Self::Moto),
            _ => Err(format!("unknown vehicle_class: {value}")),
        }
    }

    const fn model_id(self) -> StockModelId {
        match self {
            Self::Point => StockModelId::PointMass,
            Self::Car => StockModelId::CarV1,
            Self::Moto => StockModelId::MotoV1,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
struct StockPreset {
    preset_id: String,
    vehicle_class: VehicleClass,
    class_data: JsonObject,
    parameters: JsonObject,
}

#[derive(Clone, Debug, PartialEq)]
pub struct PublicStockCatalogue {
    catalog_version: String,
    presets: Vec<StockPreset>,
}

impl PublicStockCatalogue {
    pub fn from_json_str(json: &str) -> Result<Self, String> {
        let value = parse_json_str(json)
            .map_err(|error| format!("invalid public stock catalogue JSON: {error}"))?;
        reject_duplicate_fields(&value, "catalog")?;
        let (catalog_version, presets) = parse_catalogue(&value)?;
        let file_sha256 = sha256_hex(json.as_bytes());
        if file_sha256 != STOCK_CATALOG_FILE_SHA256 {
            return Err(format!(
                "immutable public stock catalogue SHA-256 mismatch: expected {STOCK_CATALOG_FILE_SHA256}, got {file_sha256}"
            ));
        }
        Ok(Self {
            catalog_version,
            presets,
        })
    }

    #[must_use]
    pub fn catalog_version(&self) -> &str {
        &self.catalog_version
    }

    #[must_use]
    pub fn preset_count(&self) -> usize {
        self.presets.len()
    }

    pub fn resolve_stock_profile(
        &self,
        catalog_version: &str,
        model_id: &str,
        preset_id: &str,
    ) -> Result<ResolvedStockProfile, String> {
        if catalog_version != self.catalog_version {
            return Err(format!(
                "unknown public stock catalog version: {catalog_version}"
            ));
        }
        let model_id = StockModelId::parse(model_id)?;
        let preset = self
            .presets
            .iter()
            .find(|entry| {
                entry.preset_id == preset_id && entry.vehicle_class.model_id() == model_id
            })
            .ok_or_else(|| {
                format!(
                    "unknown public stock preset for {}: {preset_id}",
                    model_id.as_str()
                )
            })?;
        materialize_profile(model_id, preset)
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum ResolvedStockProfile {
    PointMass {
        preset_id: String,
        class_data: JsonObject,
        native_parameters: JsonObject,
        width_opt_m: f64,
        profile: PointMassProfileV1,
        acceleration_envelope: AccelerationEnvelopeV1,
    },
    VehicleDynamics {
        model_id: StockModelId,
        preset_id: String,
        class_data: JsonObject,
        profile: VehicleDynamicsProfileV1,
    },
}

impl ResolvedStockProfile {
    #[must_use]
    pub fn model_id(&self) -> StockModelId {
        match self {
            Self::PointMass { .. } => StockModelId::PointMass,
            Self::VehicleDynamics { model_id, .. } => *model_id,
        }
    }

    #[must_use]
    pub fn preset_id(&self) -> &str {
        match self {
            Self::PointMass { preset_id, .. } | Self::VehicleDynamics { preset_id, .. } => {
                preset_id
            }
        }
    }

    #[must_use]
    pub fn class_data(&self) -> &JsonObject {
        match self {
            Self::PointMass { class_data, .. } | Self::VehicleDynamics { class_data, .. } => {
                class_data
            }
        }
    }

    #[must_use]
    pub fn native_parameters(&self) -> &JsonObject {
        match self {
            Self::PointMass {
                native_parameters, ..
            } => native_parameters,
            Self::VehicleDynamics { profile, .. } => &profile.parameters,
        }
    }

    #[must_use]
    pub fn width_opt_m(&self) -> Option<f64> {
        match self {
            Self::PointMass { width_opt_m, .. } => Some(*width_opt_m),
            Self::VehicleDynamics { .. } => None,
        }
    }
}

pub fn embedded_public_stock_catalogue() -> Result<PublicStockCatalogue, String> {
    PublicStockCatalogue::from_json_str(STOCK_CATALOG_JSON)
}

pub fn resolve_embedded_stock_profile(
    catalog_version: &str,
    model_id: &str,
    preset_id: &str,
) -> Result<ResolvedStockProfile, String> {
    embedded_public_stock_catalogue()?.resolve_stock_profile(catalog_version, model_id, preset_id)
}

fn parse_catalogue(value: &JsonValue) -> Result<(String, Vec<StockPreset>), String> {
    strict_object(
        value,
        &[
            "schema_version",
            "catalog_id",
            "catalog_version",
            "units",
            "reading_semantics",
            "presets",
        ],
        "catalog",
    )?;
    require_string_eq(
        value,
        "schema_version",
        "vehicle_preset_catalog.v1",
        "catalog",
    )?;
    require_string_eq(
        value,
        "catalog_id",
        "racelinecalc.common.vehicle_presets",
        "catalog",
    )?;
    require_string_eq(value, "units", "si", "catalog")?;
    require_string_eq(
        value,
        "reading_semantics",
        STOCK_READING_SEMANTICS,
        "catalog",
    )?;
    let catalog_version = required_string(value, "catalog_version", "catalog")?;
    if catalog_version != STOCK_CATALOG_VERSION {
        return Err(format!(
            "unsupported embedded public catalog version: {catalog_version}"
        ));
    }
    let presets = required_array(value, "presets", "catalog")?
        .iter()
        .enumerate()
        .map(|(index, preset)| parse_preset(preset, index))
        .collect::<Result<Vec<_>, _>>()?;
    validate_coverage(&presets)?;
    Ok((catalog_version, presets))
}

fn parse_preset(value: &JsonValue, index: usize) -> Result<StockPreset, String> {
    let context = format!("catalog.presets[{index}]");
    strict_object(
        value,
        &["preset_id", "vehicle_class", "class_data", "adapters"],
        &context,
    )?;
    let preset_id = required_string(value, "preset_id", &context)?;
    let vehicle_class = VehicleClass::parse(&required_string(value, "vehicle_class", &context)?)?;
    let class_fields = if vehicle_class == VehicleClass::Point {
        &[][..]
    } else {
        CLASS_DATA_FIELDS
    };
    let class_data = parse_parameter_object(
        required_field(value, "class_data", &context)?,
        class_fields,
        &format!("{context}.class_data"),
    )?;
    let adapters = required_field(value, "adapters", &context)?;
    let model_id = vehicle_class.model_id();
    strict_object(
        adapters,
        &[model_id.as_str()],
        &format!("{context}.adapters"),
    )?;
    let adapter = required_field(adapters, model_id.as_str(), &context)?;
    strict_object(adapter, &["parameters"], &format!("{context}.adapter"))?;
    let parameters = parse_parameter_object(
        required_field(adapter, "parameters", &context)?,
        fields_for_model(model_id),
        &format!("{context}.parameters"),
    )?;
    validate_invariants(vehicle_class, &preset_id, &class_data, &parameters)?;
    Ok(StockPreset {
        preset_id,
        vehicle_class,
        class_data,
        parameters,
    })
}

fn materialize_profile(
    model_id: StockModelId,
    preset: &StockPreset,
) -> Result<ResolvedStockProfile, String> {
    match model_id {
        StockModelId::PointMass => {
            let width_opt_m = required_object_number(&preset.parameters, "width_opt_m")?;
            let profile = PointMassProfileV1 {
                schema_version: PointMassProfileV1::SCHEMA_VERSION.to_owned(),
                profile_id: preset.preset_id.clone(),
                model_kind: PointMassProfileV1::MODEL_KIND.to_owned(),
                params: point_profile_parameters(&preset.parameters)?,
                metadata: Vec::new(),
            };
            let acceleration_envelope =
                profile.to_acceleration_envelope(9.81).ok_or_else(|| {
                    format!("{} cannot generate acceleration envelope", preset.preset_id)
                })?;
            Ok(ResolvedStockProfile::PointMass {
                preset_id: preset.preset_id.clone(),
                class_data: preset.class_data.clone(),
                native_parameters: preset.parameters.clone(),
                width_opt_m,
                profile,
                acceleration_envelope,
            })
        }
        StockModelId::CarV1 | StockModelId::MotoV1 => {
            let (model_family, solver_id) = match model_id {
                StockModelId::CarV1 => (VehicleDynamicsModelFamily::CarDynamics, "old_car_mintime"),
                StockModelId::MotoV1 => (
                    VehicleDynamicsModelFamily::BikeDynamics,
                    "bike_single_track_mintime",
                ),
                StockModelId::PointMass => unreachable!(),
            };
            Ok(ResolvedStockProfile::VehicleDynamics {
                model_id,
                preset_id: preset.preset_id.clone(),
                class_data: preset.class_data.clone(),
                profile: VehicleDynamicsProfileV1 {
                    schema_version: VehicleDynamicsProfileV1::SCHEMA_VERSION.to_owned(),
                    profile_id: format!("{}:{}", model_id.as_str(), preset.preset_id),
                    model_family,
                    preset_id: Some(preset.preset_id.clone()),
                    solver_id: Some(solver_id.to_owned()),
                    parameters: preset.parameters.clone(),
                    native_parameters: Vec::new(),
                    metadata: Vec::new(),
                },
            })
        }
    }
}

fn point_profile_parameters(native_parameters: &JsonObject) -> Result<JsonObject, String> {
    let mut parameters = Vec::new();
    for key in [
        "v_max_mps",
        "ax_forward_max_g",
        "ax_brake_max_g",
        "ay_left_max_g",
        "ay_right_max_g",
        "coupling_exponent",
    ] {
        let value = native_parameters
            .iter()
            .find(|(candidate, _)| candidate == key)
            .map(|(_, value)| value.clone())
            .ok_or_else(|| format!("missing point profile parameter {key}"))?;
        parameters.push((key.to_owned(), value));
    }
    for key in [
        "yaw_rate_max_deg_s",
        "yaw_accel_max_deg_s2",
        "curvature_max_1pm",
        "curvature_slew_max_1pm2",
        "heading_step_max_deg",
    ] {
        parameters.push((key.to_owned(), JsonValue::Null));
    }
    Ok(parameters)
}

fn validate_coverage(presets: &[StockPreset]) -> Result<(), String> {
    let ids_for = |class| {
        presets
            .iter()
            .filter(|preset| preset.vehicle_class == class)
            .map(|preset| preset.preset_id.as_str())
            .collect::<Vec<_>>()
    };
    require_exact_ids(
        &ids_for(VehicleClass::Point),
        POINT_PRESETS,
        "point presets",
    )?;
    require_exact_ids(&ids_for(VehicleClass::Car), CAR_PRESETS, "car presets")?;
    require_exact_ids(&ids_for(VehicleClass::Moto), MOTO_PRESETS, "moto presets")?;
    Ok(())
}

fn validate_invariants(
    class: VehicleClass,
    preset_id: &str,
    class_data: &JsonObject,
    parameters: &JsonObject,
) -> Result<(), String> {
    if class == VehicleClass::Point {
        if required_object_number(parameters, "width_opt_m")? < 0.0 {
            return Err(format!("{preset_id} point width_opt_m must be nonnegative"));
        }
        return Ok(());
    }
    let operator = required_object_number(class_data, "operator_mass_kg")?;
    let vehicle_mass = required_object_number(class_data, "vehicle_mass_kg")?;
    let body_width = required_object_number(class_data, "vehicle_width_m")?;
    let length = required_object_number(class_data, "vehicle_length_m")?;
    let width_opt = required_object_number(parameters, "width_opt_m")?;
    if operator != 75.0 {
        return Err(format!("{preset_id} operator_mass_kg must be 75"));
    }
    if vehicle_mass <= 0.0 || body_width <= 0.0 || length <= 0.0 {
        return Err(format!(
            "{preset_id} class dimensions and mass must be positive"
        ));
    }
    if body_width > width_opt {
        return Err(format!(
            "{preset_id} body width exceeds stock width_opt_m; no autocorrection"
        ));
    }
    let (mass_key, wheelbase) = match class {
        VehicleClass::Car => (
            "mass_kg",
            required_object_number(parameters, "wheelbase_front")?
                + required_object_number(parameters, "wheelbase_rear")?,
        ),
        VehicleClass::Moto => (
            "rider_bike_mass_kg",
            required_object_number(parameters, "wheelbase_m")?,
        ),
        VehicleClass::Point => unreachable!(),
    };
    if required_object_number(parameters, mass_key)? != vehicle_mass + operator {
        return Err(format!("{preset_id} total mass must count operator once"));
    }
    if length < wheelbase {
        return Err(format!("{preset_id} vehicle length is below wheelbase"));
    }
    Ok(())
}

fn fields_for_model(model_id: StockModelId) -> &'static [&'static str] {
    match model_id {
        StockModelId::PointMass => POINT_FIELDS,
        StockModelId::CarV1 => CAR_FIELDS,
        StockModelId::MotoV1 => MOTO_FIELDS,
    }
}

fn reject_duplicate_fields(value: &JsonValue, context: &str) -> Result<(), String> {
    match value {
        JsonValue::Object(entries) => {
            let mut seen = BTreeSet::new();
            for (key, entry) in entries {
                if !seen.insert(key.as_str()) {
                    return Err(format!("duplicate field {context}.{key}"));
                }
                reject_duplicate_fields(entry, &format!("{context}.{key}"))?;
            }
        }
        JsonValue::Array(entries) => {
            for (index, entry) in entries.iter().enumerate() {
                reject_duplicate_fields(entry, &format!("{context}[{index}]"))?;
            }
        }
        _ => {}
    }
    Ok(())
}

fn strict_object<'a>(
    value: &'a JsonValue,
    expected_fields: &[&str],
    context: &str,
) -> Result<&'a JsonObject, String> {
    let JsonValue::Object(entries) = value else {
        return Err(format!("{context} must be an object"));
    };
    let actual = entries
        .iter()
        .map(|(key, _)| key.as_str())
        .collect::<Vec<_>>();
    require_exact_ids(&actual, expected_fields, context)?;
    Ok(entries)
}

fn require_exact_ids(actual: &[&str], expected: &[&str], context: &str) -> Result<(), String> {
    let actual_set = actual.iter().copied().collect::<BTreeSet<_>>();
    if actual_set.len() != actual.len() {
        return Err(format!("{context} contains duplicate IDs"));
    }
    let expected_set = expected.iter().copied().collect::<BTreeSet<_>>();
    if actual_set != expected_set {
        let unknown = actual_set
            .difference(&expected_set)
            .copied()
            .collect::<Vec<_>>();
        let missing = expected_set
            .difference(&actual_set)
            .copied()
            .collect::<Vec<_>>();
        return Err(format!(
            "{context} fields/IDs mismatch; unknown={unknown:?} missing={missing:?}"
        ));
    }
    Ok(())
}

fn parse_parameter_object(
    value: &JsonValue,
    expected_fields: &[&str],
    context: &str,
) -> Result<JsonObject, String> {
    let entries = strict_object(value, expected_fields, context)?;
    for (key, value) in entries {
        match value {
            JsonValue::Integer(_) | JsonValue::String(_) | JsonValue::Bool(_) => {}
            JsonValue::Number(number) if number.is_finite() => {}
            _ => return Err(format!("{context}.{key} has unsupported value")),
        }
    }
    Ok(entries.clone())
}

fn required_field<'a>(
    value: &'a JsonValue,
    key: &str,
    context: &str,
) -> Result<&'a JsonValue, String> {
    value
        .get(key)
        .ok_or_else(|| format!("missing {context}.{key}"))
}

fn required_array<'a>(
    value: &'a JsonValue,
    key: &str,
    context: &str,
) -> Result<&'a [JsonValue], String> {
    required_field(value, key, context)?
        .as_array()
        .ok_or_else(|| format!("{context}.{key} must be an array"))
}

fn required_string(value: &JsonValue, key: &str, context: &str) -> Result<String, String> {
    required_field(value, key, context)?
        .as_str()
        .map(str::to_owned)
        .ok_or_else(|| format!("{context}.{key} must be a string"))
}

fn require_string_eq(
    value: &JsonValue,
    key: &str,
    expected: &str,
    context: &str,
) -> Result<(), String> {
    let actual = required_string(value, key, context)?;
    if actual != expected {
        return Err(format!("{context}.{key} must be {expected}; got {actual}"));
    }
    Ok(())
}

fn required_object_number(value: &JsonObject, key: &str) -> Result<f64, String> {
    value
        .iter()
        .find(|(candidate, _)| candidate == key)
        .and_then(|(_, value)| value.as_f64())
        .filter(|number| number.is_finite())
        .ok_or_else(|| format!("missing finite numeric parameter {key}"))
}
