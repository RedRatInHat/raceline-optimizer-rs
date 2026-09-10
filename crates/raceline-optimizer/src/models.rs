use crate::json::JsonValue;
use crate::{JsonObject, ToJsonValue};

pub const POINT_MASS_FAMILY: &str = "point_mass";
pub const CAR_DYNAMICS_FAMILY: &str = "car_dynamics";
pub const BIKE_DYNAMICS_FAMILY: &str = "bike_dynamics";

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ModelFamily {
    PointMass,
    CarDynamics,
    BikeDynamics,
}

impl ModelFamily {
    #[must_use]
    pub fn as_contract_key(&self) -> &'static str {
        match self {
            Self::PointMass => POINT_MASS_FAMILY,
            Self::CarDynamics => CAR_DYNAMICS_FAMILY,
            Self::BikeDynamics => BIKE_DYNAMICS_FAMILY,
        }
    }

    #[must_use]
    pub fn legacy_model_kind(&self) -> &'static str {
        match self {
            Self::PointMass => "point_mass_envelope",
            Self::CarDynamics => "car_double_track",
            Self::BikeDynamics => "bike_single_track_lean",
        }
    }
}

impl From<&ModelFamily> for JsonValue {
    fn from(value: &ModelFamily) -> Self {
        value.as_contract_key().into()
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ModelContractV1 {
    pub schema_version: String,
    pub model_family: ModelFamily,
    pub contract_key: String,
    pub legacy_model_kind: String,
    pub solver_id: String,
    pub supported_in_rust_runtime: bool,
    pub notes: Vec<String>,
}

impl ModelContractV1 {
    pub const SCHEMA_VERSION: &'static str = "ModelContractV1";

    #[must_use]
    pub fn point_mass() -> Self {
        Self {
            schema_version: Self::SCHEMA_VERSION.to_owned(),
            model_family: ModelFamily::PointMass,
            contract_key: POINT_MASS_FAMILY.to_owned(),
            legacy_model_kind: "point_mass_envelope".to_owned(),
            solver_id: "point_mass_envelope_sections".to_owned(),
            supported_in_rust_runtime: true,
            notes: vec![
                "Rust point-mass runtime owns station preprocessing and sectioned corridor solve."
                    .to_owned(),
                "Python velocity_vector_ocp_v1 remains a reference/parity source, not a runtime dependency."
                    .to_owned(),
            ],
        }
    }

    #[must_use]
    pub fn car_dynamics() -> Self {
        Self {
            schema_version: Self::SCHEMA_VERSION.to_owned(),
            model_family: ModelFamily::CarDynamics,
            contract_key: CAR_DYNAMICS_FAMILY.to_owned(),
            legacy_model_kind: "car_double_track".to_owned(),
            solver_id: "old_car_mintime".to_owned(),
            supported_in_rust_runtime: true,
            notes: vec![
                "Rust car mintime production path uses direct-dense targets for parity acceptance."
                    .to_owned(),
                "Warm-start, staged profile, and continuation artifacts remain diagnostic-only."
                    .to_owned(),
            ],
        }
    }

    #[must_use]
    pub fn bike_dynamics() -> Self {
        Self {
            schema_version: Self::SCHEMA_VERSION.to_owned(),
            model_family: ModelFamily::BikeDynamics,
            contract_key: BIKE_DYNAMICS_FAMILY.to_owned(),
            legacy_model_kind: "bike_single_track_lean".to_owned(),
            solver_id: "bike_single_track_mintime".to_owned(),
            supported_in_rust_runtime: true,
            notes: vec![
                "Rust exposes the bike_single_track_mintime solver through the public solver API."
                    .to_owned(),
                "Bike settings keep their own vocabulary instead of being narrowed car settings."
                    .to_owned(),
            ],
        }
    }
}

impl ToJsonValue for ModelContractV1 {
    fn to_json_value(&self) -> JsonValue {
        JsonValue::Object(vec![
            (
                "schema_version".to_owned(),
                self.schema_version.clone().into(),
            ),
            ("model_family".to_owned(), (&self.model_family).into()),
            ("contract_key".to_owned(), self.contract_key.clone().into()),
            (
                "legacy_model_kind".to_owned(),
                self.legacy_model_kind.clone().into(),
            ),
            ("solver_id".to_owned(), self.solver_id.clone().into()),
            (
                "supported_in_rust_runtime".to_owned(),
                self.supported_in_rust_runtime.into(),
            ),
            (
                "notes".to_owned(),
                JsonValue::Array(self.notes.iter().cloned().map(JsonValue::from).collect()),
            ),
        ])
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct PresetCatalogV1 {
    pub schema_version: String,
    pub catalog_id: String,
    pub model_family: ModelFamily,
    pub usage: JsonObject,
    pub presets: Vec<ModelPresetV1>,
}

impl PresetCatalogV1 {
    pub const SCHEMA_VERSION: &'static str = "PresetCatalogV1";
}

impl ToJsonValue for PresetCatalogV1 {
    fn to_json_value(&self) -> JsonValue {
        JsonValue::Object(vec![
            (
                "schema_version".to_owned(),
                self.schema_version.clone().into(),
            ),
            ("catalog_id".to_owned(), self.catalog_id.clone().into()),
            ("model_family".to_owned(), (&self.model_family).into()),
            ("usage".to_owned(), JsonValue::Object(self.usage.clone())),
            (
                "presets".to_owned(),
                JsonValue::Array(
                    self.presets
                        .iter()
                        .map(ModelPresetV1::to_json_value)
                        .collect(),
                ),
            ),
        ])
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct ModelPresetV1 {
    pub preset_id: String,
    pub label: String,
    pub params_file: String,
    pub class: Option<String>,
    pub preset_group: Option<String>,
    pub model_family: Option<String>,
    pub mass_kg: Option<f64>,
    pub power_kw: Option<f64>,
    pub v_max_mps: Option<f64>,
    pub drive_layout: Option<String>,
    pub mue: Option<f64>,
    pub phi_max_rad: Option<f64>,
    pub extra: JsonObject,
}

impl ToJsonValue for ModelPresetV1 {
    fn to_json_value(&self) -> JsonValue {
        let mut entries = vec![
            ("preset_id".to_owned(), self.preset_id.clone().into()),
            ("label".to_owned(), self.label.clone().into()),
            ("params_file".to_owned(), self.params_file.clone().into()),
        ];
        push_opt_string(&mut entries, "class", &self.class);
        push_opt_string(&mut entries, "preset_group", &self.preset_group);
        push_opt_string(&mut entries, "model_family", &self.model_family);
        push_opt_f64(&mut entries, "mass_kg", self.mass_kg);
        push_opt_f64(&mut entries, "power_kw", self.power_kw);
        push_opt_f64(&mut entries, "v_max_mps", self.v_max_mps);
        push_opt_string(&mut entries, "drive_layout", &self.drive_layout);
        push_opt_f64(&mut entries, "mue", self.mue);
        push_opt_f64(&mut entries, "phi_max_rad", self.phi_max_rad);
        entries.extend(self.extra.clone());
        JsonValue::Object(entries)
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct LegacyPresetCatalog {
    pub schema_version: String,
    pub usage: JsonObject,
    pub presets: Vec<ModelPresetV1>,
}

impl LegacyPresetCatalog {
    pub fn from_json(value: &JsonValue) -> Result<Self, String> {
        let schema_version = required_string(value, "schema_version")?;
        let usage = match value.get("usage") {
            Some(JsonValue::Object(entries)) => entries.clone(),
            _ => Vec::new(),
        };
        let presets = value
            .get("presets")
            .and_then(JsonValue::as_array)
            .ok_or_else(|| "missing presets array".to_owned())?
            .iter()
            .map(ModelPresetV1::from_json)
            .collect::<Result<Vec<_>, _>>()?;
        Ok(Self {
            schema_version,
            usage,
            presets,
        })
    }
}

impl ModelPresetV1 {
    fn from_json(value: &JsonValue) -> Result<Self, String> {
        let mut extra = Vec::new();
        if let JsonValue::Object(entries) = value {
            for (key, item) in entries {
                if !matches!(
                    key.as_str(),
                    "preset_id"
                        | "label"
                        | "params_file"
                        | "class"
                        | "preset_group"
                        | "model_family"
                        | "mass_kg"
                        | "power_kw"
                        | "v_max_mps"
                        | "drive_layout"
                        | "mue"
                        | "phi_max_rad"
                ) {
                    extra.push((key.clone(), item.clone()));
                }
            }
        }
        Ok(Self {
            preset_id: required_string(value, "preset_id")?,
            label: required_string(value, "label")?,
            params_file: required_string(value, "params_file")?,
            class: optional_string(value, "class"),
            preset_group: optional_string(value, "preset_group"),
            model_family: optional_string(value, "model_family"),
            mass_kg: optional_f64(value, "mass_kg"),
            power_kw: optional_f64(value, "power_kw"),
            v_max_mps: optional_f64(value, "v_max_mps"),
            drive_layout: optional_string(value, "drive_layout"),
            mue: optional_f64(value, "mue"),
            phi_max_rad: optional_f64(value, "phi_max_rad"),
            extra,
        })
    }
}

#[must_use]
pub fn product_catalog_from_legacy(legacy: LegacyPresetCatalog) -> PresetCatalogV1 {
    PresetCatalogV1 {
        schema_version: PresetCatalogV1::SCHEMA_VERSION.to_owned(),
        catalog_id: legacy.schema_version,
        model_family: ModelFamily::CarDynamics,
        usage: legacy.usage,
        presets: legacy
            .presets
            .into_iter()
            .map(|mut preset| {
                preset.preset_group = Some("car".to_owned());
                preset.model_family = Some(ModelFamily::CarDynamics.legacy_model_kind().to_owned());
                preset
            })
            .collect(),
    }
}

#[must_use]
pub fn moto_catalog_from_legacy(legacy: LegacyPresetCatalog) -> PresetCatalogV1 {
    PresetCatalogV1 {
        schema_version: PresetCatalogV1::SCHEMA_VERSION.to_owned(),
        catalog_id: legacy.schema_version,
        model_family: ModelFamily::BikeDynamics,
        usage: legacy.usage,
        presets: legacy
            .presets
            .into_iter()
            .map(|mut preset| {
                preset.preset_group = Some("moto".to_owned());
                preset.model_family =
                    Some(ModelFamily::BikeDynamics.legacy_model_kind().to_owned());
                preset
            })
            .collect(),
    }
}

#[must_use]
pub fn point_mass_default_catalog() -> PresetCatalogV1 {
    PresetCatalogV1 {
        schema_version: PresetCatalogV1::SCHEMA_VERSION.to_owned(),
        catalog_id: "point_mass_presets.v1".to_owned(),
        model_family: ModelFamily::PointMass,
        usage: Vec::new(),
        presets: vec![ModelPresetV1 {
            preset_id: "point_reference".to_owned(),
            label: "Point mass reference".to_owned(),
            params_file: "gt3_track_car_mintime.ini".to_owned(),
            class: Some("point_reference".to_owned()),
            preset_group: Some("point".to_owned()),
            model_family: Some(ModelFamily::PointMass.legacy_model_kind().to_owned()),
            mass_kg: None,
            power_kw: None,
            v_max_mps: Some(200.0),
            drive_layout: None,
            mue: None,
            phi_max_rad: None,
            extra: vec![
                ("ax_forward_max_g".to_owned(), 0.5.into()),
                ("ax_brake_max_g".to_owned(), 1.0.into()),
                ("ay_left_max_g".to_owned(), 1.5.into()),
                ("ay_right_max_g".to_owned(), 1.5.into()),
                ("coupling_exponent".to_owned(), 2.0.into()),
                ("yaw_rate_max_deg_s".to_owned(), JsonValue::Null),
                ("yaw_accel_max_deg_s2".to_owned(), JsonValue::Null),
                ("curvature_max_1pm".to_owned(), JsonValue::Null),
                ("curvature_slew_max_1pm2".to_owned(), JsonValue::Null),
                ("heading_step_max_deg".to_owned(), JsonValue::Null),
            ],
        }],
    }
}

#[must_use]
pub fn all_model_contracts() -> Vec<ModelContractV1> {
    vec![
        ModelContractV1::point_mass(),
        ModelContractV1::car_dynamics(),
        ModelContractV1::bike_dynamics(),
    ]
}

#[derive(Clone, Debug, PartialEq)]
pub struct ModelContractsManifestV1 {
    pub schema_version: String,
    pub source: String,
    pub model_contracts: Vec<ModelContractV1>,
    pub preset_catalogs: Vec<PresetCatalogV1>,
}

impl ModelContractsManifestV1 {
    pub const SCHEMA_VERSION: &'static str = "ModelContractsManifestV1";
}

impl ToJsonValue for ModelContractsManifestV1 {
    fn to_json_value(&self) -> JsonValue {
        JsonValue::Object(vec![
            (
                "schema_version".to_owned(),
                self.schema_version.clone().into(),
            ),
            ("source".to_owned(), self.source.clone().into()),
            (
                "model_contracts".to_owned(),
                JsonValue::Array(
                    self.model_contracts
                        .iter()
                        .map(ModelContractV1::to_json_value)
                        .collect(),
                ),
            ),
            (
                "preset_catalogs".to_owned(),
                JsonValue::Array(
                    self.preset_catalogs
                        .iter()
                        .map(PresetCatalogV1::to_json_value)
                        .collect(),
                ),
            ),
        ])
    }
}

fn push_opt_string(entries: &mut JsonObject, key: &str, value: &Option<String>) {
    if let Some(value) = value {
        entries.push((key.to_owned(), value.clone().into()));
    }
}

fn push_opt_f64(entries: &mut JsonObject, key: &str, value: Option<f64>) {
    if let Some(value) = value {
        entries.push((key.to_owned(), value.into()));
    }
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

#[cfg(test)]
mod tests {
    use super::{all_model_contracts, ModelFamily};

    #[test]
    fn three_product_model_families_are_registered() {
        let contracts = all_model_contracts();
        let families = contracts
            .iter()
            .map(|contract| contract.model_family.clone())
            .collect::<Vec<_>>();

        assert_eq!(
            families,
            vec![
                ModelFamily::PointMass,
                ModelFamily::CarDynamics,
                ModelFamily::BikeDynamics
            ]
        );
        assert!(contracts
            .iter()
            .find(|contract| contract.model_family == ModelFamily::PointMass)
            .is_some_and(|contract| contract.supported_in_rust_runtime));
        assert!(contracts
            .iter()
            .find(|contract| contract.model_family == ModelFamily::CarDynamics)
            .is_some_and(|contract| contract.supported_in_rust_runtime));
        assert!(contracts
            .iter()
            .find(|contract| contract.model_family == ModelFamily::BikeDynamics)
            .is_some_and(|contract| contract.supported_in_rust_runtime));
    }
}
