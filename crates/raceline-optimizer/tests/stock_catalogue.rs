use raceline_optimizer::json::{parse_json_str, JsonValue};
use raceline_optimizer::models::stock_catalogue::{
    embedded_public_stock_catalogue, PublicStockCatalogue, ResolvedStockProfile, StockModelId,
    STOCK_CATALOG_FILE_SHA256, STOCK_CATALOG_VERSION, STOCK_READING_SEMANTICS,
};
use raceline_optimizer::vehicle_dynamics::{
    BikeSingleTrackLeanParams, CarDoubleTrackParams, VehicleDynamicsProfileV1,
};
use raceline_optimizer::JsonObject;

const CATALOG_JSON: &str = include_str!("../catalogs/vehicle-presets.v1.json");
const PARITY_FIXTURE_JSON: &str = include_str!("fixtures/stock-profile-parity-public.v1.json");

#[test]
fn embedded_catalogue_is_the_exact_public_snapshot() {
    let catalogue = embedded_public_stock_catalogue().expect("public stock catalogue must load");
    assert_eq!(catalogue.catalog_version(), STOCK_CATALOG_VERSION);
    assert_eq!(catalogue.preset_count(), 14);
    assert_eq!(
        STOCK_CATALOG_FILE_SHA256,
        "778e131b2f646e9a589e763543edf89e051b2c4efdd07635c24793e604d8fd12"
    );
    assert!(!CATALOG_JSON.contains("\"car_v2\""));
    assert!(!CATALOG_JSON.contains("\"moto_v2\""));
    assert!(!CATALOG_JSON.contains("parameter_set_version"));
    assert!(!CATALOG_JSON.contains("parameter_provenance"));
}

#[test]
fn all_fourteen_native_parameter_maps_match_the_public_fixture() {
    let fixture = parse_json_str(PARITY_FIXTURE_JSON).expect("public parity fixture must parse");
    assert_eq!(
        required_string(&fixture, "schema_version"),
        "stock_profile_parity_public.v1"
    );
    assert_eq!(
        required_string(&fixture, "catalog_version"),
        STOCK_CATALOG_VERSION
    );
    assert_eq!(
        required_string(&fixture, "reading_semantics"),
        STOCK_READING_SEMANTICS
    );
    let profiles = required_array(&fixture, "profiles");
    assert_eq!(profiles.len(), 14);

    let catalogue = embedded_public_stock_catalogue().unwrap();
    for expected in profiles {
        let model_id = required_string(expected, "model_id");
        let preset_id = required_string(expected, "preset_id");
        let resolved = catalogue
            .resolve_stock_profile(STOCK_CATALOG_VERSION, model_id, preset_id)
            .unwrap_or_else(|error| panic!("{model_id}/{preset_id}: {error}"));
        assert_eq!(resolved.model_id(), StockModelId::parse(model_id).unwrap());
        assert_eq!(resolved.preset_id(), preset_id);
        assert_object_fields_eq(
            resolved.class_data(),
            required_object(expected, "class_data"),
            &format!("{model_id}/{preset_id} class_data"),
        );
        assert_object_fields_eq(
            resolved.native_parameters(),
            required_object(expected, "parameters"),
            &format!("{model_id}/{preset_id} parameters"),
        );
    }
}

#[test]
fn all_vehicle_profiles_pass_the_existing_public_native_constructors() {
    let fixture = parse_json_str(PARITY_FIXTURE_JSON).unwrap();
    let catalogue = embedded_public_stock_catalogue().unwrap();
    for expected in required_array(&fixture, "profiles") {
        let model_id = required_string(expected, "model_id");
        if model_id == "point_mass" {
            continue;
        }
        let preset_id = required_string(expected, "preset_id");
        let profile = vehicle_profile(
            catalogue.resolve_stock_profile(STOCK_CATALOG_VERSION, model_id, preset_id),
            model_id,
            preset_id,
        );
        match model_id {
            "car_v1" => {
                let params = CarDoubleTrackParams::from_profile(&profile)
                    .unwrap_or_else(|error| panic!("{model_id}/{preset_id}: {error}"));
                assert!(params.mass_kg.is_finite() && params.mass_kg > 0.0);
                assert!(params.wheelbase_m.is_finite() && params.wheelbase_m > 0.0);
            }
            "moto_v1" => {
                let params = BikeSingleTrackLeanParams::from_profile(&profile)
                    .unwrap_or_else(|error| panic!("{model_id}/{preset_id}: {error}"));
                assert!(params.rider_bike_mass_kg.is_finite() && params.rider_bike_mass_kg > 0.0);
                assert!(params.wheelbase_m.is_finite() && params.wheelbase_m > 0.0);
            }
            _ => panic!("unexpected public model {model_id}"),
        }
    }
}

#[test]
fn point_stock_materializes_profile_and_envelope() {
    let resolved = embedded_public_stock_catalogue()
        .unwrap()
        .resolve_stock_profile(STOCK_CATALOG_VERSION, "point_mass", "point_reference")
        .unwrap();
    let ResolvedStockProfile::PointMass {
        width_opt_m,
        profile,
        acceleration_envelope,
        ..
    } = resolved
    else {
        panic!("point stock must materialize a point profile");
    };
    assert_eq!(width_opt_m, 0.0);
    assert_eq!(profile.profile_id, "point_reference");
    assert!(!profile.params.iter().any(|(key, _)| key == "width_opt_m"));
    assert_eq!(acceleration_envelope.speed_mps, vec![0.0, 200.0]);
}

#[test]
fn lookup_rejects_unknown_versions_models_and_class_mismatches() {
    let catalogue = embedded_public_stock_catalogue().unwrap();
    assert_error_contains(
        catalogue.resolve_stock_profile("unknown.v1", "car_v1", "kart_125cc"),
        "unknown public stock catalog version",
    );
    for model_id in ["car_v2", "moto_v2"] {
        assert_error_contains(
            catalogue.resolve_stock_profile(STOCK_CATALOG_VERSION, model_id, "kart_125cc"),
            "unknown public stock model_id",
        );
    }
    assert_error_contains(
        catalogue.resolve_stock_profile(STOCK_CATALOG_VERSION, "car_v1", "unknown"),
        "unknown public stock preset",
    );
    assert_error_contains(
        catalogue.resolve_stock_profile(STOCK_CATALOG_VERSION, "car_v1", "moto_700_naked"),
        "unknown public stock preset",
    );
}

#[test]
fn strict_schema_rejects_unknown_duplicate_and_changed_bytes() {
    let unknown_field = CATALOG_JSON.replacen(
        "\"schema_version\":",
        "\"unexpected\": true,\n  \"schema_version\":",
        1,
    );
    assert_catalog_error(&unknown_field, "unknown=[\"unexpected\"]");

    let duplicate_nested = CATALOG_JSON.replacen(
        "\"operator_mass_kg\": 75",
        "\"operator_mass_kg\": 75,\n        \"operator_mass_kg\": 75",
        1,
    );
    assert_catalog_error(&duplicate_nested, "duplicate field");

    let duplicate_id = CATALOG_JSON.replacen(
        "\"preset_id\": \"micro_city_oka_matiz\"",
        "\"preset_id\": \"kart_125cc\"",
        1,
    );
    assert_catalog_error(&duplicate_id, "duplicate IDs");

    let changed_bytes = CATALOG_JSON.replacen(
        "\"vehicle_length_m\": 1.83",
        "\"vehicle_length_m\": 1.84",
        1,
    );
    assert_catalog_error(
        &changed_bytes,
        "immutable public stock catalogue SHA-256 mismatch",
    );
}

fn assert_catalog_error(json: &str, expected: &str) {
    assert_error_contains(PublicStockCatalogue::from_json_str(json), expected);
}

fn vehicle_profile(
    result: Result<ResolvedStockProfile, String>,
    model_id: &str,
    preset_id: &str,
) -> VehicleDynamicsProfileV1 {
    match result.unwrap_or_else(|error| panic!("{model_id}/{preset_id}: {error}")) {
        ResolvedStockProfile::VehicleDynamics { profile, .. } => profile,
        ResolvedStockProfile::PointMass { .. } => {
            panic!("{model_id}/{preset_id} unexpectedly materialized as point mass")
        }
    }
}

fn required_string<'a>(value: &'a JsonValue, key: &str) -> &'a str {
    value.get(key).and_then(JsonValue::as_str).unwrap()
}

fn required_array<'a>(value: &'a JsonValue, key: &str) -> &'a [JsonValue] {
    value.get(key).and_then(JsonValue::as_array).unwrap()
}

fn required_object<'a>(value: &'a JsonValue, key: &str) -> &'a JsonObject {
    match value.get(key).unwrap() {
        JsonValue::Object(entries) => entries,
        _ => panic!("{key} must be an object"),
    }
}

fn assert_object_fields_eq(actual: &JsonObject, expected: &JsonObject, context: &str) {
    assert_eq!(actual.len(), expected.len(), "{context} field count");
    for (key, expected_value) in expected {
        let actual_value = actual
            .iter()
            .find(|(candidate, _)| candidate == key)
            .map(|(_, value)| value)
            .unwrap_or_else(|| panic!("{context} is missing {key}"));
        assert_eq!(actual_value, expected_value, "{context}.{key}");
    }
}

fn assert_error_contains<T>(result: Result<T, String>, expected: &str) {
    let error = result.err().expect("operation unexpectedly succeeded");
    assert!(
        error.contains(expected),
        "expected error containing {expected:?}, got {error:?}"
    );
}
