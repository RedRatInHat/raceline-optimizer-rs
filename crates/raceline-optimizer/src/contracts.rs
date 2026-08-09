use crate::json::JsonValue;
use crate::{JsonObject, ToJsonValue};
use std::fmt::Write;

pub type Point2 = [f64; 2];
pub const SECTIONS_TRACK_VIEW_HASH_V1: &str = "sections_track_view_hash.v1";
pub const SECTIONS_TRACK_VIEW_HASH_V2: &str = "sections_track_view_hash.v2";
pub const STATION_GEOMETRY_CONTENT_HASH_V1: &str = "station_geometry_content_hash.v1";
pub const STATION_GEOMETRY_CONTENT_HASH_V2: &str = "station_geometry_content_hash.v2";
pub const PREPARED_STATION_BUNDLE_HASH_V2: &str = "prepared_station_bundle_hash.v2";
pub const PREPARED_STATION_BUNDLE_HASH_V3: &str = "prepared_station_bundle_hash.v3";

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct StationSourceRefV1 {
    pub project_id: String,
    pub geometry_id: String,
    pub geometry_content_hash: String,
    pub route_id: String,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct StationRecipeV1 {
    pub direction: String,
    pub station_options_hash: String,
    pub resolved_station_count: usize,
    pub generator_contract: String,
    pub generator_version: String,
    pub validation_contract: String,
    pub validation_version: String,
}

#[derive(Clone, Debug, PartialEq)]
pub struct StartFinish {
    pub p1_m: Point2,
    pub p2_m: Point2,
}

#[derive(Clone, Debug, PartialEq)]
pub struct TrackAreaContractV1 {
    pub schema_version: String,
    pub track_id: String,
    pub units: String,
    pub left_boundary_xy_m: Vec<Point2>,
    pub right_boundary_xy_m: Vec<Point2>,
    pub start_finish_xy_m: Option<StartFinish>,
    pub finish_line_xy_m: Option<StartFinish>,
    pub trajectory_mode: String,
    pub direction: Option<String>,
    pub metadata: JsonObject,
    pub image_path: Option<String>,
    pub image_width_px: Option<u32>,
    pub image_height_px: Option<u32>,
    pub meters_per_pixel: Option<f64>,
}

impl TrackAreaContractV1 {
    pub const SCHEMA_VERSION: &'static str = "TrackAreaContractV1";

    #[must_use]
    pub fn new(
        track_id: impl Into<String>,
        left_boundary_xy_m: Vec<Point2>,
        right_boundary_xy_m: Vec<Point2>,
    ) -> Self {
        Self {
            schema_version: Self::SCHEMA_VERSION.to_owned(),
            track_id: track_id.into(),
            units: "m".to_owned(),
            left_boundary_xy_m,
            right_boundary_xy_m,
            start_finish_xy_m: None,
            finish_line_xy_m: None,
            trajectory_mode: "closed".to_owned(),
            direction: None,
            metadata: Vec::new(),
            image_path: None,
            image_width_px: None,
            image_height_px: None,
            meters_per_pixel: None,
        }
    }

    pub fn from_json(value: &JsonValue) -> Result<Self, String> {
        Ok(Self {
            schema_version: required_string(value, "schema_version")?,
            track_id: required_string(value, "track_id")?,
            units: required_string(value, "units")?,
            left_boundary_xy_m: required_points(value, "left_boundary_xy_m")?,
            right_boundary_xy_m: required_points(value, "right_boundary_xy_m")?,
            start_finish_xy_m: optional_start_finish(value, "start_finish_xy_m")?,
            finish_line_xy_m: optional_start_finish(value, "finish_line_xy_m")?,
            trajectory_mode: optional_string(value, "trajectory_mode")
                .unwrap_or_else(|| "closed".to_owned()),
            direction: optional_string(value, "direction"),
            metadata: match value.get("metadata") {
                Some(JsonValue::Object(entries)) => entries.clone(),
                _ => Vec::new(),
            },
            image_path: optional_string(value, "image_path"),
            image_width_px: value.get("image_width_px").and_then(JsonValue::as_u32),
            image_height_px: value.get("image_height_px").and_then(JsonValue::as_u32),
            meters_per_pixel: value.get("meters_per_pixel").and_then(JsonValue::as_f64),
        })
    }
}

impl ToJsonValue for TrackAreaContractV1 {
    fn to_json_value(&self) -> JsonValue {
        JsonValue::Object(vec![
            (
                "schema_version".to_owned(),
                self.schema_version.clone().into(),
            ),
            ("track_id".to_owned(), self.track_id.clone().into()),
            ("units".to_owned(), self.units.clone().into()),
            (
                "left_boundary_xy_m".to_owned(),
                points_to_json(&self.left_boundary_xy_m),
            ),
            (
                "right_boundary_xy_m".to_owned(),
                points_to_json(&self.right_boundary_xy_m),
            ),
            (
                "start_finish_xy_m".to_owned(),
                option_start_finish_to_json(&self.start_finish_xy_m),
            ),
            (
                "finish_line_xy_m".to_owned(),
                option_start_finish_to_json(&self.finish_line_xy_m),
            ),
            (
                "trajectory_mode".to_owned(),
                self.trajectory_mode.clone().into(),
            ),
            (
                "direction".to_owned(),
                option_string_to_json(&self.direction),
            ),
            (
                "metadata".to_owned(),
                JsonValue::Object(self.metadata.clone()),
            ),
            (
                "image_path".to_owned(),
                option_string_to_json(&self.image_path),
            ),
            (
                "image_width_px".to_owned(),
                option_u32_to_json(self.image_width_px),
            ),
            (
                "image_height_px".to_owned(),
                option_u32_to_json(self.image_height_px),
            ),
            (
                "meters_per_pixel".to_owned(),
                option_f64_to_json(self.meters_per_pixel),
            ),
        ])
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct SectionsTrackViewV1 {
    pub schema_version: String,
    pub view_id: String,
    pub track_id: String,
    pub station_s_m: Vec<f64>,
    pub centerline_xy_m: Vec<Point2>,
    pub left_boundary_xy_m: Vec<Point2>,
    pub right_boundary_xy_m: Vec<Point2>,
    pub normals_xy: Vec<Point2>,
    pub width_left_m: Vec<f64>,
    pub width_right_m: Vec<f64>,
    pub section_dirs_xy: Vec<Point2>,
    pub quality_metrics: JsonObject,
    pub metadata: JsonObject,
}

impl SectionsTrackViewV1 {
    pub const SCHEMA_VERSION: &'static str = "SectionsTrackViewV1";

    pub fn from_json(value: &JsonValue) -> Result<Self, String> {
        Ok(Self {
            schema_version: required_string(value, "schema_version")?,
            view_id: required_string(value, "view_id")?,
            track_id: required_string(value, "track_id")?,
            station_s_m: required_f64s(value, "station_s_m")?,
            centerline_xy_m: required_points(value, "centerline_xy_m")?,
            left_boundary_xy_m: required_points(value, "left_boundary_xy_m")?,
            right_boundary_xy_m: required_points(value, "right_boundary_xy_m")?,
            normals_xy: required_points(value, "normals_xy")?,
            width_left_m: required_f64s(value, "width_left_m")?,
            width_right_m: required_f64s(value, "width_right_m")?,
            section_dirs_xy: required_points(value, "section_dirs_xy")?,
            quality_metrics: optional_object(value, "quality_metrics"),
            metadata: optional_object(value, "metadata"),
        })
    }
}

fn fnv1a_append(hash: &mut u32, bytes: &[u8]) {
    for byte in bytes {
        *hash ^= u32::from(*byte);
        *hash = hash.wrapping_mul(16_777_619);
    }
}

fn hash_append_u32(hash: &mut u32, value: usize) {
    fnv1a_append(hash, &(value as u32).to_le_bytes());
}

fn hash_append_string(hash: &mut u32, value: &str) {
    hash_append_u32(hash, value.len());
    fnv1a_append(hash, value.as_bytes());
}

fn hash_append_f64(hash: &mut u32, value: f64) {
    let normalized = if value == 0.0 { 0.0 } else { value };
    fnv1a_append(hash, &normalized.to_le_bytes());
}

fn hash_append_scalars(hash: &mut u32, values: &[f64]) {
    hash_append_u32(hash, values.len());
    for value in values {
        hash_append_f64(hash, *value);
    }
}

fn hash_append_points(hash: &mut u32, values: &[Point2]) {
    hash_append_u32(hash, values.len());
    for [x, y] in values {
        hash_append_f64(hash, *x);
        hash_append_f64(hash, *y);
    }
}

fn bytes_append_u32(target: &mut Vec<u8>, value: usize) {
    target.extend_from_slice(&(value as u32).to_le_bytes());
}

fn bytes_append_string(target: &mut Vec<u8>, value: &str) {
    bytes_append_u32(target, value.len());
    target.extend_from_slice(value.as_bytes());
}

fn bytes_append_f64(target: &mut Vec<u8>, value: f64) {
    let normalized = if value == 0.0 { 0.0 } else { value };
    target.extend_from_slice(&normalized.to_le_bytes());
}

fn bytes_append_scalars(target: &mut Vec<u8>, values: &[f64]) {
    bytes_append_u32(target, values.len());
    for value in values {
        bytes_append_f64(target, *value);
    }
}

fn bytes_append_points(target: &mut Vec<u8>, values: &[Point2]) {
    bytes_append_u32(target, values.len());
    for [x, y] in values {
        bytes_append_f64(target, *x);
        bytes_append_f64(target, *y);
    }
}

fn bytes_append_optional_start_finish(target: &mut Vec<u8>, value: Option<&StartFinish>) {
    bytes_append_u32(target, usize::from(value.is_some()));
    if let Some(value) = value {
        bytes_append_f64(target, value.p1_m[0]);
        bytes_append_f64(target, value.p1_m[1]);
        bytes_append_f64(target, value.p2_m[0]);
        bytes_append_f64(target, value.p2_m[1]);
    }
}

pub(crate) fn sha256_hex(input: &[u8]) -> String {
    const INITIAL: [u32; 8] = [
        0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab,
        0x5be0cd19,
    ];
    const K: [u32; 64] = [
        0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4,
        0xab1c5ed5, 0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe,
        0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f,
        0x4a7484aa, 0x5cb0a9dc, 0x76f988da, 0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
        0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc,
        0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b,
        0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070, 0x19a4c116,
        0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
        0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7,
        0xc67178f2,
    ];
    let bit_len = (input.len() as u64) * 8;
    let mut padded = input.to_vec();
    padded.push(0x80);
    while padded.len() % 64 != 56 {
        padded.push(0);
    }
    padded.extend_from_slice(&bit_len.to_be_bytes());
    let mut hash = INITIAL;
    for chunk in padded.chunks_exact(64) {
        let mut words = [0_u32; 64];
        for (index, word) in words.iter_mut().take(16).enumerate() {
            *word = u32::from_be_bytes(chunk[index * 4..index * 4 + 4].try_into().unwrap());
        }
        for index in 16..64 {
            let x = words[index - 15];
            let y = words[index - 2];
            let s0 = x.rotate_right(7) ^ x.rotate_right(18) ^ (x >> 3);
            let s1 = y.rotate_right(17) ^ y.rotate_right(19) ^ (y >> 10);
            words[index] = words[index - 16]
                .wrapping_add(s0)
                .wrapping_add(words[index - 7])
                .wrapping_add(s1);
        }
        let [mut a, mut b, mut c, mut d, mut e, mut f, mut g, mut h] = hash;
        for index in 0..64 {
            let sum1 = e.rotate_right(6) ^ e.rotate_right(11) ^ e.rotate_right(25);
            let choice = (e & f) ^ (!e & g);
            let temp1 = h
                .wrapping_add(sum1)
                .wrapping_add(choice)
                .wrapping_add(K[index])
                .wrapping_add(words[index]);
            let sum0 = a.rotate_right(2) ^ a.rotate_right(13) ^ a.rotate_right(22);
            let majority = (a & b) ^ (a & c) ^ (b & c);
            let temp2 = sum0.wrapping_add(majority);
            h = g;
            g = f;
            f = e;
            e = d.wrapping_add(temp1);
            d = c;
            c = b;
            b = a;
            a = temp1.wrapping_add(temp2);
        }
        for (slot, value) in hash.iter_mut().zip([a, b, c, d, e, f, g, h]) {
            *slot = slot.wrapping_add(value);
        }
    }
    let mut encoded = String::with_capacity(hash.len() * 8);
    for value in hash {
        write!(&mut encoded, "{value:08x}").expect("writing into a String cannot fail");
    }
    encoded
}

fn canonical_json(value: &JsonValue) -> String {
    match value {
        JsonValue::Null | JsonValue::Bool(_) | JsonValue::Integer(_) | JsonValue::Number(_) => {
            value.to_pretty_string()
        }
        JsonValue::String(_) => value.to_pretty_string(),
        JsonValue::Array(values) => format!(
            "[{}]",
            values
                .iter()
                .map(canonical_json)
                .collect::<Vec<_>>()
                .join(",")
        ),
        JsonValue::Object(entries) => {
            let mut entries = entries.clone();
            entries.sort_by(|left, right| left.0.cmp(&right.0));
            format!(
                "{{{}}}",
                entries
                    .iter()
                    .map(|(key, value)| format!(
                        "{}:{}",
                        JsonValue::String(key.clone()).to_pretty_string(),
                        canonical_json(value)
                    ))
                    .collect::<Vec<_>>()
                    .join(",")
            )
        }
    }
}

#[must_use]
pub fn station_options_hash_v2(options: &JsonValue) -> String {
    sha256_hex(format!("RLC:station-options:v2\0{}", canonical_json(options)).as_bytes())
}

#[cfg(test)]
mod identity_hash_tests {
    use super::{sha256_hex, station_options_hash_v2};
    use crate::parse_json_str;

    #[test]
    fn sha256_matches_standard_vector() {
        assert_eq!(
            sha256_hex(b"abc"),
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"
        );
    }

    #[test]
    fn station_options_hash_matches_typescript_unicode_vector() {
        let options = parse_json_str(r#"{"unicode":"Трасса 🏁","array":[0,-0,1.5,null]}"#).unwrap();
        assert_eq!(
            station_options_hash_v2(&options),
            "6bfb159901dbeedc7890414e3ac663342a4eab391a76b99012803f8a06391f17"
        );
    }
}

#[must_use]
pub fn station_generation_request_key_v3(
    source_ref: &StationSourceRefV1,
    count_mode: &str,
    requested_station_count: Option<usize>,
    direction: &str,
    station_options_hash: &str,
    generator_contract: &str,
    generator_version: &str,
    validation_contract: &str,
    validation_version: &str,
) -> String {
    let value = JsonValue::Object(vec![
        (
            "source_ref".into(),
            JsonValue::Object(vec![
                ("schema_version".into(), "station_source_ref.v1".into()),
                ("project_id".into(), source_ref.project_id.clone().into()),
                ("geometry_id".into(), source_ref.geometry_id.clone().into()),
                (
                    "geometry_content_hash".into(),
                    source_ref.geometry_content_hash.clone().into(),
                ),
                ("route_id".into(), source_ref.route_id.clone().into()),
            ]),
        ),
        ("requested_count_mode".into(), count_mode.into()),
        (
            "requested_station_count".into(),
            requested_station_count
                .map_or(JsonValue::Null, |value| JsonValue::Integer(value as i64)),
        ),
        ("direction".into(), direction.into()),
        ("station_options_hash".into(), station_options_hash.into()),
        ("generator_contract".into(), generator_contract.into()),
        ("generator_version".into(), generator_version.into()),
        ("validation_contract".into(), validation_contract.into()),
        ("validation_version".into(), validation_version.into()),
    ]);
    sha256_hex(format!("RLC:station-request:v3\0{}", canonical_json(&value)).as_bytes())
}

/// Stable TypeScript/Rust identity for computational station geometry only.
/// Metadata and quality metrics are deliberately excluded from the solve input hash.
#[must_use]
pub fn sections_track_view_hash_v1(sections: &SectionsTrackViewV1) -> String {
    let mut hash = 2_166_136_261;
    hash_append_string(&mut hash, SECTIONS_TRACK_VIEW_HASH_V1);
    hash_append_string(&mut hash, &sections.schema_version);
    hash_append_string(&mut hash, &sections.view_id);
    hash_append_string(&mut hash, &sections.track_id);
    hash_append_scalars(&mut hash, &sections.station_s_m);
    hash_append_points(&mut hash, &sections.centerline_xy_m);
    hash_append_points(&mut hash, &sections.left_boundary_xy_m);
    hash_append_points(&mut hash, &sections.right_boundary_xy_m);
    hash_append_points(&mut hash, &sections.normals_xy);
    hash_append_scalars(&mut hash, &sections.width_left_m);
    hash_append_scalars(&mut hash, &sections.width_right_m);
    hash_append_points(&mut hash, &sections.section_dirs_xy);
    format!("fnv1a_{hash:08x}")
}

#[must_use]
pub fn sections_track_view_hash_v2(sections: &SectionsTrackViewV1) -> String {
    let mut bytes = Vec::new();
    bytes_append_string(&mut bytes, "RLC:sections:v2");
    bytes_append_string(&mut bytes, &sections.schema_version);
    bytes_append_string(&mut bytes, &sections.view_id);
    bytes_append_string(&mut bytes, &sections.track_id);
    bytes_append_scalars(&mut bytes, &sections.station_s_m);
    bytes_append_points(&mut bytes, &sections.centerline_xy_m);
    bytes_append_points(&mut bytes, &sections.left_boundary_xy_m);
    bytes_append_points(&mut bytes, &sections.right_boundary_xy_m);
    bytes_append_points(&mut bytes, &sections.normals_xy);
    bytes_append_scalars(&mut bytes, &sections.width_left_m);
    bytes_append_scalars(&mut bytes, &sections.width_right_m);
    bytes_append_points(&mut bytes, &sections.section_dirs_xy);
    sha256_hex(&bytes)
}

pub const PREPARED_STATION_BUNDLE_HASH_V1: &str = "prepared_station_bundle_hash.v1";

fn hash_append_optional_start_finish(hash: &mut u32, value: Option<&StartFinish>) {
    hash_append_u32(hash, usize::from(value.is_some()));
    if let Some(value) = value {
        hash_append_f64(hash, value.p1_m[0]);
        hash_append_f64(hash, value.p1_m[1]);
        hash_append_f64(hash, value.p2_m[0]);
        hash_append_f64(hash, value.p2_m[1]);
    }
}

/// Stable identity for route semantics and the complete computational station frame.
#[must_use]
pub fn prepared_station_bundle_hash_v1(
    track_id: &str,
    units: &str,
    trajectory_mode: &str,
    direction: Option<&str>,
    start_finish_xy_m: Option<&StartFinish>,
    finish_line_xy_m: Option<&StartFinish>,
    sections: &SectionsTrackViewV1,
) -> String {
    let mut hash = 2_166_136_261;
    hash_append_string(&mut hash, PREPARED_STATION_BUNDLE_HASH_V1);
    hash_append_string(&mut hash, "prepared_route_identity.v1");
    hash_append_string(&mut hash, track_id);
    hash_append_string(&mut hash, units);
    hash_append_string(&mut hash, trajectory_mode);
    hash_append_u32(&mut hash, usize::from(direction.is_some()));
    if let Some(direction) = direction {
        hash_append_string(&mut hash, direction);
    }
    hash_append_optional_start_finish(&mut hash, start_finish_xy_m);
    hash_append_optional_start_finish(&mut hash, finish_line_xy_m);
    hash_append_string(&mut hash, &sections_track_view_hash_v1(sections));
    format!("fnv1a_{hash:08x}")
}

#[must_use]
pub fn station_geometry_content_hash_v1(area: &TrackAreaContractV1) -> String {
    let mut hash = 2_166_136_261;
    hash_append_string(&mut hash, STATION_GEOMETRY_CONTENT_HASH_V1);
    hash_append_string(&mut hash, &area.units);
    hash_append_string(&mut hash, area.direction.as_deref().unwrap_or(""));
    hash_append_string(&mut hash, &area.trajectory_mode);
    hash_append_points(&mut hash, &area.left_boundary_xy_m);
    hash_append_points(&mut hash, &area.right_boundary_xy_m);
    hash_append_optional_start_finish(&mut hash, area.start_finish_xy_m.as_ref());
    hash_append_optional_start_finish(&mut hash, area.finish_line_xy_m.as_ref());
    format!("fnv1a_{hash:08x}")
}

#[must_use]
pub fn station_geometry_content_hash_v2(area: &TrackAreaContractV1) -> String {
    let mut bytes = Vec::new();
    bytes_append_string(&mut bytes, "RLC:geometry:v2");
    bytes_append_string(&mut bytes, &area.units);
    bytes_append_string(&mut bytes, &area.trajectory_mode);
    bytes_append_points(&mut bytes, &area.left_boundary_xy_m);
    bytes_append_points(&mut bytes, &area.right_boundary_xy_m);
    bytes_append_optional_start_finish(&mut bytes, area.start_finish_xy_m.as_ref());
    bytes_append_optional_start_finish(&mut bytes, area.finish_line_xy_m.as_ref());
    sha256_hex(&bytes)
}

#[must_use]
pub fn prepared_station_bundle_hash_v2(
    source_ref: &StationSourceRefV1,
    recipe: &StationRecipeV1,
    units: &str,
    trajectory_mode: &str,
    direction: Option<&str>,
    start_finish_xy_m: Option<&StartFinish>,
    finish_line_xy_m: Option<&StartFinish>,
    sections_hash: &str,
) -> String {
    let mut hash = 2_166_136_261;
    hash_append_string(&mut hash, PREPARED_STATION_BUNDLE_HASH_V2);
    hash_append_string(&mut hash, "station_source_ref.v1");
    hash_append_string(&mut hash, &source_ref.project_id);
    hash_append_string(&mut hash, &source_ref.geometry_id);
    hash_append_string(&mut hash, &source_ref.geometry_content_hash);
    hash_append_string(&mut hash, &source_ref.route_id);
    hash_append_string(&mut hash, "station_recipe.v1");
    hash_append_string(&mut hash, &recipe.direction);
    hash_append_string(&mut hash, &recipe.station_options_hash);
    hash_append_u32(&mut hash, recipe.resolved_station_count);
    hash_append_string(&mut hash, &recipe.generator_contract);
    hash_append_string(&mut hash, &recipe.generator_version);
    hash_append_string(&mut hash, &recipe.validation_contract);
    hash_append_string(&mut hash, &recipe.validation_version);
    hash_append_string(&mut hash, "prepared_route_identity.v1");
    hash_append_string(&mut hash, &source_ref.route_id);
    hash_append_string(&mut hash, units);
    hash_append_string(&mut hash, trajectory_mode);
    hash_append_u32(&mut hash, usize::from(direction.is_some()));
    if let Some(direction) = direction {
        hash_append_string(&mut hash, direction);
    }
    hash_append_optional_start_finish(&mut hash, start_finish_xy_m);
    hash_append_optional_start_finish(&mut hash, finish_line_xy_m);
    hash_append_string(&mut hash, sections_hash);
    hash_append_string(&mut hash, &recipe.validation_contract);
    hash_append_string(&mut hash, &recipe.validation_version);
    hash_append_string(&mut hash, "passed");
    format!("fnv1a_{hash:08x}")
}

#[must_use]
pub fn prepared_station_bundle_hash_v3(
    source_ref: &StationSourceRefV1,
    recipe: &StationRecipeV1,
    units: &str,
    trajectory_mode: &str,
    direction: Option<&str>,
    start_finish_xy_m: Option<&StartFinish>,
    finish_line_xy_m: Option<&StartFinish>,
    sections_hash: &str,
) -> String {
    let mut bytes = Vec::new();
    bytes_append_string(&mut bytes, "RLC:prepared-bundle:v3");
    bytes_append_string(&mut bytes, "station_source_ref.v1");
    bytes_append_string(&mut bytes, &source_ref.project_id);
    bytes_append_string(&mut bytes, &source_ref.geometry_id);
    bytes_append_string(&mut bytes, &source_ref.geometry_content_hash);
    bytes_append_string(&mut bytes, &source_ref.route_id);
    bytes_append_string(&mut bytes, "station_recipe.v1");
    bytes_append_string(&mut bytes, &recipe.direction);
    bytes_append_string(&mut bytes, &recipe.station_options_hash);
    bytes_append_u32(&mut bytes, recipe.resolved_station_count);
    bytes_append_string(&mut bytes, &recipe.generator_contract);
    bytes_append_string(&mut bytes, &recipe.generator_version);
    bytes_append_string(&mut bytes, &recipe.validation_contract);
    bytes_append_string(&mut bytes, &recipe.validation_version);
    bytes_append_string(&mut bytes, "prepared_route_identity.v1");
    bytes_append_string(&mut bytes, &source_ref.route_id);
    bytes_append_string(&mut bytes, units);
    bytes_append_string(&mut bytes, trajectory_mode);
    bytes_append_u32(&mut bytes, usize::from(direction.is_some()));
    if let Some(direction) = direction {
        bytes_append_string(&mut bytes, direction);
    }
    bytes_append_optional_start_finish(&mut bytes, start_finish_xy_m);
    bytes_append_optional_start_finish(&mut bytes, finish_line_xy_m);
    bytes_append_string(&mut bytes, sections_hash);
    bytes_append_string(&mut bytes, &recipe.validation_contract);
    bytes_append_string(&mut bytes, &recipe.validation_version);
    bytes_append_string(&mut bytes, "passed");
    sha256_hex(&bytes)
}

impl ToJsonValue for SectionsTrackViewV1 {
    fn to_json_value(&self) -> JsonValue {
        JsonValue::Object(vec![
            (
                "schema_version".to_owned(),
                self.schema_version.clone().into(),
            ),
            ("view_id".to_owned(), self.view_id.clone().into()),
            ("track_id".to_owned(), self.track_id.clone().into()),
            ("station_s_m".to_owned(), f64s_to_json(&self.station_s_m)),
            (
                "centerline_xy_m".to_owned(),
                points_to_json(&self.centerline_xy_m),
            ),
            (
                "left_boundary_xy_m".to_owned(),
                points_to_json(&self.left_boundary_xy_m),
            ),
            (
                "right_boundary_xy_m".to_owned(),
                points_to_json(&self.right_boundary_xy_m),
            ),
            ("normals_xy".to_owned(), points_to_json(&self.normals_xy)),
            ("width_left_m".to_owned(), f64s_to_json(&self.width_left_m)),
            (
                "width_right_m".to_owned(),
                f64s_to_json(&self.width_right_m),
            ),
            (
                "section_dirs_xy".to_owned(),
                points_to_json(&self.section_dirs_xy),
            ),
            (
                "quality_metrics".to_owned(),
                JsonValue::Object(self.quality_metrics.clone()),
            ),
            (
                "metadata".to_owned(),
                JsonValue::Object(self.metadata.clone()),
            ),
        ])
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct AccelerationEnvelopeV1 {
    pub schema_version: String,
    pub envelope_id: String,
    pub speed_mps: Vec<f64>,
    pub ax_drive_max_mps2: Vec<f64>,
    pub ax_brake_max_mps2: Vec<f64>,
    pub ay_left_max_mps2: Vec<f64>,
    pub ay_right_max_mps2: Vec<f64>,
    pub coupling_exponent: f64,
    pub metadata: JsonObject,
}

impl AccelerationEnvelopeV1 {
    pub const SCHEMA_VERSION: &'static str = "AccelerationEnvelopeV1";

    pub fn from_json(value: &JsonValue) -> Result<Self, String> {
        Ok(Self {
            schema_version: required_string(value, "schema_version")?,
            envelope_id: required_string(value, "envelope_id")?,
            speed_mps: required_f64s(value, "speed_mps")?,
            ax_drive_max_mps2: required_f64s(value, "ax_drive_max_mps2")?,
            ax_brake_max_mps2: required_f64s(value, "ax_brake_max_mps2")?,
            ay_left_max_mps2: required_f64s(value, "ay_left_max_mps2")?,
            ay_right_max_mps2: required_f64s(value, "ay_right_max_mps2")?,
            coupling_exponent: required_f64(value, "coupling_exponent")?,
            metadata: optional_object(value, "metadata"),
        })
    }

    #[must_use]
    pub fn limits(&self, speed_mps: f64) -> EnvelopeLimits {
        EnvelopeLimits {
            ax_drive_max_mps2: interp(&self.speed_mps, &self.ax_drive_max_mps2, speed_mps),
            ax_brake_max_mps2: interp(&self.speed_mps, &self.ax_brake_max_mps2, speed_mps),
            ay_left_max_mps2: interp(&self.speed_mps, &self.ay_left_max_mps2, speed_mps),
            ay_right_max_mps2: interp(&self.speed_mps, &self.ay_right_max_mps2, speed_mps),
        }
    }
}

impl ToJsonValue for AccelerationEnvelopeV1 {
    fn to_json_value(&self) -> JsonValue {
        JsonValue::Object(vec![
            (
                "schema_version".to_owned(),
                self.schema_version.clone().into(),
            ),
            ("envelope_id".to_owned(), self.envelope_id.clone().into()),
            ("speed_mps".to_owned(), f64s_to_json(&self.speed_mps)),
            (
                "ax_drive_max_mps2".to_owned(),
                f64s_to_json(&self.ax_drive_max_mps2),
            ),
            (
                "ax_brake_max_mps2".to_owned(),
                f64s_to_json(&self.ax_brake_max_mps2),
            ),
            (
                "ay_left_max_mps2".to_owned(),
                f64s_to_json(&self.ay_left_max_mps2),
            ),
            (
                "ay_right_max_mps2".to_owned(),
                f64s_to_json(&self.ay_right_max_mps2),
            ),
            (
                "coupling_exponent".to_owned(),
                self.coupling_exponent.into(),
            ),
            (
                "metadata".to_owned(),
                JsonValue::Object(self.metadata.clone()),
            ),
        ])
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EnvelopeLimits {
    pub ax_drive_max_mps2: f64,
    pub ax_brake_max_mps2: f64,
    pub ay_left_max_mps2: f64,
    pub ay_right_max_mps2: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct DynamicsProfileV1 {
    pub schema_version: String,
    pub profile_id: String,
    pub model_kind: String,
    pub params: JsonObject,
    pub metadata: JsonObject,
}

impl DynamicsProfileV1 {
    pub const SCHEMA_VERSION: &'static str = "DynamicsProfileV1";
}

#[derive(Clone, Debug, PartialEq)]
pub struct PointMassProfileV1 {
    pub schema_version: String,
    pub profile_id: String,
    pub model_kind: String,
    pub params: JsonObject,
    pub metadata: JsonObject,
}

impl PointMassProfileV1 {
    pub const SCHEMA_VERSION: &'static str = "PointMassProfileV1";
    pub const MODEL_KIND: &'static str = "point_mass_envelope";

    pub fn from_json(value: &JsonValue) -> Result<Self, String> {
        Ok(Self {
            schema_version: required_string(value, "schema_version")?,
            profile_id: required_string(value, "profile_id")?,
            model_kind: required_string(value, "model_kind")?,
            params: optional_object(value, "params"),
            metadata: optional_object(value, "metadata"),
        })
    }

    #[must_use]
    pub fn to_acceleration_envelope(&self, g_mps2: f64) -> Option<AccelerationEnvelopeV1> {
        let v_max_mps = object_f64(&self.params, "v_max_mps")?;
        let ax_forward_max_g = object_f64(&self.params, "ax_forward_max_g")?;
        let ax_brake_max_g = object_f64(&self.params, "ax_brake_max_g")?;
        let ay_left_max_g = object_f64(&self.params, "ay_left_max_g")?;
        let ay_right_max_g = object_f64(&self.params, "ay_right_max_g")?;
        let coupling_exponent = object_f64(&self.params, "coupling_exponent").unwrap_or(2.0);

        Some(AccelerationEnvelopeV1 {
            schema_version: AccelerationEnvelopeV1::SCHEMA_VERSION.to_owned(),
            envelope_id: format!("{}_acceleration_envelope", self.profile_id),
            speed_mps: vec![0.0, v_max_mps],
            ax_drive_max_mps2: vec![ax_forward_max_g * g_mps2; 2],
            ax_brake_max_mps2: vec![ax_brake_max_g * g_mps2; 2],
            ay_left_max_mps2: vec![ay_left_max_g * g_mps2; 2],
            ay_right_max_mps2: vec![ay_right_max_g * g_mps2; 2],
            coupling_exponent,
            metadata: vec![
                (
                    "source_profile_id".to_owned(),
                    self.profile_id.clone().into(),
                ),
                (
                    "source_profile_kind".to_owned(),
                    self.model_kind.clone().into(),
                ),
            ],
        })
    }
}

impl ToJsonValue for PointMassProfileV1 {
    fn to_json_value(&self) -> JsonValue {
        JsonValue::Object(vec![
            (
                "schema_version".to_owned(),
                self.schema_version.clone().into(),
            ),
            ("profile_id".to_owned(), self.profile_id.clone().into()),
            ("model_kind".to_owned(), self.model_kind.clone().into()),
            ("params".to_owned(), JsonValue::Object(self.params.clone())),
            (
                "metadata".to_owned(),
                JsonValue::Object(self.metadata.clone()),
            ),
        ])
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct VehicleProfileV1 {
    pub schema_version: String,
    pub vehicle_id: String,
    pub mass_kg: f64,
    pub length_m: f64,
    pub width_m: f64,
    pub v_max_mps: f64,
    pub envelope_id: String,
    pub native_params: JsonObject,
}

impl VehicleProfileV1 {
    pub const SCHEMA_VERSION: &'static str = "VehicleProfileV1";
}

#[derive(Clone, Debug, PartialEq)]
pub struct SolveRequestV1 {
    pub schema_version: String,
    pub request_id: String,
    pub solver_id: String,
    pub track_id: String,
    pub track_view_id: String,
    pub vehicle_id: String,
    pub target_station_count: u32,
    pub objective: String,
    pub output_prefix: String,
    pub options: JsonObject,
    pub dynamics_profile_id: Option<String>,
}

impl SolveRequestV1 {
    pub const SCHEMA_VERSION: &'static str = "SolveRequestV1";
}

#[derive(Clone, Debug, PartialEq)]
pub struct TrajectoryResultSummaryV1 {
    pub schema_version: String,
    pub result_id: String,
    pub request_id: String,
    pub solver_id: String,
    pub track_id: String,
    pub status: String,
    pub lap_time_s: f64,
    pub utilization_kind: String,
    pub dynamics_profile_id: Option<String>,
    pub model_kind: Option<String>,
    pub native_utilization: JsonObject,
    pub diagnostics: JsonObject,
    pub artifacts: JsonObject,
}

impl TrajectoryResultSummaryV1 {
    pub const SCHEMA_VERSION: &'static str = "TrajectoryResultV1";
}

#[derive(Clone, Debug, PartialEq)]
pub struct TrajectoryResultSeriesV1 {
    pub s_m: Vec<f64>,
    pub x_m: Vec<f64>,
    pub y_m: Vec<f64>,
    pub heading_rad: Vec<f64>,
    pub kappa_1pm: Vec<f64>,
    pub v_mps: Vec<f64>,
    pub ax_mps2: Vec<f64>,
    pub ay_mps2: Vec<f64>,
    pub utilization_cornering: Vec<f64>,
    pub utilization_longitudinal: Vec<f64>,
    pub utilization_combined: Vec<f64>,
    pub station_index: Option<Vec<i64>>,
}

impl TrajectoryResultSeriesV1 {
    pub fn from_columns(
        column_names: &[String],
        rows: &[Vec<f64>],
    ) -> Result<TrajectoryResultSeriesV1, String> {
        Ok(Self {
            s_m: required_column(column_names, rows, "s_m")?,
            x_m: required_column(column_names, rows, "x_m")?,
            y_m: required_column(column_names, rows, "y_m")?,
            heading_rad: required_column(column_names, rows, "heading_rad")?,
            kappa_1pm: required_column(column_names, rows, "kappa_1pm")?,
            v_mps: required_column(column_names, rows, "v_mps")?,
            ax_mps2: required_column(column_names, rows, "ax_mps2")?,
            ay_mps2: required_column(column_names, rows, "ay_mps2")?,
            utilization_cornering: required_column(column_names, rows, "utilization_cornering")?,
            utilization_longitudinal: required_column(
                column_names,
                rows,
                "utilization_longitudinal",
            )?,
            utilization_combined: required_column(column_names, rows, "utilization_combined")?,
            station_index: optional_column(column_names, rows, "station_index").map(|values| {
                values
                    .into_iter()
                    .map(|value| value.round() as i64)
                    .collect()
            }),
        })
    }
}

impl ToJsonValue for TrajectoryResultSeriesV1 {
    fn to_json_value(&self) -> JsonValue {
        JsonValue::Object(vec![
            (
                "schema_version".to_owned(),
                "trajectory_result_series.v1".into(),
            ),
            ("s_m".to_owned(), f64s_to_json(&self.s_m)),
            ("x_m".to_owned(), f64s_to_json(&self.x_m)),
            ("y_m".to_owned(), f64s_to_json(&self.y_m)),
            ("heading_rad".to_owned(), f64s_to_json(&self.heading_rad)),
            ("kappa_1pm".to_owned(), f64s_to_json(&self.kappa_1pm)),
            ("v_mps".to_owned(), f64s_to_json(&self.v_mps)),
            ("ax_mps2".to_owned(), f64s_to_json(&self.ax_mps2)),
            ("ay_mps2".to_owned(), f64s_to_json(&self.ay_mps2)),
            (
                "utilization_cornering".to_owned(),
                f64s_to_json(&self.utilization_cornering),
            ),
            (
                "utilization_longitudinal".to_owned(),
                f64s_to_json(&self.utilization_longitudinal),
            ),
            (
                "utilization_combined".to_owned(),
                f64s_to_json(&self.utilization_combined),
            ),
            (
                "station_index".to_owned(),
                self.station_index
                    .as_ref()
                    .map_or(JsonValue::Null, |values| {
                        JsonValue::Array(
                            values
                                .iter()
                                .map(|value| JsonValue::Integer(*value))
                                .collect(),
                        )
                    }),
            ),
        ])
    }
}

fn points_to_json(points: &[Point2]) -> JsonValue {
    JsonValue::Array(
        points
            .iter()
            .map(|point| JsonValue::Array(vec![point[0].into(), point[1].into()]))
            .collect(),
    )
}

fn f64s_to_json(values: &[f64]) -> JsonValue {
    JsonValue::Array(values.iter().copied().map(JsonValue::from).collect())
}

fn option_string_to_json(value: &Option<String>) -> JsonValue {
    value.clone().map_or(JsonValue::Null, JsonValue::String)
}

fn option_u32_to_json(value: Option<u32>) -> JsonValue {
    value.map_or(JsonValue::Null, JsonValue::from)
}

fn option_f64_to_json(value: Option<f64>) -> JsonValue {
    value.map_or(JsonValue::Null, JsonValue::from)
}

fn start_finish_to_json(value: &StartFinish) -> JsonValue {
    JsonValue::Object(vec![
        (
            "p1_m".to_owned(),
            JsonValue::Array(vec![value.p1_m[0].into(), value.p1_m[1].into()]),
        ),
        (
            "p2_m".to_owned(),
            JsonValue::Array(vec![value.p2_m[0].into(), value.p2_m[1].into()]),
        ),
    ])
}

pub(crate) fn option_start_finish_to_json(value: &Option<StartFinish>) -> JsonValue {
    value.as_ref().map_or(JsonValue::Null, start_finish_to_json)
}

fn object_f64(object: &JsonObject, key: &str) -> Option<f64> {
    object
        .iter()
        .find(|(entry_key, _)| entry_key == key)
        .and_then(|(_, value)| value.as_f64())
}

fn optional_object(value: &JsonValue, key: &str) -> JsonObject {
    match value.get(key) {
        Some(JsonValue::Object(entries)) => entries.clone(),
        _ => Vec::new(),
    }
}

fn required_f64(value: &JsonValue, key: &str) -> Result<f64, String> {
    value
        .get(key)
        .and_then(JsonValue::as_f64)
        .ok_or_else(|| format!("missing number field: {key}"))
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

pub(crate) fn optional_start_finish(
    value: &JsonValue,
    key: &str,
) -> Result<Option<StartFinish>, String> {
    let Some(row) = value.get(key) else {
        return Ok(None);
    };

    if matches!(row, JsonValue::Null) {
        return Ok(None);
    }

    Ok(Some(StartFinish {
        p1_m: start_finish_point(row, "p1_m", "a_m", key)?,
        p2_m: start_finish_point(row, "p2_m", "b_m", key)?,
    }))
}

fn start_finish_point(
    value: &JsonValue,
    primary_key: &str,
    legacy_key: &str,
    context_key: &str,
) -> Result<Point2, String> {
    value
        .get(primary_key)
        .or_else(|| value.get(legacy_key))
        .ok_or_else(|| format!("missing {primary_key} in {context_key}"))
        .and_then(|point| point_from_json(point, context_key))
}

fn point_from_json(value: &JsonValue, context_key: &str) -> Result<Point2, String> {
    let values = value
        .as_array()
        .ok_or_else(|| format!("invalid point row in {context_key}"))?;
    if values.len() != 2 {
        return Err(format!("invalid point width in {context_key}"));
    }
    Ok([
        values[0]
            .as_f64()
            .ok_or_else(|| format!("invalid x coordinate in {context_key}"))?,
        values[1]
            .as_f64()
            .ok_or_else(|| format!("invalid y coordinate in {context_key}"))?,
    ])
}

fn required_points(value: &JsonValue, key: &str) -> Result<Vec<Point2>, String> {
    let rows = value
        .get(key)
        .and_then(JsonValue::as_array)
        .ok_or_else(|| format!("missing point array field: {key}"))?;
    rows.iter().map(|row| point_from_json(row, key)).collect()
}

fn required_f64s(value: &JsonValue, key: &str) -> Result<Vec<f64>, String> {
    value
        .get(key)
        .and_then(JsonValue::as_array)
        .ok_or_else(|| format!("missing number array field: {key}"))?
        .iter()
        .map(|entry| {
            entry
                .as_f64()
                .ok_or_else(|| format!("invalid number in {key}"))
        })
        .collect()
}

fn required_column(
    column_names: &[String],
    rows: &[Vec<f64>],
    column_name: &str,
) -> Result<Vec<f64>, String> {
    optional_column(column_names, rows, column_name)
        .ok_or_else(|| format!("missing CSV column: {column_name}"))
}

fn optional_column(
    column_names: &[String],
    rows: &[Vec<f64>],
    column_name: &str,
) -> Option<Vec<f64>> {
    let index = column_names.iter().position(|name| name == column_name)?;
    Some(
        rows.iter()
            .filter_map(|row| row.get(index).copied())
            .collect(),
    )
}

fn interp(xs: &[f64], ys: &[f64], query: f64) -> f64 {
    if xs.is_empty() || ys.is_empty() {
        return f64::NAN;
    }
    let last = xs.len().min(ys.len()) - 1;
    if query <= xs[0] {
        return ys[0];
    }
    if query >= xs[last] {
        return ys[last];
    }
    for idx in 0..last {
        let x0 = xs[idx];
        let x1 = xs[idx + 1];
        if query >= x0 && query <= x1 {
            let span = (x1 - x0).max(f64::EPSILON);
            let t = (query - x0) / span;
            return ys[idx] * (1.0 - t) + ys[idx + 1] * t;
        }
    }
    ys[last]
}

#[cfg(test)]
mod tests {
    use super::{
        interp, sections_track_view_hash_v1, PointMassProfileV1, SectionsTrackViewV1, StartFinish,
        TrackAreaContractV1,
    };
    use crate::json::parse_json_str;
    use crate::ToJsonValue;

    #[test]
    fn interpolation_matches_numpy_style_edges() {
        let xs = [0.0, 10.0, 20.0];
        let ys = [0.0, 100.0, 200.0];

        assert_eq!(interp(&xs, &ys, -1.0), 0.0);
        assert_eq!(interp(&xs, &ys, 25.0), 200.0);
        assert_eq!(interp(&xs, &ys, 5.0), 50.0);
    }

    #[test]
    fn point_mass_profile_resolves_scalar_acceleration_envelope() {
        let profile = PointMassProfileV1 {
            schema_version: PointMassProfileV1::SCHEMA_VERSION.to_owned(),
            profile_id: "point_test".to_owned(),
            model_kind: PointMassProfileV1::MODEL_KIND.to_owned(),
            params: vec![
                ("v_max_mps".to_owned(), 50.0.into()),
                ("ax_forward_max_g".to_owned(), 0.5.into()),
                ("ax_brake_max_g".to_owned(), 1.0.into()),
                ("ay_left_max_g".to_owned(), 1.5.into()),
                ("ay_right_max_g".to_owned(), 1.25.into()),
                ("coupling_exponent".to_owned(), 2.0.into()),
            ],
            metadata: Vec::new(),
        };

        let envelope = profile.to_acceleration_envelope(9.81).unwrap();

        assert_eq!(envelope.envelope_id, "point_test_acceleration_envelope");
        assert_eq!(envelope.ax_drive_max_mps2, vec![4.905, 4.905]);
        assert_eq!(envelope.ay_right_max_mps2, vec![12.262500000000001; 2]);
    }

    #[test]
    fn track_area_contract_preserves_open_trajectory_fields() {
        let value = parse_json_str(
            r#"{
              "schema_version": "TrackAreaContractV1",
              "track_id": "open_test",
              "units": "m",
              "left_boundary_xy_m": [[0, 0], [10, 0]],
              "right_boundary_xy_m": [[0, 2], [10, 2]],
              "start_finish_xy_m": {"p1_m": [0, 0], "p2_m": [0, 2]},
              "finish_line_xy_m": {"p1_m": [10, 0], "p2_m": [10, 2]},
              "trajectory_mode": "open",
              "direction": "clockwise",
              "metadata": {}
            }"#,
        )
        .unwrap();

        let contract = TrackAreaContractV1::from_json(&value).unwrap();

        assert_eq!(contract.trajectory_mode, "open");
        assert_eq!(contract.direction.as_deref(), Some("clockwise"));
        assert_eq!(
            contract.start_finish_xy_m,
            Some(StartFinish {
                p1_m: [0.0, 0.0],
                p2_m: [0.0, 2.0],
            })
        );
        assert_eq!(
            contract.finish_line_xy_m,
            Some(StartFinish {
                p1_m: [10.0, 0.0],
                p2_m: [10.0, 2.0],
            })
        );
        assert!(contract.to_json_value().get("finish_line_xy_m").is_some());
    }

    #[test]
    fn sections_track_view_hash_matches_cross_runtime_fixture() {
        let fixture = parse_json_str(include_str!(
            "../tests/public-fixtures/sections-track-view-hash-v1.json"
        ))
        .unwrap();
        let expected = fixture
            .get("expected_hash")
            .and_then(crate::json::JsonValue::as_str)
            .unwrap();
        let sections =
            SectionsTrackViewV1::from_json(fixture.get("sections_track_view").unwrap()).unwrap();

        assert_eq!(sections_track_view_hash_v1(&sections), expected);
    }
}
