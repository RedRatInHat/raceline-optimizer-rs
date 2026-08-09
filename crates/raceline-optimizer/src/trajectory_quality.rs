use crate::json::JsonValue;

const QUALITY_SCHEMA_VERSION: &str = "unified_trajectory_quality.v1";

const AY_JUMP_WEIGHT_S_PER_MPS2: f64 = 1.30;
const AX_JUMP_WEIGHT_S_PER_MPS2: f64 = 0.15;
const LATERAL_SLOPE_WEIGHT_S: f64 = 6.00;
const SPEED_TROUGH_WEIGHT_S_PER_MPS: f64 = 0.10;
const KAPPA_JUMP_WEIGHT_S_PER_1PM: f64 = 0.50;

const HARD_MAX_SCALED_VIOLATION: f64 = 1.0e-6;
const HARD_MIN_ABS_SECTION_DET: f64 = 0.05;
const HARD_MIN_FORWARD_PROGRESS: f64 = 0.20;

#[derive(Clone, Debug, PartialEq)]
pub struct UnifiedTrajectoryQuality {
    pub sample_source: &'static str,
    pub sample_count: usize,
    pub lap_time_s: Option<f64>,
    pub unified_quality_penalty_s: f64,
    pub product_score_s: Option<f64>,
    pub hard_gate_clean: bool,
    pub hard_gate_reasons: Vec<String>,
    pub ay_jump_p95_mps2: Option<f64>,
    pub ay_jump_max_mps2: Option<f64>,
    pub ax_jump_p95_mps2: Option<f64>,
    pub ax_jump_max_mps2: Option<f64>,
    pub lateral_slope_p95_abs: Option<f64>,
    pub lateral_slope_max_abs: Option<f64>,
    pub speed_trough_p95_mps: Option<f64>,
    pub speed_trough_max_mps: Option<f64>,
    pub kappa_jump_p95_1pm: Option<f64>,
    pub kappa_jump_max_1pm: Option<f64>,
    pub max_scaled_violation: Option<f64>,
    pub min_section_det_dense: Option<f64>,
    pub min_abs_section_det_dense: Option<f64>,
    pub section_det_reference_sign: Option<f64>,
    pub section_det_sign_flip_count: Option<f64>,
    pub min_forward_progress_dense: Option<f64>,
    pub sigma_clamp_count: Option<f64>,
}

impl UnifiedTrajectoryQuality {
    #[must_use]
    pub fn to_json_value(&self) -> JsonValue {
        JsonValue::Object(vec![
            ("schema_version".to_owned(), QUALITY_SCHEMA_VERSION.into()),
            ("sample_source".to_owned(), self.sample_source.into()),
            (
                "sample_count".to_owned(),
                JsonValue::Integer(self.sample_count as i64),
            ),
            ("lap_time_s".to_owned(), option_number_json(self.lap_time_s)),
            (
                "unified_quality_penalty_s".to_owned(),
                JsonValue::from(self.unified_quality_penalty_s),
            ),
            (
                "product_score_s".to_owned(),
                option_number_json(self.product_score_s),
            ),
            (
                "hard_gate".to_owned(),
                JsonValue::Object(vec![
                    ("clean".to_owned(), JsonValue::Bool(self.hard_gate_clean)),
                    (
                        "reasons".to_owned(),
                        JsonValue::Array(
                            self.hard_gate_reasons
                                .iter()
                                .cloned()
                                .map(JsonValue::from)
                                .collect(),
                        ),
                    ),
                    (
                        "max_scaled_violation".to_owned(),
                        option_number_json(self.max_scaled_violation),
                    ),
                    (
                        "min_section_det_dense".to_owned(),
                        option_number_json(self.min_section_det_dense),
                    ),
                    (
                        "min_abs_section_det_dense".to_owned(),
                        option_number_json(self.min_abs_section_det_dense),
                    ),
                    (
                        "section_det_reference_sign".to_owned(),
                        option_number_json(self.section_det_reference_sign),
                    ),
                    (
                        "section_det_sign_flip_count".to_owned(),
                        option_number_json(self.section_det_sign_flip_count),
                    ),
                    (
                        "min_forward_progress_dense".to_owned(),
                        option_number_json(self.min_forward_progress_dense),
                    ),
                    (
                        "sigma_clamp_count".to_owned(),
                        option_number_json(self.sigma_clamp_count),
                    ),
                ]),
            ),
            (
                "components".to_owned(),
                JsonValue::Object(vec![
                    (
                        "ay_jump_p95_mps2".to_owned(),
                        option_number_json(self.ay_jump_p95_mps2),
                    ),
                    (
                        "ay_jump_max_mps2".to_owned(),
                        option_number_json(self.ay_jump_max_mps2),
                    ),
                    (
                        "ay_jump_penalty_s".to_owned(),
                        option_weighted_json(self.ay_jump_p95_mps2, AY_JUMP_WEIGHT_S_PER_MPS2),
                    ),
                    (
                        "ax_jump_p95_mps2".to_owned(),
                        option_number_json(self.ax_jump_p95_mps2),
                    ),
                    (
                        "ax_jump_max_mps2".to_owned(),
                        option_number_json(self.ax_jump_max_mps2),
                    ),
                    (
                        "ax_jump_penalty_s".to_owned(),
                        option_weighted_json(self.ax_jump_p95_mps2, AX_JUMP_WEIGHT_S_PER_MPS2),
                    ),
                    (
                        "lateral_slope_p95_abs".to_owned(),
                        option_number_json(self.lateral_slope_p95_abs),
                    ),
                    (
                        "lateral_slope_max_abs".to_owned(),
                        option_number_json(self.lateral_slope_max_abs),
                    ),
                    (
                        "lateral_slope_penalty_s".to_owned(),
                        option_weighted_json(self.lateral_slope_p95_abs, LATERAL_SLOPE_WEIGHT_S),
                    ),
                    (
                        "speed_trough_p95_mps".to_owned(),
                        option_number_json(self.speed_trough_p95_mps),
                    ),
                    (
                        "speed_trough_max_mps".to_owned(),
                        option_number_json(self.speed_trough_max_mps),
                    ),
                    (
                        "speed_trough_penalty_s".to_owned(),
                        option_weighted_json(
                            self.speed_trough_p95_mps,
                            SPEED_TROUGH_WEIGHT_S_PER_MPS,
                        ),
                    ),
                    (
                        "kappa_jump_p95_1pm".to_owned(),
                        option_number_json(self.kappa_jump_p95_1pm),
                    ),
                    (
                        "kappa_jump_max_1pm".to_owned(),
                        option_number_json(self.kappa_jump_max_1pm),
                    ),
                    (
                        "kappa_jump_penalty_s".to_owned(),
                        option_weighted_json(self.kappa_jump_p95_1pm, KAPPA_JUMP_WEIGHT_S_PER_1PM),
                    ),
                ]),
            ),
            (
                "weights".to_owned(),
                JsonValue::Object(vec![
                    (
                        "ay_jump_p95_s_per_mps2".to_owned(),
                        JsonValue::from(AY_JUMP_WEIGHT_S_PER_MPS2),
                    ),
                    (
                        "ax_jump_p95_s_per_mps2".to_owned(),
                        JsonValue::from(AX_JUMP_WEIGHT_S_PER_MPS2),
                    ),
                    (
                        "lateral_slope_p95_s".to_owned(),
                        JsonValue::from(LATERAL_SLOPE_WEIGHT_S),
                    ),
                    (
                        "speed_trough_p95_s_per_mps".to_owned(),
                        JsonValue::from(SPEED_TROUGH_WEIGHT_S_PER_MPS),
                    ),
                    (
                        "kappa_jump_p95_s_per_1pm".to_owned(),
                        JsonValue::from(KAPPA_JUMP_WEIGHT_S_PER_1PM),
                    ),
                ]),
            ),
        ])
    }
}

#[must_use]
pub fn with_unified_trajectory_quality(
    mut diagnostics: JsonValue,
    lap_time_s: Option<f64>,
    trajectory_result: &JsonValue,
    trajectory_dense: Option<&JsonValue>,
    closed: bool,
) -> JsonValue {
    let quality = unified_trajectory_quality(
        lap_time_s,
        trajectory_result,
        trajectory_dense,
        Some(&diagnostics),
        closed,
    )
    .to_json_value();

    match &mut diagnostics {
        JsonValue::Object(entries) => {
            entries.retain(|(key, _)| key != "unified_trajectory_quality");
            entries.push(("unified_trajectory_quality".to_owned(), quality));
            diagnostics
        }
        _ => JsonValue::Object(vec![
            ("source_diagnostics".to_owned(), diagnostics),
            ("unified_trajectory_quality".to_owned(), quality),
        ]),
    }
}

#[must_use]
pub fn unified_trajectory_quality(
    lap_time_s: Option<f64>,
    trajectory_result: &JsonValue,
    trajectory_dense: Option<&JsonValue>,
    diagnostics: Option<&JsonValue>,
    closed: bool,
) -> UnifiedTrajectoryQuality {
    let dense_count = trajectory_dense
        .and_then(|value| series_numbers(value, &["v_mps"]))
        .map_or(0, |values| values.len());
    let (sample_source, sample) = if dense_count >= 3 {
        ("trajectory_dense", trajectory_dense.unwrap())
    } else {
        ("trajectory_result", trajectory_result)
    };

    let v_mps = series_numbers(sample, &["v_mps"]).unwrap_or_default();
    let ay_mps2 = series_numbers(sample, &["ay_model_mps2", "ay_mps2"]).unwrap_or_default();
    let ax_mps2 = series_numbers(sample, &["ax_model_mps2", "ax_mps2"]).unwrap_or_default();
    let dn_ds = series_numbers(sample, &["dn_ds"]).unwrap_or_default();
    let kappa_1pm = series_numbers(sample, &["kappa_geo_1pm", "kappa_1pm"]).unwrap_or_default();
    let sample_count = [
        v_mps.len(),
        ay_mps2.len(),
        ax_mps2.len(),
        dn_ds.len(),
        kappa_1pm.len(),
    ]
    .into_iter()
    .max()
    .unwrap_or(0);

    let ay_jumps = adjacent_abs_deltas(&ay_mps2, closed);
    let ax_jumps = adjacent_abs_deltas(&ax_mps2, closed);
    let kappa_jumps = adjacent_abs_deltas(&kappa_1pm, closed);
    let speed_troughs = local_speed_troughs(&v_mps);
    let abs_dn_ds: Vec<f64> = dn_ds.iter().map(|value| value.abs()).collect();

    let ay_jump_p95_mps2 = percentile(&ay_jumps, 0.95);
    let ax_jump_p95_mps2 = percentile(&ax_jumps, 0.95);
    let lateral_slope_p95_abs = percentile(&abs_dn_ds, 0.95);
    let speed_trough_p95_mps = percentile(&speed_troughs, 0.95);
    let kappa_jump_p95_1pm = percentile(&kappa_jumps, 0.95);

    let penalty = weighted_or_zero(ay_jump_p95_mps2, AY_JUMP_WEIGHT_S_PER_MPS2)
        + weighted_or_zero(ax_jump_p95_mps2, AX_JUMP_WEIGHT_S_PER_MPS2)
        + weighted_or_zero(lateral_slope_p95_abs, LATERAL_SLOPE_WEIGHT_S)
        + weighted_or_zero(speed_trough_p95_mps, SPEED_TROUGH_WEIGHT_S_PER_MPS)
        + weighted_or_zero(kappa_jump_p95_1pm, KAPPA_JUMP_WEIGHT_S_PER_1PM);

    let max_scaled_violation = diagnostics
        .and_then(|value| value.get("final_residuals"))
        .and_then(|value| value.get("max_scaled_violation"))
        .and_then(JsonValue::as_f64);
    let geometry = diagnostics.and_then(|value| value.get("geometry_diagnostics"));
    let min_section_det_dense = geometry
        .and_then(|value| value.get("min_section_det_dense"))
        .and_then(JsonValue::as_f64);
    let min_abs_section_det_dense = geometry
        .and_then(|value| value.get("min_abs_section_det_dense"))
        .and_then(JsonValue::as_f64)
        .or_else(|| min_section_det_dense.map(f64::abs));
    let section_det_reference_sign = geometry
        .and_then(|value| value.get("section_det_reference_sign"))
        .and_then(JsonValue::as_f64);
    let section_det_sign_flip_count = geometry
        .and_then(|value| value.get("section_det_sign_flip_count"))
        .and_then(JsonValue::as_f64);
    let min_forward_progress_dense = geometry
        .and_then(|value| value.get("min_forward_progress_dense"))
        .and_then(JsonValue::as_f64);
    let sigma_clamp_count = geometry
        .and_then(|value| value.get("sigma_clamp_count"))
        .and_then(JsonValue::as_f64);

    let mut hard_gate_reasons = Vec::new();
    if let Some(value) = max_scaled_violation {
        if value > HARD_MAX_SCALED_VIOLATION {
            hard_gate_reasons.push(format!(
                "max_scaled_violation {value:.3e} > {HARD_MAX_SCALED_VIOLATION:.3e}"
            ));
        }
    }
    if let Some(value) = min_abs_section_det_dense {
        if value < HARD_MIN_ABS_SECTION_DET {
            hard_gate_reasons.push(format!(
                "min_abs_section_det_dense {value:.6} < {HARD_MIN_ABS_SECTION_DET:.6}"
            ));
        }
    }
    if let Some(value) = section_det_sign_flip_count {
        if value > 0.0 {
            hard_gate_reasons.push(format!("section_det_sign_flip_count {value:.0} > 0"));
        }
    }
    if let Some(value) = min_forward_progress_dense {
        if value < HARD_MIN_FORWARD_PROGRESS {
            hard_gate_reasons.push(format!(
                "min_forward_progress_dense {value:.6} < {HARD_MIN_FORWARD_PROGRESS:.6}"
            ));
        }
    }
    if let Some(value) = sigma_clamp_count {
        if value > 0.0 {
            hard_gate_reasons.push(format!("sigma_clamp_count {value:.0} > 0"));
        }
    }

    UnifiedTrajectoryQuality {
        sample_source,
        sample_count,
        lap_time_s,
        unified_quality_penalty_s: penalty,
        product_score_s: lap_time_s.map(|lap| lap + penalty),
        hard_gate_clean: hard_gate_reasons.is_empty(),
        hard_gate_reasons,
        ay_jump_p95_mps2,
        ay_jump_max_mps2: max_finite(&ay_jumps),
        ax_jump_p95_mps2,
        ax_jump_max_mps2: max_finite(&ax_jumps),
        lateral_slope_p95_abs,
        lateral_slope_max_abs: max_finite(&abs_dn_ds),
        speed_trough_p95_mps,
        speed_trough_max_mps: max_finite(&speed_troughs),
        kappa_jump_p95_1pm,
        kappa_jump_max_1pm: max_finite(&kappa_jumps),
        max_scaled_violation,
        min_section_det_dense,
        min_abs_section_det_dense,
        section_det_reference_sign,
        section_det_sign_flip_count,
        min_forward_progress_dense,
        sigma_clamp_count,
    }
}

fn series_numbers(value: &JsonValue, keys: &[&str]) -> Option<Vec<f64>> {
    keys.iter().find_map(|key| {
        value.get(key).and_then(|field| {
            let numbers: Vec<f64> = field
                .as_array()?
                .iter()
                .filter_map(JsonValue::as_f64)
                .filter(|value| value.is_finite())
                .collect();
            if numbers.is_empty() {
                None
            } else {
                Some(numbers)
            }
        })
    })
}

fn adjacent_abs_deltas(values: &[f64], closed: bool) -> Vec<f64> {
    if values.len() < 2 {
        return Vec::new();
    }
    let mut deltas: Vec<f64> = values
        .windows(2)
        .map(|pair| (pair[1] - pair[0]).abs())
        .collect();
    if closed && values.len() > 2 {
        deltas.push((values[0] - values[values.len() - 1]).abs());
    }
    deltas
}

fn local_speed_troughs(v_mps: &[f64]) -> Vec<f64> {
    if v_mps.len() < 3 {
        return Vec::new();
    }
    let window = (v_mps.len() / 16).clamp(8, 80);
    let mut troughs = Vec::with_capacity(v_mps.len());
    for index in 0..v_mps.len() {
        let start = index.saturating_sub(window);
        let end = (index + window + 1).min(v_mps.len());
        let local_max = v_mps[start..end]
            .iter()
            .copied()
            .filter(|value| value.is_finite())
            .fold(f64::NEG_INFINITY, f64::max);
        if local_max.is_finite() {
            troughs.push((local_max - v_mps[index]).max(0.0));
        }
    }
    troughs
}

fn percentile(values: &[f64], quantile: f64) -> Option<f64> {
    let mut sorted: Vec<f64> = values
        .iter()
        .copied()
        .filter(|value| value.is_finite())
        .collect();
    if sorted.is_empty() {
        return None;
    }
    sorted.sort_by(f64::total_cmp);
    if sorted.len() == 1 {
        return Some(sorted[0]);
    }
    let position = quantile.clamp(0.0, 1.0) * (sorted.len() - 1) as f64;
    let lower = position.floor() as usize;
    let upper = position.ceil() as usize;
    if lower == upper {
        Some(sorted[lower])
    } else {
        let t = position - lower as f64;
        Some(sorted[lower] * (1.0 - t) + sorted[upper] * t)
    }
}

fn max_finite(values: &[f64]) -> Option<f64> {
    values
        .iter()
        .copied()
        .filter(|value| value.is_finite())
        .reduce(f64::max)
}

fn option_number_json(value: Option<f64>) -> JsonValue {
    value.map(JsonValue::from).unwrap_or(JsonValue::Null)
}

fn option_weighted_json(value: Option<f64>, weight: f64) -> JsonValue {
    value
        .map(|value| JsonValue::from(value * weight))
        .unwrap_or(JsonValue::Null)
}

fn weighted_or_zero(value: Option<f64>, weight: f64) -> f64 {
    value.map_or(0.0, |value| value * weight)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn array(values: &[f64]) -> JsonValue {
        JsonValue::Array(values.iter().copied().map(JsonValue::from).collect())
    }

    fn trajectory(v: &[f64], ay: &[f64], ax: &[f64], dn_ds: &[f64]) -> JsonValue {
        JsonValue::Object(vec![
            ("v_mps".to_owned(), array(v)),
            ("ay_model_mps2".to_owned(), array(ay)),
            ("ax_model_mps2".to_owned(), array(ax)),
            ("dn_ds".to_owned(), array(dn_ds)),
            ("kappa_geo_1pm".to_owned(), array(&vec![0.0; v.len()])),
        ])
    }

    #[test]
    fn smooth_trajectory_has_lower_penalty_than_spiky_trajectory() {
        let smooth = trajectory(
            &[10.0, 10.2, 10.1, 10.3, 10.2],
            &[0.0, 0.1, 0.2, 0.1, 0.0],
            &[0.0, 0.1, 0.0, -0.1, 0.0],
            &[0.01, 0.02, 0.02, 0.01, 0.0],
        );
        let spiky = trajectory(
            &[14.0, 5.0, 14.0, 5.0, 14.0],
            &[0.0, 4.0, -4.0, 4.0, 0.0],
            &[0.0, -5.0, 5.0, -5.0, 0.0],
            &[0.0, 0.5, -0.5, 0.5, 0.0],
        );

        let smooth_quality = unified_trajectory_quality(Some(60.0), &smooth, None, None, false);
        let spiky_quality = unified_trajectory_quality(Some(60.0), &spiky, None, None, false);

        assert!(smooth_quality.unified_quality_penalty_s < spiky_quality.unified_quality_penalty_s);
        assert!(smooth_quality.product_score_s < spiky_quality.product_score_s);
    }

    #[test]
    fn hard_gate_uses_residual_and_section_geometry_diagnostics() {
        let trajectory = trajectory(
            &[10.0, 10.0, 10.0],
            &[0.0, 0.0, 0.0],
            &[0.0, 0.0, 0.0],
            &[0.0, 0.0, 0.0],
        );
        let diagnostics = JsonValue::Object(vec![
            (
                "final_residuals".to_owned(),
                JsonValue::Object(vec![(
                    "max_scaled_violation".to_owned(),
                    JsonValue::from(2.0e-6),
                )]),
            ),
            (
                "geometry_diagnostics".to_owned(),
                JsonValue::Object(vec![
                    ("min_section_det_dense".to_owned(), JsonValue::from(0.04)),
                    (
                        "min_forward_progress_dense".to_owned(),
                        JsonValue::from(0.19),
                    ),
                    ("sigma_clamp_count".to_owned(), JsonValue::from(1.0)),
                ]),
            ),
        ]);

        let quality =
            unified_trajectory_quality(Some(60.0), &trajectory, None, Some(&diagnostics), false);

        assert!(!quality.hard_gate_clean);
        assert_eq!(quality.hard_gate_reasons.len(), 4);
    }

    #[test]
    fn hard_gate_accepts_consistently_negative_section_orientation() {
        let trajectory = trajectory(
            &[10.0, 10.0, 10.0],
            &[0.0, 0.0, 0.0],
            &[0.0, 0.0, 0.0],
            &[0.0, 0.0, 0.0],
        );
        let diagnostics = JsonValue::Object(vec![(
            "geometry_diagnostics".to_owned(),
            JsonValue::Object(vec![
                ("min_section_det_dense".to_owned(), JsonValue::from(-1.1)),
                ("min_abs_section_det_dense".to_owned(), JsonValue::from(0.8)),
                (
                    "section_det_reference_sign".to_owned(),
                    JsonValue::from(-1.0),
                ),
                (
                    "section_det_sign_flip_count".to_owned(),
                    JsonValue::from(0.0),
                ),
                (
                    "min_forward_progress_dense".to_owned(),
                    JsonValue::from(0.9),
                ),
                ("sigma_clamp_count".to_owned(), JsonValue::from(0.0)),
            ]),
        )]);

        let quality =
            unified_trajectory_quality(Some(60.0), &trajectory, None, Some(&diagnostics), false);

        assert!(quality.hard_gate_clean);
        assert!(quality.hard_gate_reasons.is_empty());
        assert_eq!(quality.min_abs_section_det_dense, Some(0.8));
        assert_eq!(quality.section_det_reference_sign, Some(-1.0));
    }

    #[test]
    fn hard_gate_rejects_section_orientation_sign_flips() {
        let trajectory = trajectory(
            &[10.0, 10.0, 10.0],
            &[0.0, 0.0, 0.0],
            &[0.0, 0.0, 0.0],
            &[0.0, 0.0, 0.0],
        );
        let diagnostics = JsonValue::Object(vec![(
            "geometry_diagnostics".to_owned(),
            JsonValue::Object(vec![
                ("min_abs_section_det_dense".to_owned(), JsonValue::from(0.8)),
                (
                    "section_det_reference_sign".to_owned(),
                    JsonValue::from(-1.0),
                ),
                (
                    "section_det_sign_flip_count".to_owned(),
                    JsonValue::from(1.0),
                ),
                (
                    "min_forward_progress_dense".to_owned(),
                    JsonValue::from(0.9),
                ),
                ("sigma_clamp_count".to_owned(), JsonValue::from(0.0)),
            ]),
        )]);

        let quality =
            unified_trajectory_quality(Some(60.0), &trajectory, None, Some(&diagnostics), false);

        assert!(!quality.hard_gate_clean);
        assert_eq!(quality.hard_gate_reasons.len(), 1);
        assert!(quality.hard_gate_reasons[0].contains("section_det_sign_flip_count"));
    }
}
