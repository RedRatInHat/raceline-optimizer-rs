use std::time::{SystemTime, UNIX_EPOCH};

use crate::json::JsonValue;

/// Physical capability telemetry published while a native solver is running.
///
/// This is deliberately not a UI progress percentage. Consumers may derive a
/// presentation estimate from the values, but solver code only reports the
/// physical metric and the acceptance threshold that applies to its scope.
#[derive(Clone, Debug, PartialEq)]
pub struct SolverCapabilityEventV2 {
    pub metric_id: String,
    pub sample_scope: String,
    pub max_utilization: f64,
    pub clean_limit: f64,
    pub max_violation: f64,
    pub is_dense_clean: bool,
    pub objective: Option<f64>,
    pub timestamp_ms: u128,
}

impl SolverCapabilityEventV2 {
    pub fn from_max_utilization(
        metric_id: impl Into<String>,
        sample_scope: impl Into<String>,
        max_utilization: f64,
        clean_limit: f64,
        objective: Option<f64>,
    ) -> Option<Self> {
        if !max_utilization.is_finite() || !clean_limit.is_finite() || clean_limit <= 0.0 {
            return None;
        }

        Some(Self {
            metric_id: metric_id.into(),
            sample_scope: sample_scope.into(),
            max_utilization,
            clean_limit,
            max_violation: (max_utilization - clean_limit).max(0.0),
            is_dense_clean: max_utilization <= clean_limit,
            objective,
            timestamp_ms: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .map(|duration| duration.as_millis())
                .unwrap_or_default(),
        })
    }

    pub fn to_json_value(&self) -> JsonValue {
        JsonValue::Object(vec![
            (
                "schema_version".to_owned(),
                "solver_capability_event.v2".into(),
            ),
            ("metric_id".to_owned(), self.metric_id.clone().into()),
            ("sample_scope".to_owned(), self.sample_scope.clone().into()),
            ("max_utilization".to_owned(), self.max_utilization.into()),
            ("clean_limit".to_owned(), self.clean_limit.into()),
            ("max_violation".to_owned(), self.max_violation.into()),
            (
                "is_dense_clean".to_owned(),
                JsonValue::Bool(self.is_dense_clean),
            ),
            (
                "objective".to_owned(),
                self.objective
                    .map(JsonValue::from)
                    .unwrap_or(JsonValue::Null),
            ),
            (
                "timestamp_ms".to_owned(),
                JsonValue::Integer(self.timestamp_ms.min(i64::MAX as u128) as i64),
            ),
        ])
    }
}

#[cfg(test)]
mod tests {
    use super::SolverCapabilityEventV2;
    use crate::json::JsonValue;

    #[test]
    fn capability_event_reports_physical_violation_without_a_ui_percentage() {
        let event = SolverCapabilityEventV2::from_max_utilization(
            "car.active_kamm_utilization",
            "station_collocation_linear_dense",
            1.014,
            1.02,
            Some(58.2),
        )
        .expect("finite physical telemetry must serialize");

        assert_eq!(event.max_violation, 0.0);
        assert!(event.is_dense_clean);
        let json = event.to_json_value();
        assert_eq!(
            json.get("schema_version").and_then(JsonValue::as_str),
            Some("solver_capability_event.v2")
        );
        assert!(json.get("progress").is_none());
    }

    #[test]
    fn capability_event_rejects_non_finite_physical_values() {
        assert!(SolverCapabilityEventV2::from_max_utilization(
            "metric",
            "scope",
            f64::NAN,
            1.0,
            None,
        )
        .is_none());
    }
}
