use std::time::{SystemTime, UNIX_EPOCH};

use crate::json::JsonValue;

#[derive(Clone, Debug, PartialEq)]
pub struct SolverConvergenceEventV2 {
    pub iteration: u32,
    pub algorithm_mode: i32,
    pub objective: f64,
    pub primal_infeasibility: f64,
    pub dual_infeasibility: f64,
    pub barrier_parameter: f64,
    pub step_norm: f64,
    pub regularization_size: f64,
    pub dual_step_size: f64,
    pub primal_step_size: f64,
    pub line_search_trials: u32,
    pub target_tolerance: f64,
    pub acceptable_tolerance: f64,
    pub timestamp_ms: u128,
}

impl SolverConvergenceEventV2 {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        iteration: u32,
        algorithm_mode: i32,
        objective: f64,
        primal_infeasibility: f64,
        dual_infeasibility: f64,
        barrier_parameter: f64,
        step_norm: f64,
        regularization_size: f64,
        dual_step_size: f64,
        primal_step_size: f64,
        line_search_trials: i32,
        target_tolerance: f64,
        acceptable_tolerance: f64,
    ) -> Option<Self> {
        let finite = [
            objective,
            primal_infeasibility,
            dual_infeasibility,
            barrier_parameter,
            step_norm,
            regularization_size,
            dual_step_size,
            primal_step_size,
            target_tolerance,
            acceptable_tolerance,
        ]
        .into_iter()
        .all(f64::is_finite);
        if !finite || target_tolerance <= 0.0 || acceptable_tolerance <= 0.0 {
            return None;
        }

        Some(Self {
            iteration,
            algorithm_mode,
            objective,
            primal_infeasibility: primal_infeasibility.max(0.0),
            dual_infeasibility: dual_infeasibility.max(0.0),
            barrier_parameter: barrier_parameter.max(0.0),
            step_norm: step_norm.max(0.0),
            regularization_size: regularization_size.max(0.0),
            dual_step_size: dual_step_size.clamp(0.0, 1.0),
            primal_step_size: primal_step_size.clamp(0.0, 1.0),
            line_search_trials: u32::try_from(line_search_trials.max(0)).unwrap_or(u32::MAX),
            target_tolerance,
            acceptable_tolerance,
            timestamp_ms: timestamp_ms(),
        })
    }

    pub fn to_json_value(&self) -> JsonValue {
        JsonValue::Object(vec![
            (
                "schema_version".to_owned(),
                "solver_convergence_event.v2".into(),
            ),
            (
                "iteration".to_owned(),
                JsonValue::Integer(i64::from(self.iteration)),
            ),
            (
                "algorithm_mode".to_owned(),
                JsonValue::Integer(i64::from(self.algorithm_mode)),
            ),
            ("objective".to_owned(), self.objective.into()),
            (
                "primal_infeasibility".to_owned(),
                self.primal_infeasibility.into(),
            ),
            (
                "dual_infeasibility".to_owned(),
                self.dual_infeasibility.into(),
            ),
            (
                "barrier_parameter".to_owned(),
                self.barrier_parameter.into(),
            ),
            ("step_norm".to_owned(), self.step_norm.into()),
            (
                "regularization_size".to_owned(),
                self.regularization_size.into(),
            ),
            ("dual_step_size".to_owned(), self.dual_step_size.into()),
            ("primal_step_size".to_owned(), self.primal_step_size.into()),
            (
                "line_search_trials".to_owned(),
                JsonValue::Integer(i64::from(self.line_search_trials)),
            ),
            ("target_tolerance".to_owned(), self.target_tolerance.into()),
            (
                "acceptable_tolerance".to_owned(),
                self.acceptable_tolerance.into(),
            ),
            (
                "timestamp_ms".to_owned(),
                JsonValue::Integer(self.timestamp_ms.min(i64::MAX as u128) as i64),
            ),
        ])
    }
}

fn timestamp_ms() -> u128 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|duration| duration.as_millis())
        .unwrap_or_default()
}

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
            timestamp_ms: timestamp_ms(),
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
    use super::{SolverCapabilityEventV2, SolverConvergenceEventV2};
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

    #[test]
    fn convergence_event_reports_raw_ipopt_values_without_a_ui_percentage() {
        let event = SolverConvergenceEventV2::new(
            42, 0, 58.2, 1e-4, 2e-3, 1e-5, 0.4, 0.0, 1.0, 0.5, 3, 1e-5, 1e-4,
        )
        .expect("finite convergence telemetry must serialize");

        let json = event.to_json_value();
        assert_eq!(
            json.get("schema_version").and_then(JsonValue::as_str),
            Some("solver_convergence_event.v2")
        );
        assert_eq!(
            json.get("iteration").and_then(JsonValue::as_f64),
            Some(42.0)
        );
        assert!(json.get("progress").is_none());
    }
}
