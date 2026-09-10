use crate::vehicle_dynamics::VehicleDynamicsModelFamily;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MintimeLayout {
    pub model_family: VehicleDynamicsModelFamily,
    pub state_columns: &'static [&'static str],
    pub control_columns: &'static [&'static str],
}

impl MintimeLayout {
    #[must_use]
    pub fn dimensions_for_station_count(
        self,
        station_count: usize,
        closed: bool,
    ) -> MintimeDimensions {
        let interval_count = if closed {
            station_count
        } else {
            station_count.saturating_sub(1)
        };
        MintimeDimensions {
            station_count,
            interval_count,
            state_variable_count: station_count * self.state_columns.len(),
            control_variable_count: interval_count * self.control_columns.len(),
            collocation_state_variable_count: 0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MintimeDimensions {
    pub station_count: usize,
    pub interval_count: usize,
    pub state_variable_count: usize,
    pub control_variable_count: usize,
    pub collocation_state_variable_count: usize,
}

impl MintimeDimensions {
    #[must_use]
    pub fn decision_variable_count(self) -> usize {
        self.state_variable_count
            + self.control_variable_count
            + self.collocation_state_variable_count
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DecisionLayout {
    pub dimensions: MintimeDimensions,
    pub state_len: usize,
    pub control_len: usize,
    pub collocation_degree: usize,
}

impl DecisionLayout {
    #[must_use]
    pub fn new(
        station_count: usize,
        closed: bool,
        state_len: usize,
        control_len: usize,
        collocation_degree: usize,
    ) -> Self {
        let interval_count = if closed {
            station_count
        } else {
            station_count.saturating_sub(1)
        };
        let mut dimensions = MintimeDimensions {
            station_count,
            interval_count,
            state_variable_count: station_count * state_len,
            control_variable_count: interval_count * control_len,
            collocation_state_variable_count: 0,
        };
        dimensions.collocation_state_variable_count =
            interval_count * collocation_degree * state_len;
        Self {
            dimensions,
            state_len,
            control_len,
            collocation_degree,
        }
    }

    #[must_use]
    pub fn state_offset(self, station: usize) -> usize {
        state_offset(station, self.state_len)
    }

    #[must_use]
    pub fn control_offset(self, interval: usize) -> usize {
        control_offset(self.dimensions, interval, self.control_len)
    }

    #[must_use]
    pub fn collocation_state_offset(self, interval: usize, point: usize) -> usize {
        collocation_state_offset(
            self.dimensions,
            interval,
            point,
            self.state_len,
            self.collocation_degree,
        )
    }
}

#[must_use]
pub fn state_offset(station: usize, state_len: usize) -> usize {
    station * state_len
}

#[must_use]
pub fn control_offset(dimensions: MintimeDimensions, interval: usize, control_len: usize) -> usize {
    dimensions.state_variable_count + interval * control_len
}

#[must_use]
pub fn collocation_state_offset(
    dimensions: MintimeDimensions,
    interval: usize,
    point: usize,
    state_len: usize,
    collocation_degree: usize,
) -> usize {
    dimensions.state_variable_count
        + dimensions.control_variable_count
        + (interval * collocation_degree + point) * state_len
}

#[must_use]
pub fn next_station_index(station_count: usize, interval: usize) -> usize {
    (interval + 1) % station_count.max(1)
}

pub trait MintimeModelSpec {
    const STATE_LEN: usize;
    const CONTROL_LEN: usize;

    type Params: Copy;
    type State;
    type Control;

    fn model_id() -> &'static str;
    fn state_columns() -> &'static [&'static str];
    fn control_columns() -> &'static [&'static str];
    fn state_from_slice(values: &[f64]) -> Self::State;
    fn control_from_slice(values: &[f64]) -> Self::Control;
    fn initial_state(params: Self::Params) -> Vec<f64>;
    fn initial_control(params: Self::Params) -> Vec<f64>;
    fn state_bounds(params: Self::Params, lower_n_m: f64, upper_n_m: f64) -> (Vec<f64>, Vec<f64>);
    fn control_bounds(params: Self::Params) -> (Vec<f64>, Vec<f64>);
    fn dynamics_s(
        params: Self::Params,
        state: Self::State,
        control: Self::Control,
        kappa_1pm: f64,
    ) -> Vec<f64>;
    fn sigma_dt_ds(
        params: Self::Params,
        state: Self::State,
        control: Self::Control,
        kappa_1pm: f64,
    ) -> f64;

    #[must_use]
    fn layout(model_family: VehicleDynamicsModelFamily) -> MintimeLayout {
        MintimeLayout {
            model_family,
            state_columns: Self::state_columns(),
            control_columns: Self::control_columns(),
        }
    }

    #[must_use]
    fn decision_layout(station_count: usize, closed: bool) -> DecisionLayout {
        DecisionLayout::new(
            station_count,
            closed,
            Self::STATE_LEN,
            Self::CONTROL_LEN,
            CollocationDegree3::DEGREE,
        )
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct CollocationIntervalInput {
    pub start_state: Vec<f64>,
    pub collocation_states: Vec<Vec<f64>>,
    pub end_state: Vec<f64>,
    pub control: Vec<f64>,
    pub ds_m: f64,
    pub kappa_1pm: Vec<f64>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct CollocationIntervalDefects {
    pub dynamics: Vec<Vec<f64>>,
    pub continuity: Vec<f64>,
}

pub fn collocation_defects<S: MintimeModelSpec>(
    params: S::Params,
    input: &CollocationIntervalInput,
) -> Result<CollocationIntervalDefects, String> {
    validate_collocation_input::<S>(input)?;

    let collocation = CollocationDegree3::legendre();
    let mut states = Vec::with_capacity(CollocationDegree3::DEGREE + 1);
    states.push(input.start_state.clone());
    states.extend(input.collocation_states.iter().cloned());

    let mut dynamics = vec![vec![0.0; S::STATE_LEN]; CollocationDegree3::DEGREE];
    for point in 1..=CollocationDegree3::DEGREE {
        let state = S::state_from_slice(&states[point]);
        let control = S::control_from_slice(&input.control);
        let rhs = S::dynamics_s(params, state, control, input.kappa_1pm[point - 1]);
        if rhs.len() != S::STATE_LEN {
            return Err(format!(
                "{} dynamics_s returned {} values, expected {}",
                S::model_id(),
                rhs.len(),
                S::STATE_LEN
            ));
        }

        for state_index in 0..S::STATE_LEN {
            let mut polynomial_derivative = 0.0;
            for basis_index in 0..=CollocationDegree3::DEGREE {
                polynomial_derivative +=
                    collocation.c[basis_index][point] * states[basis_index][state_index];
            }
            dynamics[point - 1][state_index] =
                polynomial_derivative / input.ds_m - rhs[state_index];
        }
    }

    let mut continuity = vec![0.0; S::STATE_LEN];
    for state_index in 0..S::STATE_LEN {
        let mut endpoint = 0.0;
        for basis_index in 0..=CollocationDegree3::DEGREE {
            endpoint += collocation.d[basis_index] * states[basis_index][state_index];
        }
        continuity[state_index] = endpoint - input.end_state[state_index];
    }

    Ok(CollocationIntervalDefects {
        dynamics,
        continuity,
    })
}

fn validate_collocation_input<S: MintimeModelSpec>(
    input: &CollocationIntervalInput,
) -> Result<(), String> {
    validate_len::<S>("start_state", input.start_state.len(), S::STATE_LEN)?;
    validate_len::<S>("end_state", input.end_state.len(), S::STATE_LEN)?;
    validate_len::<S>("control", input.control.len(), S::CONTROL_LEN)?;
    validate_len::<S>(
        "collocation_states",
        input.collocation_states.len(),
        CollocationDegree3::DEGREE,
    )?;
    validate_len::<S>(
        "kappa_1pm",
        input.kappa_1pm.len(),
        CollocationDegree3::DEGREE,
    )?;
    if input.ds_m <= 0.0 {
        return Err(format!(
            "{} collocation ds_m must be positive, got {}",
            S::model_id(),
            input.ds_m
        ));
    }
    for (point, state) in input.collocation_states.iter().enumerate() {
        validate_len::<S>(
            &format!("collocation_states[{point}]"),
            state.len(),
            S::STATE_LEN,
        )?;
    }
    Ok(())
}

#[derive(Clone, Debug, PartialEq)]
pub struct MintimeSeedBounds {
    pub layout: MintimeLayout,
    pub decision_layout: DecisionLayout,
    pub dimensions: MintimeDimensions,
    pub station_initial_state: Vec<f64>,
    pub initial_control: Vec<f64>,
    pub state_lower_bounds: Vec<f64>,
    pub state_upper_bounds: Vec<f64>,
    pub control_lower_bounds: Vec<f64>,
    pub control_upper_bounds: Vec<f64>,
    pub initial_decision: Vec<f64>,
    pub lower_bounds: Vec<f64>,
    pub upper_bounds: Vec<f64>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GenericMintimeConstraintRow {
    Continuity {
        interval: usize,
        state_index: usize,
    },
    CollocationDynamics {
        interval: usize,
        point: usize,
        state_index: usize,
    },
    ControlRate {
        interval: usize,
        control_index: usize,
    },
    TrackBounds {
        station: usize,
    },
    DtDs {
        interval: usize,
    },
    ModelSpecific {
        kind: &'static str,
        index: usize,
    },
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NumericSparseJacobianEntry {
    pub row: usize,
    pub variable: usize,
    pub value: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct GenericConstraintRowOptions {
    pub include_control_rate: bool,
    pub include_track_bounds: bool,
    pub include_dt_ds: bool,
}

impl GenericConstraintRowOptions {
    #[must_use]
    pub const fn collocation_only() -> Self {
        Self {
            include_control_rate: false,
            include_track_bounds: false,
            include_dt_ds: false,
        }
    }

    #[must_use]
    pub const fn with_control_rate() -> Self {
        Self {
            include_control_rate: true,
            include_track_bounds: false,
            include_dt_ds: false,
        }
    }
}

#[must_use]
pub fn generic_mintime_constraint_rows(
    decision_layout: DecisionLayout,
    options: GenericConstraintRowOptions,
) -> Vec<GenericMintimeConstraintRow> {
    let dimensions = decision_layout.dimensions;
    let mut rows = Vec::new();

    for interval in 0..dimensions.interval_count {
        for state_index in 0..decision_layout.state_len {
            rows.push(GenericMintimeConstraintRow::Continuity {
                interval,
                state_index,
            });
        }
    }

    for interval in 0..dimensions.interval_count {
        for point in 0..decision_layout.collocation_degree {
            for state_index in 0..decision_layout.state_len {
                rows.push(GenericMintimeConstraintRow::CollocationDynamics {
                    interval,
                    point,
                    state_index,
                });
            }
        }
    }

    if options.include_control_rate {
        for interval in 0..dimensions.interval_count {
            for control_index in 0..decision_layout.control_len {
                rows.push(GenericMintimeConstraintRow::ControlRate {
                    interval,
                    control_index,
                });
            }
        }
    }

    if options.include_track_bounds {
        for station in 0..dimensions.station_count {
            rows.push(GenericMintimeConstraintRow::TrackBounds { station });
        }
    }

    if options.include_dt_ds {
        for interval in 0..dimensions.interval_count {
            rows.push(GenericMintimeConstraintRow::DtDs { interval });
        }
    }

    rows
}

pub fn generic_mintime_constraint_values<S: MintimeModelSpec>(
    params: S::Params,
    decision_layout: DecisionLayout,
    decision: &[f64],
    rows: &[GenericMintimeConstraintRow],
    interval_ds_m: &[f64],
    kappa_1pm: &[Vec<f64>],
) -> Result<Vec<f64>, String> {
    validate_generic_constraint_inputs::<S>(decision_layout, decision, interval_ds_m, kappa_1pm)?;

    let collocation = CollocationDegree3::legendre();
    let mut values = Vec::with_capacity(rows.len());
    for row in rows {
        values.push(generic_mintime_constraint_value_unchecked::<S>(
            params,
            decision_layout,
            decision,
            *row,
            interval_ds_m,
            kappa_1pm,
            collocation,
        )?);
    }
    Ok(values)
}

#[must_use]
pub fn generic_mintime_sparse_pattern(
    decision_layout: DecisionLayout,
    rows: &[GenericMintimeConstraintRow],
) -> Vec<(usize, usize)> {
    let mut pattern = Vec::new();
    for (row_index, row) in rows.iter().enumerate() {
        let mut variables = std::collections::BTreeSet::new();
        match *row {
            GenericMintimeConstraintRow::Continuity {
                interval,
                state_index,
            } => {
                variables.insert(decision_layout.state_offset(interval) + state_index);
                let next_station =
                    next_station_index(decision_layout.dimensions.station_count, interval);
                variables.insert(decision_layout.state_offset(next_station) + state_index);
                for point in 0..decision_layout.collocation_degree {
                    variables.insert(
                        decision_layout.collocation_state_offset(interval, point) + state_index,
                    );
                }
            }
            GenericMintimeConstraintRow::CollocationDynamics { interval, .. } => {
                for state_index in 0..decision_layout.state_len {
                    variables.insert(decision_layout.state_offset(interval) + state_index);
                }
                for point in 0..decision_layout.collocation_degree {
                    let offset = decision_layout.collocation_state_offset(interval, point);
                    for state_index in 0..decision_layout.state_len {
                        variables.insert(offset + state_index);
                    }
                }
                let control_offset = decision_layout.control_offset(interval);
                for control_index in 0..decision_layout.control_len {
                    variables.insert(control_offset + control_index);
                }
            }
            GenericMintimeConstraintRow::ControlRate {
                interval,
                control_index,
            } => {
                let next_interval =
                    next_interval_index(decision_layout.dimensions.interval_count, interval);
                variables.insert(decision_layout.control_offset(interval) + control_index);
                variables.insert(decision_layout.control_offset(next_interval) + control_index);
            }
            GenericMintimeConstraintRow::TrackBounds { station } => {
                let offset = decision_layout.state_offset(station);
                for state_index in 0..decision_layout.state_len {
                    variables.insert(offset + state_index);
                }
            }
            GenericMintimeConstraintRow::DtDs { interval } => {
                let offset = decision_layout.state_offset(interval);
                for state_index in 0..decision_layout.state_len {
                    variables.insert(offset + state_index);
                }
            }
            GenericMintimeConstraintRow::ModelSpecific { .. } => {}
        }
        pattern.extend(variables.into_iter().map(|variable| (row_index, variable)));
    }
    pattern
}

pub fn generic_mintime_sparse_jacobian_numeric<S: MintimeModelSpec>(
    params: S::Params,
    decision_layout: DecisionLayout,
    decision: &[f64],
    rows: &[GenericMintimeConstraintRow],
    interval_ds_m: &[f64],
    kappa_1pm: &[Vec<f64>],
    epsilon: f64,
) -> Result<Vec<NumericSparseJacobianEntry>, String> {
    if epsilon <= 0.0 {
        return Err(format!(
            "{} sparse numeric Jacobian epsilon must be positive, got {}",
            S::model_id(),
            epsilon
        ));
    }
    let pattern = generic_mintime_sparse_pattern(decision_layout, rows);
    let mut entries = Vec::with_capacity(pattern.len());
    let mut plus = decision.to_vec();
    let mut minus = decision.to_vec();
    for (row, variable) in pattern {
        plus[variable] += epsilon;
        minus[variable] -= epsilon;
        let plus_value = generic_mintime_constraint_value_unchecked::<S>(
            params,
            decision_layout,
            &plus,
            rows[row],
            interval_ds_m,
            kappa_1pm,
            CollocationDegree3::legendre(),
        )?;
        let minus_value = generic_mintime_constraint_value_unchecked::<S>(
            params,
            decision_layout,
            &minus,
            rows[row],
            interval_ds_m,
            kappa_1pm,
            CollocationDegree3::legendre(),
        )?;
        entries.push(NumericSparseJacobianEntry {
            row,
            variable,
            value: (plus_value - minus_value) / (2.0 * epsilon),
        });
        plus[variable] = decision[variable];
        minus[variable] = decision[variable];
    }
    Ok(entries)
}

fn generic_mintime_constraint_value_unchecked<S: MintimeModelSpec>(
    params: S::Params,
    decision_layout: DecisionLayout,
    decision: &[f64],
    row: GenericMintimeConstraintRow,
    interval_ds_m: &[f64],
    kappa_1pm: &[Vec<f64>],
    collocation: CollocationDegree3,
) -> Result<f64, String> {
    Ok(match row {
        GenericMintimeConstraintRow::Continuity {
            interval,
            state_index,
        } => {
            let next_station =
                next_station_index(decision_layout.dimensions.station_count, interval);
            let mut endpoint =
                collocation.d[0] * decision[decision_layout.state_offset(interval) + state_index];
            for point in 0..decision_layout.collocation_degree {
                endpoint += collocation.d[point + 1]
                    * decision
                        [decision_layout.collocation_state_offset(interval, point) + state_index];
            }
            endpoint - decision[decision_layout.state_offset(next_station) + state_index]
        }
        GenericMintimeConstraintRow::CollocationDynamics {
            interval,
            point,
            state_index,
        } => {
            let mut states = Vec::with_capacity(CollocationDegree3::DEGREE + 1);
            states.push(read_station_state(decision_layout, decision, interval));
            for collocation_point in 0..decision_layout.collocation_degree {
                states.push(read_collocation_state(
                    decision_layout,
                    decision,
                    interval,
                    collocation_point,
                ));
            }
            let control = read_control(decision_layout, decision, interval);
            let state = S::state_from_slice(&states[point + 1]);
            let control = S::control_from_slice(&control);
            let rhs = S::dynamics_s(params, state, control, kappa_1pm[interval][point]);
            if rhs.len() != S::STATE_LEN {
                return Err(format!(
                    "{} dynamics_s returned {} values, expected {}",
                    S::model_id(),
                    rhs.len(),
                    S::STATE_LEN
                ));
            }

            let mut polynomial_derivative = 0.0;
            for basis_index in 0..=decision_layout.collocation_degree {
                polynomial_derivative +=
                    collocation.c[basis_index][point + 1] * states[basis_index][state_index];
            }
            polynomial_derivative / interval_ds_m[interval] - rhs[state_index]
        }
        GenericMintimeConstraintRow::ControlRate {
            interval,
            control_index,
        } => {
            let next_interval =
                next_interval_index(decision_layout.dimensions.interval_count, interval);
            decision[decision_layout.control_offset(next_interval) + control_index]
                - decision[decision_layout.control_offset(interval) + control_index]
        }
        GenericMintimeConstraintRow::TrackBounds { .. }
        | GenericMintimeConstraintRow::DtDs { .. }
        | GenericMintimeConstraintRow::ModelSpecific { .. } => {
            return Err(format!(
                "{} row {:?} has no generic evaluator",
                S::model_id(),
                row
            ));
        }
    })
}

pub fn generic_mintime_lap_time_objective_s<S: MintimeModelSpec>(
    params: S::Params,
    decision_layout: DecisionLayout,
    decision: &[f64],
    interval_ds_m: &[f64],
    interval_kappa_1pm: &[f64],
) -> Result<f64, String> {
    validate_len::<S>(
        "decision",
        decision.len(),
        decision_layout.dimensions.decision_variable_count(),
    )?;
    validate_len::<S>(
        "interval_ds_m",
        interval_ds_m.len(),
        decision_layout.dimensions.interval_count,
    )?;
    validate_len::<S>(
        "interval_kappa_1pm",
        interval_kappa_1pm.len(),
        decision_layout.dimensions.interval_count,
    )?;

    let mut objective_s = 0.0;
    for interval in 0..decision_layout.dimensions.interval_count {
        let ds_m = interval_ds_m[interval];
        if ds_m <= 0.0 {
            return Err(format!(
                "{} interval_ds_m[{interval}] must be positive, got {ds_m}",
                S::model_id()
            ));
        }
        let state = S::state_from_slice(&read_station_state(decision_layout, decision, interval));
        let control = S::control_from_slice(&read_control(decision_layout, decision, interval));
        let sigma = S::sigma_dt_ds(params, state, control, interval_kappa_1pm[interval]);
        if !sigma.is_finite() {
            return Err(format!(
                "{} sigma_dt_ds returned non-finite value at interval {interval}: {sigma}",
                S::model_id()
            ));
        }
        objective_s += sigma * ds_m;
    }
    Ok(objective_s)
}

pub fn generic_mintime_objective_gradient_numeric<S: MintimeModelSpec>(
    params: S::Params,
    decision_layout: DecisionLayout,
    decision: &[f64],
    interval_ds_m: &[f64],
    interval_kappa_1pm: &[f64],
    epsilon: f64,
) -> Result<Vec<f64>, String> {
    if epsilon <= 0.0 {
        return Err(format!(
            "{} objective gradient epsilon must be positive, got {}",
            S::model_id(),
            epsilon
        ));
    }
    let mut gradient = vec![0.0; decision.len()];
    let mut plus = decision.to_vec();
    let mut minus = decision.to_vec();
    for variable in 0..decision.len() {
        plus[variable] += epsilon;
        minus[variable] -= epsilon;
        let plus_value = generic_mintime_lap_time_objective_s::<S>(
            params,
            decision_layout,
            &plus,
            interval_ds_m,
            interval_kappa_1pm,
        )?;
        let minus_value = generic_mintime_lap_time_objective_s::<S>(
            params,
            decision_layout,
            &minus,
            interval_ds_m,
            interval_kappa_1pm,
        )?;
        gradient[variable] = (plus_value - minus_value) / (2.0 * epsilon);
        plus[variable] = decision[variable];
        minus[variable] = decision[variable];
    }
    Ok(gradient)
}

pub fn build_mintime_seed_bounds<S: MintimeModelSpec>(
    model_family: VehicleDynamicsModelFamily,
    station_count: usize,
    closed: bool,
    params: S::Params,
    lower_n_m: f64,
    upper_n_m: f64,
) -> Result<MintimeSeedBounds, String> {
    let layout = S::layout(model_family);
    let decision_layout = S::decision_layout(station_count, closed);
    let dimensions = decision_layout.dimensions;

    let station_initial_state = S::initial_state(params);
    let initial_control = S::initial_control(params);
    let (state_lower_bounds, state_upper_bounds) = S::state_bounds(params, lower_n_m, upper_n_m);
    let (control_lower_bounds, control_upper_bounds) = S::control_bounds(params);

    validate_len::<S>("initial_state", station_initial_state.len(), S::STATE_LEN)?;
    validate_len::<S>("initial_control", initial_control.len(), S::CONTROL_LEN)?;
    validate_len::<S>("state_lower_bounds", state_lower_bounds.len(), S::STATE_LEN)?;
    validate_len::<S>("state_upper_bounds", state_upper_bounds.len(), S::STATE_LEN)?;
    validate_len::<S>(
        "control_lower_bounds",
        control_lower_bounds.len(),
        S::CONTROL_LEN,
    )?;
    validate_len::<S>(
        "control_upper_bounds",
        control_upper_bounds.len(),
        S::CONTROL_LEN,
    )?;

    let mut initial_decision = Vec::with_capacity(dimensions.decision_variable_count());
    let mut lower_bounds = Vec::with_capacity(dimensions.decision_variable_count());
    let mut upper_bounds = Vec::with_capacity(dimensions.decision_variable_count());

    for _ in 0..dimensions.station_count {
        initial_decision.extend_from_slice(&station_initial_state);
        lower_bounds.extend_from_slice(&state_lower_bounds);
        upper_bounds.extend_from_slice(&state_upper_bounds);
    }
    for _ in 0..dimensions.interval_count {
        initial_decision.extend_from_slice(&initial_control);
        lower_bounds.extend_from_slice(&control_lower_bounds);
        upper_bounds.extend_from_slice(&control_upper_bounds);
    }
    for _ in 0..dimensions.interval_count * CollocationDegree3::DEGREE {
        initial_decision.extend_from_slice(&station_initial_state);
        lower_bounds.extend_from_slice(&state_lower_bounds);
        upper_bounds.extend_from_slice(&state_upper_bounds);
    }

    debug_assert_eq!(initial_decision.len(), dimensions.decision_variable_count());
    debug_assert_eq!(lower_bounds.len(), dimensions.decision_variable_count());
    debug_assert_eq!(upper_bounds.len(), dimensions.decision_variable_count());

    Ok(MintimeSeedBounds {
        layout,
        decision_layout,
        dimensions,
        station_initial_state,
        initial_control,
        state_lower_bounds,
        state_upper_bounds,
        control_lower_bounds,
        control_upper_bounds,
        initial_decision,
        lower_bounds,
        upper_bounds,
    })
}

fn validate_generic_constraint_inputs<S: MintimeModelSpec>(
    decision_layout: DecisionLayout,
    decision: &[f64],
    interval_ds_m: &[f64],
    kappa_1pm: &[Vec<f64>],
) -> Result<(), String> {
    validate_len::<S>(
        "decision",
        decision.len(),
        decision_layout.dimensions.decision_variable_count(),
    )?;
    validate_len::<S>(
        "interval_ds_m",
        interval_ds_m.len(),
        decision_layout.dimensions.interval_count,
    )?;
    validate_len::<S>(
        "kappa_1pm",
        kappa_1pm.len(),
        decision_layout.dimensions.interval_count,
    )?;
    for (interval, ds_m) in interval_ds_m.iter().enumerate() {
        if *ds_m <= 0.0 {
            return Err(format!(
                "{} interval_ds_m[{interval}] must be positive, got {ds_m}",
                S::model_id()
            ));
        }
    }
    for (interval, kappa) in kappa_1pm.iter().enumerate() {
        validate_len::<S>(
            &format!("kappa_1pm[{interval}]"),
            kappa.len(),
            decision_layout.collocation_degree,
        )?;
    }
    Ok(())
}

fn read_station_state(
    decision_layout: DecisionLayout,
    decision: &[f64],
    station: usize,
) -> Vec<f64> {
    let offset = decision_layout.state_offset(station);
    decision[offset..offset + decision_layout.state_len].to_vec()
}

fn read_collocation_state(
    decision_layout: DecisionLayout,
    decision: &[f64],
    interval: usize,
    point: usize,
) -> Vec<f64> {
    let offset = decision_layout.collocation_state_offset(interval, point);
    decision[offset..offset + decision_layout.state_len].to_vec()
}

fn read_control(decision_layout: DecisionLayout, decision: &[f64], interval: usize) -> Vec<f64> {
    let offset = decision_layout.control_offset(interval);
    decision[offset..offset + decision_layout.control_len].to_vec()
}

#[must_use]
fn next_interval_index(interval_count: usize, interval: usize) -> usize {
    (interval + 1) % interval_count.max(1)
}

fn validate_len<S: MintimeModelSpec>(
    name: &str,
    actual: usize,
    expected: usize,
) -> Result<(), String> {
    if actual == expected {
        return Ok(());
    }
    Err(format!(
        "{} {name} length {actual}, expected {expected}",
        S::model_id()
    ))
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CollocationDegree3 {
    pub tau: [f64; 4],
    pub c: [[f64; 4]; 4],
    pub d: [f64; 4],
    pub b: [f64; 4],
}

impl CollocationDegree3 {
    pub const DEGREE: usize = 3;

    #[must_use]
    pub const fn legendre() -> Self {
        Self {
            tau: [0.0, 0.112_701_665_379_258_3, 0.5, 0.887_298_334_620_741_7],
            c: [
                [
                    -12.000_000_000_000_004,
                    -6.000_000_000_000_004,
                    2.999_999_999_999_996_4,
                    -6.000_000_000_000_012,
                ],
                [
                    13.121_638_910_345_695,
                    5.000_000_000_000_002,
                    -5.727_486_121_839_512,
                    10.163_977_794_943_227,
                ],
                [
                    -1.333_333_333_333_333_3,
                    1.163_977_794_943_223,
                    2.000_000_000_000_002_7,
                    -9.163_977_794_943_216,
                ],
                [
                    0.211_694_422_987_638_52,
                    -0.163_977_794_943_222_5,
                    0.727_486_121_839_514_1,
                    5.0,
                ],
            ],
            d: [
                -1.000_000_000_000_003_6,
                1.666_666_666_666_671_4,
                -1.333_333_333_333_329_7,
                1.666_666_666_666_666_7,
            ],
            b: [
                0.0,
                0.277_777_777_777_778_57,
                0.444_444_444_444_444_2,
                0.277_777_777_777_777_85,
            ],
        }
    }

    #[must_use]
    pub fn basis_at_tau(self, tau: f64) -> [f64; 4] {
        let mut basis = [0.0; 4];
        for node_index in 0..=Self::DEGREE {
            let mut value = 1.0;
            for other_index in 0..=Self::DEGREE {
                if other_index != node_index {
                    value *= (tau - self.tau[other_index])
                        / (self.tau[node_index] - self.tau[other_index]);
                }
            }
            basis[node_index] = value;
        }
        basis
    }

    #[must_use]
    pub fn basis_derivative_at_tau(self, tau: f64) -> [f64; 4] {
        let mut basis = [0.0; 4];
        for node_index in 0..=Self::DEGREE {
            let mut sum = 0.0;
            for derivative_index in 0..=Self::DEGREE {
                if derivative_index == node_index {
                    continue;
                }
                let mut term = 1.0 / (self.tau[node_index] - self.tau[derivative_index]);
                for product_index in 0..=Self::DEGREE {
                    if product_index != node_index && product_index != derivative_index {
                        term *= (tau - self.tau[product_index])
                            / (self.tau[node_index] - self.tau[product_index]);
                    }
                }
                sum += term;
            }
            basis[node_index] = sum;
        }
        basis
    }

    #[must_use]
    pub fn basis_second_derivative_at_tau(self, tau: f64) -> [f64; 4] {
        let mut basis = [0.0; 4];
        for node_index in 0..=Self::DEGREE {
            let mut sum = 0.0;
            for first_derivative_index in 0..=Self::DEGREE {
                if first_derivative_index == node_index {
                    continue;
                }
                for second_derivative_index in 0..=Self::DEGREE {
                    if second_derivative_index == node_index
                        || second_derivative_index == first_derivative_index
                    {
                        continue;
                    }
                    let mut term = 1.0
                        / ((self.tau[node_index] - self.tau[first_derivative_index])
                            * (self.tau[node_index] - self.tau[second_derivative_index]));
                    for product_index in 0..=Self::DEGREE {
                        if product_index != node_index
                            && product_index != first_derivative_index
                            && product_index != second_derivative_index
                        {
                            term *= (tau - self.tau[product_index])
                                / (self.tau[node_index] - self.tau[product_index]);
                        }
                    }
                    sum += term;
                }
            }
            basis[node_index] = sum;
        }
        basis
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_close(left: f64, right: f64, tolerance: f64) {
        assert!(
            (left - right).abs() <= tolerance,
            "left={left} right={right} delta={}",
            (left - right).abs()
        );
    }

    #[test]
    fn mintime_common_degree3_basis_matches_collocation_coefficients() {
        let coefficients = CollocationDegree3::legendre();

        for point in 0..=CollocationDegree3::DEGREE {
            let basis = coefficients.basis_at_tau(coefficients.tau[point]);
            for node in 0..=CollocationDegree3::DEGREE {
                assert_close(basis[node], if node == point { 1.0 } else { 0.0 }, 1e-12);
            }
        }

        for point in 1..=CollocationDegree3::DEGREE {
            let basis_derivative = coefficients.basis_derivative_at_tau(coefficients.tau[point]);
            for node in 0..=CollocationDegree3::DEGREE {
                assert_close(basis_derivative[node], coefficients.c[node][point], 1e-11);
            }
        }

        let endpoint_basis = coefficients.basis_at_tau(1.0);
        for node in 0..=CollocationDegree3::DEGREE {
            assert_close(endpoint_basis[node], coefficients.d[node], 1e-12);
        }
    }

    #[test]
    fn mintime_common_decision_layout_matches_existing_car_and_bike_offsets() {
        let car = DecisionLayout::new(32, true, 5, 4, 3);
        assert_eq!(car.dimensions.station_count, 32);
        assert_eq!(car.dimensions.interval_count, 32);
        assert_eq!(car.dimensions.state_variable_count, 32 * 5);
        assert_eq!(car.dimensions.control_variable_count, 32 * 4);
        assert_eq!(car.dimensions.collocation_state_variable_count, 32 * 3 * 5);
        assert_eq!(car.state_offset(7), 7 * 5);
        assert_eq!(car.control_offset(4), 32 * 5 + 4 * 4);
        assert_eq!(
            car.collocation_state_offset(4, 2),
            32 * 5 + 32 * 4 + (4 * 3 + 2) * 5
        );

        let bike_v05 = DecisionLayout::new(24, true, 7, 4, 3);
        assert_eq!(bike_v05.state_offset(3), 3 * 7);
        assert_eq!(bike_v05.control_offset(2), 24 * 7 + 2 * 4);
        assert_eq!(
            bike_v05.collocation_state_offset(2, 1),
            24 * 7 + 24 * 4 + (2 * 3 + 1) * 7
        );

        let bike_v1 = DecisionLayout::new(40, true, 9, 3, 3);
        assert_eq!(bike_v1.state_offset(5), 5 * 9);
        assert_eq!(bike_v1.control_offset(6), 40 * 9 + 6 * 3);
        assert_eq!(
            bike_v1.collocation_state_offset(6, 0),
            40 * 9 + 40 * 3 + (6 * 3) * 9
        );
    }

    #[test]
    fn mintime_common_open_dimensions_use_station_minus_one_intervals() {
        let layout = DecisionLayout::new(12, false, 9, 3, 3);

        assert_eq!(layout.dimensions.station_count, 12);
        assert_eq!(layout.dimensions.interval_count, 11);
        assert_eq!(layout.dimensions.state_variable_count, 12 * 9);
        assert_eq!(layout.dimensions.control_variable_count, 11 * 3);
        assert_eq!(
            layout.dimensions.collocation_state_variable_count,
            11 * 3 * 9
        );
        assert_eq!(
            layout.dimensions.decision_variable_count(),
            12 * 9 + 11 * 3 + 11 * 3 * 9
        );
    }

    #[test]
    fn mintime_common_next_station_index_wraps_closed_style() {
        assert_eq!(next_station_index(10, 0), 1);
        assert_eq!(next_station_index(10, 8), 9);
        assert_eq!(next_station_index(10, 9), 0);
        assert_eq!(next_station_index(0, 3), 0);
    }
}
