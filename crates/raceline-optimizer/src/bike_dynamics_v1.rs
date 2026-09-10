use crate::vehicle_dynamics::{BikeSingleTrackLeanParams, TireLoadSensitivityMode};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BikeCountersteerLeanParamsV1 {
    pub base: BikeSingleTrackLeanParams,
    pub steering_inertia_kgm2: f64,
    pub steering_damping_nm_per_radps: f64,
    pub steering_stiffness_nm_per_rad: f64,
    pub steering_torque_max_nm: f64,
    pub pneumatic_trail_m: f64,
    pub roll_steer_gain_nm_per_rad: f64,
}

impl BikeCountersteerLeanParamsV1 {
    #[must_use]
    pub fn from_v05(base: BikeSingleTrackLeanParams) -> Self {
        let steering_inertia_kgm2 = (0.06_f64).max(0.00025 * base.rider_bike_mass_kg);
        let steering_response = base.steering_response_s.max(0.02);
        let steering_damping_nm_per_radps = 2.0 * steering_inertia_kgm2 / steering_response;
        let steering_stiffness_nm_per_rad = steering_inertia_kgm2 / steering_response.powi(2);

        Self {
            base,
            steering_inertia_kgm2,
            steering_damping_nm_per_radps,
            steering_stiffness_nm_per_rad,
            steering_torque_max_nm: 80.0,
            pneumatic_trail_m: 0.045,
            roll_steer_gain_nm_per_rad: 8.0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BikeCountersteerLeanStateV1 {
    pub v_mps: f64,
    pub beta_rad: f64,
    pub omega_z_radps: f64,
    pub n_m: f64,
    pub xi_rad: f64,
    pub phi_rad: f64,
    pub phi_dot_radps: f64,
    pub delta_rad: f64,
    pub delta_dot_radps: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BikeCountersteerLeanControlV1 {
    pub steering_torque_nm: f64,
    pub f_drive_n: f64,
    pub f_brake_n: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BikeCountersteerLeanDynamicsV1 {
    pub dv_ds: f64,
    pub dbeta_ds: f64,
    pub domega_z_ds: f64,
    pub dn_ds: f64,
    pub dxi_ds: f64,
    pub dphi_ds: f64,
    pub dphi_dot_ds: f64,
    pub ddelta_ds: f64,
    pub ddelta_dot_ds: f64,
    pub sigma_dt_ds: f64,
    pub ax_body_mps2: f64,
    pub ay_body_mps2: f64,
    pub ax_mps2: f64,
    pub ay_mps2: f64,
    pub roll_moment_nm: f64,
    pub phi_ddot_radps2: f64,
    pub steering_input_torque_nm: f64,
    pub steering_aligning_torque_nm: f64,
    pub steering_roll_torque_nm: f64,
    pub steering_centering_torque_nm: f64,
    pub steering_damping_torque_nm: f64,
    pub steering_net_torque_nm: f64,
    pub delta_ddot_radps2: f64,
    pub tire_forces: BikeCountersteerLeanTireForcesV1,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BikeCountersteerLeanTimeDynamicsV1 {
    pub dv_dt: f64,
    pub dbeta_dt: f64,
    pub domega_z_dt: f64,
    pub dphi_dt: f64,
    pub dphi_dot_dt: f64,
    pub ddelta_dt: f64,
    pub ddelta_dot_dt: f64,
    pub ax_body_mps2: f64,
    pub ay_body_mps2: f64,
    pub ax_mps2: f64,
    pub ay_mps2: f64,
    pub roll_moment_nm: f64,
    pub phi_ddot_radps2: f64,
    pub steering_input_torque_nm: f64,
    pub steering_aligning_torque_nm: f64,
    pub steering_roll_torque_nm: f64,
    pub steering_centering_torque_nm: f64,
    pub steering_damping_torque_nm: f64,
    pub steering_net_torque_nm: f64,
    pub delta_ddot_radps2: f64,
    pub tire_forces: BikeCountersteerLeanTireForcesV1,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BikeCountersteerLeanTireForcesV1 {
    pub fx_front_n: f64,
    pub fx_rear_n: f64,
    pub fy_front_n: f64,
    pub fy_rear_n: f64,
    pub fz_front_n: f64,
    pub fz_rear_n: f64,
    pub alpha_front_rad: f64,
    pub alpha_rear_rad: f64,
    pub alpha_front_effective_rad: f64,
    pub alpha_rear_effective_rad: f64,
}

#[must_use]
pub fn bike_countersteer_lean_dynamics_v1(
    params: BikeCountersteerLeanParamsV1,
    state: BikeCountersteerLeanStateV1,
    control: BikeCountersteerLeanControlV1,
    kappa_1pm: f64,
) -> BikeCountersteerLeanDynamicsV1 {
    let time = bike_countersteer_lean_time_dynamics_v1(params, state, control);
    let sigma = bike_countersteer_lean_pure_frenet_sigma_dt_ds_v1(state, kappa_1pm);
    let dn_ds = sigma * state.v_mps.max(1e-6) * (state.xi_rad + state.beta_rad).sin();
    bike_countersteer_lean_spatial_dynamics_v1(time, state, kappa_1pm, sigma, dn_ds)
}

#[must_use]
pub fn bike_countersteer_lean_time_dynamics_v1(
    params: BikeCountersteerLeanParamsV1,
    state: BikeCountersteerLeanStateV1,
    control: BikeCountersteerLeanControlV1,
) -> BikeCountersteerLeanTimeDynamicsV1 {
    let base = params.base;
    let v = state.v_mps.max(1e-6);
    let beta = state.beta_rad;
    let omega_z = state.omega_z_radps;
    let delta = state.delta_rad;
    let mass = base.rider_bike_mass_kg;
    let gravity = base.gravity_mps2;

    let drag = base.drag_coeff_n_per_mps2 * v.powi(2);
    let rolling_total = base.rolling_resistance_coeff * mass * gravity;
    let rolling_front = rolling_total * base.wheelbase_rear_m / base.wheelbase_m;
    let rolling_rear = rolling_total * base.wheelbase_front_m / base.wheelbase_m;

    let fz_static_front = mass * gravity * base.wheelbase_rear_m / base.wheelbase_m;
    let fz_static_rear = mass * gravity * base.wheelbase_front_m / base.wheelbase_m;
    let longitudinal_force_for_transfer =
        control.f_drive_n + control.f_brake_n - drag - rolling_total;
    let fz_dyn_front = -base.cg_height_m / base.wheelbase_m * longitudinal_force_for_transfer;
    let fz_dyn_rear = -fz_dyn_front;
    let fz_front = fz_static_front + base.liftcoeff_front * v.powi(2) + fz_dyn_front;
    let fz_rear = fz_static_rear + base.liftcoeff_rear * v.powi(2) + fz_dyn_rear;

    let vx = (v * beta.cos()).signed_max_abs(1e-6);
    let alpha_front = delta - ((v * beta.sin() + base.wheelbase_front_m * omega_z) / vx).atan();
    let alpha_rear = ((-v * beta.sin() + base.wheelbase_rear_m * omega_z) / vx).atan();

    let fy_front = pacejka_lateral_force_n(
        base.lateral_grip_level,
        fz_front,
        base.tire_eps_front,
        base.tire_fz0_front_n,
        base.tire_load_sensitivity_mode,
        base.tire_b_front,
        base.tire_c_front,
        base.tire_e_front,
        alpha_front,
    );
    let fy_rear = pacejka_lateral_force_n(
        base.lateral_grip_level,
        fz_rear,
        base.tire_eps_rear,
        base.tire_fz0_rear_n,
        base.tire_load_sensitivity_mode,
        base.tire_b_rear,
        base.tire_c_rear,
        base.tire_e_rear,
        alpha_rear,
    );

    let fx_front = base.front_brake_bias * control.f_brake_n - rolling_front;
    let fx_rear =
        control.f_drive_n + (1.0 - base.front_brake_bias) * control.f_brake_n - rolling_rear;

    let ax_body = (fx_rear + fx_front * delta.cos() - fy_front * delta.sin() - drag) / mass;
    let ay_body = (fx_front * delta.sin() + fy_rear + fy_front * delta.cos()) / mass;
    let ax = ax_body * beta.cos() + ay_body * beta.sin();
    let ay = -ax_body * beta.sin() + ay_body * beta.cos();

    let dv_dt = (fx_rear * beta.cos() + fx_front * (delta - beta).cos() + fy_rear * beta.sin()
        - fy_front * (delta - beta).sin()
        - drag * beta.cos())
        / mass;
    let dbeta_dt = -omega_z
        + (-fx_rear * beta.sin()
            + fx_front * (delta - beta).sin()
            + fy_rear * beta.cos()
            + fy_front * (delta - beta).cos()
            + drag * beta.sin())
            / (mass * v);
    let domega_z_dt = (-fy_rear * base.wheelbase_rear_m
        + (fy_front * delta.cos() + fx_front * delta.sin()) * base.wheelbase_front_m)
        / base.yaw_inertia_kgm2;

    let roll_moment =
        mass * base.cg_height_m * (ay * state.phi_rad.cos() - gravity * state.phi_rad.sin());
    let phi_ddot = (roll_moment - base.roll_damping * state.phi_dot_radps) / base.roll_inertia_kgm2;

    let steering_input_torque = control.steering_torque_nm;
    let steering_aligning_torque = -params.pneumatic_trail_m * fy_front;
    let steering_roll_torque = params.roll_steer_gain_nm_per_rad * state.phi_rad;
    let steering_centering_torque = -params.steering_stiffness_nm_per_rad * delta;
    let steering_damping_torque = -params.steering_damping_nm_per_radps * state.delta_dot_radps;
    let steering_net_torque = steering_input_torque
        + steering_aligning_torque
        + steering_roll_torque
        + steering_centering_torque
        + steering_damping_torque;
    let delta_ddot = steering_net_torque / params.steering_inertia_kgm2;

    BikeCountersteerLeanTimeDynamicsV1 {
        dv_dt,
        dbeta_dt,
        domega_z_dt,
        dphi_dt: state.phi_dot_radps,
        dphi_dot_dt: phi_ddot,
        ddelta_dt: state.delta_dot_radps,
        ddelta_dot_dt: delta_ddot,
        ax_body_mps2: ax_body,
        ay_body_mps2: ay_body,
        ax_mps2: ax,
        ay_mps2: ay,
        roll_moment_nm: roll_moment,
        phi_ddot_radps2: phi_ddot,
        steering_input_torque_nm: steering_input_torque,
        steering_aligning_torque_nm: steering_aligning_torque,
        steering_roll_torque_nm: steering_roll_torque,
        steering_centering_torque_nm: steering_centering_torque,
        steering_damping_torque_nm: steering_damping_torque,
        steering_net_torque_nm: steering_net_torque,
        delta_ddot_radps2: delta_ddot,
        tire_forces: BikeCountersteerLeanTireForcesV1 {
            fx_front_n: fx_front,
            fx_rear_n: fx_rear,
            fy_front_n: fy_front,
            fy_rear_n: fy_rear,
            fz_front_n: fz_front,
            fz_rear_n: fz_rear,
            alpha_front_rad: alpha_front,
            alpha_rear_rad: alpha_rear,
            alpha_front_effective_rad: alpha_front,
            alpha_rear_effective_rad: alpha_rear,
        },
    }
}

#[must_use]
pub fn bike_countersteer_lean_spatial_dynamics_v1(
    time: BikeCountersteerLeanTimeDynamicsV1,
    state: BikeCountersteerLeanStateV1,
    kappa_1pm: f64,
    sigma_dt_ds: f64,
    dn_ds: f64,
) -> BikeCountersteerLeanDynamicsV1 {
    BikeCountersteerLeanDynamicsV1 {
        dv_ds: sigma_dt_ds * time.dv_dt,
        dbeta_ds: sigma_dt_ds * time.dbeta_dt,
        domega_z_ds: sigma_dt_ds * time.domega_z_dt,
        dn_ds,
        dxi_ds: sigma_dt_ds * state.omega_z_radps - kappa_1pm,
        dphi_ds: sigma_dt_ds * time.dphi_dt,
        dphi_dot_ds: sigma_dt_ds * time.dphi_dot_dt,
        ddelta_ds: sigma_dt_ds * time.ddelta_dt,
        ddelta_dot_ds: sigma_dt_ds * time.ddelta_dot_dt,
        sigma_dt_ds,
        ax_body_mps2: time.ax_body_mps2,
        ay_body_mps2: time.ay_body_mps2,
        ax_mps2: time.ax_mps2,
        ay_mps2: time.ay_mps2,
        roll_moment_nm: time.roll_moment_nm,
        phi_ddot_radps2: time.phi_ddot_radps2,
        steering_input_torque_nm: time.steering_input_torque_nm,
        steering_aligning_torque_nm: time.steering_aligning_torque_nm,
        steering_roll_torque_nm: time.steering_roll_torque_nm,
        steering_centering_torque_nm: time.steering_centering_torque_nm,
        steering_damping_torque_nm: time.steering_damping_torque_nm,
        steering_net_torque_nm: time.steering_net_torque_nm,
        delta_ddot_radps2: time.delta_ddot_radps2,
        tire_forces: time.tire_forces,
    }
}

#[must_use]
pub fn bike_countersteer_lean_pure_frenet_sigma_dt_ds_v1(
    state: BikeCountersteerLeanStateV1,
    kappa_1pm: f64,
) -> f64 {
    let v = state.v_mps.max(1e-6);
    ((1.0 - state.n_m * kappa_1pm)
        / (v * (state.xi_rad + state.beta_rad).cos()).signed_max_abs(1e-6))
    .max(1e-9)
}

fn pacejka_lateral_force_n(
    grip: f64,
    normal_load_n: f64,
    eps: f64,
    fz0_n: f64,
    mode: TireLoadSensitivityMode,
    b: f64,
    c: f64,
    e: f64,
    alpha_rad: f64,
) -> f64 {
    let normal_load_n = normal_load_n.max(1e-6);
    grip * normal_load_n
        * tire_capacity_factor(normal_load_n, eps, fz0_n, mode)
        * (c * (b * alpha_rad - e * (b * alpha_rad - (b * alpha_rad).atan())).atan()).sin()
}

fn tire_capacity_factor(
    normal_load_n: f64,
    eps: f64,
    fz0_n: f64,
    mode: TireLoadSensitivityMode,
) -> f64 {
    let normal_load_n = normal_load_n.max(1e-6);
    let fz0_n = fz0_n.max(1e-6);
    match mode {
        TireLoadSensitivityMode::UpstreamRaw => 1.0 + eps * normal_load_n / fz0_n,
        TireLoadSensitivityMode::ReferenceNormalizedDfz => {
            1.0 + eps * ((normal_load_n - fz0_n) / fz0_n)
        }
    }
    .max(0.05)
}

trait SignedMaxAbs {
    fn signed_max_abs(self, min_abs: f64) -> Self;
}

impl SignedMaxAbs for f64 {
    fn signed_max_abs(self, min_abs: f64) -> Self {
        if self.abs() >= min_abs {
            self
        } else if self.is_sign_negative() {
            -min_abs
        } else {
            min_abs
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::vehicle_dynamics::{VehicleDynamicsModelFamily, VehicleDynamicsProfileV1, G_MPS2};

    fn test_params() -> BikeCountersteerLeanParamsV1 {
        let profile = VehicleDynamicsProfileV1 {
            schema_version: VehicleDynamicsProfileV1::SCHEMA_VERSION.to_owned(),
            profile_id: "bike_dynamics:v1_test".to_owned(),
            model_family: VehicleDynamicsModelFamily::BikeDynamics,
            preset_id: None,
            solver_id: None,
            parameters: vec![
                ("mass".to_owned(), 200.0.into()),
                ("g".to_owned(), G_MPS2.into()),
                ("wheelbase".to_owned(), 1.4.into()),
                ("front_weight_bias".to_owned(), 0.5.into()),
                ("cog_z".to_owned(), 0.6.into()),
                ("I_z".to_owned(), 80.0.into()),
                ("mue".to_owned(), 1.3.into()),
                ("delta_max".to_owned(), 0.8.into()),
                ("phi_max".to_owned(), 1.2.into()),
                ("phi_dot_max".to_owned(), 3.0.into()),
                ("t_delta".to_owned(), 0.1.into()),
                ("strict_product_gates".to_owned(), false.into()),
                (
                    "physics_version".to_owned(),
                    "bike_single_track_lean_v2".into(),
                ),
            ],
            native_parameters: Vec::new(),
            metadata: Vec::new(),
        };
        BikeCountersteerLeanParamsV1::from_v05(
            BikeSingleTrackLeanParams::from_profile(&profile).unwrap(),
        )
    }

    fn straight_state() -> BikeCountersteerLeanStateV1 {
        BikeCountersteerLeanStateV1 {
            v_mps: 20.0,
            beta_rad: 0.0,
            omega_z_radps: 0.0,
            n_m: 0.0,
            xi_rad: 0.0,
            phi_rad: 0.0,
            phi_dot_radps: 0.0,
            delta_rad: 0.0,
            delta_dot_radps: 0.0,
        }
    }

    fn zero_control() -> BikeCountersteerLeanControlV1 {
        BikeCountersteerLeanControlV1 {
            steering_torque_nm: 0.0,
            f_drive_n: 0.0,
            f_brake_n: 0.0,
        }
    }

    fn assert_near(actual: f64, expected: f64, tolerance: f64) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "expected {actual:.12} near {expected:.12} with tolerance {tolerance:.3e}"
        );
    }

    #[test]
    fn v1_straight_line_has_no_fake_ay_roll_or_steering() {
        let dynamics = bike_countersteer_lean_dynamics_v1(
            test_params(),
            straight_state(),
            zero_control(),
            0.0,
        );

        assert!(dynamics.ax_mps2 < 0.0);
        assert!(dynamics.dv_ds < 0.0);
        assert!(dynamics.ay_mps2.abs() < 1e-9);
        assert!(dynamics.roll_moment_nm.abs() < 1e-9);
        assert!(dynamics.phi_ddot_radps2.abs() < 1e-9);
        assert!(dynamics.delta_ddot_radps2.abs() < 1e-9);
    }

    #[test]
    fn v1_positive_steering_torque_accelerates_steering_assembly() {
        let mut control = zero_control();
        control.steering_torque_nm = 5.0;
        let dynamics =
            bike_countersteer_lean_dynamics_v1(test_params(), straight_state(), control, 0.0);

        assert_eq!(dynamics.steering_input_torque_nm, 5.0);
        assert!(dynamics.delta_ddot_radps2 > 0.0);
        assert!(dynamics.ddelta_dot_ds > 0.0);
        assert!(dynamics.ay_mps2.abs() < 1e-9);
    }

    #[test]
    fn v1_time_dynamics_do_not_depend_on_spatial_offset_or_heading_error() {
        let params = test_params();
        let mut shifted = straight_state();
        shifted.n_m = -4.8;
        shifted.xi_rad = 0.35;
        shifted.beta_rad = 0.04;
        shifted.delta_rad = 0.05;

        let mut same_physics = shifted;
        same_physics.n_m = 0.2;
        same_physics.xi_rad = -0.2;

        let left = bike_countersteer_lean_time_dynamics_v1(params, shifted, zero_control());
        let right = bike_countersteer_lean_time_dynamics_v1(params, same_physics, zero_control());

        assert_near(left.dv_dt, right.dv_dt, 1e-12);
        assert_near(left.dbeta_dt, right.dbeta_dt, 1e-12);
        assert_near(left.domega_z_dt, right.domega_z_dt, 1e-12);
        assert_near(left.dphi_dot_dt, right.dphi_dot_dt, 1e-12);
        assert_near(left.ddelta_dot_dt, right.ddelta_dot_dt, 1e-12);
    }

    #[test]
    fn v1_positive_delta_creates_positive_front_slip_ay_and_roll() {
        let mut state = straight_state();
        state.delta_rad = 0.12;
        let dynamics =
            bike_countersteer_lean_dynamics_v1(test_params(), state, zero_control(), 0.0);

        assert!(dynamics.tire_forces.alpha_front_rad > 0.0);
        assert!(dynamics.tire_forces.fy_front_n > 0.0);
        assert!(dynamics.ay_mps2 > 0.0);
        assert!(dynamics.roll_moment_nm > 0.0);
    }

    #[test]
    fn v1_negative_delta_mirrors_positive_delta_lateral_response() {
        let params = test_params();
        let mut positive = straight_state();
        positive.delta_rad = 0.12;
        let mut negative = straight_state();
        negative.delta_rad = -positive.delta_rad;

        let pos = bike_countersteer_lean_dynamics_v1(params, positive, zero_control(), 0.0);
        let neg = bike_countersteer_lean_dynamics_v1(params, negative, zero_control(), 0.0);

        assert!(pos.tire_forces.alpha_front_rad > 0.0);
        assert!(neg.tire_forces.alpha_front_rad < 0.0);
        assert_near(
            neg.tire_forces.alpha_front_rad,
            -pos.tire_forces.alpha_front_rad,
            1e-12,
        );
        assert_near(
            neg.tire_forces.fy_front_n,
            -pos.tire_forces.fy_front_n,
            1e-9,
        );
        assert_near(neg.ay_mps2, -pos.ay_mps2, 1e-9);
        assert_near(neg.roll_moment_nm, -pos.roll_moment_nm, 1e-7);
        assert_near(
            neg.steering_aligning_torque_nm,
            -pos.steering_aligning_torque_nm,
            1e-10,
        );
        assert_near(
            neg.steering_centering_torque_nm,
            -pos.steering_centering_torque_nm,
            1e-10,
        );
        assert_near(neg.delta_ddot_radps2, -pos.delta_ddot_radps2, 1e-9);
    }

    #[test]
    fn v1_aligning_torque_opposes_front_slip() {
        let mut state = straight_state();
        state.delta_rad = 0.12;
        let dynamics =
            bike_countersteer_lean_dynamics_v1(test_params(), state, zero_control(), 0.0);

        assert!(dynamics.tire_forces.alpha_front_rad > 0.0);
        assert!(dynamics.steering_aligning_torque_nm < 0.0);
        assert!(dynamics.steering_centering_torque_nm < 0.0);
        assert!(dynamics.delta_ddot_radps2 < 0.0);
    }

    #[test]
    fn v1_aligning_torque_opposes_positive_and_negative_front_slip() {
        let params = test_params();
        for delta in [0.12, -0.12] {
            let mut state = straight_state();
            state.delta_rad = delta;
            let dynamics = bike_countersteer_lean_dynamics_v1(params, state, zero_control(), 0.0);

            assert!(
                dynamics.tire_forces.alpha_front_rad * dynamics.steering_aligning_torque_nm < 0.0,
                "aligning torque must oppose front slip for delta={delta}"
            );
        }
    }

    #[test]
    fn v1_centering_torque_opposes_positive_and_negative_delta() {
        let params = test_params();
        for delta in [0.20, -0.20] {
            let mut state = straight_state();
            state.delta_rad = delta;
            let dynamics = bike_countersteer_lean_dynamics_v1(params, state, zero_control(), 0.0);

            assert!(
                state.delta_rad * dynamics.steering_centering_torque_nm < 0.0,
                "centering torque must oppose delta={delta}"
            );
        }
    }

    #[test]
    fn v1_damping_torque_opposes_positive_and_negative_delta_dot() {
        let params = test_params();
        for delta_dot in [1.5, -1.5] {
            let mut state = straight_state();
            state.delta_dot_radps = delta_dot;
            let dynamics = bike_countersteer_lean_dynamics_v1(params, state, zero_control(), 0.0);

            assert!(
                state.delta_dot_radps * dynamics.steering_damping_torque_nm < 0.0,
                "steering damping torque must oppose delta_dot={delta_dot}"
            );
            assert!(
                state.delta_dot_radps * dynamics.delta_ddot_radps2 < 0.0,
                "damping-only delta acceleration must oppose delta_dot={delta_dot}"
            );
        }
    }

    #[test]
    fn v1_positive_lean_produces_self_steering_torque_into_lean() {
        let mut state = straight_state();
        state.phi_rad = 0.2;
        let dynamics =
            bike_countersteer_lean_dynamics_v1(test_params(), state, zero_control(), 0.0);

        assert!(dynamics.steering_roll_torque_nm > 0.0);
        assert!(dynamics.delta_ddot_radps2 > 0.0);
        assert!(dynamics.roll_moment_nm < 0.0);
    }

    #[test]
    fn v1_negative_lean_mirrors_positive_lean_roll_and_steering_feedback() {
        let params = test_params();
        let mut positive = straight_state();
        positive.phi_rad = 0.2;
        let mut negative = straight_state();
        negative.phi_rad = -positive.phi_rad;

        let pos = bike_countersteer_lean_dynamics_v1(params, positive, zero_control(), 0.0);
        let neg = bike_countersteer_lean_dynamics_v1(params, negative, zero_control(), 0.0);

        assert_near(
            neg.steering_roll_torque_nm,
            -pos.steering_roll_torque_nm,
            1e-12,
        );
        assert_near(neg.roll_moment_nm, -pos.roll_moment_nm, 1e-9);
        assert_near(neg.phi_ddot_radps2, -pos.phi_ddot_radps2, 1e-9);
        assert_near(neg.delta_ddot_radps2, -pos.delta_ddot_radps2, 1e-9);
    }

    #[test]
    fn v1_braking_transfers_load_to_front_and_unloads_rear() {
        let params = test_params();
        let baseline =
            bike_countersteer_lean_dynamics_v1(params, straight_state(), zero_control(), 0.0);
        let mut control = zero_control();
        control.f_brake_n = -700.0;
        let braking = bike_countersteer_lean_dynamics_v1(params, straight_state(), control, 0.0);

        assert!(braking.tire_forces.fz_front_n > baseline.tire_forces.fz_front_n);
        assert!(braking.tire_forces.fz_rear_n < baseline.tire_forces.fz_rear_n);
        assert_near(
            braking.tire_forces.fz_front_n - baseline.tire_forces.fz_front_n,
            -(braking.tire_forces.fz_rear_n - baseline.tire_forces.fz_rear_n),
            1e-9,
        );
    }

    #[test]
    fn v1_drive_transfers_load_to_rear_and_unloads_front() {
        let params = test_params();
        let baseline =
            bike_countersteer_lean_dynamics_v1(params, straight_state(), zero_control(), 0.0);
        let mut control = zero_control();
        control.f_drive_n = 700.0;
        let driving = bike_countersteer_lean_dynamics_v1(params, straight_state(), control, 0.0);

        assert!(driving.tire_forces.fz_front_n < baseline.tire_forces.fz_front_n);
        assert!(driving.tire_forces.fz_rear_n > baseline.tire_forces.fz_rear_n);
        assert_near(
            driving.tire_forces.fz_front_n - baseline.tire_forces.fz_front_n,
            -(driving.tire_forces.fz_rear_n - baseline.tire_forces.fz_rear_n),
            1e-9,
        );
    }

    #[test]
    fn v1_steady_turn_at_phi_atan_ay_over_g_has_no_roll_accel() {
        let params = test_params();
        let mut state = straight_state();
        state.v_mps = 18.0;
        state.delta_rad = 0.18;
        state.omega_z_radps = 0.20;

        let unleaned = bike_countersteer_lean_dynamics_v1(params, state, zero_control(), 0.0);
        assert!(unleaned.ay_mps2 > 0.0);
        assert!(unleaned.roll_moment_nm > 0.0);

        state.phi_rad = (unleaned.ay_mps2 / params.base.gravity_mps2).atan();
        let leaned = bike_countersteer_lean_dynamics_v1(params, state, zero_control(), 0.0);

        assert!(state.phi_rad > 0.0);
        assert!(leaned.roll_moment_nm.abs() < 1e-9);
        assert!(leaned.phi_ddot_radps2.abs() < 1e-9);
    }
}
