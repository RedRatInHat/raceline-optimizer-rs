use crate::json::JsonValue;
use crate::{JsonObject, ToJsonValue};

pub const G_MPS2: f64 = 9.80665;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VehicleDynamicsModelFamily {
    CarDynamics,
    BikeDynamics,
}

impl VehicleDynamicsModelFamily {
    pub fn parse(value: &str) -> Result<Self, String> {
        match value {
            "car_dynamics" => Ok(Self::CarDynamics),
            "bike_dynamics" => Ok(Self::BikeDynamics),
            _ => Err(format!(
                "unsupported vehicle dynamics model_family: {value}"
            )),
        }
    }

    #[must_use]
    pub fn as_str(self) -> &'static str {
        match self {
            Self::CarDynamics => "car_dynamics",
            Self::BikeDynamics => "bike_dynamics",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TireLoadSensitivityMode {
    UpstreamRaw,
    ReferenceNormalizedDfz,
}

impl TireLoadSensitivityMode {
    #[must_use]
    pub fn from_profile(profile: &VehicleDynamicsProfileV1) -> Self {
        match profile
            .string_param("tire_load_sensitivity_mode")
            .or_else(|| profile.string_param("lambda_mode"))
            .map(|value| value.trim().to_ascii_lowercase())
            .as_deref()
        {
            Some("reference_normalized_dfz" | "reference_dfz" | "dfz") => {
                Self::ReferenceNormalizedDfz
            }
            Some("upstream_raw" | "raw" | "legacy_raw") | None => Self::UpstreamRaw,
            Some(_) => Self::UpstreamRaw,
        }
    }

    #[must_use]
    pub fn as_str(self) -> &'static str {
        match self {
            Self::UpstreamRaw => "upstream_raw",
            Self::ReferenceNormalizedDfz => "reference_normalized_dfz",
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct VehicleDynamicsProfileV1 {
    pub schema_version: String,
    pub profile_id: String,
    pub model_family: VehicleDynamicsModelFamily,
    pub preset_id: Option<String>,
    pub solver_id: Option<String>,
    pub parameters: JsonObject,
    pub native_parameters: JsonObject,
    pub metadata: JsonObject,
}

impl VehicleDynamicsProfileV1 {
    pub const SCHEMA_VERSION: &'static str = "vehicle_dynamics_profile.v1";

    pub fn from_json(value: &JsonValue) -> Result<Self, String> {
        let schema_version = required_string(value, "schema_version")?;
        if schema_version != Self::SCHEMA_VERSION {
            return Err(format!(
                "vehicle_dynamics_profile schema_version must be {}",
                Self::SCHEMA_VERSION
            ));
        }

        Ok(Self {
            schema_version,
            profile_id: required_string(value, "profile_id")?,
            model_family: VehicleDynamicsModelFamily::parse(&required_string(
                value,
                "model_family",
            )?)?,
            preset_id: optional_string(value, "preset_id"),
            solver_id: optional_string(value, "solver_id"),
            parameters: optional_object(value, "parameters"),
            native_parameters: optional_object(value, "native_parameters"),
            metadata: optional_object(value, "metadata"),
        })
    }

    #[must_use]
    pub fn numeric_param(&self, key: &str) -> Option<f64> {
        object_f64(&self.parameters, key)
    }

    #[must_use]
    pub fn string_param(&self, key: &str) -> Option<&str> {
        object_str(&self.parameters, key)
    }
}

impl ToJsonValue for VehicleDynamicsProfileV1 {
    fn to_json_value(&self) -> JsonValue {
        JsonValue::Object(vec![
            (
                "schema_version".to_owned(),
                self.schema_version.clone().into(),
            ),
            ("profile_id".to_owned(), self.profile_id.clone().into()),
            ("model_family".to_owned(), self.model_family.as_str().into()),
            (
                "preset_id".to_owned(),
                self.preset_id
                    .clone()
                    .map_or(JsonValue::Null, JsonValue::from),
            ),
            (
                "solver_id".to_owned(),
                self.solver_id
                    .clone()
                    .map_or(JsonValue::Null, JsonValue::from),
            ),
            (
                "parameters".to_owned(),
                JsonValue::Object(self.parameters.clone()),
            ),
            (
                "native_parameters".to_owned(),
                JsonValue::Object(self.native_parameters.clone()),
            ),
            (
                "metadata".to_owned(),
                JsonValue::Object(self.metadata.clone()),
            ),
        ])
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CarDoubleTrackParams {
    pub max_speed_mps: f64,
    pub mass_kg: f64,
    pub gravity_mps2: f64,
    pub wheelbase_m: f64,
    pub cg_to_front_axle_m: f64,
    pub cg_height_m: f64,
    pub track_width_front_m: f64,
    pub track_width_rear_m: f64,
    pub grip_level: f64,
    pub longitudinal_grip_level: f64,
    pub drive_grip_level: f64,
    pub lateral_grip_level: f64,
    pub brake_grip_level: f64,
    pub steering_angle_max_rad: f64,
    pub drive_force_max_n: f64,
    pub brake_force_max_n: f64,
    pub steering_response_s: f64,
    pub throttle_response_s: f64,
    pub brake_response_s: f64,
    pub yaw_inertia_kgm2: f64,
    pub drag_coeff_n_per_mps2: f64,
    pub rolling_resistance_coeff: f64,
    pub roll_stiffness_distribution: f64,
    pub drive_front_fraction: f64,
    pub brake_front_fraction: f64,
    pub power_max_w: f64,
    pub tire_b_front: f64,
    pub tire_c_front: f64,
    pub tire_e_front: f64,
    pub tire_eps_front: f64,
    pub tire_b_rear: f64,
    pub tire_c_rear: f64,
    pub tire_e_rear: f64,
    pub tire_eps_rear: f64,
    pub tire_fz0_n: f64,
    pub tire_fz0_front_n: f64,
    pub tire_fz0_rear_n: f64,
    pub tire_load_sensitivity_mode: TireLoadSensitivityMode,
    pub liftcoeff_front: f64,
    pub liftcoeff_rear: f64,
}

impl CarDoubleTrackParams {
    pub fn from_profile(profile: &VehicleDynamicsProfileV1) -> Result<Self, String> {
        if profile.model_family != VehicleDynamicsModelFamily::CarDynamics {
            return Err("car_double_track params require car_dynamics profile".to_owned());
        }

        let max_speed_mps = positive_param_any(profile, &["max_speed_mps", "v_max_mps"], 70.0)?;
        let mass_kg = positive_param(profile, "mass_kg", 1180.0)?;
        let gravity_mps2 = positive_param_any(profile, &["gravity_mps2", "g"], G_MPS2)?;
        let wheelbase_front_m = profile.numeric_param("wheelbase_front");
        let wheelbase_rear_m = profile.numeric_param("wheelbase_rear");
        let wheelbase_m = profile
            .numeric_param("wheelbase_m")
            .or_else(|| Some(wheelbase_front_m? + wheelbase_rear_m?))
            .unwrap_or(2.65);
        let tire_fz0_n = positive_param_any(
            profile,
            &["tire_fz0_n", "f_z0"],
            (mass_kg * gravity_mps2 / 4.0).max(1.0),
        )?;

        let grip_level = positive_param_any(profile, &["grip_level", "mue"], 1.1)?;
        let longitudinal_grip_level = positive_param_any(
            profile,
            &["longitudinal_grip_level", "mu_x", "mue_x"],
            grip_level,
        )?;
        let drive_grip_level = positive_param_any(
            profile,
            &["drive_grip_level", "drive_mu", "mue_drive"],
            longitudinal_grip_level,
        )?;
        let lateral_grip_level = positive_param_any(
            profile,
            &["lateral_grip_level", "mu_y", "mue_y"],
            grip_level,
        )?;
        let brake_grip_level = positive_param_any(
            profile,
            &["brake_grip_level", "brake_mu", "mue_brake"],
            longitudinal_grip_level,
        )?;

        Ok(Self {
            max_speed_mps,
            mass_kg,
            gravity_mps2,
            wheelbase_m,
            cg_to_front_axle_m: profile
                .numeric_param("cg_to_front_axle_m")
                .or(wheelbase_front_m)
                .unwrap_or(wheelbase_m * 0.5),
            cg_height_m: positive_param_any(profile, &["cg_height_m", "cog_z"], 0.55)?,
            track_width_front_m: positive_param_any(
                profile,
                &["track_width_front_m", "track_width_front"],
                1.6,
            )?,
            track_width_rear_m: positive_param_any(
                profile,
                &["track_width_rear_m", "track_width_rear"],
                1.6,
            )?,
            grip_level,
            longitudinal_grip_level,
            drive_grip_level,
            lateral_grip_level,
            brake_grip_level,
            steering_angle_max_rad: positive_param_any(
                profile,
                &["delta_max_rad", "delta_max"],
                0.7,
            )?,
            drive_force_max_n: positive_param_any(
                profile,
                &["f_drive_max_n", "f_drive_max"],
                mass_kg * gravity_mps2 * 0.6,
            )?,
            brake_force_max_n: positive_param_any(
                profile,
                &["f_brake_max_n", "f_brake_max"],
                mass_kg * gravity_mps2 * 1.2,
            )?,
            steering_response_s: positive_param_any(
                profile,
                &["steering_response_s", "t_delta"],
                0.08,
            )?,
            throttle_response_s: positive_param_any(
                profile,
                &["throttle_response_s", "t_drive"],
                0.08,
            )?,
            brake_response_s: positive_param_any(profile, &["brake_response_s", "t_brake"], 0.07)?,
            yaw_inertia_kgm2: positive_param_any(
                profile,
                &["yaw_inertia_kgm2", "I_z"],
                mass_kg * wheelbase_m.powi(2) * 0.28,
            )?,
            drag_coeff_n_per_mps2: profile
                .numeric_param("drag_coeff_n_per_mps2")
                .or_else(|| profile.numeric_param("dragcoeff"))
                .unwrap_or(0.35),
            rolling_resistance_coeff: profile
                .numeric_param("rolling_resistance_coeff")
                .or_else(|| profile.numeric_param("rolling_resistance"))
                .or_else(|| profile.numeric_param("c_roll"))
                .unwrap_or(0.013),
            roll_stiffness_distribution: profile
                .numeric_param("roll_stiffness_distribution")
                .or_else(|| profile.numeric_param("k_roll"))
                .unwrap_or(0.5)
                .clamp(0.0, 1.0),
            drive_front_fraction: profile
                .numeric_param("drive_front_fraction")
                .or_else(|| profile.numeric_param("k_drive_front"))
                .unwrap_or(0.0)
                .clamp(0.0, 1.0),
            brake_front_fraction: profile
                .numeric_param("front_brake_bias")
                .or_else(|| profile.numeric_param("brake_bias_front"))
                .or_else(|| profile.numeric_param("k_brake_front"))
                .unwrap_or(0.65)
                .clamp(0.0, 1.0),
            power_max_w: positive_param(
                profile,
                "power_max_w",
                profile
                    .numeric_param("power_max")
                    .or_else(|| {
                        profile
                            .numeric_param("power_kw")
                            .map(|value| value * 1000.0)
                    })
                    .unwrap_or(mass_kg * gravity_mps2 * 45.0),
            )?,
            tire_b_front: positive_param_any(profile, &["tire_b_front", "B_front"], 10.0)?,
            tire_c_front: positive_param_any(profile, &["tire_c_front", "C_front"], 1.9)?,
            tire_e_front: profile
                .numeric_param("tire_e_front")
                .or_else(|| profile.numeric_param("E_front"))
                .unwrap_or(0.97),
            tire_eps_front: profile
                .numeric_param("tire_eps_front")
                .or_else(|| profile.numeric_param("eps_front"))
                .unwrap_or(0.0),
            tire_b_rear: positive_param_any(profile, &["tire_b_rear", "B_rear"], 10.0)?,
            tire_c_rear: positive_param_any(profile, &["tire_c_rear", "C_rear"], 1.9)?,
            tire_e_rear: profile
                .numeric_param("tire_e_rear")
                .or_else(|| profile.numeric_param("E_rear"))
                .unwrap_or(0.97),
            tire_eps_rear: profile
                .numeric_param("tire_eps_rear")
                .or_else(|| profile.numeric_param("eps_rear"))
                .unwrap_or(0.0),
            tire_fz0_n,
            tire_fz0_front_n: positive_param_any(
                profile,
                &["tire_fz0_front_n", "f_z0_front", "fz0_front"],
                tire_fz0_n,
            )?,
            tire_fz0_rear_n: positive_param_any(
                profile,
                &["tire_fz0_rear_n", "f_z0_rear", "fz0_rear"],
                tire_fz0_n,
            )?,
            tire_load_sensitivity_mode: TireLoadSensitivityMode::from_profile(profile),
            liftcoeff_front: profile.numeric_param("liftcoeff_front").unwrap_or(0.0),
            liftcoeff_rear: profile.numeric_param("liftcoeff_rear").unwrap_or(0.0),
        })
    }

    #[must_use]
    pub fn static_axle_loads_n(self) -> AxleLoadsN {
        let front_fraction =
            ((self.wheelbase_m - self.cg_to_front_axle_m) / self.wheelbase_m).clamp(0.0, 1.0);
        let total = self.mass_kg * self.gravity_mps2;
        AxleLoadsN {
            front_n: total * front_fraction,
            rear_n: total * (1.0 - front_fraction),
        }
    }

    #[must_use]
    pub fn tire_reference_load_n(self, wheel: &str) -> f64 {
        match wheel {
            "fl" | "fr" | "front" => self.tire_fz0_front_n,
            "rl" | "rr" | "rear" => self.tire_fz0_rear_n,
            _ => self.tire_fz0_n,
        }
    }

    #[must_use]
    pub fn longitudinal_load_transfer_n(self, ax_mps2: f64) -> f64 {
        self.mass_kg * ax_mps2 * self.cg_height_m / self.wheelbase_m
    }

    #[must_use]
    pub fn lateral_load_transfer_n(self, ay_mps2: f64, front_fraction: f64) -> AxleLoadsN {
        let front_fraction = front_fraction.clamp(0.0, 1.0);
        let front =
            self.mass_kg * ay_mps2 * self.cg_height_m * front_fraction / self.track_width_front_m;
        let rear = self.mass_kg * ay_mps2 * self.cg_height_m * (1.0 - front_fraction)
            / self.track_width_rear_m;
        AxleLoadsN {
            front_n: front,
            rear_n: rear,
        }
    }

    #[must_use]
    pub fn aero_vertical_load_n(self, speed_mps: f64) -> AxleLoadsN {
        let q = speed_mps.max(0.0).powi(2);
        AxleLoadsN {
            front_n: -self.liftcoeff_front * q,
            rear_n: -self.liftcoeff_rear * q,
        }
    }

    #[must_use]
    pub fn wheelbase_front_m(self) -> f64 {
        self.cg_to_front_axle_m
    }

    #[must_use]
    pub fn wheelbase_rear_m(self) -> f64 {
        (self.wheelbase_m - self.cg_to_front_axle_m).max(1e-9)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CarDoubleTrackState {
    pub v_mps: f64,
    pub beta_rad: f64,
    pub omega_z_radps: f64,
    pub n_m: f64,
    pub xi_rad: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CarDoubleTrackControl {
    pub delta_rad: f64,
    pub f_drive_n: f64,
    pub f_brake_n: f64,
    pub gamma_y_n: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CarDoubleTrackDynamics {
    pub dv_ds: f64,
    pub dbeta_ds: f64,
    pub domega_z_ds: f64,
    pub dn_ds: f64,
    pub dxi_ds: f64,
    pub sigma_dt_ds: f64,
    pub ax_mps2: f64,
    pub ay_mps2: f64,
    pub tire_forces: CarDoubleTrackTireForces,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CarDoubleTrackTireForces {
    pub fx_fl_n: f64,
    pub fx_fr_n: f64,
    pub fx_rl_n: f64,
    pub fx_rr_n: f64,
    pub fy_fl_n: f64,
    pub fy_fr_n: f64,
    pub fy_rl_n: f64,
    pub fy_rr_n: f64,
    pub fz_fl_n: f64,
    pub fz_fr_n: f64,
    pub fz_rl_n: f64,
    pub fz_rr_n: f64,
    pub alpha_fl_rad: f64,
    pub alpha_fr_rad: f64,
    pub alpha_rl_rad: f64,
    pub alpha_rr_rad: f64,
}

impl CarDoubleTrackTireForces {
    #[must_use]
    pub fn wheel_ellipse_utilization(self, params: CarDoubleTrackParams, wheel: &str) -> f64 {
        let (fx, fy, fz, eps) = match wheel {
            "fl" => (
                self.fx_fl_n,
                self.fy_fl_n,
                self.fz_fl_n,
                params.tire_eps_front,
            ),
            "fr" => (
                self.fx_fr_n,
                self.fy_fr_n,
                self.fz_fr_n,
                params.tire_eps_front,
            ),
            "rl" => (
                self.fx_rl_n,
                self.fy_rl_n,
                self.fz_rl_n,
                params.tire_eps_rear,
            ),
            "rr" => (
                self.fx_rr_n,
                self.fy_rr_n,
                self.fz_rr_n,
                params.tire_eps_rear,
            ),
            _ => return 0.0,
        };
        let longitudinal_capacity = directional_longitudinal_tire_capacity_n(
            params.drive_grip_level,
            params.brake_grip_level,
            fx,
            fz,
        );
        let lateral_capacity = tire_load_sensitive_capacity_n(
            params.lateral_grip_level,
            fz,
            eps,
            params.tire_reference_load_n(wheel),
            params.tire_load_sensitivity_mode,
        );
        (fx / longitudinal_capacity).powi(2) + (fy / lateral_capacity).powi(2)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BikeSingleTrackLeanParams {
    pub rider_bike_mass_kg: f64,
    pub max_speed_mps: f64,
    pub gravity_mps2: f64,
    pub wheelbase_m: f64,
    pub wheelbase_front_m: f64,
    pub wheelbase_rear_m: f64,
    pub cg_height_m: f64,
    pub yaw_inertia_kgm2: f64,
    pub drag_coeff_n_per_mps2: f64,
    pub rolling_resistance_coeff: f64,
    pub grip_level: f64,
    pub longitudinal_grip_level: f64,
    pub drive_grip_level: f64,
    pub lateral_grip_level: f64,
    pub brake_grip_level: f64,
    pub front_brake_bias: f64,
    pub steering_response_s: f64,
    pub throttle_response_s: f64,
    pub brake_response_s: f64,
    pub lean_response_s: f64,
    pub power_max_w: f64,
    pub drive_force_max_n: f64,
    pub brake_force_max_n: f64,
    pub steering_angle_max_rad: f64,
    pub lean_angle_max_rad: f64,
    pub lean_rate_max_radps: f64,
    pub beta_max_rad: f64,
    pub alpha_front_max_rad: f64,
    pub alpha_rear_max_rad: f64,
    pub xi_max_rad: f64,
    pub min_normal_force_n: f64,
    pub min_normal_force_static_fraction: f64,
    pub dt_ds_min: f64,
    pub dt_ds_max: f64,
    pub tire_b_front: f64,
    pub tire_c_front: f64,
    pub tire_e_front: f64,
    pub tire_eps_front: f64,
    pub tire_b_rear: f64,
    pub tire_c_rear: f64,
    pub tire_e_rear: f64,
    pub tire_eps_rear: f64,
    pub tire_fz0_n: f64,
    pub tire_fz0_front_n: f64,
    pub tire_fz0_rear_n: f64,
    pub tire_load_sensitivity_mode: TireLoadSensitivityMode,
    pub tire_min_capacity_factor: f64,
    pub liftcoeff_front: f64,
    pub liftcoeff_rear: f64,
    pub roll_inertia_kgm2: f64,
    pub roll_tau_max_nm: f64,
    pub roll_rate_servo_gain: f64,
    pub roll_damping: f64,
    pub physics_version_v2: bool,
    pub strict_product_gates: bool,
}

impl BikeSingleTrackLeanParams {
    pub fn from_profile(profile: &VehicleDynamicsProfileV1) -> Result<Self, String> {
        if profile.model_family != VehicleDynamicsModelFamily::BikeDynamics {
            return Err("bike_single_track params require bike_dynamics profile".to_owned());
        }

        let mass_kg =
            positive_param_any(profile, &["rider_bike_mass_kg", "mass_kg", "mass"], 215.0)?;
        let gravity_mps2 = positive_param_any(profile, &["gravity_mps2", "g"], G_MPS2)?;
        let max_speed_mps =
            positive_param_any(profile, &["max_speed_mps", "v_max_mps", "v_max"], 52.0)?;
        let wheelbase_m = positive_param_any(profile, &["wheelbase_m", "wheelbase"], 1.42)?;
        let front_weight_bias = profile.numeric_param("front_weight_bias").unwrap_or(0.48);
        let wheelbase_front_m = positive_param_any(
            profile,
            &["wheelbase_front_m", "wheelbase_front"],
            wheelbase_m * (1.0 - front_weight_bias),
        )?;
        let wheelbase_rear_m = positive_param_any(
            profile,
            &["wheelbase_rear_m", "wheelbase_rear"],
            wheelbase_m * front_weight_bias,
        )?;
        let wheelbase_m = wheelbase_front_m + wheelbase_rear_m;
        let cg_height_m = positive_param_any(profile, &["cg_height_m", "cog_z"], 0.58)?;
        let lean_angle_max_rad = positive_param_any(
            profile,
            &[
                "lean_angle_max_rad",
                "phi_max_rad",
                "phi_max",
                "lean_capability",
            ],
            1.05,
        )?;
        let lean_rate_max_radps = positive_param_any(
            profile,
            &["lean_rate_max_radps", "roll_rate_max_rad_s", "phi_dot_max"],
            2.4,
        )?;
        let lean_response_s = positive_param_any(profile, &["lean_response_s", "t_lean"], 0.10)?;
        let yaw_inertia_kgm2 = positive_param_any(
            profile,
            &["yaw_inertia_kgm2", "I_z"],
            (45.0_f64).max(0.45 * mass_kg * wheelbase_m.powi(2)),
        )?;
        let power_max_w = positive_param_any(
            profile,
            &["power_max_w", "power_max"],
            profile
                .numeric_param("power_kw")
                .map(|value| value * 1000.0)
                .unwrap_or(28_000.0),
        )?;
        let physics_version_v2 = profile
            .string_param("physics_version")
            .is_none_or(|version| version == "bike_single_track_lean_v2");
        let strict_product_gates =
            object_bool(&profile.parameters, "strict_product_gates").unwrap_or(physics_version_v2);
        let mut beta_max_rad = positive_param(profile, "beta_max", 0.35)?;
        let mut alpha_front_max_rad = positive_param(profile, "alpha_front_max", 0.35)?;
        let mut alpha_rear_max_rad = positive_param(profile, "alpha_rear_max", 0.35)?;
        let mut xi_max_rad = positive_param(profile, "xi_max", std::f64::consts::FRAC_PI_2)?;
        let mut min_normal_force_static_fraction =
            positive_param(profile, "min_normal_force_static_fraction", 0.10)?;
        let mut dt_ds_min = positive_param(profile, "dt_ds_min", 1e-5)?;
        let mut dt_ds_max = positive_param(profile, "dt_ds_max", 2.0)?;
        if physics_version_v2 && strict_product_gates {
            beta_max_rad = beta_max_rad.min(0.28);
            alpha_front_max_rad = alpha_front_max_rad.min(0.28);
            alpha_rear_max_rad = alpha_rear_max_rad.min(0.32);
            xi_max_rad = xi_max_rad.min(0.75);
            min_normal_force_static_fraction = min_normal_force_static_fraction.max(0.22);
            dt_ds_min = dt_ds_min.max(1e-5);
            dt_ds_max = dt_ds_max.min(2.0);
        }
        let roll_inertia_kgm2 = positive_param_any(
            profile,
            &["roll_inertia_kgm2", "I_roll"],
            (5.0_f64).max(0.35 * mass_kg * cg_height_m.powi(2)),
        )?;
        let roll_tau_max_nm = positive_param_any(
            profile,
            &["roll_tau_max_nm", "roll_tau_max"],
            (200.0_f64).max(roll_inertia_kgm2 * lean_rate_max_radps / lean_response_s.max(0.05)),
        )?;
        let roll_rate_servo_gain = positive_param(
            profile,
            "roll_rate_servo_gain",
            roll_tau_max_nm / lean_rate_max_radps.max(1e-3),
        )?;

        let tire_fz0_n = positive_param_any(
            profile,
            &["tire_fz0_n", "f_z0"],
            (250.0_f64).max(0.5 * mass_kg * gravity_mps2),
        )?;

        let grip_level = positive_param_any(profile, &["grip_level", "mue"], 0.9)?;
        let longitudinal_grip_level = positive_param_any(
            profile,
            &["longitudinal_grip_level", "mu_x", "mue_x"],
            grip_level,
        )?;
        let drive_grip_level = positive_param_any(
            profile,
            &["drive_grip_level", "drive_mu", "mue_drive"],
            longitudinal_grip_level,
        )?;
        let lateral_grip_level = positive_param_any(
            profile,
            &["lateral_grip_level", "mu_y", "mue_y"],
            grip_level,
        )?;
        let brake_grip_level = positive_param_any(
            profile,
            &["brake_grip_level", "brake_mu", "mue_brake"],
            longitudinal_grip_level,
        )?;

        Ok(Self {
            rider_bike_mass_kg: mass_kg,
            max_speed_mps,
            gravity_mps2,
            wheelbase_m,
            wheelbase_front_m,
            wheelbase_rear_m,
            cg_height_m,
            yaw_inertia_kgm2,
            drag_coeff_n_per_mps2: profile
                .numeric_param("drag_coeff_n_per_mps2")
                .or_else(|| profile.numeric_param("dragcoeff"))
                .unwrap_or(0.44),
            rolling_resistance_coeff: profile
                .numeric_param("rolling_resistance_coeff")
                .or_else(|| profile.numeric_param("c_roll"))
                .unwrap_or(0.015),
            grip_level,
            longitudinal_grip_level,
            drive_grip_level,
            lateral_grip_level,
            brake_grip_level,
            front_brake_bias: profile
                .numeric_param("front_brake_bias")
                .unwrap_or(0.76)
                .clamp(0.0, 1.0),
            steering_response_s: positive_param_any(
                profile,
                &["steering_response_s", "t_delta"],
                0.08,
            )?,
            throttle_response_s: positive_param_any(
                profile,
                &["throttle_response_s", "t_drive"],
                0.08,
            )?,
            brake_response_s: positive_param_any(profile, &["brake_response_s", "t_brake"], 0.07)?,
            lean_response_s,
            power_max_w,
            drive_force_max_n: positive_param_any(
                profile,
                &["drive_force_max_n", "f_drive_max"],
                2500.0,
            )?,
            brake_force_max_n: positive_param_any(
                profile,
                &["brake_force_max_n", "f_brake_max"],
                5200.0,
            )?,
            steering_angle_max_rad: positive_param_any(
                profile,
                &["steering_angle_max_rad", "delta_max"],
                0.85,
            )?,
            lean_angle_max_rad,
            lean_rate_max_radps,
            beta_max_rad,
            alpha_front_max_rad,
            alpha_rear_max_rad,
            xi_max_rad,
            min_normal_force_n: positive_param(profile, "min_normal_force", 25.0)?,
            min_normal_force_static_fraction,
            dt_ds_min,
            dt_ds_max,
            tire_b_front: positive_param_any(profile, &["tire_b_front", "B_front"], 10.0)?,
            tire_c_front: positive_param_any(profile, &["tire_c_front", "C_front"], 2.2)?,
            tire_e_front: profile
                .numeric_param("tire_e_front")
                .or_else(|| profile.numeric_param("E_front"))
                .unwrap_or(1.0),
            tire_eps_front: profile
                .numeric_param("tire_eps_front")
                .or_else(|| profile.numeric_param("eps_front"))
                .unwrap_or(-0.10),
            tire_b_rear: positive_param_any(profile, &["tire_b_rear", "B_rear"], 10.0)?,
            tire_c_rear: positive_param_any(profile, &["tire_c_rear", "C_rear"], 2.2)?,
            tire_e_rear: profile
                .numeric_param("tire_e_rear")
                .or_else(|| profile.numeric_param("E_rear"))
                .unwrap_or(1.0),
            tire_eps_rear: profile
                .numeric_param("tire_eps_rear")
                .or_else(|| profile.numeric_param("eps_rear"))
                .unwrap_or(-0.10),
            tire_fz0_n,
            tire_fz0_front_n: positive_param_any(
                profile,
                &["tire_fz0_front_n", "f_z0_front", "fz0_front"],
                tire_fz0_n,
            )?,
            tire_fz0_rear_n: positive_param_any(
                profile,
                &["tire_fz0_rear_n", "f_z0_rear", "fz0_rear"],
                tire_fz0_n,
            )?,
            tire_load_sensitivity_mode: TireLoadSensitivityMode::from_profile(profile),
            tire_min_capacity_factor: positive_param(profile, "min_capacity_factor", 0.05)?,
            liftcoeff_front: profile.numeric_param("liftcoeff_front").unwrap_or(0.0),
            liftcoeff_rear: profile.numeric_param("liftcoeff_rear").unwrap_or(0.0),
            roll_inertia_kgm2,
            roll_tau_max_nm,
            roll_rate_servo_gain,
            roll_damping: positive_param(profile, "roll_damping", 0.05 * roll_rate_servo_gain)?,
            physics_version_v2,
            strict_product_gates,
        })
    }

    #[must_use]
    pub fn lean_limited_lateral_accel_mps2(self) -> f64 {
        self.gravity_mps2 * self.lean_angle_max_rad.tan()
    }

    #[must_use]
    pub fn tire_reference_load_n(self, axle: &str) -> f64 {
        match axle {
            "front" => self.tire_fz0_front_n,
            "rear" => self.tire_fz0_rear_n,
            _ => self.tire_fz0_n,
        }
    }

    #[must_use]
    pub fn tire_limited_lateral_accel_mps2(self) -> f64 {
        self.gravity_mps2 * self.lateral_grip_level
    }

    #[must_use]
    pub fn effective_lateral_accel_limit_mps2(self) -> f64 {
        self.lean_limited_lateral_accel_mps2()
            .min(self.tire_limited_lateral_accel_mps2())
    }

    #[must_use]
    pub fn steady_state_lean_rad_for_ay(self, ay_mps2: f64) -> f64 {
        (ay_mps2 / self.gravity_mps2).atan()
    }

    #[must_use]
    pub fn static_axle_loads_n(self) -> AxleLoadsN {
        AxleLoadsN {
            front_n: self.rider_bike_mass_kg * self.gravity_mps2 * self.wheelbase_rear_m
                / self.wheelbase_m,
            rear_n: self.rider_bike_mass_kg * self.gravity_mps2 * self.wheelbase_front_m
                / self.wheelbase_m,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BikeSingleTrackLeanState {
    pub v_mps: f64,
    pub beta_rad: f64,
    pub omega_z_radps: f64,
    pub n_m: f64,
    pub xi_rad: f64,
    pub phi_rad: f64,
    pub phi_dot_radps: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BikeSingleTrackLeanControl {
    pub delta_rad: f64,
    pub f_drive_n: f64,
    pub f_brake_n: f64,
    pub phi_dot_cmd_radps: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BikeSingleTrackLeanDynamics {
    pub dv_ds: f64,
    pub dbeta_ds: f64,
    pub domega_z_ds: f64,
    pub dn_ds: f64,
    pub dxi_ds: f64,
    pub dphi_ds: f64,
    pub dphi_dot_ds: f64,
    pub sigma_dt_ds: f64,
    pub ax_body_mps2: f64,
    pub ay_body_mps2: f64,
    pub ax_mps2: f64,
    pub ay_mps2: f64,
    pub roll_tau_cmd_nm: f64,
    pub roll_moment_nm: f64,
    pub phi_ddot_radps2: f64,
    pub tire_forces: BikeSingleTrackLeanTireForces,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BikeSingleTrackLeanTireForces {
    pub fx_front_n: f64,
    pub fx_rear_n: f64,
    pub fy_front_n: f64,
    pub fy_rear_n: f64,
    pub fz_front_n: f64,
    pub fz_rear_n: f64,
    pub alpha_front_rad: f64,
    pub alpha_rear_rad: f64,
}

impl BikeSingleTrackLeanTireForces {
    #[must_use]
    pub fn front_kamm_utilization(self, params: BikeSingleTrackLeanParams) -> f64 {
        let longitudinal_capacity = directional_longitudinal_tire_capacity_n(
            params.drive_grip_level,
            params.brake_grip_level,
            self.fx_front_n,
            self.fz_front_n,
        );
        let lateral_capacity = tire_load_sensitive_capacity_n(
            params.lateral_grip_level,
            self.fz_front_n,
            params.tire_eps_front,
            params.tire_reference_load_n("front"),
            params.tire_load_sensitivity_mode,
        );
        (self.fx_front_n / longitudinal_capacity).powi(2)
            + (self.fy_front_n / lateral_capacity).powi(2)
    }

    #[must_use]
    pub fn rear_kamm_utilization(self, params: BikeSingleTrackLeanParams) -> f64 {
        let longitudinal_capacity = directional_longitudinal_tire_capacity_n(
            params.drive_grip_level,
            params.brake_grip_level,
            self.fx_rear_n,
            self.fz_rear_n,
        );
        let lateral_capacity = tire_load_sensitive_capacity_n(
            params.lateral_grip_level,
            self.fz_rear_n,
            params.tire_eps_rear,
            params.tire_reference_load_n("rear"),
            params.tire_load_sensitivity_mode,
        );
        (self.fx_rear_n / longitudinal_capacity).powi(2)
            + (self.fy_rear_n / lateral_capacity).powi(2)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AxleLoadsN {
    pub front_n: f64,
    pub rear_n: f64,
}

impl AxleLoadsN {
    #[must_use]
    pub fn total_n(self) -> f64 {
        self.front_n + self.rear_n
    }
}

#[must_use]
pub fn car_double_track_dynamics(
    params: CarDoubleTrackParams,
    state: CarDoubleTrackState,
    control: CarDoubleTrackControl,
    kappa_1pm: f64,
) -> CarDoubleTrackDynamics {
    let v = state.v_mps.max(1e-6);
    let beta = state.beta_rad;
    let omega_z = state.omega_z_radps;
    let delta = control.delta_rad;
    let f_drive = control.f_drive_n;
    let f_brake = control.f_brake_n;
    let gamma_y = control.gamma_y_n;
    let mass = params.mass_kg;
    let wheelbase_front = params.wheelbase_front_m();
    let wheelbase_rear = params.wheelbase_rear_m();
    let drag = params.drag_coeff_n_per_mps2 * v.powi(2);
    let gravity = params.gravity_mps2;
    let rolling_total = params.rolling_resistance_coeff * mass * gravity;
    let rolling_fl = 0.5 * rolling_total * wheelbase_rear / params.wheelbase_m;
    let rolling_fr = rolling_fl;
    let rolling_rl = 0.5 * rolling_total * wheelbase_front / params.wheelbase_m;
    let rolling_rr = rolling_rl;

    let fz_static_fl = 0.5 * mass * gravity * wheelbase_rear / params.wheelbase_m;
    let fz_static_fr = fz_static_fl;
    let fz_static_rl = 0.5 * mass * gravity * wheelbase_front / params.wheelbase_m;
    let fz_static_rr = fz_static_rl;
    let fz_lift_fl = 0.5 * params.liftcoeff_front * v.powi(2);
    let fz_lift_fr = fz_lift_fl;
    let fz_lift_rl = 0.5 * params.liftcoeff_rear * v.powi(2);
    let fz_lift_rr = fz_lift_rl;
    let longitudinal_transfer =
        params.cg_height_m / params.wheelbase_m * (f_drive + f_brake - drag - rolling_total);
    let fz_dyn_fl = -0.5 * longitudinal_transfer - params.roll_stiffness_distribution * gamma_y;
    let fz_dyn_fr = -0.5 * longitudinal_transfer + params.roll_stiffness_distribution * gamma_y;
    let fz_dyn_rl =
        0.5 * longitudinal_transfer - (1.0 - params.roll_stiffness_distribution) * gamma_y;
    let fz_dyn_rr =
        0.5 * longitudinal_transfer + (1.0 - params.roll_stiffness_distribution) * gamma_y;

    let fz_fl = fz_static_fl + fz_lift_fl + fz_dyn_fl;
    let fz_fr = fz_static_fr + fz_lift_fr + fz_dyn_fr;
    let fz_rl = fz_static_rl + fz_lift_rl + fz_dyn_rl;
    let fz_rr = fz_static_rr + fz_lift_rr + fz_dyn_rr;

    let alpha_fl = delta
        - ((v * beta.sin() + wheelbase_front * omega_z)
            / (v * beta.cos() - 0.5 * params.track_width_front_m * omega_z).max_abs(1e-6))
        .atan();
    let alpha_fr = delta
        - ((v * beta.sin() + wheelbase_front * omega_z)
            / (v * beta.cos() + 0.5 * params.track_width_front_m * omega_z).max_abs(1e-6))
        .atan();
    let alpha_rl = ((-v * beta.sin() + wheelbase_rear * omega_z)
        / (v * beta.cos() - 0.5 * params.track_width_rear_m * omega_z).max_abs(1e-6))
    .atan();
    let alpha_rr = ((-v * beta.sin() + wheelbase_rear * omega_z)
        / (v * beta.cos() + 0.5 * params.track_width_rear_m * omega_z).max_abs(1e-6))
    .atan();

    let fy_fl = pacejka_lateral_force_n(
        params.lateral_grip_level,
        fz_fl,
        params.tire_eps_front,
        params.tire_fz0_front_n,
        params.tire_load_sensitivity_mode,
        params.tire_b_front,
        params.tire_c_front,
        params.tire_e_front,
        alpha_fl,
    );
    let fy_fr = pacejka_lateral_force_n(
        params.lateral_grip_level,
        fz_fr,
        params.tire_eps_front,
        params.tire_fz0_front_n,
        params.tire_load_sensitivity_mode,
        params.tire_b_front,
        params.tire_c_front,
        params.tire_e_front,
        alpha_fr,
    );
    let fy_rl = pacejka_lateral_force_n(
        params.lateral_grip_level,
        fz_rl,
        params.tire_eps_rear,
        params.tire_fz0_rear_n,
        params.tire_load_sensitivity_mode,
        params.tire_b_rear,
        params.tire_c_rear,
        params.tire_e_rear,
        alpha_rl,
    );
    let fy_rr = pacejka_lateral_force_n(
        params.lateral_grip_level,
        fz_rr,
        params.tire_eps_rear,
        params.tire_fz0_rear_n,
        params.tire_load_sensitivity_mode,
        params.tire_b_rear,
        params.tire_c_rear,
        params.tire_e_rear,
        alpha_rr,
    );

    let fx_fl = 0.5 * f_drive * params.drive_front_fraction
        + 0.5 * f_brake * params.brake_front_fraction
        - rolling_fl;
    let fx_fr = 0.5 * f_drive * params.drive_front_fraction
        + 0.5 * f_brake * params.brake_front_fraction
        - rolling_fr;
    let fx_rl = 0.5 * f_drive * (1.0 - params.drive_front_fraction)
        + 0.5 * f_brake * (1.0 - params.brake_front_fraction)
        - rolling_rl;
    let fx_rr = 0.5 * f_drive * (1.0 - params.drive_front_fraction)
        + 0.5 * f_brake * (1.0 - params.brake_front_fraction)
        - rolling_rr;

    let ax = (fx_rl + fx_rr + (fx_fl + fx_fr) * delta.cos() - (fy_fl + fy_fr) * delta.sin() - drag)
        / mass;
    let ay = ((fx_fl + fx_fr) * delta.sin() + fy_rl + fy_rr + (fy_fl + fy_fr) * delta.cos()) / mass;
    let sigma =
        ((1.0 - state.n_m * kappa_1pm) / (v * (state.xi_rad + beta).cos()).max_abs(1e-6)).max(1e-9);
    let dv_ds = sigma
        * ((fx_rl + fx_rr) * beta.cos()
            + (fx_fl + fx_fr) * (delta - beta).cos()
            + (fy_rl + fy_rr) * beta.sin()
            - (fy_fl + fy_fr) * (delta - beta).sin()
            - drag * beta.cos())
        / mass;
    let dbeta_ds = sigma
        * (-omega_z
            + (-(fx_rl + fx_rr) * beta.sin()
                + (fx_fl + fx_fr) * (delta - beta).sin()
                + (fy_rl + fy_rr) * beta.cos()
                + (fy_fl + fy_fr) * (delta - beta).cos()
                + drag * beta.sin())
                / (mass * v));
    let domega_z_ds = sigma
        * ((fx_rr - fx_rl) * params.track_width_rear_m / 2.0 - (fy_rl + fy_rr) * wheelbase_rear
            + ((fx_fr - fx_fl) * delta.cos() + (fy_fl - fy_fr) * delta.sin())
                * params.track_width_front_m
                / 2.0
            + ((fy_fl + fy_fr) * delta.cos() + (fx_fl + fx_fr) * delta.sin()) * wheelbase_front)
        / params.yaw_inertia_kgm2;
    let dn_ds = sigma * v * (state.xi_rad + beta).sin();
    let dxi_ds = sigma * omega_z - kappa_1pm;

    CarDoubleTrackDynamics {
        dv_ds,
        dbeta_ds,
        domega_z_ds,
        dn_ds,
        dxi_ds,
        sigma_dt_ds: sigma,
        ax_mps2: ax,
        ay_mps2: ay,
        tire_forces: CarDoubleTrackTireForces {
            fx_fl_n: fx_fl,
            fx_fr_n: fx_fr,
            fx_rl_n: fx_rl,
            fx_rr_n: fx_rr,
            fy_fl_n: fy_fl,
            fy_fr_n: fy_fr,
            fy_rl_n: fy_rl,
            fy_rr_n: fy_rr,
            fz_fl_n: fz_fl,
            fz_fr_n: fz_fr,
            fz_rl_n: fz_rl,
            fz_rr_n: fz_rr,
            alpha_fl_rad: alpha_fl,
            alpha_fr_rad: alpha_fr,
            alpha_rl_rad: alpha_rl,
            alpha_rr_rad: alpha_rr,
        },
    }
}

#[must_use]
pub fn bike_single_track_lean_dynamics(
    params: BikeSingleTrackLeanParams,
    state: BikeSingleTrackLeanState,
    control: BikeSingleTrackLeanControl,
    kappa_1pm: f64,
) -> BikeSingleTrackLeanDynamics {
    let v = state.v_mps.max(1e-6);
    let beta = state.beta_rad;
    let omega_z = state.omega_z_radps;
    let delta = control.delta_rad;
    let f_drive = control.f_drive_n;
    let f_brake = control.f_brake_n;
    let mass = params.rider_bike_mass_kg;
    let gravity = params.gravity_mps2;
    let drag = params.drag_coeff_n_per_mps2 * v.powi(2);
    let rolling_total = params.rolling_resistance_coeff * mass * gravity;
    let rolling_front = rolling_total * params.wheelbase_rear_m / params.wheelbase_m;
    let rolling_rear = rolling_total * params.wheelbase_front_m / params.wheelbase_m;

    let fz_static_front = mass * gravity * params.wheelbase_rear_m / params.wheelbase_m;
    let fz_static_rear = mass * gravity * params.wheelbase_front_m / params.wheelbase_m;
    let longitudinal_force_for_transfer = f_drive + f_brake - drag - rolling_total;
    let fz_dyn_front = -params.cg_height_m / params.wheelbase_m * longitudinal_force_for_transfer;
    let fz_dyn_rear = -fz_dyn_front;
    let fz_front = fz_static_front + params.liftcoeff_front * v.powi(2) + fz_dyn_front;
    let fz_rear = fz_static_rear + params.liftcoeff_rear * v.powi(2) + fz_dyn_rear;

    let alpha_front = delta
        - ((v * beta.sin() + params.wheelbase_front_m * omega_z) / (v * beta.cos()).max_abs(1e-6))
            .atan();
    let alpha_rear = ((-v * beta.sin() + params.wheelbase_rear_m * omega_z)
        / (v * beta.cos()).max_abs(1e-6))
    .atan();
    let fy_front = pacejka_lateral_force_n(
        params.lateral_grip_level,
        fz_front,
        params.tire_eps_front,
        params.tire_fz0_front_n,
        params.tire_load_sensitivity_mode,
        params.tire_b_front,
        params.tire_c_front,
        params.tire_e_front,
        alpha_front,
    );
    let fy_rear = pacejka_lateral_force_n(
        params.lateral_grip_level,
        fz_rear,
        params.tire_eps_rear,
        params.tire_fz0_rear_n,
        params.tire_load_sensitivity_mode,
        params.tire_b_rear,
        params.tire_c_rear,
        params.tire_e_rear,
        alpha_rear,
    );

    let fx_front = params.front_brake_bias * f_brake - rolling_front;
    let fx_rear = f_drive + (1.0 - params.front_brake_bias) * f_brake - rolling_rear;
    let ax_body = (fx_rear + fx_front * delta.cos() - fy_front * delta.sin() - drag) / mass;
    let ay_body = (fx_front * delta.sin() + fy_rear + fy_front * delta.cos()) / mass;
    let ax = ax_body * beta.cos() + ay_body * beta.sin();
    let ay = -ax_body * beta.sin() + ay_body * beta.cos();
    let sigma =
        ((1.0 - state.n_m * kappa_1pm) / (v * (state.xi_rad + beta).cos()).max_abs(1e-6)).max(1e-9);

    let dv_ds = sigma
        * (fx_rear * beta.cos() + fx_front * (delta - beta).cos() + fy_rear * beta.sin()
            - fy_front * (delta - beta).sin()
            - drag * beta.cos())
        / mass;
    let dbeta_ds = sigma
        * (-omega_z
            + (-fx_rear * beta.sin()
                + fx_front * (delta - beta).sin()
                + fy_rear * beta.cos()
                + fy_front * (delta - beta).cos()
                + drag * beta.sin())
                / (mass * v));
    let domega_z_ds = sigma
        * (-fy_rear * params.wheelbase_rear_m
            + (fy_front * delta.cos() + fx_front * delta.sin()) * params.wheelbase_front_m)
        / params.yaw_inertia_kgm2;
    let dn_ds = sigma * v * (state.xi_rad + beta).sin();
    let dxi_ds = sigma * omega_z - kappa_1pm;

    let roll_tau_cmd =
        params.roll_rate_servo_gain * (control.phi_dot_cmd_radps - state.phi_dot_radps);
    let roll_moment =
        mass * params.cg_height_m * (ay * state.phi_rad.cos() - gravity * state.phi_rad.sin());
    let phi_ddot = (roll_moment + roll_tau_cmd - params.roll_damping * state.phi_dot_radps)
        / params.roll_inertia_kgm2;
    let (dphi_ds, dphi_dot_ds) = if params.physics_version_v2 {
        (sigma * state.phi_dot_radps, sigma * phi_ddot)
    } else {
        (sigma * control.phi_dot_cmd_radps, 0.0)
    };

    BikeSingleTrackLeanDynamics {
        dv_ds,
        dbeta_ds,
        domega_z_ds,
        dn_ds,
        dxi_ds,
        dphi_ds,
        dphi_dot_ds,
        sigma_dt_ds: sigma,
        ax_body_mps2: ax_body,
        ay_body_mps2: ay_body,
        ax_mps2: ax,
        ay_mps2: ay,
        roll_tau_cmd_nm: roll_tau_cmd,
        roll_moment_nm: roll_moment,
        phi_ddot_radps2: phi_ddot,
        tire_forces: BikeSingleTrackLeanTireForces {
            fx_front_n: fx_front,
            fx_rear_n: fx_rear,
            fy_front_n: fy_front,
            fy_rear_n: fy_rear,
            fz_front_n: fz_front,
            fz_rear_n: fz_rear,
            alpha_front_rad: alpha_front,
            alpha_rear_rad: alpha_rear,
        },
    }
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
}

fn tire_nominal_capacity_n(grip: f64, normal_load_n: f64) -> f64 {
    (grip * normal_load_n.max(1e-6)).max(1e-6)
}

pub(crate) fn directional_longitudinal_tire_capacity_n(
    drive_grip: f64,
    brake_grip: f64,
    longitudinal_force_n: f64,
    normal_load_n: f64,
) -> f64 {
    let fx_smooth_n = (0.01 * normal_load_n.abs()).max(10.0);
    let drive_weight = 0.5 * (1.0 + (longitudinal_force_n / fx_smooth_n).tanh());
    let brake_weight = 1.0 - drive_weight;
    let drive_capacity = tire_nominal_capacity_n(drive_grip, normal_load_n);
    let brake_capacity = tire_nominal_capacity_n(brake_grip, normal_load_n);
    let inv_capacity = drive_weight / drive_capacity + brake_weight / brake_capacity;
    (1.0 / inv_capacity.max(1e-12)).max(1e-6)
}

fn tire_load_sensitive_capacity_n(
    grip: f64,
    normal_load_n: f64,
    eps: f64,
    fz0_n: f64,
    mode: TireLoadSensitivityMode,
) -> f64 {
    (grip * normal_load_n.max(1e-6) * tire_capacity_factor(normal_load_n, eps, fz0_n, mode))
        .max(1e-6)
}

trait SignedMaxAbs {
    fn max_abs(self, min_abs: f64) -> Self;
}

impl SignedMaxAbs for f64 {
    fn max_abs(self, min_abs: f64) -> Self {
        if self.abs() >= min_abs {
            self
        } else if self.is_sign_negative() {
            -min_abs
        } else {
            min_abs
        }
    }
}

fn positive_param(
    profile: &VehicleDynamicsProfileV1,
    key: &str,
    fallback: f64,
) -> Result<f64, String> {
    let value = profile.numeric_param(key).unwrap_or(fallback);
    if value.is_finite() && value > 0.0 {
        Ok(value)
    } else {
        Err(format!("{key} must be positive finite"))
    }
}

fn positive_param_any(
    profile: &VehicleDynamicsProfileV1,
    keys: &[&str],
    fallback: f64,
) -> Result<f64, String> {
    let value = keys
        .iter()
        .find_map(|key| profile.numeric_param(key))
        .unwrap_or(fallback);
    if value.is_finite() && value > 0.0 {
        Ok(value)
    } else {
        Err(format!("{} must be positive finite", keys.join(" or ")))
    }
}

fn required_string(value: &JsonValue, key: &str) -> Result<String, String> {
    value
        .get(key)
        .and_then(JsonValue::as_str)
        .map(str::to_owned)
        .ok_or_else(|| format!("{key} must be a string"))
}

fn optional_string(value: &JsonValue, key: &str) -> Option<String> {
    value
        .get(key)
        .and_then(JsonValue::as_str)
        .map(str::to_owned)
}

fn optional_object(value: &JsonValue, key: &str) -> JsonObject {
    match value.get(key) {
        Some(JsonValue::Object(entries)) => entries.clone(),
        _ => Vec::new(),
    }
}

fn object_f64(object: &JsonObject, key: &str) -> Option<f64> {
    object
        .iter()
        .find(|(entry_key, _)| entry_key == key)
        .and_then(|(_, value)| value.as_f64())
}

fn object_str<'a>(object: &'a JsonObject, key: &str) -> Option<&'a str> {
    object
        .iter()
        .find(|(entry_key, _)| entry_key == key)
        .and_then(|(_, value)| value.as_str())
}

fn object_bool(object: &JsonObject, key: &str) -> Option<bool> {
    object
        .iter()
        .find(|(entry_key, _)| entry_key == key)
        .and_then(|(_, value)| match value {
            JsonValue::Bool(value) => Some(*value),
            _ => None,
        })
}

#[cfg(test)]
mod tests {
    use super::{
        bike_single_track_lean_dynamics, car_double_track_dynamics,
        directional_longitudinal_tire_capacity_n, tire_capacity_factor, BikeSingleTrackLeanControl,
        BikeSingleTrackLeanParams, BikeSingleTrackLeanState, BikeSingleTrackLeanTireForces,
        CarDoubleTrackControl, CarDoubleTrackParams, CarDoubleTrackState, CarDoubleTrackTireForces,
        TireLoadSensitivityMode, VehicleDynamicsModelFamily, VehicleDynamicsProfileV1, G_MPS2,
    };
    use crate::json::parse_json_str;

    #[test]
    fn parses_vehicle_dynamics_profile_contract() {
        let value = parse_json_str(
            r#"{
              "schema_version": "vehicle_dynamics_profile.v1",
              "profile_id": "car_dynamics:gt3_track_car",
              "model_family": "car_dynamics",
              "preset_id": "gt3_track_car",
              "solver_id": "old_car_mintime",
              "parameters": {
                "mass_kg": 1340,
                "wheelbase_m": 2.72,
                "drive_layout": "rwd"
              },
              "native_parameters": {},
              "metadata": {}
            }"#,
        )
        .unwrap();
        let profile = VehicleDynamicsProfileV1::from_json(&value).unwrap();

        assert_eq!(
            profile.model_family,
            VehicleDynamicsModelFamily::CarDynamics
        );
        assert_eq!(profile.numeric_param("mass_kg"), Some(1340.0));
        assert_eq!(profile.string_param("drive_layout"), Some("rwd"));
    }

    #[test]
    fn car_params_accept_python_mintime_aliases() {
        let value = parse_json_str(
            r#"{
              "schema_version": "vehicle_dynamics_profile.v1",
              "profile_id": "car_dynamics:gt3_track_car",
              "model_family": "car_dynamics",
              "preset_id": "gt3_track_car",
              "solver_id": "old_car_mintime",
              "parameters": {
                "v_max_mps": 82.0,
                "mass_kg": 1340.0,
                "wheelbase_front": 1.13,
                "wheelbase_rear": 1.38,
                "track_width_front": 1.70,
                "track_width_rear": 1.68,
                "cog_z": 0.35,
                "mue": 1.45,
                "I_z": 1500.0,
                "delta_max": 0.50,
                "t_delta": 0.09,
                "t_drive": 0.08,
                "t_brake": 0.07,
                "power_max": 416000.0,
                "f_drive_max": 11500.0,
                "f_brake_max": 26000.0,
                "f_z0": 3285.0,
                "B_front": 11.0,
                "C_front": 2.3,
                "eps_front": -0.12,
                "E_front": 1.0,
                "B_rear": 12.0,
                "C_rear": 2.4,
                "eps_rear": -0.13,
                "E_rear": 0.9
              }
            }"#,
        )
        .unwrap();
        let profile = VehicleDynamicsProfileV1::from_json(&value).unwrap();
        let params = CarDoubleTrackParams::from_profile(&profile).unwrap();

        assert_eq!(params.max_speed_mps, 82.0);
        assert!((params.wheelbase_m - 2.51).abs() < 1e-12);
        assert_eq!(params.cg_to_front_axle_m, 1.13);
        assert_eq!(params.track_width_front_m, 1.70);
        assert_eq!(params.track_width_rear_m, 1.68);
        assert_eq!(params.cg_height_m, 0.35);
        assert_eq!(params.grip_level, 1.45);
        assert_eq!(params.longitudinal_grip_level, 1.45);
        assert_eq!(params.drive_grip_level, 1.45);
        assert_eq!(params.lateral_grip_level, 1.45);
        assert_eq!(params.brake_grip_level, 1.45);
        assert_eq!(params.yaw_inertia_kgm2, 1500.0);
        assert_eq!(params.steering_angle_max_rad, 0.5);
        assert_eq!(params.steering_response_s, 0.09);
        assert_eq!(params.throttle_response_s, 0.08);
        assert_eq!(params.brake_response_s, 0.07);
        assert_eq!(params.power_max_w, 416000.0);
        assert_eq!(params.drive_force_max_n, 11500.0);
        assert_eq!(params.brake_force_max_n, 26000.0);
        assert_eq!(params.tire_fz0_n, 3285.0);
        assert_eq!(params.tire_b_front, 11.0);
        assert_eq!(params.tire_c_front, 2.3);
        assert_eq!(params.tire_eps_front, -0.12);
        assert_eq!(params.tire_e_rear, 0.9);
        assert_eq!(
            params.tire_load_sensitivity_mode,
            TireLoadSensitivityMode::UpstreamRaw
        );
    }

    #[test]
    fn tire_load_sensitivity_modes_have_distinct_reference_semantics() {
        let raw_at_reference =
            tire_capacity_factor(1000.0, -0.25, 1000.0, TireLoadSensitivityMode::UpstreamRaw);
        let dfz_at_reference = tire_capacity_factor(
            1000.0,
            -0.25,
            1000.0,
            TireLoadSensitivityMode::ReferenceNormalizedDfz,
        );
        let dfz_above_reference = tire_capacity_factor(
            1500.0,
            -0.25,
            1000.0,
            TireLoadSensitivityMode::ReferenceNormalizedDfz,
        );

        assert!((raw_at_reference - 0.75).abs() < 1e-12);
        assert!((dfz_at_reference - 1.0).abs() < 1e-12);
        assert!((dfz_above_reference - 0.875).abs() < 1e-12);
    }

    #[test]
    fn tire_load_sensitivity_mode_parses_from_profile() {
        let value = parse_json_str(
            r#"{
              "schema_version": "vehicle_dynamics_profile.v1",
              "profile_id": "car_dynamics:test",
              "model_family": "car_dynamics",
              "solver_id": "old_car_mintime",
              "parameters": {
                "tire_load_sensitivity_mode": "reference_normalized_dfz"
              }
            }"#,
        )
        .unwrap();
        let profile = VehicleDynamicsProfileV1::from_json(&value).unwrap();
        let params = CarDoubleTrackParams::from_profile(&profile).unwrap();

        assert_eq!(
            params.tire_load_sensitivity_mode,
            TireLoadSensitivityMode::ReferenceNormalizedDfz
        );
    }

    #[test]
    fn car_tire_fz0_accepts_axle_specific_aliases_with_shared_fallback() {
        let value = parse_json_str(
            r#"{
              "schema_version": "vehicle_dynamics_profile.v1",
              "profile_id": "car_dynamics:test",
              "model_family": "car_dynamics",
              "solver_id": "old_car_mintime",
              "parameters": {
                "f_z0": 1200.0,
                "f_z0_front": 900.0,
                "f_z0_rear": 1500.0
              }
            }"#,
        )
        .unwrap();
        let profile = VehicleDynamicsProfileV1::from_json(&value).unwrap();
        let params = CarDoubleTrackParams::from_profile(&profile).unwrap();

        assert_eq!(params.tire_fz0_n, 1200.0);
        assert_eq!(params.tire_fz0_front_n, 900.0);
        assert_eq!(params.tire_fz0_rear_n, 1500.0);

        let fallback_value = parse_json_str(
            r#"{
              "schema_version": "vehicle_dynamics_profile.v1",
              "profile_id": "car_dynamics:test",
              "model_family": "car_dynamics",
              "solver_id": "old_car_mintime",
              "parameters": {"f_z0": 1200.0}
            }"#,
        )
        .unwrap();
        let fallback_profile = VehicleDynamicsProfileV1::from_json(&fallback_value).unwrap();
        let fallback_params = CarDoubleTrackParams::from_profile(&fallback_profile).unwrap();
        assert_eq!(fallback_params.tire_fz0_front_n, 1200.0);
        assert_eq!(fallback_params.tire_fz0_rear_n, 1200.0);
    }

    #[test]
    fn car_static_axle_loads_conserve_weight() {
        let params = CarDoubleTrackParams {
            max_speed_mps: 50.0,
            mass_kg: 1000.0,
            gravity_mps2: G_MPS2,
            wheelbase_m: 2.5,
            cg_to_front_axle_m: 1.25,
            cg_height_m: 0.5,
            track_width_front_m: 1.5,
            track_width_rear_m: 1.5,
            grip_level: 1.0,
            longitudinal_grip_level: 1.0,
            drive_grip_level: 1.0,
            lateral_grip_level: 1.0,
            brake_grip_level: 1.0,
            steering_angle_max_rad: 0.7,
            drive_force_max_n: 6000.0,
            brake_force_max_n: 12000.0,
            steering_response_s: 0.08,
            throttle_response_s: 0.08,
            brake_response_s: 0.07,
            yaw_inertia_kgm2: 1750.0,
            drag_coeff_n_per_mps2: 0.35,
            rolling_resistance_coeff: 0.013,
            roll_stiffness_distribution: 0.5,
            drive_front_fraction: 0.0,
            brake_front_fraction: 0.65,
            power_max_w: 250000.0,
            tire_b_front: 10.0,
            tire_c_front: 1.9,
            tire_e_front: 0.97,
            tire_eps_front: 0.0,
            tire_b_rear: 10.0,
            tire_c_rear: 1.9,
            tire_e_rear: 0.97,
            tire_eps_rear: 0.0,
            tire_fz0_n: 2500.0,
            tire_fz0_front_n: 2500.0,
            tire_fz0_rear_n: 2500.0,
            tire_load_sensitivity_mode: TireLoadSensitivityMode::UpstreamRaw,
            liftcoeff_front: 0.0,
            liftcoeff_rear: 0.0,
        };
        let loads = params.static_axle_loads_n();

        assert!((loads.front_n - 0.5 * 1000.0 * G_MPS2).abs() < 1e-9);
        assert!((loads.total_n() - 1000.0 * G_MPS2).abs() < 1e-9);
    }

    #[test]
    fn car_tire_ellipse_uses_load_sensitive_capacity() {
        let params = CarDoubleTrackParams {
            max_speed_mps: 50.0,
            mass_kg: 1000.0,
            gravity_mps2: G_MPS2,
            wheelbase_m: 2.5,
            cg_to_front_axle_m: 1.25,
            cg_height_m: 0.5,
            track_width_front_m: 1.5,
            track_width_rear_m: 1.5,
            grip_level: 2.0,
            longitudinal_grip_level: 2.0,
            drive_grip_level: 2.0,
            lateral_grip_level: 2.0,
            brake_grip_level: 2.0,
            steering_angle_max_rad: 0.7,
            drive_force_max_n: 6000.0,
            brake_force_max_n: 12000.0,
            steering_response_s: 0.08,
            throttle_response_s: 0.08,
            brake_response_s: 0.07,
            yaw_inertia_kgm2: 1750.0,
            drag_coeff_n_per_mps2: 0.35,
            rolling_resistance_coeff: 0.013,
            roll_stiffness_distribution: 0.5,
            drive_front_fraction: 0.0,
            brake_front_fraction: 0.65,
            power_max_w: 250000.0,
            tire_b_front: 10.0,
            tire_c_front: 1.9,
            tire_e_front: 0.97,
            tire_eps_front: 0.0,
            tire_b_rear: 10.0,
            tire_c_rear: 1.9,
            tire_e_rear: 0.97,
            tire_eps_rear: -0.25,
            tire_fz0_n: 1000.0,
            tire_fz0_front_n: 1000.0,
            tire_fz0_rear_n: 1000.0,
            tire_load_sensitivity_mode: TireLoadSensitivityMode::UpstreamRaw,
            liftcoeff_front: 0.0,
            liftcoeff_rear: 0.0,
        };
        let tire = CarDoubleTrackTireForces {
            fx_fl_n: 0.0,
            fx_fr_n: 0.0,
            fx_rl_n: 1000.0,
            fx_rr_n: 0.0,
            fy_fl_n: 0.0,
            fy_fr_n: 0.0,
            fy_rl_n: 750.0,
            fy_rr_n: 0.0,
            fz_fl_n: 1.0,
            fz_fr_n: 1.0,
            fz_rl_n: 1000.0,
            fz_rr_n: 1.0,
            alpha_fl_rad: 0.0,
            alpha_fr_rad: 0.0,
            alpha_rl_rad: 0.0,
            alpha_rr_rad: 0.0,
        };

        assert!((tire.wheel_ellipse_utilization(params, "rl") - 0.5).abs() < 1e-12);
    }

    #[test]
    fn car_tire_ellipse_uses_separate_longitudinal_and_lateral_grip() {
        let params = CarDoubleTrackParams {
            max_speed_mps: 50.0,
            mass_kg: 1000.0,
            gravity_mps2: G_MPS2,
            wheelbase_m: 2.5,
            cg_to_front_axle_m: 1.25,
            cg_height_m: 0.5,
            track_width_front_m: 1.5,
            track_width_rear_m: 1.5,
            grip_level: 1.0,
            longitudinal_grip_level: 2.0,
            drive_grip_level: 2.0,
            lateral_grip_level: 0.5,
            brake_grip_level: 2.0,
            steering_angle_max_rad: 0.7,
            drive_force_max_n: 6000.0,
            brake_force_max_n: 12000.0,
            steering_response_s: 0.08,
            throttle_response_s: 0.08,
            brake_response_s: 0.07,
            yaw_inertia_kgm2: 1750.0,
            drag_coeff_n_per_mps2: 0.35,
            rolling_resistance_coeff: 0.013,
            roll_stiffness_distribution: 0.5,
            drive_front_fraction: 0.0,
            brake_front_fraction: 0.65,
            power_max_w: 250000.0,
            tire_b_front: 10.0,
            tire_c_front: 1.9,
            tire_e_front: 0.97,
            tire_eps_front: 0.0,
            tire_b_rear: 10.0,
            tire_c_rear: 1.9,
            tire_e_rear: 0.97,
            tire_eps_rear: 0.0,
            tire_fz0_n: 1000.0,
            tire_fz0_front_n: 1000.0,
            tire_fz0_rear_n: 1000.0,
            tire_load_sensitivity_mode: TireLoadSensitivityMode::UpstreamRaw,
            liftcoeff_front: 0.0,
            liftcoeff_rear: 0.0,
        };
        let tire = CarDoubleTrackTireForces {
            fx_fl_n: 0.0,
            fx_fr_n: 0.0,
            fx_rl_n: 1000.0,
            fx_rr_n: 0.0,
            fy_fl_n: 0.0,
            fy_fr_n: 0.0,
            fy_rl_n: 250.0,
            fy_rr_n: 0.0,
            fz_fl_n: 1.0,
            fz_fr_n: 1.0,
            fz_rl_n: 1000.0,
            fz_rr_n: 1.0,
            alpha_fl_rad: 0.0,
            alpha_fr_rad: 0.0,
            alpha_rl_rad: 0.0,
            alpha_rr_rad: 0.0,
        };

        assert!((tire.wheel_ellipse_utilization(params, "rl") - 0.5).abs() < 1e-12);
    }

    #[test]
    fn car_params_parse_separate_longitudinal_and_lateral_grip() {
        let value = parse_json_str(
            r#"{
              "schema_version": "vehicle_dynamics_profile.v1",
              "profile_id": "car_dynamics:test",
              "model_family": "car_dynamics",
              "preset_id": "test",
              "solver_id": "old_car_mintime",
              "parameters": {
                "mue": 1.0,
                "longitudinal_grip_level": 1.5,
                "lateral_grip_level": 0.75
              }
            }"#,
        )
        .unwrap();
        let profile = VehicleDynamicsProfileV1::from_json(&value).unwrap();
        let params = CarDoubleTrackParams::from_profile(&profile).unwrap();

        assert_eq!(params.grip_level, 1.0);
        assert_eq!(params.longitudinal_grip_level, 1.5);
        assert_eq!(params.drive_grip_level, 1.5);
        assert_eq!(params.lateral_grip_level, 0.75);
        assert_eq!(params.brake_grip_level, 1.5);
    }

    #[test]
    fn bike_params_parse_separate_longitudinal_and_lateral_grip() {
        let value = parse_json_str(
            r#"{
              "schema_version": "vehicle_dynamics_profile.v1",
              "profile_id": "bike_dynamics:test",
              "model_family": "bike_dynamics",
              "preset_id": "test",
              "solver_id": "bike_single_track_mintime",
              "parameters": {
                "mue": 1.0,
                "longitudinal_grip_level": 1.5,
                "lateral_grip_level": 0.75
              }
            }"#,
        )
        .unwrap();
        let profile = VehicleDynamicsProfileV1::from_json(&value).unwrap();
        let params = BikeSingleTrackLeanParams::from_profile(&profile).unwrap();

        assert_eq!(params.grip_level, 1.0);
        assert_eq!(params.longitudinal_grip_level, 1.5);
        assert_eq!(params.drive_grip_level, 1.5);
        assert_eq!(params.lateral_grip_level, 0.75);
        assert_eq!(params.brake_grip_level, 1.5);
    }

    #[test]
    fn car_params_parse_directional_grip_overrides_legacy_longitudinal() {
        let value = parse_json_str(
            r#"{
              "schema_version": "vehicle_dynamics_profile.v1",
              "profile_id": "car_dynamics:test",
              "model_family": "car_dynamics",
              "preset_id": "test",
              "solver_id": "old_car_mintime",
              "parameters": {
                "mue": 1.0,
                "longitudinal_grip_level": 1.5,
                "drive_grip_level": 0.8,
                "lateral_grip_level": 1.1,
                "brake_grip_level": 1.9
              }
            }"#,
        )
        .unwrap();
        let profile = VehicleDynamicsProfileV1::from_json(&value).unwrap();
        let params = CarDoubleTrackParams::from_profile(&profile).unwrap();

        assert_eq!(params.grip_level, 1.0);
        assert_eq!(params.longitudinal_grip_level, 1.5);
        assert_eq!(params.drive_grip_level, 0.8);
        assert_eq!(params.lateral_grip_level, 1.1);
        assert_eq!(params.brake_grip_level, 1.9);
    }

    #[test]
    fn bike_params_parse_directional_grip_overrides_legacy_longitudinal() {
        let value = parse_json_str(
            r#"{
              "schema_version": "vehicle_dynamics_profile.v1",
              "profile_id": "bike_dynamics:test",
              "model_family": "bike_dynamics",
              "preset_id": "test",
              "solver_id": "bike_single_track_mintime",
              "parameters": {
                "mue": 1.0,
                "longitudinal_grip_level": 1.5,
                "drive_grip_level": 0.8,
                "lateral_grip_level": 1.1,
                "brake_grip_level": 1.9
              }
            }"#,
        )
        .unwrap();
        let profile = VehicleDynamicsProfileV1::from_json(&value).unwrap();
        let params = BikeSingleTrackLeanParams::from_profile(&profile).unwrap();

        assert_eq!(params.grip_level, 1.0);
        assert_eq!(params.longitudinal_grip_level, 1.5);
        assert_eq!(params.drive_grip_level, 0.8);
        assert_eq!(params.lateral_grip_level, 1.1);
        assert_eq!(params.brake_grip_level, 1.9);
    }

    #[test]
    fn directional_longitudinal_capacity_uses_force_sign_smoothly() {
        let forward_capacity = directional_longitudinal_tire_capacity_n(0.5, 2.0, 500.0, 1000.0);
        let brake_capacity = directional_longitudinal_tire_capacity_n(0.5, 2.0, -500.0, 1000.0);
        let neutral_capacity = directional_longitudinal_tire_capacity_n(0.5, 2.0, 0.0, 1000.0);

        assert!((forward_capacity - 500.0).abs() < 1e-6);
        assert!((brake_capacity - 2000.0).abs() < 1e-6);
        assert!(neutral_capacity.is_finite());
        assert!(neutral_capacity > forward_capacity);
        assert!(neutral_capacity < brake_capacity);
    }

    #[test]
    fn bike_tire_ellipse_uses_separate_longitudinal_and_lateral_grip() {
        let value = parse_json_str(
            r#"{
              "schema_version": "vehicle_dynamics_profile.v1",
              "profile_id": "bike_dynamics:test",
              "model_family": "bike_dynamics",
              "preset_id": "test",
              "solver_id": "bike_single_track_mintime",
              "parameters": {
                "mue": 1.0,
                "longitudinal_grip_level": 2.0,
                "lateral_grip_level": 0.5,
                "eps_rear": 0.0,
                "f_z0": 1000.0
              }
            }"#,
        )
        .unwrap();
        let profile = VehicleDynamicsProfileV1::from_json(&value).unwrap();
        let params = BikeSingleTrackLeanParams::from_profile(&profile).unwrap();
        let tire = BikeSingleTrackLeanTireForces {
            fx_front_n: 0.0,
            fx_rear_n: 1000.0,
            fy_front_n: 0.0,
            fy_rear_n: 250.0,
            fz_front_n: 1.0,
            fz_rear_n: 1000.0,
            alpha_front_rad: 0.0,
            alpha_rear_rad: 0.0,
        };

        assert!((tire.rear_kamm_utilization(params) - 0.5).abs() < 1e-12);
    }

    #[test]
    fn bike_kamm_uses_load_sensitive_capacity() {
        let value = parse_json_str(
            r#"{
              "schema_version": "vehicle_dynamics_profile.v1",
              "profile_id": "bike_dynamics:test",
              "model_family": "bike_dynamics",
              "preset_id": "test",
              "solver_id": "bike_single_track_mintime",
              "parameters": {
                "mue": 2.0,
                "eps_rear": -0.25,
                "f_z0": 1000.0,
                "physics_version": "bike_single_track_lean_v2"
              }
            }"#,
        )
        .unwrap();
        let profile = VehicleDynamicsProfileV1::from_json(&value).unwrap();
        let params = BikeSingleTrackLeanParams::from_profile(&profile).unwrap();
        let tire = BikeSingleTrackLeanTireForces {
            fx_front_n: 0.0,
            fx_rear_n: 1000.0,
            fy_front_n: 0.0,
            fy_rear_n: 750.0,
            fz_front_n: 1.0,
            fz_rear_n: 1000.0,
            alpha_front_rad: 0.0,
            alpha_rear_rad: 0.0,
        };

        assert!((tire.rear_kamm_utilization(params) - 0.5).abs() < 1e-12);
    }

    #[test]
    fn car_longitudinal_load_transfer_matches_rigid_body_formula() {
        let params = CarDoubleTrackParams {
            max_speed_mps: 50.0,
            mass_kg: 1000.0,
            gravity_mps2: G_MPS2,
            wheelbase_m: 2.0,
            cg_to_front_axle_m: 1.0,
            cg_height_m: 0.5,
            track_width_front_m: 1.5,
            track_width_rear_m: 1.5,
            grip_level: 1.0,
            longitudinal_grip_level: 1.0,
            drive_grip_level: 1.0,
            lateral_grip_level: 1.0,
            brake_grip_level: 1.0,
            steering_angle_max_rad: 0.7,
            drive_force_max_n: 6000.0,
            brake_force_max_n: 12000.0,
            steering_response_s: 0.08,
            throttle_response_s: 0.08,
            brake_response_s: 0.07,
            yaw_inertia_kgm2: 1750.0,
            drag_coeff_n_per_mps2: 0.35,
            rolling_resistance_coeff: 0.013,
            roll_stiffness_distribution: 0.5,
            drive_front_fraction: 0.0,
            brake_front_fraction: 0.65,
            power_max_w: 250000.0,
            tire_b_front: 10.0,
            tire_c_front: 1.9,
            tire_e_front: 0.97,
            tire_eps_front: 0.0,
            tire_b_rear: 10.0,
            tire_c_rear: 1.9,
            tire_e_rear: 0.97,
            tire_eps_rear: 0.0,
            tire_fz0_n: 2500.0,
            tire_fz0_front_n: 2500.0,
            tire_fz0_rear_n: 2500.0,
            tire_load_sensitivity_mode: TireLoadSensitivityMode::UpstreamRaw,
            liftcoeff_front: 0.0,
            liftcoeff_rear: 0.0,
        };

        assert!((params.longitudinal_load_transfer_n(4.0) - 1000.0).abs() < 1e-9);
    }

    #[test]
    fn car_double_track_dynamics_matches_straight_line_rolling_drag_signs() {
        let params = CarDoubleTrackParams {
            max_speed_mps: 50.0,
            mass_kg: 1000.0,
            gravity_mps2: G_MPS2,
            wheelbase_m: 2.0,
            cg_to_front_axle_m: 1.0,
            cg_height_m: 0.5,
            track_width_front_m: 1.5,
            track_width_rear_m: 1.5,
            grip_level: 1.0,
            longitudinal_grip_level: 1.0,
            drive_grip_level: 1.0,
            lateral_grip_level: 1.0,
            brake_grip_level: 1.0,
            steering_angle_max_rad: 0.7,
            drive_force_max_n: 6000.0,
            brake_force_max_n: 12000.0,
            steering_response_s: 0.08,
            throttle_response_s: 0.08,
            brake_response_s: 0.07,
            yaw_inertia_kgm2: 1750.0,
            drag_coeff_n_per_mps2: 0.35,
            rolling_resistance_coeff: 0.013,
            roll_stiffness_distribution: 0.5,
            drive_front_fraction: 0.0,
            brake_front_fraction: 0.65,
            power_max_w: 250000.0,
            tire_b_front: 10.0,
            tire_c_front: 1.9,
            tire_e_front: 0.97,
            tire_eps_front: 0.0,
            tire_b_rear: 10.0,
            tire_c_rear: 1.9,
            tire_e_rear: 0.97,
            tire_eps_rear: 0.0,
            tire_fz0_n: 2500.0,
            tire_fz0_front_n: 2500.0,
            tire_fz0_rear_n: 2500.0,
            tire_load_sensitivity_mode: TireLoadSensitivityMode::UpstreamRaw,
            liftcoeff_front: 0.0,
            liftcoeff_rear: 0.0,
        };
        let dynamics = car_double_track_dynamics(
            params,
            CarDoubleTrackState {
                v_mps: 20.0,
                beta_rad: 0.0,
                omega_z_radps: 0.0,
                n_m: 0.0,
                xi_rad: 0.0,
            },
            CarDoubleTrackControl {
                delta_rad: 0.0,
                f_drive_n: 0.0,
                f_brake_n: 0.0,
                gamma_y_n: 0.0,
            },
            0.0,
        );

        assert!(dynamics.ax_mps2 < 0.0);
        assert!(dynamics.dv_ds < 0.0);
        assert!(dynamics.ay_mps2.abs() < 1e-9);
        assert!((dynamics.sigma_dt_ds - 0.05).abs() < 1e-12);
        assert!(
            (dynamics.tire_forces.fz_fl_n
                + dynamics.tire_forces.fz_fr_n
                + dynamics.tire_forces.fz_rl_n
                + dynamics.tire_forces.fz_rr_n
                - params.mass_kg * G_MPS2)
                .abs()
                < 1e-9
        );
    }

    #[test]
    fn bike_params_accept_python_v2_contract_aliases_and_gates() {
        let value = parse_json_str(
            r#"{
              "schema_version": "vehicle_dynamics_profile.v1",
              "profile_id": "bike_dynamics:moto_450_motard",
              "model_family": "bike_dynamics",
              "preset_id": "moto_450_motard",
              "solver_id": "bike_single_track_mintime",
              "parameters": {
                "mass": 188.0,
                "v_max": 52.0,
                "g": 9.81,
                "wheelbase": 1.48,
                "front_weight_bias": 0.47,
                "cog_z": 0.64,
                "I_z": 105.0,
                "dragcoeff": 0.44,
                "mue": 1.12,
                "front_brake_bias": 0.76,
                "t_delta": 0.11,
                "t_drive": 0.10,
                "t_brake": 0.09,
                "t_lean": 0.10,
                "power_max": 45000.0,
                "f_drive_max": 2700.0,
                "f_brake_max": 5200.0,
                "delta_max": 0.90,
                "phi_max": 1.00,
                "phi_dot_max": 2.50,
                "beta_max": 0.35,
                "alpha_front_max": 0.35,
                "alpha_rear_max": 0.45,
                "min_normal_force": 25.0,
                "min_normal_force_static_fraction": 0.10,
                "f_z0": 920.0,
                "B_front": 9.8,
                "C_front": 2.20,
                "eps_front": -0.10,
                "E_front": 1.0,
                "B_rear": 9.8,
                "C_rear": 2.20,
                "eps_rear": -0.10,
                "E_rear": 1.0,
                "physics_version": "bike_single_track_lean_v2"
              },
              "native_parameters": {},
              "metadata": {}
            }"#,
        )
        .unwrap();
        let profile = VehicleDynamicsProfileV1::from_json(&value).unwrap();
        let params = BikeSingleTrackLeanParams::from_profile(&profile).unwrap();

        assert_eq!(params.rider_bike_mass_kg, 188.0);
        assert!((params.wheelbase_front_m - 1.48 * 0.53).abs() < 1e-12);
        assert!((params.wheelbase_rear_m - 1.48 * 0.47).abs() < 1e-12);
        assert_eq!(params.beta_max_rad, 0.28);
        assert_eq!(params.alpha_front_max_rad, 0.28);
        assert_eq!(params.alpha_rear_max_rad, 0.32);
        assert_eq!(params.xi_max_rad, 0.75);
        assert_eq!(params.min_normal_force_static_fraction, 0.22);
        assert!(params.roll_inertia_kgm2 > 0.0);
        assert!(params.roll_tau_max_nm > 0.0);
        assert!((params.effective_lateral_accel_limit_mps2() - 1.12 * 9.81).abs() < 1e-9);
    }

    #[test]
    fn bike_tire_fz0_accepts_axle_specific_aliases_with_shared_fallback() {
        let value = parse_json_str(
            r#"{
              "schema_version": "vehicle_dynamics_profile.v1",
              "profile_id": "bike_dynamics:test",
              "model_family": "bike_dynamics",
              "solver_id": "bike_single_track_mintime",
              "parameters": {
                "f_z0": 920.0,
                "f_z0_front": 700.0,
                "f_z0_rear": 1100.0,
                "physics_version": "bike_single_track_lean_v2"
              }
            }"#,
        )
        .unwrap();
        let profile = VehicleDynamicsProfileV1::from_json(&value).unwrap();
        let params = BikeSingleTrackLeanParams::from_profile(&profile).unwrap();

        assert_eq!(params.tire_fz0_n, 920.0);
        assert_eq!(params.tire_fz0_front_n, 700.0);
        assert_eq!(params.tire_fz0_rear_n, 1100.0);

        let fallback_value = parse_json_str(
            r#"{
              "schema_version": "vehicle_dynamics_profile.v1",
              "profile_id": "bike_dynamics:test",
              "model_family": "bike_dynamics",
              "solver_id": "bike_single_track_mintime",
              "parameters": {
                "f_z0": 920.0,
                "physics_version": "bike_single_track_lean_v2"
              }
            }"#,
        )
        .unwrap();
        let fallback_profile = VehicleDynamicsProfileV1::from_json(&fallback_value).unwrap();
        let fallback_params = BikeSingleTrackLeanParams::from_profile(&fallback_profile).unwrap();
        assert_eq!(fallback_params.tire_fz0_front_n, 920.0);
        assert_eq!(fallback_params.tire_fz0_rear_n, 920.0);
    }

    #[test]
    fn bike_single_track_dynamics_matches_straight_line_loads_and_roll_signs() {
        let profile = VehicleDynamicsProfileV1 {
            schema_version: VehicleDynamicsProfileV1::SCHEMA_VERSION.to_owned(),
            profile_id: "bike_dynamics:test".to_owned(),
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
                ("mue".to_owned(), 1.0.into()),
                ("phi_max".to_owned(), 1.0.into()),
                ("phi_dot_max".to_owned(), 2.5.into()),
                ("strict_product_gates".to_owned(), false.into()),
            ],
            native_parameters: Vec::new(),
            metadata: Vec::new(),
        };
        let params = BikeSingleTrackLeanParams::from_profile(&profile).unwrap();
        let dynamics = bike_single_track_lean_dynamics(
            params,
            BikeSingleTrackLeanState {
                v_mps: 20.0,
                beta_rad: 0.0,
                omega_z_radps: 0.0,
                n_m: 0.0,
                xi_rad: 0.0,
                phi_rad: 0.0,
                phi_dot_radps: 0.0,
            },
            BikeSingleTrackLeanControl {
                delta_rad: 0.0,
                f_drive_n: 0.0,
                f_brake_n: 0.0,
                phi_dot_cmd_radps: 1.0,
            },
            0.0,
        );

        assert!(
            (params.steady_state_lean_rad_for_ay(G_MPS2) - std::f64::consts::FRAC_PI_4).abs()
                < 1e-12
        );
        assert!(dynamics.ax_mps2 < 0.0);
        assert!(dynamics.dv_ds < 0.0);
        assert!(dynamics.ay_mps2.abs() < 1e-9);
        assert!((dynamics.sigma_dt_ds - 0.05).abs() < 1e-12);
        assert!(
            (dynamics.tire_forces.fz_front_n + dynamics.tire_forces.fz_rear_n
                - params.rider_bike_mass_kg * G_MPS2)
                .abs()
                < 1e-9
        );
        assert!(dynamics.roll_tau_cmd_nm > 0.0);
        assert!(dynamics.phi_ddot_radps2 > 0.0);
    }

    #[test]
    fn bike_roll_dynamics_matches_lean_equilibrium() {
        let profile = VehicleDynamicsProfileV1 {
            schema_version: VehicleDynamicsProfileV1::SCHEMA_VERSION.to_owned(),
            profile_id: "bike_dynamics:test".to_owned(),
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
                ("t_lean".to_owned(), 0.1.into()),
                ("strict_product_gates".to_owned(), false.into()),
                (
                    "physics_version".to_owned(),
                    "bike_single_track_lean_v2".into(),
                ),
            ],
            native_parameters: Vec::new(),
            metadata: Vec::new(),
        };
        let params = BikeSingleTrackLeanParams::from_profile(&profile).unwrap();
        let straight_state = BikeSingleTrackLeanState {
            v_mps: 20.0,
            beta_rad: 0.0,
            omega_z_radps: 0.0,
            n_m: 0.0,
            xi_rad: 0.0,
            phi_rad: 0.0,
            phi_dot_radps: 0.0,
        };
        let zero_control = BikeSingleTrackLeanControl {
            delta_rad: 0.0,
            f_drive_n: 0.0,
            f_brake_n: 0.0,
            phi_dot_cmd_radps: 0.0,
        };
        let straight = bike_single_track_lean_dynamics(params, straight_state, zero_control, 0.0);
        assert!(straight.roll_moment_nm.abs() < 1e-9);
        assert!(straight.phi_ddot_radps2.abs() < 1e-9);

        let turning_state = BikeSingleTrackLeanState {
            v_mps: 18.0,
            beta_rad: 0.0,
            omega_z_radps: 0.28,
            n_m: 0.0,
            xi_rad: 0.0,
            phi_rad: 0.0,
            phi_dot_radps: 0.0,
        };
        let turning_control = BikeSingleTrackLeanControl {
            delta_rad: 0.25,
            f_drive_n: 0.0,
            f_brake_n: 0.0,
            phi_dot_cmd_radps: 0.0,
        };
        let unleaned = bike_single_track_lean_dynamics(params, turning_state, turning_control, 0.0);
        assert!(
            unleaned.ay_mps2 > 0.0,
            "positive steering/yaw test case should produce positive lateral acceleration"
        );
        assert!(unleaned.roll_moment_nm > 0.0);

        let equilibrium_phi = params.steady_state_lean_rad_for_ay(unleaned.ay_mps2);
        let leaned = bike_single_track_lean_dynamics(
            params,
            BikeSingleTrackLeanState {
                phi_rad: equilibrium_phi,
                ..turning_state
            },
            turning_control,
            0.0,
        );

        assert!(equilibrium_phi > 0.0);
        assert!((equilibrium_phi.tan() - unleaned.ay_mps2 / G_MPS2).abs() < 1e-12);
        assert!(
            leaned.roll_moment_nm.abs() < 1e-9,
            "steady turn at phi=atan(ay/g) should not require roll moment: {}",
            leaned.roll_moment_nm
        );
        assert!(
            leaned.phi_ddot_radps2.abs() < 1e-9,
            "steady turn with zero roll command/damping should not roll accelerate: {}",
            leaned.phi_ddot_radps2
        );
    }
}
