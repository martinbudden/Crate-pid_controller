use num_traits::{ConstOne, ConstZero, Float};
use serde::{Deserialize, Serialize};

/// `Pid` using `f32` values.
pub type Pidf32 = Pid<f32>;
/// `PidGains` using `f32` values.
pub type PidGainsf32 = PidGains<f32>;
/// `PidError` using `f32` values.
pub type PidErrorf32 = PidErrors<f32>;
/// `PidLimits` using `f32` values.
pub type PidLimitsf32 = PidLimits<f32>;

/// `Pid` using `f64` values.
pub type Pidf64 = Pid<f64>;
/// `PidGains` using `f64` values.
pub type PidGainsf64 = PidGains<f64>;
/// `PidError` using `f64` values.
pub type PidErrorf64 = PidErrors<f64>;
/// `PidLimits` using `f32` values.
pub type PidLimitsf64 = PidLimits<f64>;

/// PID controller function-call trait.<br>
/// Declares the various forms of the `update` functions.<br><br>
///
pub trait PidController<T> {
    fn update(&mut self, measurement: T, delta_t: T) -> T;
    fn update_delta(&mut self, measurement: T, measurement_delta: T, delta_t: T) -> T;
    fn update_delta_iterm(&mut self, measurement: T, measurement_delta: T, iterm_error: T, delta_t: T) -> T;
    fn update_sp(&mut self, measurement: T) -> T;
    fn update_spd(&mut self, measurement: T, measurement_delta: T, delta_t: T) -> T;
    fn update_skpd(&mut self, measurement: T, measurement_delta: T, delta_t: T) -> T;
}

/// PID controller with open loop control (generic form).<br>
/// `Pidf32` and `Pidf64` aliases are available.<br>
/// This includes setpoint gain (classical feed forward) and<br>
/// setpoint derivative gain (kick - called feedforward by Betaflight).<br><br>
///
/// Uses "independent PID" notation, where the gains are denoted as kp, ki, kd etc.<br>
/// (In the "dependent PID" notation `kc`, `tau_i`, and `tau_d` parameters are used, where `kp = kc`, `ki = kc/tau_i`, `kd = kc*tau_d`).
///
#[derive(Clone, Copy, Debug, PartialEq, Deserialize, Serialize)]
pub struct Pid<T> {
    gains: PidGains<T>,
    limits: PidLimits<T>,
    /// saved value of pid.ki, so integration can be switched on and off.
    ki_saved: T,
    measurement_previous: T,

    setpoint: T,
    setpoint_previous: T,
    setpoint_derivative: T,

    error_derivative: T,
    error_integral: T,
    error_previous: T,
}

/// Default `Pid`.
/// ```
/// # use pidsk_controller::Pidf32;
/// # use num_traits::Zero;
///
/// let pid = Pidf32::default();
///
/// assert_eq!(1.0, pid.gains().kp);
/// ```
impl<T: Float + ConstZero + ConstOne> Default for Pid<T> {
    fn default() -> Self {
        Self::new(PidGains::new(T::ONE, T::ZERO, T::ZERO, T::ZERO, T::ZERO))
    }
}

// Constructors
impl<T: Float + ConstZero + ConstOne> Pid<T> {
    pub const fn with_limits(gains: PidGains<T>, limits: PidLimits<T>) -> Self {
        Self {
            gains,
            limits,
            ki_saved: gains.ki,
            measurement_previous: T::ZERO,
            setpoint: T::ZERO,
            setpoint_previous: T::ZERO,
            setpoint_derivative: T::ZERO,
            error_derivative: T::ZERO,
            error_integral: T::ZERO,
            error_previous: T::ZERO,
        }
    }

    pub const fn new(gains: PidGains<T>) -> Self {
        Self::with_limits(gains, PidLimits::new())
    }
}

impl<T: Float> PidController<T> for Pid<T> {
    /// PID update.
    /// ```
    /// # use pidsk_controller::{Pidf32,PidController,PidGainsf32};
    /// let delta_t: f32 = 0.01;
    /// let mut pid = Pidf32::new(PidGainsf32 { kp:0.1, ki:0.0, kd:0.0, ks:0.0, kk:0.0 });
    ///
    /// pid.set_setpoint(8.7);
    ///
    /// let measurement:f32 = 9.2;
    /// let output = pid.update(measurement, delta_t);
    ///
    /// assert_eq!(-0.05, output);
    /// ```
    fn update(&mut self, measurement: T, delta_t: T) -> T {
        self.update_delta(measurement, measurement - self.measurement_previous, delta_t)
    }

    /// PID update with `measurement_delta` specified.
    /// This allows the user to filter `measurement_delta` with a filter of their choice.
    ///
    /// ```
    /// # use pidsk_controller::{Pidf32,PidController,PidGainsf32};
    /// # use signal_filters::{Pt1Filterf32,SignalFilter};
    /// let delta_t: f32 = 0.01;
    /// let mut pid = Pidf32::new(PidGainsf32 { kp:0.1, ki:0.0, kd:0.01, ks:0.0, kk:0.0 });
    /// let mut filter = Pt1Filterf32::with_k(1.0);
    ///
    /// pid.set_setpoint(2.1);
    ///
    /// let measurement:f32 = 0.2;
    /// let measurement_delta = measurement - pid.previous_measurement();
    /// let measurement_delta_filtered = filter.update(measurement_delta);
    ///
    /// let output = pid.update_delta(measurement, measurement_delta_filtered, delta_t);
    ///
    /// assert_eq!(-0.010_000_005, output);
    ///
    fn update_delta(&mut self, measurement: T, measurement_delta: T, delta_t: T) -> T {
        self.update_delta_iterm(measurement, measurement_delta, self.setpoint - measurement, delta_t)
    }

    fn update_delta_iterm(&mut self, measurement: T, measurement_delta: T, iterm_error: T, delta_t: T) -> T {
        self.measurement_previous = measurement;

        let error = self.setpoint - measurement;
        self.error_previous = error;
        self.error_derivative = -measurement_delta / delta_t; // note minus sign, error delta has reverse polarity to measurement delta

        let partial_sum = self.partial_sum();

        // Perform Euler integration
        self.error_integral = self.error_integral + self.gains.ki * iterm_error * delta_t;

        // Anti-windup via integral clamping
        if let Some(max) = self.limits.integral_max
            && self.error_integral > max
        {
            self.error_integral = max;
        }
        if let Some(min) = self.limits.integral_min
            && self.error_integral < min
        {
            self.error_integral = min;
        }

        // Dynamic Anti-Windup Clamping based on Output Saturation Limit
        if let Some(output_saturation_value) = self.limits.output_saturation_value {
            // Determine dynamic upper and lower boundaries for the integral term
            let max_allowed_integral = output_saturation_value - partial_sum;
            let min_allowed_integral = -output_saturation_value - partial_sum;

            // Clamp the accumulator within the calculated dynamic window
            if self.error_integral > max_allowed_integral {
                self.error_integral = max_allowed_integral;
            } else if self.error_integral < min_allowed_integral {
                self.error_integral = min_allowed_integral;
            }
        }

        // The PID calculation with additional S setpoint(openloop) and K kick(setpoint derivative) terms
        // P+D+S+K  + I
        partial_sum + self.error_integral
    }

    fn update_sp(&mut self, measurement: T) -> T {
        self.measurement_previous = measurement;
        let error = self.setpoint - measurement;
        self.error_previous = error;

        // The P (no I, no D) calculation with additional S setpoint(openloop) term
        //         P          + S
        self.gains.kp * error + self.gains.ks * self.setpoint
    }

    fn update_spd(&mut self, measurement: T, measurement_delta: T, delta_t: T) -> T {
        self.measurement_previous = measurement;
        let error = self.setpoint - measurement;
        self.error_previous = error;

        self.error_derivative = -measurement_delta / delta_t; // note minus sign, error delta has reverse polarity to measurement delta

        // The PD (no I) calculation with additional S setpoint(openloop) term
        //         P          + D                                     + S
        self.gains.kp * error + self.gains.kd * self.error_derivative + self.gains.ks * self.setpoint
    }

    fn update_skpd(&mut self, measurement: T, measurement_delta: T, delta_t: T) -> T {
        self.update_spd(measurement, measurement_delta, delta_t) + self.gains.kk * self.setpoint_derivative
    }
}

impl<T: Float> Pid<T> {
    /// Partial PID sum, excludes `Iterm`,
    /// has additional S setpoint(openloop) and K kick(setpoint derivative) terms.
    #[inline]
    pub fn partial_sum(&self) -> T {
        self.gains.kp * self.error_previous
            + self.gains.kd * self.error_derivative
            + self.gains.ks * self.setpoint
            + self.gains.kk * self.setpoint_derivative
    }

    pub fn set_setpoint(&mut self, setpoint: T) {
        self.setpoint_previous = self.setpoint;
        self.setpoint = setpoint;
    }

    pub fn set_setpoint_derivative(&mut self, setpoint_derivative: T) {
        self.setpoint_derivative = setpoint_derivative;
    }

    pub fn set_setpoint_for_delta_t(&mut self, setpoint: T, delta_t: T) {
        self.setpoint_previous = self.setpoint;
        self.setpoint = setpoint;
        self.setpoint_derivative = (self.setpoint - self.setpoint_previous) / delta_t;
    }

    pub fn setpoint(&self) -> T {
        self.setpoint
    }

    pub fn previous_setpoint(&self) -> T {
        self.setpoint_previous
    }

    pub fn setpoint_delta(&self) -> T {
        self.setpoint - self.setpoint_previous
    }

    /// previous measurement, useful for `Dterm` filtering.
    pub fn previous_measurement(&self) -> T {
        self.measurement_previous
    }

    pub fn reset_integral(&mut self) {
        self.error_integral = T::zero();
    }

    pub fn switch_integration_off(&mut self) {
        self.ki_saved = self.gains.ki;
        self.gains.ki = T::zero();
        self.error_integral = T::zero();
    }

    pub fn switch_integration_on(&mut self) {
        self.gains.ki = self.ki_saved;
        self.error_integral = T::zero();
    }

    /// Safely update PID gains on the fly without causing an output bump.
    pub fn update_gains(&mut self, new_gains: PidGains<T>) {
        // Calculate the current components using OLD gains
        let old_partial_sum = self.partial_sum();
        let old_error_integral = self.error_integral;

        // Commit the new gains to the struct
        self.set_gains(new_gains);
        if self.error_integral == T::zero() {
            return;
        }

        // Calculate partial_sum using NEW gains
        let new_partial_sum = self.partial_sum();

        self.error_integral = old_error_integral + old_partial_sum - new_partial_sum;

        // Force-clamp the newly calculated accumulator within the Option bounds
        if let Some(output_saturation_value) = self.limits.output_saturation_value {
            let max = output_saturation_value - new_partial_sum;
            let min = -output_saturation_value - new_partial_sum;
            self.error_integral = self.error_integral.clamp(min, max);
        }

        if let Some(max) = self.limits.integral_max {
            self.error_integral = self.error_integral.min(max);
        }
        if let Some(min) = self.limits.integral_min {
            self.error_integral = self.error_integral.max(min);
        }
    }
}

/// Gains for PID controller.
/// Includes classical PID (`kp`, `ki`, and `kd`) gains and also<br>
/// setpoint gain (`ks` - classical feed forward) and<br>
/// setpoint derivative gain (`kk`. kick - called feedforward by Betaflight).<br>
///
/// Uses "independent PID" notation, where the gains are denoted as kp, ki, kd etc.<br>
/// (In the "dependent PID" notation `kc`, `tau_i`, and `tau_d` parameters are used, where `kp = kc`, `ki = kc/tau_i`, `kd = kc*tau_d`).
///
#[derive(Clone, Copy, Debug, PartialEq, Deserialize, Serialize)]
pub struct PidGains<T> {
    /// proportional gain.
    pub kp: T,
    /// integral gain.
    pub ki: T,
    /// derivative gain.
    pub kd: T,
    /// setpoint gain.
    pub ks: T,
    /// setpoint derivative gain ('kick').
    pub kk: T,
}

impl<T: Float> Default for PidGains<T> {
    fn default() -> Self {
        Self::new(T::one(), T::zero(), T::zero(), T::zero(), T::zero())
    }
}

impl<T: Float> PidGains<T> {
    pub const fn new(kp: T, ki: T, kd: T, ks: T, kk: T) -> Self {
        Self { kp, ki, kd, ks, kk }
    }
}

impl<T: Float> Pid<T> {
    /// Return the pid gains. The set value of ki is returned, whether integration is turned on or not.
    pub fn gains(&self) -> PidGains<T> {
        PidGains {
            kp: self.gains.kp,
            ki: self.ki_saved,
            kd: self.gains.kd,
            ks: self.gains.ks,
            kk: self.gains.kk,
        }
    }

    pub fn set_gains(&mut self, gains: PidGains<T>) {
        self.gains = gains;
        self.ki_saved = self.gains.ki;
        if self.gains.ki.abs() <= T::epsilon() {
            // If the new Ki is zero, the integral term cannot contribute to the output.
            // Clear the accumulator to prevent massive hidden windup if Ki is re-enabled later.
            self.error_integral = T::zero();
        }
    }
}

impl<T: Float + Default> From<PidGains<T>> for Pid<T> {
    fn from(pid: PidGains<T>) -> Self {
        Self {
            gains: PidGains {
                kp: pid.kp,
                ki: pid.ki,
                kd: pid.kd,
                ks: pid.ks,
                kk: pid.kk,
            },
            limits: PidLimits {
                integral_max: None,
                integral_min: None,
                output_saturation_value: None,
            },
            ki_saved: pid.ki,
            measurement_previous: T::default(),
            setpoint: T::default(),
            setpoint_previous: T::default(),
            setpoint_derivative: T::default(),
            error_derivative: T::default(),
            error_integral: T::default(),
            error_previous: T::default(),
        }
    }
}

/// Pid integral anti-windup parameters.<br>
///
#[derive(Clone, Copy, Debug, PartialEq, Deserialize, Serialize)]
pub struct PidLimits<T> {
    /// Integral windup limit for positive integral.
    pub integral_max: Option<T>,
    /// Integral windup limit for negative integral.
    pub integral_min: Option<T>,
    /// For integral windup control.
    pub output_saturation_value: Option<T>,
}

impl<T: Float + ConstZero> Default for PidLimits<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T: Float + ConstZero> PidLimits<T> {
    #[must_use]
    pub const fn new() -> Self {
        Self {
            integral_max: None,
            integral_min: None,
            output_saturation_value: None,
        }
    }
}

impl<T: Float> Pid<T> {
    pub fn limits(&self) -> PidLimits<T> {
        self.limits
    }

    pub fn set_limits(&mut self, limits: PidLimits<T>) {
        self.limits = limits;
    }

    pub fn set_integral_max(&mut self, integral_max: T) {
        self.limits.integral_max = Some(integral_max);
    }

    pub fn set_integral_min(&mut self, integral_min: T) {
        self.limits.integral_min = Some(integral_min);
    }

    pub fn set_integral_limit(&mut self, integral_limit: T) {
        self.limits.integral_max = Some(integral_limit);
        self.limits.integral_min = Some(-integral_limit);
    }

    pub fn set_output_saturation_value(&mut self, output_saturation_value: T) {
        self.limits.output_saturation_value = Some(output_saturation_value);
    }
}

/// P, I, D, S, and K errors as calculated by PID controller.<br><br>
///
#[derive(Clone, Copy, Debug, PartialEq, Deserialize, Serialize)]
pub struct PidErrors<T> {
    pub p: T,
    pub i: T,
    pub d: T,
    pub s: T,
    pub k: T,
}

impl<T: Float> Default for PidErrors<T> {
    fn default() -> Self {
        Self::new(T::zero(), T::zero(), T::zero(), T::zero(), T::zero())
    }
}

impl<T: Float> PidErrors<T> {
    #[allow(clippy::many_single_char_names)]
    pub const fn new(p: T, i: T, d: T, s: T, k: T) -> Self {
        Self { p, i, d, s, k }
    }
}

impl<T: Float> Pid<T> {
    // accessor functions to obtain error values
    pub fn error(&self) -> PidErrors<T> {
        PidErrors {
            p: self.error_previous * self.gains.kp,
            i: self.error_integral, // _error_integral is already multiplied by self.pid.ki
            d: self.error_derivative * self.gains.kd,
            s: self.setpoint * self.gains.ks,
            k: self.setpoint_derivative * self.gains.kk,
        }
    }

    pub fn error_raw(&self) -> PidErrors<T> {
        PidErrors {
            p: self.error_previous,
            i: if self.gains.ki == T::zero() {
                T::zero()
            } else {
                self.error_integral / self.gains.ki
            },
            d: self.error_derivative,
            s: self.setpoint,
            k: self.setpoint_derivative,
        }
    }

    /// get previous error, for test code.
    pub fn previous_error(&self) -> T {
        self.error_previous
    }

    /// reset all, for test code.
    pub fn reset_all(&mut self) {
        self.measurement_previous = T::zero();
        self.setpoint = T::zero();
        self.setpoint_previous = T::zero();
        self.setpoint_derivative = T::zero();
        self.error_derivative = T::zero();
        self.error_integral = T::zero();
        self.error_previous = T::zero();
    }
}
