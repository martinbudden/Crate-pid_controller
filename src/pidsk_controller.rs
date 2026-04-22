use core::ops::{Add, Div, Mul, Neg, Sub};
use num_traits::{One, Signed, Zero};

use crate::pid::PidController;

pub type Pidf32 = Pid<f32>;
pub type PidGainsf32 = PidGains<f32>;
pub type PidErrorf32 = PidErrors<f32>;

pub type Pidf64 = Pid<f64>;
pub type PidGainsf64 = PidGains<f64>;
pub type PidErrorf64 = PidErrors<f64>;

/// PID controller with open loop control.
/// This includes setpoint gain (classical feed forward) and setpoint derivative gain (kick - called feedforward by Betaflight).
///
/// Uses "independent PID" notation, where the gains are denoted as kp, ki, kd etc.
///
/// (In the "dependent PID" notation `kc`, `tau_i`, and `tau_d` parameters are used, where `kp = kc`, `ki = kc/tau_i`, `kd = kc*tau_d`).
///
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PidGains<T> {
    pub kp: T, // proportional gain
    pub ki: T, // integral gain
    pub kd: T, // derivative gain
    pub ks: T, // setpoint gain
    pub kk: T, // setpoint derivative gain ('kick')
}
impl<T> Default for PidGains<T>
where
    T: Copy + Default + Zero + One,
{
    fn default() -> Self {
        Self::new(T::one(), T::zero(), T::zero())
    }
}

impl<T> PidGains<T>
where
    T: Copy + Default,
{
    pub fn new(kp: T, ki: T, kd: T) -> Self {
        Self {
            kp,
            ki,
            kd,
            ks: T::default(),
            kk: T::default(),
        }
    }
}

/// Pid integral anti-windup parameters.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PidLimits<T> {
    /// Integral windup limit for positive integral.
    integral_max: T,
    /// Integral windup limit for negative integral.
    integral_min: T,
    /// Threshold for PID integration. Can be set to avoid integral wind-up due to movement in motor's backlash zone.
    integral_threshold: T,
    /// For integral windup control.
    output_saturation_value: T,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PidErrors<T> {
    pub p: T,
    pub i: T,
    pub d: T,
    pub s: T,
    pub k: T,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pid<T> {
    gains: PidGains<T>,
    /// saved value of pid.ki, so integration can be switched on and off.
    ki_saved: T,
    measurement_previous: T,

    setpoint: T,
    setpoint_previous: T,
    setpoint_derivative: T,

    error_derivative: T,
    error_integral: T,
    error_previous: T,

    limits: PidLimits<T>,
}

/// Default `Pid`.
/// ```
/// # use pidsk_controller::Pidf32;
/// # use num_traits::Zero;
///
/// let pid = Pidf32::default();
///
/// assert_eq!(1.0, pid.kp());
/// ```
impl<T> Default for Pid<T>
where
    T: Copy + Default + Zero + One,
{
    fn default() -> Self {
        Self::new(T::one(), T::zero(), T::zero())
    }
}

impl<T> Pid<T>
where
    T: Copy + Default,
{
    pub fn new(kp: T, ki: T, kd: T) -> Self {
        Self {
            gains: PidGains::new(kp, ki, kd),
            ki_saved: ki,
            measurement_previous: T::default(),
            setpoint: T::default(),
            setpoint_previous: T::default(),
            setpoint_derivative: T::default(),
            error_derivative: T::default(),
            error_integral: T::default(),
            error_previous: T::default(),
            limits: PidLimits::default(),
        }
    }
}

impl<T> PidController<T> for Pid<T>
where
    T: Copy + Zero + Signed + PartialOrd + Add<Output = T> + Mul<Output = T>,
{
    /// PID update.
    /// ```
    /// # use pidsk_controller::{Pidf32,PidController};
    /// let delta_t: f32 = 0.01;
    /// let mut pid = Pidf32::new(0.1, 0.0, 0.0);
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
    /// # use pidsk_controller::{Pidf32,PidController};
    /// # use signal_filters::{Pt1Filterf32,SignalFilter};
    /// let delta_t: f32 = 0.01;
    /// let mut pid = Pidf32::new(0.1, 0.0, 0.01);
    /// let mut filter = Pt1Filterf32::new(1.0);
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
        self.error_derivative = -measurement_delta / delta_t; // note minus sign, error delta has reverse polarity to measurement delta
        // Partial PID sum, excludes ITerm
        // has additional S setpoint(openloop) and K kick(setpoint derivative) terms
        let partial_sum = self.gains.kp * error
            + self.gains.kd * self.error_derivative
            + self.gains.ks * self.setpoint
            + self.gains.kk * self.setpoint_derivative;

        if self.limits.integral_threshold == T::zero() || (error).abs() >= self.limits.integral_threshold {
            // "integrate" the error
            self.error_integral = self.error_integral + self.gains.ki * iterm_error * delta_t; // Euler integration
            //self.error_integral += self.pid.ki*0.5F*(iTermError + _errorPrevious)*delta_t; // integration using trapezoid rule
            // Anti-windup via integral clamping
            if self.limits.integral_max > T::zero() && self.error_integral > self.limits.integral_max {
                self.error_integral = self.limits.integral_max;
            } else if self.limits.integral_min < T::zero() && self.error_integral < self.limits.integral_min {
                self.error_integral = self.limits.integral_min;
            }
        }
        self.error_previous = error;

        if self.limits.output_saturation_value > T::zero() {
            // Anti-windup by avoiding output saturation.
            // Check if partialSum + self.error_integral saturates the output
            // If so, the excess value above saturation does not help convergence to the setpoint and will result in
            // overshoot when the P value eventually comes down.
            // So limit the self.error_integral to a value that avoids output saturation.
            if self.error_integral > self.limits.output_saturation_value - partial_sum {
                self.error_integral = self.limits.output_saturation_value - partial_sum;
                if self.error_integral < T::zero() {
                    self.error_integral = T::zero();
                }
            } else if self.error_integral < -self.limits.output_saturation_value - partial_sum {
                self.error_integral = -self.limits.output_saturation_value - partial_sum;
                if self.error_integral > T::zero() {
                    self.error_integral = T::zero();
                }
            }
        }

        // The PID calculation with additional S setpoint(openloop) and K kick(setpoint derivative) terms
        //  P+D+S+K +  I
        partial_sum + self.error_integral
    }

    fn update_sp(&mut self, measurement: T) -> T {
        self.measurement_previous = measurement;
        let error = self.setpoint - measurement;
        self.error_previous = error;

        // The P (no I, no D) calculation with additional S setpoint(openloop) term
        //       P                   + S
        self.gains.kp * error + self.gains.ks * self.setpoint
    }

    fn update_spi(&mut self, measurement: T, delta_t: T) -> T {
        self.measurement_previous = measurement;
        let error = self.setpoint - measurement;
        let partial_sum: T = self.gains.kp * error + self.gains.ks * self.setpoint;

        if self.limits.integral_threshold == T::zero() || error.abs() >= self.limits.integral_threshold {
            // "integrate" the error
            self.error_integral = self.error_integral + self.gains.ki * error * delta_t; // Euler integration

            //_error_integral += self.pid.ki*0.5F*(error + self.errorPrevious)*delta_t; // integration using trapezoid rule
            // Anti-windup via integral clamping
            if self.limits.integral_max > T::zero() && self.error_integral > self.limits.integral_max {
                self.error_integral = self.limits.integral_max;
            } else if self.limits.integral_min < T::zero() && self.error_integral < self.limits.integral_min {
                self.error_integral = self.limits.integral_min;
            }
        }
        self.error_previous = error;

        if self.limits.output_saturation_value > T::zero() {
            // Anti-windup by avoiding output saturation.
            // Check if partial_sum + self.error_integral saturates the output
            // If so, the excess value above saturation does not help convergence to the setpoint and will result in
            // overshoot when the P value eventually comes down.
            // So limit the self.error_integral to a value that avoids output saturation.
            if self.error_integral > self.limits.output_saturation_value - partial_sum {
                self.error_integral = self.limits.output_saturation_value - partial_sum;
                if self.error_integral < T::zero() {
                    self.error_integral = T::zero();
                }
            } else if self.error_integral < -self.limits.output_saturation_value - partial_sum {
                self.error_integral = -self.limits.output_saturation_value - partial_sum;
                if self.error_integral < T::zero() {
                    self.error_integral = T::zero();
                }
            }
        }

        // The PI (no D) calculation with additional S setpoint(openloop) term
        // P + S    + I
        partial_sum + self.error_integral
    }

    fn update_skpi(&mut self, measurement: T, delta_t: T) -> T {
        self.update_spi(measurement, delta_t) + self.gains.kk * self.setpoint_derivative
    }

    fn update_spd(&mut self, measurement: T, measurement_delta: T, delta_t: T) -> T {
        self.measurement_previous = measurement;
        let error = self.setpoint - measurement;
        self.error_previous = error;

        self.error_derivative = -measurement_delta / delta_t; // note minus sign, error delta has reverse polarity to measurement delta

        // The PD (no I) calculation with additional S setpoint(openloop) term
        //       P                   + D                                   + S
        self.gains.kp * error + self.gains.kd * self.error_derivative + self.gains.ks * self.setpoint
    }

    fn update_skpd(&mut self, measurement: T, measurement_delta: T, delta_t: T) -> T {
        self.update_spd(measurement, measurement_delta, delta_t) + self.gains.kk * self.setpoint_derivative
    }
}

impl<T> Pid<T>
where
    T: Copy + Zero + Neg<Output = T>,
{
    pub fn set_kp(&mut self, kp: T) {
        self.gains.kp = kp;
    }

    pub fn set_ki(&mut self, ki: T) {
        self.gains.ki = ki;
        self.ki_saved = self.gains.ki;
    }

    pub fn set_kd(&mut self, kd: T) {
        self.gains.kd = kd;
    }

    pub fn set_ks(&mut self, ks: T) {
        self.gains.ks = ks;
    }

    pub fn set_kk(&mut self, kk: T) {
        self.gains.kk = kk;
    }

    pub fn set_pid(&mut self, pid: PidGains<T>) {
        self.gains = pid;
        self.ki_saved = self.gains.ki;
    }

    pub fn kp(&self) -> T {
        self.gains.kp
    }

    /// Return the set value of ki, whether integration is turned on or not.
    pub fn ki(&self) -> T {
        self.ki_saved
    }

    pub fn kd(&self) -> T {
        self.gains.kd
    }

    pub fn ks(&self) -> T {
        self.gains.ks
    }

    pub fn kk(&self) -> T {
        self.gains.kk
    }

    /// Return the set value of ki, whether integration is turned on or not.
    pub fn pid_constants(&self) -> PidGains<T> {
        PidGains {
            kp: self.gains.kp,
            ki: self.ki_saved,
            kd: self.gains.kd,
            ks: self.gains.ks,
            kk: self.gains.kk,
        }
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

    pub fn set_integral_max(&mut self, integral_max: T) {
        self.limits.integral_max = integral_max;
    }

    pub fn set_integral_min(&mut self, integral_min: T) {
        self.limits.integral_min = integral_min;
    }

    pub fn set_integral_limit(&mut self, integral_limit: T) {
        self.limits.integral_max = integral_limit;
        self.limits.integral_min = -integral_limit;
    }

    pub fn set_integral_threshold(&mut self, integral_threshold: T) {
        self.limits.integral_threshold = integral_threshold;
    }

    pub fn set_output_saturation_value(&mut self, output_saturation_value: T) {
        self.limits.output_saturation_value = output_saturation_value;
    }

    pub fn set_setpoint(&mut self, setpoint: T) {
        self.setpoint_previous = self.setpoint;
        self.setpoint = setpoint;
    }

    pub fn set_setpoint_derivative(&mut self, setpoint_derivative: T) {
        self.setpoint_derivative = setpoint_derivative;
    }

    pub fn setpoint(&self) -> T {
        self.setpoint
    }

    pub fn previous_setpoint(&self) -> T {
        self.setpoint_previous
    }

    /// previous measurement, useful for `Dterm` filtering.
    pub fn previous_measurement(&self) -> T {
        self.measurement_previous
    }
}

impl<T> Pid<T>
where
    T: Copy + Sub<Output = T> + Div<Output = T>,
{
    pub fn setpoint_delta(&self) -> T {
        self.setpoint - self.setpoint_previous
    }

    pub fn set_setpoint_for_delta_t(&mut self, setpoint: T, delta_t: T) {
        self.setpoint_previous = self.setpoint;
        self.setpoint = setpoint;
        self.setpoint_derivative = (self.setpoint - self.setpoint_previous) / delta_t;
    }
}

impl<T> Pid<T>
where
    T: Copy + Zero + Signed + PartialEq + Add<Output = T> + Mul<Output = T>,
{
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
                self.gains.ki
            } else {
                self.error_integral / self.gains.ki
            },
            d: self.error_derivative,
            s: self.setpoint,
            k: self.setpoint_derivative,
        }
    }

    pub fn error_p(&self) -> T {
        self.error_previous * self.gains.kp
    }

    pub fn error_i(&self) -> T {
        // self.error_integral is already multiplied by self.pid.ki
        self.error_integral
    }

    pub fn error_d(&self) -> T {
        self.error_derivative * self.gains.kd
    }

    pub fn error_s(&self) -> T {
        self.setpoint * self.gains.ks
    }

    pub fn error_k(&self) -> T {
        self.setpoint_derivative * self.gains.kk
    }

    pub fn error_raw_p(&self) -> T {
        self.error_previous
    }

    pub fn error_raw_i(&self) -> T {
        if self.gains.ki == T::zero() {
            self.gains.ki
        } else {
            self.error_integral / self.gains.ki
        }
    }

    pub fn error_raw_d(&self) -> T {
        self.error_derivative
    }

    pub fn error_raw_s(&self) -> T {
        self.setpoint
    }

    pub fn error_raw_k(&self) -> T {
        self.setpoint_derivative
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

impl<T> From<PidGains<T>> for Pid<T>
where
    T: Default + Copy,
{
    fn from(pid: PidGains<T>) -> Self {
        Self {
            gains: PidGains {
                kp: pid.kp,
                ki: pid.ki,
                kd: pid.kd,
                ks: pid.ks,
                kk: pid.kk,
            },
            ki_saved: pid.ki,
            measurement_previous: T::default(),
            setpoint: T::default(),
            setpoint_previous: T::default(),
            setpoint_derivative: T::default(),
            error_derivative: T::default(),
            error_integral: T::default(),
            error_previous: T::default(),
            limits: PidLimits {
                integral_max: T::default(),
                integral_min: T::default(),
                integral_threshold: T::default(),
                output_saturation_value: T::default(),
            },
        }
    }
}
