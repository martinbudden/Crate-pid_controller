#![allow(unused)]
use assert_float_eq::assert_f32_near;

/// PID controller with feed forward (open loop) control.
///
/// Uses "independent PID" notation, where the gains are denoted as kp, ki, kd etc.
///
/// (In the "dependent PID" notation Kc, tauI, and tauD parameters are used, where kp = Kc, ki = Kc/tauI, kd = Kc*tauD)
///
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PidConstants {
    pub kp: f32, // proportional gain
    pub ki: f32, // integral gain
    pub kd: f32, // derivative gain
    pub ks: f32, // setpoint gain
    pub kk: f32, // setpoint derivative gain ('kick')
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PidError {
    pub p: f32,
    pub i: f32,
    pub d: f32,
    pub s: f32,
    pub k: f32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PidController {
    pid: PidConstants,
    /// saved value of pid.ki, so integration can be switched on and off
    ki_saved: f32,
    measurement_previous: f32,

    setpoint: f32,
    setpoint_previous: f32,
    setpoint_derivative: f32,

    error_derivative: f32,
    error_integral: f32,
    error_previous: f32,

    // integral anti-windup parameters
    /// Integral windup limit for positive integral
    integral_max: f32,
    /// Integral windup limit for negative integral
    integral_min: f32,
    /// Threshold for PID integration. Can be set to afn integral wind-up due to movement in motor's backlash zone.
    integral_threshold: f32,
    /// For integral windup control
    output_saturation_value: f32,
}

impl PidController {
    pub fn new(kp: f32, ki: f32, kd: f32) -> Self {
        Self {
            pid: PidConstants {
                kp: kp,
                ki: ki,
                kd: kd,
                ks: 0.0,
                kk: 0.0,
            },
            ki_saved: ki,
            measurement_previous: 0.0,
            setpoint: 0.0,
            setpoint_previous: 0.0,
            setpoint_derivative: 0.0,
            error_derivative: 0.0,
            error_integral: 0.0,
            error_previous: 0.0,
            integral_max: 0.0,
            integral_min: 0.0,
            integral_threshold: 0.0,
            output_saturation_value: 0.0,
        }
    }

    pub fn set_kp(&mut self, kp: f32) {
        self.pid.kp = kp;
    }

    pub fn set_ki(&mut self, ki: f32) {
        self.pid.ki = ki;
        self.ki_saved = self.pid.ki;
    }

    pub fn set_kd(&mut self, kd: f32) {
        self.pid.kd = kd;
    }

    pub fn set_ks(&mut self, ks: f32) {
        self.pid.ks = ks;
    }

    pub fn set_kk(&mut self, kk: f32) {
        self.pid.kk = kk;
    }

    pub fn set_pid(&mut self, pid: PidConstants) {
        self.pid = pid;
        self.ki_saved = self.pid.ki;
    }

    pub fn kp(&self) -> f32 {
        self.pid.kp
    }

    /// Return the set value of ki, whether integration is turned on or not
    pub fn ki(&self) -> f32 {
        self.ki_saved
    }

    pub fn kd(&self) -> f32 {
        self.pid.kd
    }

    pub fn ks(&self) -> f32 {
        self.pid.ks
    }

    pub fn kk(&self) -> f32 {
        self.pid.kk
    }

    /// Return the set value of ki, whether integration is turned on or not
    pub fn pid_constants(&self) -> PidConstants {
        PidConstants {
            kp: self.pid.kp,
            ki: self.ki_saved,
            kd: self.pid.kd,
            ks: self.pid.ks,
            kk: self.pid.kk,
        }
    }

    pub fn reset_integral(&mut self) {
        self.error_integral = 0.0;
    }

    pub fn switch_integration_off(&mut self) {
        self.ki_saved = self.pid.ki;
        self.pid.ki = 0.0;
        self.error_integral = 0.0;
    }

    pub fn switch_integration_on(&mut self) {
        self.pid.ki = self.ki_saved;
        self.error_integral = 0.0;
    }

    pub fn set_integral_max(&mut self, integral_max: f32) {
        self.integral_max = integral_max;
    }

    pub fn set_integral_min(&mut self, integral_min: f32) {
        self.integral_min = integral_min;
    }

    pub fn set_integral_limit(&mut self, integral_limit: f32) {
        self.integral_max = integral_limit;
        self.integral_min = -integral_limit;
    }

    pub fn set_integral_threshold(&mut self, integral_threshold: f32) {
        self.integral_threshold = integral_threshold;
    }

    pub fn set_output_saturation_value(&mut self, output_saturation_value: f32) {
        self.output_saturation_value = output_saturation_value;
    }

    pub fn set_setpoint(&mut self, setpoint: f32) {
        self.setpoint_previous = self.setpoint;
        self.setpoint = setpoint;
    }

    pub fn set_setpoint_for_delta_t(&mut self, setpoint: f32, delta_t: f32) {
        self.setpoint_previous = self.setpoint;
        self.setpoint = setpoint;
        self.setpoint_derivative = (self.setpoint - self.setpoint_previous) / delta_t;
    }

    pub fn set_setpoint_derivative(&mut self, setpoint_derivative: f32) {
        self.setpoint_derivative = setpoint_derivative;
    }

    pub fn setpoint(&self) -> f32 {
        self.setpoint
    }

    pub fn previous_setpoint(&self) -> f32 {
        self.setpoint_previous
    }

    pub fn setpoint_delta(&self) -> f32 {
        self.setpoint - self.setpoint_previous
    }

    /// previous measurement, useful for DTerm filtering
    pub fn previous_measurement(&self) -> f32 {
        self.measurement_previous
    }

    /// PID update.
    /// ```
    /// # use pid_controller::PidController;
    ///
    /// let delta_t: f32 = 0.01;
    /// let mut pid = PidController::new(0.1, 0.0, 0.0);
    ///
    /// pid.set_setpoint(8.7);
    ///
    /// let measurement:f32 = 9.2;
    /// let output = pid.update(measurement, delta_t);
    ///
    /// assert_eq!(-0.05, output);
    /// ```
    pub fn update(&mut self, measurement: f32, delta_t: f32) -> f32 {
        self.update_delta(
            measurement,
            measurement - self.measurement_previous,
            delta_t,
        )
    }

    /// PID update with `measurement_delta` specified.
    /// This allows the user to filter `measurement_delta` with a filter of their choice.
    ///
    /// ```
    /// # use pid_controller::PidController;
    /// # use filters::FilterPT1;
    ///
    /// let delta_t: f32 = 0.01;
    /// let mut pid = PidController::new(0.1, 0.0, 0.01);
    /// let mut filter = FilterPT1::<f32>::new(1.0);
    ///
    /// pid.set_setpoint(2.1);
    ///
    /// let measurement:f32 = 0.2;
    /// let measurement_delta = measurement - pid.previous_measurement();
    /// let measurement_delta_filtered = filter.filter(measurement_delta);
    ///
    /// let output = pid.update_delta(measurement, measurement_delta_filtered, delta_t);
    ///
    /// assert_eq!(-0.010000005, output);
    ///
    pub fn update_delta(&mut self, measurement: f32, measurement_delta: f32, delta_t: f32) -> f32 {
        self.update_delta_iterm(
            measurement,
            measurement_delta,
            self.setpoint - measurement,
            delta_t,
        )
    }

    pub fn update_delta_iterm(
        &mut self,
        measurement: f32,
        measurement_delta: f32,
        i_term_error: f32,
        delta_t: f32,
    ) -> f32 {
        self.measurement_previous = measurement;
        let error = self.setpoint - measurement;
        self.error_derivative = -measurement_delta / delta_t; // note minus sign, error delta has reverse polarity to measurement delta
        // Partial PID sum, excludes ITerm
        // has additional S setpoint(openloop) and F feedforward(setpoint derivative) terms
        let partial_sum = self.pid.kp * error
            + self.pid.kd * self.error_derivative
            + self.pid.ks * self.setpoint
            + self.pid.kk * self.setpoint_derivative;

        if self.integral_threshold == 0.0 || (error).abs() >= self.integral_threshold {
            // "integrate" the error
            self.error_integral += self.pid.ki * i_term_error * delta_t; // Euler integration
            //self.error_integral += self.pid.ki*0.5F*(iTermError + _errorPrevious)*delta_t; // integration using trapezoid rule
            // Anti-windup via integral clamping
            if self.integral_max > 0.0 && self.error_integral > self.integral_max {
                self.error_integral = self.integral_max;
            } else if self.integral_min < 0.0 && self.error_integral < self.integral_min {
                self.error_integral = self.integral_min;
            }
        }
        self.error_previous = error;

        if self.output_saturation_value > 0.0 {
            // Anti-windup by avoiding output saturation.
            // Check if partialSum + self.error_integral saturates the output
            // If so, the excess value above saturation does not help convergence to the setpoint and will result in
            // overshoot when the P value eventually comes down.
            // So limit the self.error_integral to a value that avoids output saturation.
            if self.error_integral > self.output_saturation_value - partial_sum {
                self.error_integral = (self.output_saturation_value - partial_sum).max(0.0);
            } else if self.error_integral < -self.output_saturation_value - partial_sum {
                self.error_integral = (-self.output_saturation_value - partial_sum).min(0.0);
            }
        }

        // The PID calculation with additional S setpoint(openloop) and F feedforward(setpoint derivative) terms
        //                   P+D+S+F    +  I
        partial_sum + self.error_integral
    }

    pub fn update_sp(&mut self, measurement: f32) -> f32 {
        self.measurement_previous = measurement;
        let error = self.setpoint - measurement;
        self.error_previous = error;

        // The P (no I, no D) calculation with additional S setpoint(openloop) term
        //                   P             + S
        self.pid.kp * error + self.pid.ks * self.setpoint
    }

    pub fn update_spi(&mut self, measurement: f32, delta_t: f32) -> f32 {
        self.measurement_previous = measurement;
        let error = self.setpoint - measurement;
        let partial_sum: f32 = self.pid.kp * error + self.pid.ks * self.setpoint;

        if self.integral_threshold == 0.0 || error.abs() >= self.integral_threshold {
            // "integrate" the error
            self.error_integral += self.pid.ki * error * delta_t; // Euler integration

            //_error_integral += self.pid.ki*0.5F*(error + self.errorPrevious)*delta_t; // integration using trapezoid rule
            // Anti-windup via integral clamping
            if self.integral_max > 0.0 && self.error_integral > self.integral_max {
                self.error_integral = self.integral_max;
            } else if self.integral_min < 0.0 && self.error_integral < self.integral_min {
                self.error_integral = self.integral_min;
            }
        }
        self.error_previous = error;

        if self.output_saturation_value > 0.0 {
            // Anti-windup by avoiding output saturation.
            // Check if partialSum + self.error_integral saturates the output
            // If so, the excess value above saturation does not help convergence to the setpoint and will result in
            // overshoot when the P value eventually comes down.
            // So limit the self.error_integral to a value that avoids output saturation.
            if self.error_integral > self.output_saturation_value - partial_sum {
                self.error_integral = (self.output_saturation_value - partial_sum).max(0.0);
            } else if self.error_integral < -self.output_saturation_value - partial_sum {
                self.error_integral = (-self.output_saturation_value - partial_sum).min(0.0);
            }
        }

        // The PI (no D) calculation with additional S setpoint(openloop) term
        // P + S    + I
        partial_sum + self.error_integral
    }

    pub fn update_skpi(&mut self, measurement: f32, delta_t: f32) -> f32 {
        self.update_spi(measurement, delta_t) + self.pid.kk * self.setpoint_derivative
    }

    pub fn update_spd(&mut self, measurement: f32, measurement_delta: f32, delta_t: f32) -> f32 {
        self.measurement_previous = measurement;
        let error = self.setpoint - measurement;
        self.error_previous = error;

        self.error_derivative = -measurement_delta / delta_t; // note minus sign, error delta has reverse polarity to measurement delta

        // The PD (no I) calculation with additional S setpoint(openloop) term
        //       P                   + D                                   + S
        self.pid.kp * error + self.pid.kd * self.error_derivative + self.pid.ks * self.setpoint
    }

    pub fn update_skpd(&mut self, measurement: f32, measurement_delta: f32, delta_t: f32) -> f32 {
        self.update_spd(measurement, measurement_delta, delta_t)
            + self.pid.kk * self.setpoint_derivative
    }

    // accessor functions to obtain error values
    pub fn error(&self) -> PidError {
        PidError {
            p: self.error_previous * self.pid.kp,
            i: self.error_integral, // _error_integral is already multiplied by self.pid.ki
            d: self.error_derivative * self.pid.kd,
            s: self.setpoint * self.pid.ks,
            k: self.setpoint_derivative * self.pid.kk,
        }
    }

    pub fn error_raw(&self) -> PidError {
        PidError {
            p: self.error_previous,
            i: if self.pid.ki == 0.0 {
                0.0
            } else {
                self.error_integral / self.pid.ki
            },
            d: self.error_derivative,
            s: self.setpoint,
            k: self.setpoint_derivative,
        }
    }

    pub fn error_p(&self) -> f32 {
        self.error_previous * self.pid.kp
    }

    pub fn error_i(&self) -> f32 {
        // self.error_integral is already multiplied by self.pid.ki
        self.error_integral
    }

    pub fn error_d(&self) -> f32 {
        self.error_derivative * self.pid.kd
    }

    pub fn error_s(&self) -> f32 {
        self.setpoint * self.pid.ks
    }

    pub fn error_k(&self) -> f32 {
        self.setpoint_derivative * self.pid.kk
    }

    pub fn error_raw_p(&self) -> f32 {
        self.error_previous
    }

    pub fn error_raw_i(&self) -> f32 {
        if self.pid.ki == 0.0 {
            0.0
        } else {
            self.error_integral / self.pid.ki
        }
    }

    pub fn error_raw_d(&self) -> f32 {
        self.error_derivative
    }

    pub fn error_raw_s(&self) -> f32 {
        self.setpoint
    }

    pub fn error_raw_k(&self) -> f32 {
        self.setpoint_derivative
    }

    /// get previous error, for test code
    pub fn previous_error(&self) -> f32 {
        self.error_previous
    }

    /// reset all, for test code
    fn reset_all(&mut self) {
        self.measurement_previous = 0.0;
        self.setpoint = 0.0;
        self.setpoint_previous = 0.0;
        self.setpoint_derivative = 0.0;
        self.error_derivative = 0.0;
        self.error_integral = 0.0;
        self.error_previous = 0.0;
    }
}

impl From<PidConstants> for PidController {
    fn from(pid: PidConstants) -> Self {
        Self {
            pid: PidConstants {
                kp: pid.kp,
                ki: pid.ki,
                kd: pid.kd,
                ks: pid.ks,
                kk: pid.kk,
            },
            ki_saved: pid.ki,
            measurement_previous: 0.0,
            setpoint: 0.0,
            setpoint_previous: 0.0,
            setpoint_derivative: 0.0,
            error_derivative: 0.0,
            error_integral: 0.0,
            error_previous: 0.0,
            integral_max: 0.0,
            integral_min: 0.0,
            integral_threshold: 0.0,
            output_saturation_value: 0.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn is_normal<T: Sized + Send + Sync + Unpin>() {}

    #[test]
    fn normal_types() {
        is_normal::<PidController>();
    }

    #[test]
    fn default() {
        let pid: PidController = PidController::default();
    }

    #[test]
    fn test_pid_init() {
        let pid = PidController::new(0.0, 0.0, 0.0);
        assert_eq!(0.0, pid.kp());
        assert_eq!(0.0, pid.ki());
        assert_eq!(0.0, pid.kd());
        assert_eq!(0.0, pid.ks());
        assert_eq!(0.0, pid.kk());
        assert_eq!(0.0, pid.setpoint());

        let error = pid.error();
        assert_eq!(error.p, 0.0);
        assert_eq!(error.i, 0.0);
        assert_eq!(error.d, 0.0);
        assert_eq!(error.s, 0.0);
        assert_eq!(error.k, 0.0);
    }

    #[test]
    fn test_pid() {
        let mut pid = PidController::new(5.0, 3.0, 1.0);

        assert_eq!(5.0, pid.kp());
        assert_eq!(3.0, pid.ki());
        assert_eq!(1.0, pid.kd());
        assert_eq!(0.0, pid.ks());
        assert_eq!(0.0, pid.kk());
        assert_eq!(0.0, pid.setpoint());

        let delta_t: f32 = 0.01;
        let input: f32 = 0.5;
        let output = pid.update(input, delta_t);

        let error = pid.error();
        assert_eq!(-input * 5.0, error.p);
        assert_eq!(-input * 3.0 * delta_t, error.i);
        assert_eq!(-input * 1.0 / delta_t, error.d);
        assert_eq!(error.p + error.i + error.d, output);
    }

    #[test]
    fn update() {
        use crate::PidController;
        let delta_t: f32 = 0.01;
        let mut pid = PidController::new(0.1, 0.0, 0.0);
        pid.set_setpoint(8.7);

        let measurement: f32 = 9.2;
        let output = pid.update(measurement, delta_t);
        assert_eq!(-0.05, output);
    }

    #[test]
    fn update_delta() {
        use filters::FilterPT1;
        let delta_t: f32 = 0.01;
        let mut pid = PidController::new(0.1, 0.0, 0.01);
        let mut filter = FilterPT1::<f32>::new(1.0);

        pid.set_setpoint(2.1);

        let measurement: f32 = 0.2;
        let measurement_delta = measurement - pid.previous_measurement();
        let measurement_delta_filtered = filter.filter(measurement_delta);
        let output = pid.update_delta(measurement, measurement_delta_filtered, delta_t);
        assert_eq!(-0.010000005, output);
    }

    #[test]
    fn update_delta_iterm() {
        use filters::FilterPT1;
        let delta_t: f32 = 0.01;
        let mut pid = PidController::new(0.1, 0.05, 0.01);
        let mut filter = FilterPT1::<f32>::new(1.0);

        pid.set_setpoint(2.1);

        let measurement: f32 = 0.2;

        let measurement_delta = measurement - pid.previous_measurement();
        let measurement_delta_filtered = filter.filter(measurement_delta);

        let iterm_relax_factor = 0.5; // set to a constant for the example, in practice it would vary depending on setpoint and/or measurement
        let iterm_error = (pid.setpoint() - measurement)* iterm_relax_factor;

        let output = pid.update_delta_iterm(measurement, measurement_delta_filtered, iterm_error, delta_t);

        assert_eq!(-0.009525006, output);
    }

    #[test]
    fn test_p_controller() {
        let delta_t: f32 = 1.0;
        let mut pid = PidController::new(1.0, 0.0, 0.0);

        assert_eq!(1.0, pid.kp());
        assert_eq!(0.0, pid.ki());
        assert_eq!(0.0, pid.kd());
        assert_eq!(0.0, pid.ks());
        assert_eq!(0.0, pid.kk());
        assert_eq!(0.0, pid.setpoint());

        let mut output = pid.update(0.0, delta_t);
        let mut error = pid.error();
        assert_eq!(0.0, error.p);
        assert_eq!(0.0, error.i);
        assert_eq!(0.0, error.d);
        assert_eq!(0.0, error.s);
        assert_eq!(0.0, error.k);
        assert_eq!(error.p + error.i + error.d, output);

        pid.set_setpoint(5.0);
        output = pid.update(0.0, delta_t);
        error = pid.error();
        assert_eq!(5.0, output);
        assert_eq!(5.0, error.p);
        assert_eq!(error.p + error.i + error.d, output);

        output = pid.update(1.0, delta_t);
        error = pid.error();
        assert_eq!(4.0, output);
        assert_eq!(4.0, error.p);

        output = pid.update(2.0, delta_t);
        error = pid.error();
        assert_eq!(3.0, output);
        assert_eq!(3.0, error.p);

        output = pid.update(3.0, delta_t);
        error = pid.error();
        assert_eq!(2.0, output);
        assert_eq!(2.0, error.p);

        output = pid.update(4.0, delta_t);
        error = pid.error();
        assert_eq!(1.0, output);
        assert_eq!(1.0, error.p);

        output = pid.update(5.0, delta_t);
        error = pid.error();
        assert_eq!(0.0, output);
        assert_eq!(0.0, error.p);

        output = pid.update(6.0, delta_t);
        error = pid.error();
        assert_eq!(-1.0, output);
        assert_eq!(-1.0, error.p);

        output = pid.update(5.0, delta_t);
        error = pid.error();
        assert_eq!(0.0, output);
        assert_eq!(0.0, error.p);
    }

    #[test]
    fn test_pi_controller() {
        let delta_t: f32 = 1.0;
        let mut pid = PidController::new(0.3, 0.2, 0.0);

        assert_eq!(0.3, pid.kp());
        assert_eq!(0.2, pid.ki());
        assert_eq!(0.0, pid.kd());
        assert_eq!(0.0, pid.ks());
        assert_eq!(0.0, pid.kk());
        assert_eq!(0.0, pid.setpoint());

        let mut output = pid.update(0.0, delta_t);
        let mut error = pid.error();
        assert_eq!(0.0, error.p);
        assert_eq!(0.0, error.i);
        assert_eq!(0.0, error.d);
        assert_eq!(0.0, error.s);
        assert_eq!(0.0, error.k);
        assert_eq!(error.p + error.i + error.d, output);

        pid.set_setpoint(5.0);
        output = pid.update(0.0, delta_t);
        error = pid.error();
        assert_eq!(1.5, error.p);
        assert_eq!(5.0, pid.previous_error());
        assert_eq!(1.0, error.i);
        assert_eq!(error.p + error.i, output);
        assert_eq!(0.0, error.s);
        assert_eq!(0.0, error.k);
        assert_eq!(2.5, output);

        output = pid.update(1.0, delta_t);
        error = pid.error();
        assert_eq!(1.2, error.p);
        assert_eq!(4.0, pid.previous_error());
        assert_eq!(1.8, error.i); // 1.0 + (5.0 - 1.0) * 0.2
        assert_eq!(error.p + error.i, output);
        assert_eq!(0.0, error.s);
        assert_eq!(0.0, error.k);
        assert_eq!(3.0, output);

        output = pid.update(4.0, delta_t);
        error = pid.error();
        assert_eq!(0.3, error.p);
        assert_eq!(1.0, pid.previous_error());
        assert_eq!(2.0, error.i); // 1.8 + (5.0 - 4.0) * 0.2
        assert_eq!(error.p + error.i, output);
        assert_eq!(0.0, error.s);
        assert_eq!(0.0, error.k);
        assert_eq!(2.3, output);

        output = pid.update(7.0, delta_t);
        error = pid.error();
        assert_eq!(-0.6, error.p);
        assert_eq!(-2.0, pid.previous_error());
        assert_eq!(1.6, error.i); // 2.0 + -2.0 * 0.2
        assert_eq!(error.p + error.i, output);
        assert_eq!(0.0, error.s);
        assert_eq!(0.0, error.k);
        assert_eq!(1.0, output);

        output = pid.update(6.0, delta_t);
        error = pid.error();
        assert_eq!(-0.3, error.p);
        assert_eq!(-1.0, pid.previous_error());
        assert_eq!(1.4, error.i); // 1.6 + -1.0 * 0.2
        assert_eq!(error.p + error.i, output);
        assert_eq!(0.0, error.s);
        assert_eq!(0.0, error.k);
        assert_f32_near!(1.1, output);

        output = pid.update(5.0, delta_t);
        error = pid.error();
        assert_eq!(0.0, error.p);
        assert_eq!(0.0, pid.previous_error());
        assert_eq!(1.4, error.i); // 1.5 + (0.0 - 1.0) * 0.2 /2
        assert_eq!(error.p + error.i, output);
        assert_eq!(0.0, error.s);
        assert_eq!(0.0, error.k);
        assert_eq!(1.4, output);

        output = pid.update(5.0, delta_t);
        error = pid.error();
        assert_eq!(0.0, error.p);
        assert_eq!(0.0, pid.previous_error());
        assert_eq!(1.4, error.i); // 1.4 + (0.0 + 0.0) * 0.2 / 2
        assert_eq!(error.p + error.i, output);
        assert_eq!(0.0, error.s);
        assert_eq!(0.0, error.k);
        assert_eq!(1.4, output);
    }

    #[test]
    fn test_update_pi() {
        let delta_t: f32 = 1.0;
        let mut pid = PidController::new(0.3, 0.2, 0.0);

        assert_eq!(0.3, pid.kp());
        assert_eq!(0.2, pid.ki());
        assert_eq!(0.0, pid.kd());
        assert_eq!(0.0, pid.ks());
        assert_eq!(0.0, pid.kk());
        assert_eq!(0.0, pid.setpoint());

        let mut output = pid.update_spi(0.0, delta_t);
        let mut error = pid.error();
        assert_eq!(0.0, error.p);
        assert_eq!(0.0, error.i);
        assert_eq!(0.0, error.d);
        assert_eq!(0.0, error.s);
        assert_eq!(0.0, error.k);
        assert_eq!(error.p + error.i + error.d, output);

        pid.set_setpoint(5.0);
        output = pid.update_spi(0.0, delta_t);
        error = pid.error();
        assert_eq!(1.5, error.p);
        assert_eq!(5.0, pid.previous_error());
        assert_eq!(1.0, error.i); // 5.0 * 0.2
        assert_eq!(error.p + error.i, output);
        assert_eq!(0.0, error.s);
        assert_eq!(0.0, error.k);
        assert_eq!(2.5, output);

        output = pid.update_spi(1.0, delta_t);
        error = pid.error();
        assert_eq!(1.2, error.p);
        assert_eq!(4.0, pid.previous_error());
        assert_eq!(1.8, error.i); // 1.0 + 4.0 * 0.2
        assert_eq!(error.p + error.i, output);
        assert_eq!(0.0, error.s);
        assert_eq!(0.0, error.k);
        assert_eq!(3.0, output);

        output = pid.update_spi(4.0, delta_t);
        error = pid.error();
        assert_eq!(0.3, error.p);
        assert_eq!(1.0, pid.previous_error());
        assert_eq!(2.0, error.i); // 1.8 + 1.0 * 0.2
        assert_eq!(error.p + error.i, output);
        assert_eq!(0.0, error.s);
        assert_eq!(0.0, error.k);
        assert_eq!(2.3, output);

        output = pid.update_spi(7.0, delta_t);
        error = pid.error();
        assert_eq!(-0.6, error.p);
        assert_eq!(-2.0, pid.previous_error());
        assert_eq!(1.6, error.i); // 2.0 + -2.0 * 0.2
        assert_eq!(error.p + error.i, output);
        assert_eq!(0.0, error.s);
        assert_eq!(0.0, error.k);
        assert_eq!(1.0, output);

        output = pid.update_spi(6.0, delta_t);
        error = pid.error();
        assert_eq!(-0.3, error.p);
        assert_eq!(-1.0, pid.previous_error());
        assert_eq!(1.4, error.i); // 1.6 + -1.0 * 0.2
        assert_eq!(error.p + error.i, output);
        assert_eq!(0.0, error.s);
        assert_eq!(0.0, error.k);
        assert_f32_near!(1.1, output);

        output = pid.update_spi(5.0, delta_t);
        error = pid.error();
        assert_eq!(0.0, error.p);
        assert_eq!(0.0, pid.previous_error());
        assert_eq!(1.4, error.i); // 1.4 + 0.0 * 0.2
        assert_eq!(error.p + error.i, output);
        assert_eq!(0.0, error.s);
        assert_eq!(0.0, error.k);
        assert_eq!(1.4, output);

        output = pid.update_spi(5.0, delta_t);
        error = pid.error();
        assert_eq!(0.0, error.p);
        assert_eq!(0.0, pid.previous_error());
        assert_eq!(1.4, error.i); // 1.4 + (0.0 + 0.0) * 0.2 / 2
        assert_eq!(error.p + error.i, output);
        assert_eq!(0.0, error.s);
        assert_eq!(0.0, error.k);
        assert_eq!(1.4, output);
    }

    #[test]
    fn test_integration_on_off() {
        let delta_t: f32 = 1.0;
        let mut pid = PidController::new(0.2, 0.3, 0.0);

        assert_eq!(0.0, pid.setpoint());

        let mut output = pid.update(0.0, delta_t);
        assert_eq!(0.0, output);
        let mut error = pid.error();
        assert_eq!(0.0, pid.previous_error());
        assert_eq!(0.0, error.i);

        output = pid.update(2.0, delta_t);
        error = pid.error();
        assert_eq!(-0.4, error.p);
        assert_eq!(-2.0, pid.previous_error());
        assert_eq!(-0.6, error.i); // 0.0 + -2.0* 0.3
        assert_eq!(error.p + error.i, output);
        assert_eq!(-1.0, output);

        output = pid.update(2.0, delta_t);
        error = pid.error();
        assert_eq!(-0.4, error.p);
        assert_eq!(-2.0, pid.previous_error());
        assert_eq!(-1.2, error.i); // -0.6 +-2.0 * 0.3
        assert_eq!(-1.6, output);

        // Integration OFF
        pid.switch_integration_off();
        error = pid.error();
        assert_eq!(-2.0, pid.previous_error());
        assert_eq!(0.0, error.i);

        output = pid.update(0.0, delta_t);
        assert_eq!(0.0, output);
        error = pid.error();
        assert_eq!(0.0, pid.previous_error());
        assert_eq!(0.0, error.i);

        output = pid.update(2.0, delta_t);
        error = pid.error();
        assert_eq!(-0.4, error.p);
        assert_eq!(-2.0, pid.previous_error());
        assert_eq!(0.0, error.i);
        assert_eq!(error.p + error.i, output);
        assert_eq!(-0.4, output);

        output = pid.update(2.0, delta_t);
        error = pid.error();
        assert_eq!(-0.4, error.p);
        assert_eq!(-2.0, pid.previous_error());
        assert_eq!(0.0, error.i);
        assert_eq!(-0.4, output);

        // Integration back ON
        pid.switch_integration_on();
        error = pid.error();
        assert_eq!(-0.4, error.p);
        assert_eq!(-2.0, pid.previous_error());
        assert_eq!(0.0, error.i);

        output = pid.update(0.0, delta_t);
        assert_eq!(0.0, output);
        error = pid.error();
        assert_eq!(0.0, pid.previous_error());
        assert_eq!(0.0, error.i); // 0.0 + 0.0 * 0.3

        output = pid.update(2.0, delta_t);
        error = pid.error();
        assert_eq!(-0.4, error.p);
        assert_eq!(-2.0, pid.previous_error());
        assert_eq!(-0.6, error.i); // - 0.3 + (0.0 - 2.0) * 0.3 / 2
        assert_eq!(error.p + error.i, output);
        assert_eq!(-1.0, output);

        output = pid.update(2.0, delta_t);
        error = pid.error();
        assert_eq!(-0.4, error.p);
        assert_eq!(-2.0, pid.previous_error());
        assert_eq!(-1.2, error.i); // -0.6 + (-2.0 - 2.0) * 0.3 / 2
        assert_eq!(-1.6, output);

        pid.reset_all();
        error = pid.error();
        assert_eq!(0.0, pid.setpoint());
        assert_eq!(0.0, pid.previous_setpoint());
        assert_eq!(0.0, pid.setpoint_delta());
        assert_eq!(0.0, pid.previous_error());
        assert_eq!(0.0, pid.previous_measurement());
        assert_eq!(0.0, error.p);
        assert_eq!(0.0, error.i);
        assert_eq!(0.0, error.d);
        assert_eq!(0.0, error.s);
        assert_eq!(0.0, error.k);
    }

    #[test]
    fn test_integral_limit() {
        let delta_t: f32 = 1.0;
        let mut pid = PidController::new(0.2, 0.3, 0.0);
        pid.set_integral_limit(2.0);

        assert_eq!(0.0, pid.setpoint());

        let mut output = pid.update(0.0, delta_t);
        assert_eq!(0.0, output);
        let mut error = pid.error();
        assert_eq!(0.0, error.i);

        output = pid.update(2.0, delta_t);
        error = pid.error();
        assert_eq!(-0.4, error.p);
        assert_eq!(-0.6, error.i);
        assert_eq!(error.p + error.i, output);
        assert_eq!(-1.0, output);

        output = pid.update(2.0, delta_t);
        error = pid.error();
        assert_eq!(-0.4, error.p);
        assert_eq!(-1.2, error.i);
        assert_eq!(-1.6, output);

        output = pid.update(2.0, delta_t);
        error = pid.error();
        assert_eq!(-0.4, error.p);
        assert_f32_near!(-1.8, error.i);
        assert_eq!(-2.2, output);

        output = pid.update(2.0, delta_t);
        error = pid.error();
        assert_eq!(-0.4, error.p);
        assert_eq!(-2.0, error.i);
        assert_eq!(-2.4, output);

        output = pid.update(2.0, delta_t);
        error = pid.error();
        assert_eq!(-0.4, error.p);
        assert_eq!(-2.0, error.i);
        assert_eq!(-2.4, output);
    }

    #[test]
    fn test_integral_saturation_positive() {
        let delta_t: f32 = 1.0;
        let mut pid = PidController::new(0.2, 0.3, 0.0);
        pid.set_output_saturation_value(1.5);

        assert_eq!(0.0, pid.setpoint());

        let mut output = pid.update(0.0, delta_t);
        assert_eq!(0.0, output);
        let mut error = pid.error();
        assert_eq!(0.0, error.i);

        output = pid.update(2.0, delta_t);
        error = pid.error();
        assert_eq!(-0.4, error.p);
        assert_eq!(-0.6, error.i);
        assert_eq!(error.p + error.i, output);
        assert_eq!(-1.0, output);

        output = pid.update(2.0, delta_t);
        error = pid.error();
        assert_eq!(-0.4, error.p);
        assert_eq!(-1.1, error.i);
        assert_eq!(-1.5, output);

        output = pid.update(2.0, delta_t);
        error = pid.error();
        assert_eq!(-0.4, error.p);
        assert_eq!(-1.1, error.i);
        assert_eq!(-1.5, output);

        output = pid.update(2.0, delta_t);
        error = pid.error();
        assert_eq!(-0.4, error.p);
        assert_eq!(-1.1, error.i);
        assert_eq!(-1.5, output);

        output = pid.update(1.5, delta_t);
        error = pid.error();
        assert_eq!(-0.3, error.p);
        assert_eq!(-1.2, error.i);
        assert_eq!(-1.5, output);

        output = pid.update(1.0, delta_t);
        error = pid.error();
        assert_eq!(-0.2, error.p);
        assert_eq!(-1.3, error.i);
        assert_eq!(-1.5, output);

        output = pid.update(0.5, delta_t);
        error = pid.error();
        assert_eq!(-0.1, error.p);
        assert_eq!(-0.5, pid.previous_error());
        assert_eq!(-1.4, error.i);
        assert_eq!(-1.5, output);

        output = pid.update(0.2, delta_t);
        error = pid.error();
        assert_f32_near!(-0.04, error.p);
        assert_eq!(-0.2, pid.previous_error());
        assert_eq!(-1.46, error.i);
        assert_eq!(-1.5, output);

        output = pid.update(0.0, delta_t);
        error = pid.error();
        assert_eq!(-0.0, error.p);
        assert_eq!(0.0, pid.previous_error());
        assert_eq!(-1.46, error.i); // -1.46 + 0.0 * 0.3
        assert_eq!(-1.46, output);

        output = pid.update(0.0, delta_t);
        error = pid.error();
        assert_eq!(0.0, error.p);
        assert_eq!(0.0, pid.previous_error());
        assert_eq!(-1.46, error.i); // -1.495 + 0.0 * 0.3
        assert_eq!(-1.46, output);
    }

    #[test]
    fn test_integral_saturation_negative() {
        let delta_t: f32 = 1.0;
        let mut pid = PidController::new(0.2, 0.3, 0.0);
        pid.set_output_saturation_value(1.5);

        assert_eq!(0.0, pid.setpoint());

        let mut output = pid.update(0.0, delta_t);
        assert_eq!(0.0, output);
        let mut error = pid.error();
        assert_eq!(0.0, error.i);

        output = pid.update(-2.0, delta_t);
        error = pid.error();
        assert_eq!(0.4, error.p);
        assert_eq!(0.6, error.i);
        assert_eq!(error.p + error.i, output);
        assert_eq!(1.0, output);

        output = pid.update(-2.0, delta_t);
        error = pid.error();
        assert_eq!(0.4, error.p);
        assert_eq!(1.1, error.i);
        assert_eq!(1.5, output);

        output = pid.update(-2.0, delta_t);
        error = pid.error();
        assert_eq!(0.4, error.p);
        assert_eq!(1.1, error.i);
        assert_eq!(1.5, output);

        output = pid.update(-2.0, delta_t);
        error = pid.error();
        assert_eq!(0.4, error.p);
        assert_eq!(1.1, error.i);
        assert_eq!(1.5, output);
    }
}
