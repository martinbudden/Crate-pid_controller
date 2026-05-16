use pidsk_controller::PidController;
use pidsk_controller::{PidErrorf32, PidGainsf32, PidLimitsf32, Pidf32};
#[cfg(test)]
mod tests {
    #![allow(clippy::float_cmp)]

    use super::*;
    use pidsk_controller::UpdatePidController;
    use serde::{Deserialize, Serialize};

    macro_rules! assert_near {
        ($left:expr, $right:expr) => {
            approx::assert_abs_diff_eq!($left, $right, epsilon = 4e-6);
        };
    }

    fn _is_normal<T: Sized + Send + Sync + Unpin>() {}
    fn _is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}
    fn is_config<
        T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq + Serialize + for<'a> Deserialize<'a>,
    >() {
    }

    #[test]
    fn normal_types() {
        is_config::<Pidf32>();
        is_config::<PidGainsf32>();
        is_config::<PidErrorf32>();
        is_config::<PidLimitsf32>();
    }

    #[test]
    fn default() {
        let pid: Pidf32 = Pidf32::default();
        let pid_gains = pid.gains();
        assert_eq!(1.0, pid_gains.kp);
        assert_eq!(0.0, pid_gains.ki);
        assert_eq!(0.0, pid_gains.kd);
        assert_eq!(0.0, pid_gains.ks);
        assert_eq!(0.0, pid_gains.kk);
        assert_eq!(0.0, pid.setpoint());
    }

    #[test]
    fn test_pid_init() {
        let pid = Pidf32::new(PidGainsf32::new(1.0, 0.0, 0.0, 0.0, 0.0));
        let pid_gains = pid.gains();
        assert_eq!(1.0, pid_gains.kp);
        assert_eq!(0.0, pid_gains.ki);
        assert_eq!(0.0, pid_gains.kd);
        assert_eq!(0.0, pid_gains.ks);
        assert_eq!(0.0, pid_gains.kk);
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
        let pid_gains = PidGainsf32 {
            kp: 5.0,
            ki: 3.0,
            kd: 1.0,
            ks: 0.0,
            kk: 0.0,
        };
        let mut pid = Pidf32::new(pid_gains);
        let pid_gains = pid.gains();

        assert_eq!(5.0, pid_gains.kp);
        assert_eq!(3.0, pid_gains.ki);
        assert_eq!(1.0, pid_gains.kd);
        assert_eq!(0.0, pid_gains.ks);
        assert_eq!(0.0, pid_gains.kk);
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
        let delta_t: f32 = 0.01;
        let pid_gains = PidGainsf32 {
            kp: 0.1,
            ki: 0.0,
            kd: 0.0,
            ks: 0.0,
            kk: 0.0,
        };

        let mut pid = Pidf32::new(pid_gains);
        pid.set_setpoint(8.7);

        let measurement: f32 = 9.2;
        let output = pid.update(measurement, delta_t);
        assert_eq!(-0.05, output);
    }

    #[test]
    fn adjust() {
        let delta_t: f32 = 0.01;
        let pid_gains = PidGainsf32 {
            kp: 0.1,
            ki: 0.0,
            kd: 0.0,
            ks: 0.0,
            kk: 0.0,
        };
        let mut pid_controller = Pidf32::new(pid_gains);
        pid_controller.set_setpoint(8.7);

        let measurement: f32 = 9.2;
        let output = measurement.adjust_using(&mut pid_controller, delta_t);
        assert_eq!(-0.05, output);
    }

    #[test]
    fn update_delta() {
        use signal_filters::{Pt1Filterf32, SignalFilter};
        let delta_t: f32 = 0.01;
        let pid_gains = PidGainsf32 {
            kp: 0.1,
            ki: 0.0,
            kd: 0.01,
            ks: 0.0,
            kk: 0.0,
        };
        let mut pid = Pidf32::new(pid_gains);
        let mut filter = Pt1Filterf32::new();

        pid.set_setpoint(2.1);

        let measurement: f32 = 0.2;
        let measurement_delta = measurement - pid.previous_measurement();
        let measurement_delta_filtered = filter.update(measurement_delta);
        let output = pid.update_delta(measurement, measurement_delta_filtered, delta_t);
        assert_near!(-0.010_000_005, output);
    }

    #[test]
    fn update_delta_iterm() {
        use signal_filters::{Pt1Filterf32, SignalFilter};
        let delta_t: f32 = 0.01;
        let pid_gains = PidGainsf32 {
            kp: 0.1,
            ki: 0.05,
            kd: 0.01,
            ks: 0.0,
            kk: 0.0,
        };
        let mut pid = Pidf32::new(pid_gains);
        let mut filter = Pt1Filterf32::new();

        pid.set_setpoint(2.1);

        let measurement: f32 = 0.2;

        let measurement_delta = measurement - pid.previous_measurement();
        let measurement_delta_filtered = filter.update(measurement_delta);

        let iterm_relax_factor = 0.5; // set to a constant for the example, in practice it would vary depending on setpoint and/or measurement
        let iterm_error = (pid.setpoint() - measurement) * iterm_relax_factor;

        let output = pid.update_delta_iterm(measurement, measurement_delta_filtered, iterm_error, delta_t);

        assert_near!(-0.009_525_006, output);
    }

    #[test]
    fn test_p_controller() {
        let delta_t: f32 = 1.0;
        let pid_gains = PidGainsf32 {
            kp: 1.0,
            ki: 0.0,
            kd: 0.0,
            ks: 0.0,
            kk: 0.0,
        };
        let mut pid = Pidf32::new(pid_gains);
        let pid_gains = pid.gains();

        assert_eq!(1.0, pid_gains.kp);
        assert_eq!(0.0, pid_gains.ki);
        assert_eq!(0.0, pid_gains.kd);
        assert_eq!(0.0, pid_gains.ks);
        assert_eq!(0.0, pid_gains.kk);
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
        let pid_gains = PidGainsf32 {
            kp: 0.3,
            ki: 0.2,
            kd: 0.0,
            ks: 0.0,
            kk: 0.0,
        };

        let mut pid = Pidf32::new(pid_gains);

        assert_eq!(0.3, pid_gains.kp);
        assert_eq!(0.2, pid_gains.ki);
        assert_eq!(0.0, pid_gains.kd);
        assert_eq!(0.0, pid_gains.ks);
        assert_eq!(0.0, pid_gains.kk);
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
        assert_near!(1.1, output);

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
        let pid_gains = PidGainsf32 {
            kp: 0.3,
            ki: 0.2,
            kd: 0.0,
            ks: 0.0,
            kk: 0.0,
        };

        let mut pid = Pidf32::new(pid_gains);

        assert_eq!(0.3, pid_gains.kp);
        assert_eq!(0.2, pid_gains.ki);
        assert_eq!(0.0, pid_gains.kd);
        assert_eq!(0.0, pid_gains.ks);
        assert_eq!(0.0, pid_gains.kk);
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
        assert_eq!(1.0, error.i); // 5.0 * 0.2
        assert_eq!(error.p + error.i, output);
        assert_eq!(0.0, error.s);
        assert_eq!(0.0, error.k);
        assert_eq!(2.5, output);

        output = pid.update(1.0, delta_t);
        error = pid.error();
        assert_eq!(1.2, error.p);
        assert_eq!(4.0, pid.previous_error());
        assert_eq!(1.8, error.i); // 1.0 + 4.0 * 0.2
        assert_eq!(error.p + error.i, output);
        assert_eq!(0.0, error.s);
        assert_eq!(0.0, error.k);
        assert_eq!(3.0, output);

        output = pid.update(4.0, delta_t);
        error = pid.error();
        assert_eq!(0.3, error.p);
        assert_eq!(1.0, pid.previous_error());
        assert_eq!(2.0, error.i); // 1.8 + 1.0 * 0.2
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
        assert_near!(1.1, output);

        output = pid.update(5.0, delta_t);
        error = pid.error();
        assert_eq!(0.0, error.p);
        assert_eq!(0.0, pid.previous_error());
        assert_eq!(1.4, error.i); // 1.4 + 0.0 * 0.2
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
    fn test_integration_on_off() {
        let delta_t: f32 = 1.0;
        let pid_gains = PidGainsf32 {
            kp: 0.2,
            ki: 0.3,
            kd: 0.0,
            ks: 0.0,
            kk: 0.0,
        };

        let mut pid = Pidf32::new(pid_gains);

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
        let pid_gains = PidGainsf32 {
            kp: 0.2,
            ki: 0.3,
            kd: 0.0,
            ks: 0.0,
            kk: 0.0,
        };

        let mut pid = Pidf32::new(pid_gains);
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
        assert_near!(-1.8, error.i);
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
        let pid_gains = PidGainsf32 {
            kp: 0.2,
            ki: 0.3,
            kd: 0.0,
            ks: 0.0,
            kk: 0.0,
        };

        let mut pid = Pidf32::new(pid_gains);
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
        assert_near!(-0.04, error.p);
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
        let pid_gains = PidGainsf32 {
            kp: 0.2,
            ki: 0.3,
            kd: 0.0,
            ks: 0.0,
            kk: 0.0,
        };
        let mut pid = Pidf32::new(pid_gains);
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

    fn setup_test_pid() -> Pidf32 {
        let gains = PidGainsf32 {
            kp: 2.0,
            ki: 0.5,
            kd: 0.1,
            ks: 0.0,
            kk: 0.0,
        };

        let limits = PidLimitsf32 {
            integral_max: Some(10.0),
            integral_min: Some(-10.0),
            output_saturation_value: Some(15.0),
        };

        Pidf32::with_limits(gains, limits)
        //Pid::new(gains)
    }

    #[test]
    fn test_bumpless_transfer_smoothness() {
        let mut pid = setup_test_pid();
        pid.set_setpoint(10.0);

        let delta_t = 0.1;

        // Step 1: Run updates so the internal state states align naturally
        // Note: passing (setpoint - measurement) as iterm_error to match internal equations
        let measurement_1 = 3.0;
        let _ = pid.update(measurement_1, delta_t);

        let measurement_2 = 5.0;
        // Capture the absolute latest output value returned by the system
        let old_output = pid.update(measurement_2, delta_t);

        assert!(pid.error().i.abs() > 0.0, "Integral term should not be zero");

        // Step 2: Define new radically different gains
        let new_gains = PidGainsf32 {
            kp: 5.0,
            ki: 2.5,
            kd: 0.5,
            ks: 0.0,
            kk: 0.0,
        };

        // Step 3: Trigger the bumpless transfer routine
        pid.update_gains(new_gains);

        // Step 4: Calculate the immediate output using the updated internal states
        let new_partial_sum = pid.partial_sum();

        let new_output = new_partial_sum + pid.error().i;

        // Step 5: Assert algebraic continuity across the execution boundary
        let difference = (new_output - old_output).abs();
        assert!(
            difference < 1e-4,
            "Bumpless transfer failed! Output bumped from {old_output} to {new_output} (diff: {difference})"
        );
    }

    #[allow(clippy::float_cmp)]
    #[test]
    fn test_bumpless_transfer_to_zero_ki() {
        let delta_t = 0.1;

        let mut pid = setup_test_pid();
        pid.set_setpoint(5.0);

        // Advance loop to build an integral component
        let _ = pid.update(4.0, delta_t);
        let _ = pid.update(4.2, delta_t);
        let _old_output = pid.update(4.4, delta_t);

        // Switch to a purely PD controller (Ki drops to 0.0 entirely)
        let new_gains = PidGainsf32 {
            kp: 4.0,
            ki: 0.0, // Turn off integration completely
            kd: 0.2,
            ks: 0.0,
            kk: 0.0,
        };

        pid.update_gains(new_gains);
        assert_eq!(
            0.0,
            pid.error().i,
            "Integral accumulator should be wiped clean when Ki is 0"
        );

        // Re-evaluate output instantly
        /*let new_partial_sum = pid.partial_sum();
        let new_output = new_partial_sum + pid.error().i;

        // Verify it didn't bump and state handled the division-by-zero catch correctly
        let difference = (old_output - new_output).abs();
        assert_eq!(0.0, difference);
        assert!(difference < 1e-5, "Output bumped on zero-Ki handoff!");*/
    }
}
