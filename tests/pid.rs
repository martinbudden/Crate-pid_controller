use pidsk_controller::{Pid,Pidf32,PidGainsf32,PidErrorf32};
use pidsk_controller::PidController;
#[cfg(test)]
mod tests {
    #![allow(clippy::float_cmp)]
    use super::*;
    #[allow(unused)]
    use approx::assert_abs_diff_eq;
    macro_rules! assert_near {
        ($left:expr, $right:expr) => {
            approx::assert_abs_diff_eq!($left, $right, epsilon = 4e-6);
        };
    }

    #[allow(unused)]
    fn is_normal<T: Sized + Send + Sync + Unpin>() {}
    fn is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}

    #[test]
    fn normal_types() {
        is_full::<Pidf32>();
        is_full::<PidGainsf32>();
        is_full::<PidErrorf32>();
    }

    #[test]
    fn default() {
        let pid: Pidf32 = Pidf32::default();
        assert_eq!(1.0, pid.kp());
        assert_eq!(0.0, pid.ki());
        assert_eq!(0.0, pid.kd());
        assert_eq!(0.0, pid.ks());
        assert_eq!(0.0, pid.kk());
        assert_eq!(0.0, pid.setpoint());
    }

    #[test]
    fn test_pid_init() {
        let pid = Pidf32::new(0.0, 0.0, 0.0);
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
        let mut pid = Pidf32::new(5.0, 3.0, 1.0);

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
        let delta_t: f32 = 0.01;
        let mut pid = Pid::<f32>::new(0.1, 0.0, 0.0);
        pid.set_setpoint(8.7);

        let measurement: f32 = 9.2;
        let output = pid.update(measurement, delta_t);
        assert_eq!(-0.05, output);
    }

    #[test]
    fn update_delta() {
        use signal_filters::{Pt1Filterf32, SignalFilter};
        let delta_t: f32 = 0.01;
        let mut pid = Pid::<f32>::new(0.1, 0.0, 0.01);
        let mut filter = Pt1Filterf32::new(1.0);

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
        let mut pid = Pidf32::new(0.1, 0.05, 0.01);
        let mut filter = Pt1Filterf32::new(1.0);

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
        let mut pid = Pidf32::new(1.0, 0.0, 0.0);

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
        let mut pid = Pidf32::new(0.3, 0.2, 0.0);

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
        let mut pid = Pidf32::new(0.3, 0.2, 0.0);

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
        assert_near!(1.1, output);

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
        let mut pid = Pidf32::new(0.2, 0.3, 0.0);

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
        let mut pid = Pidf32::new(0.2, 0.3, 0.0);
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
        let mut pid = Pidf32::new(0.2, 0.3, 0.0);
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
        let mut pid = Pidf32::new(0.2, 0.3, 0.0);
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
