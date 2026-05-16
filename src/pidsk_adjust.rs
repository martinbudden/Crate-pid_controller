use crate::pidsk_controller::PidController;

#[allow(clippy::doc_paragraphs_missing_punctuation)]
/// Trait to allow `PidController` to be used with method-call syntax, ie:<br>
/// `let output = measurement.adjust_using(&mut pid_controller, dt)`
/// ```
/// # use pidsk_controller::{Pidf32,PidGainsf32};
/// # use pidsk_controller::UpdatePidController;
/// # let mut pid_controller = Pidf32::new(PidGainsf32 { kp:0.1, ki:0.1, kd:0.01, ks: 0.0, kk: 0.0 });
/// # let dt = 0.001_f32;
/// let measurement: f32 = 9.2;
/// let output = measurement.adjust_using(&mut pid_controller, dt);
/// ```
pub trait UpdatePidController<T> {
    fn adjust_using<P: PidController<T>>(self, pid_controller: &mut P, delta_t: T) -> Self;
    fn adjust_using_d<P: PidController<T>>(self, pid_controller: &mut P, measurement_delta: T, delta_t: T) -> Self;
    fn adjust_using_di<P: PidController<T>>(
        self,
        pid_controller: &mut P,
        measurement_delta: T,
        iterm_error: T,
        delta_t: T,
    ) -> Self;
    fn adjust_using_sp<P: PidController<T>>(self, pid_controller: &mut P) -> Self;
    fn adjust_using_spd<P: PidController<T>>(self, pid_controller: &mut P, measurement_delta: T, delta_t: T) -> Self;
    fn adjust_using_skpd<P: PidController<T>>(self, pid_controller: &mut P, measurement_delta: T, delta_t: T) -> Self;
}

impl UpdatePidController<f32> for f32 {
    fn adjust_using<P: PidController<f32>>(self, pid_controller: &mut P, delta_t: f32) -> Self {
        // self is measurement:f32, pid.update returns f32
        pid_controller.update(self, delta_t)
    }

    fn adjust_using_d<P: PidController<f32>>(
        self,
        pid_controller: &mut P,
        measurement_delta: f32,
        delta_t: f32,
    ) -> Self {
        pid_controller.update_delta(self, measurement_delta, delta_t)
    }

    fn adjust_using_di<P: PidController<f32>>(
        self,
        pid_controller: &mut P,
        measurement_delta: f32,
        iterm_error: f32,
        delta_t: f32,
    ) -> Self {
        pid_controller.update_delta_iterm(self, measurement_delta, iterm_error, delta_t)
    }

    fn adjust_using_sp<P: PidController<f32>>(self, pid_controller: &mut P) -> Self {
        pid_controller.update_sp(self)
    }

    fn adjust_using_spd<P: PidController<f32>>(
        self,
        pid_controller: &mut P,
        measurement_delta: f32,
        delta_t: f32,
    ) -> Self {
        pid_controller.update_spd(self, measurement_delta, delta_t)
    }

    fn adjust_using_skpd<P: PidController<f32>>(
        self,
        pid_controller: &mut P,
        measurement_delta: f32,
        delta_t: f32,
    ) -> Self {
        pid_controller.update_skpd(self, measurement_delta, delta_t)
    }
}
