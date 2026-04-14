/// PID controller trait.
pub trait PidController<T> {
    fn update(&mut self, measurement: T, delta_t: T) -> T;
    fn update_delta(&mut self, measurement: T, measurement_delta: T, delta_t: T) -> T;
    fn update_delta_iterm(&mut self, measurement: T, measurement_delta: T, i_term_error: T, delta_t: T) -> T;
    fn update_sp(&mut self, measurement: T) -> T;
    fn update_spi(&mut self, measurement: T, delta_t: T) -> T;
    fn update_skpi(&mut self, measurement: T, delta_t: T) -> T;
    fn update_spd(&mut self, measurement: T, measurement_delta: T, delta_t: T) -> T;
    fn update_skpd(&mut self, measurement: T, measurement_delta: T, delta_t: T) -> T;

}

pub trait UpdatePidController<T> {
    fn adjust_using<P: PidController<T>>(self, pid_controller: &mut P, delta_t: T) -> Self;
    fn adjust_using_d<P: PidController<T>>(self, pid_controller: &mut P, measurement_delta: T, delta_t: T) -> Self;
    fn adjust_using_di<P: PidController<T>>(self, pid_controller: &mut P, measurement_delta: T, i_term_error: T, delta_t: T) -> Self;
    fn adjust_using_sp<P: PidController<T>>(self, pid_controller: &mut P) -> Self;
    fn adjust_using_spi<P: PidController<f32>>(self, pid_controller: &mut P, delta_t: f32) -> Self;
    fn adjust_using_skpi<P: PidController<f32>>(self, pid_controller: &mut P, delta_t: f32) -> Self;
    fn adjust_using_spd<P: PidController<T>>(self, pid_controller: &mut P, measurement_delta: T, delta_t: T) -> Self;
    fn adjust_using_skpd<P: PidController<T>>(self, pid_controller: &mut P, measurement_delta: T, delta_t: T) -> Self;
}

impl UpdatePidController<f32> for f32 {
    fn adjust_using<P: PidController<f32>>(self, pid_controller: &mut P, delta_t: f32) -> Self {
        // self is measurement:f32, pid.update returns f32
        pid_controller.update(self, delta_t)
    }

    fn adjust_using_d<P: PidController<f32>>(self, pid_controller: &mut P, measurement_delta: f32, delta_t: f32) -> Self{
        pid_controller.update_delta(self, measurement_delta, delta_t)
    }

    fn adjust_using_di<P: PidController<f32>>(self, pid_controller: &mut P, measurement_delta: f32, i_term_error: f32, delta_t: f32) -> Self{
        pid_controller.update_delta_iterm(self, measurement_delta, i_term_error, delta_t)
    }

    fn adjust_using_sp<P: PidController<f32>>(self, pid_controller: &mut P) -> Self {
        pid_controller.update_sp(self)
    }

    fn adjust_using_spi<P: PidController<f32>>(self, pid_controller: &mut P, delta_t: f32) -> Self {
        // self is measurement:f32, pid.update returns f32
        pid_controller.update_spi(self, delta_t)
    }
    fn adjust_using_skpi<P: PidController<f32>>(self, pid_controller: &mut P, delta_t: f32) -> Self {
        // self is measurement:f32, pid.update returns f32
        pid_controller.update_skpi(self, delta_t)
    }
    fn adjust_using_spd<P: PidController<f32>>(self, pid_controller: &mut P, measurement_delta: f32, delta_t: f32) -> Self{
        pid_controller.update_spd(self, measurement_delta, delta_t)
    }

    fn adjust_using_skpd<P: PidController<f32>>(self, pid_controller: &mut P, measurement_delta: f32, delta_t: f32) -> Self{
        pid_controller.update_skpd(self, measurement_delta, delta_t)
    }
}

