use defmt::Format;

#[derive(Clone, Copy, Debug, Format)]
pub struct PidController {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub i_limit: f32,   // Max integral windup
    pub out_limit: f32, // Max total output

    prev_error: f32,
    integral: f32,
}

impl PidController {
    pub fn new(kp: f32, ki: f32, kd: f32, i_limit: f32, out_limit: f32) -> Self {
        Self {
            kp,
            ki,
            kd,
            i_limit,
            out_limit,
            prev_error: 0.0,
            integral: 0.0,
        }
    }

    pub fn update(&mut self, target: f32, current: f32, dt: f32) -> f32 {
        let error = target - current;


        let p = self.kp * error;


        self.integral += error * dt;
        self.integral = self.integral.clamp(-self.i_limit, self.i_limit);
        let i = self.ki * self.integral;


        // Note: In real flight, D is often calculated on Measurement to avoid "Derivative Kick" on setpoint change
        let d = if dt > 0.0 {
            self.kd * (error - self.prev_error) / dt
        } else {
            0.0
        };
        self.prev_error = error;

        (p + i + d).clamp(-self.out_limit, self.out_limit)
    }
}