use nalgebra::{ComplexField, Quaternion, SimdComplexField, SimdRealField, UnitQuaternion, Vector3};

pub struct MadgwickFilter {
    // The orientation quaternion (w, i, j, k)
    // We use a raw Quaternion here because during integration it might not be normalized
    pub q: Quaternion<f32>,
    pub beta: f32,
}

impl MadgwickFilter {
    pub fn new(beta: f32) -> Self {
        Self {
            // Initialize as Identity (Flat level): w=1, x=0, y=0, z=0
            q: Quaternion::new(1.0, 0.0, 0.0, 0.0), 
            beta,
        }
    }

    pub fn update_imu(&mut self, gyro: Vector3<f32>, accel: Vector3<f32>, dt: f32) {
        let ax = accel.x;
        let ay = accel.y;
        let az = accel.z;

        // 1. Check for invalid accel data (avoid divide by zero)
        if ax == 0.0 && ay == 0.0 && az == 0.0 {
            return;
        }

        // Normalise accelerometer measurement
        // (We do this manually or use nalgebra's .normalize())
        let accel_norm = accel.normalize();
        let ax = accel_norm.x;
        let ay = accel_norm.y;
        let az = accel_norm.z;

        // Short aliases for current quaternion components
        let q0 = self.q.w;
        let q1 = self.q.i;
        let q2 = self.q.j;
        let q3 = self.q.k;

        // 2. Gradient Descent Step
        // This calculates the direction to nudge the quaternion to align with gravity
        // (Derived from the Jacobian * Error matrix math)
        
        let _2q0 = 2.0 * q0;
        let _2q1 = 2.0 * q1;
        let _2q2 = 2.0 * q2;
        let _2q3 = 2.0 * q3;
        let _4q0 = 4.0 * q0;
        let _4q1 = 4.0 * q1;
        let _4q2 = 4.0 * q2;
        let _8q1 = 8.0 * q1;
        let _8q2 = 8.0 * q2;
        let q0q0 = q0 * q0;
        let q1q1 = q1 * q1;
        let q2q2 = q2 * q2;
        let q3q3 = q3 * q3;

        // Gradient (s)
        let s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        let s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        let s2 = 4.0 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        let s3 = 4.0 * q1q1 * q3 - _2q1 * ax + 4.0 * q2q2 * q3 - _2q2 * ay;

        // Normalize the gradient to treat it as a direction
        // (Manual normalization to avoid allocating a Vector4)
        let norm_s = (s0*s0 + s1*s1 + s2*s2 + s3*s3).simd_sqrt();
        let s0 = s0 / norm_s;
        let s1 = s1 / norm_s;
        let s2 = s2 / norm_s;
        let s3 = s3 / norm_s;

        // 3. Gyroscope Integration Step
        // Rate of change of quaternion from gyro = 0.5 * q * omega
        // nalgebra's quaternion multiplication is q1 * q2
        let q_dot_omega = self.q * Quaternion::new(0.0, gyro.x, gyro.y, gyro.z) * 0.5;

        // 4. Fusion Step
        // Apply feedback (beta) to the gradient and subtract from gyro rate
        let q_dot_w = q_dot_omega.w - self.beta * s0;
        let q_dot_x = q_dot_omega.i - self.beta * s1;
        let q_dot_y = q_dot_omega.j - self.beta * s2;
        let q_dot_z = q_dot_omega.k - self.beta * s3;

        // 5. Integrate to get new Quaternion
        self.q.w += q_dot_w * dt;
        self.q.i += q_dot_x * dt;
        self.q.j += q_dot_y * dt;
        self.q.k += q_dot_z * dt;

        // 6. Normalize the result
        // Essential to prevent errors from accumulating
        self.q = self.q.normalize();
    }

    pub fn to_euler_angles(&self) -> Vector3<f32> {
        let q0 = self.q.w;
        let q1 = self.q.i;
        let q2 = self.q.j;
        let q3 = self.q.k;

        // Roll (x-axis rotation)
        let sinr_cosp = 2.0 * (q0 * q1 + q2 * q3);
        let cosr_cosp = 1.0 - 2.0 * (q1 * q1 + q2 * q2);
        let roll = sinr_cosp.simd_atan2(cosr_cosp);

        // Pitch (y-axis rotation)
        let sinp = 2.0 * (q0 * q2 - q3 * q1);
        let pitch = if sinp.abs() >= 1.0 {
            // Use 90 degrees if out of range
            core::f32::consts::FRAC_PI_2.copysign(sinp) 
        } else {
            sinp.asin()
        };

        // Yaw (z-axis rotation)
        let siny_cosp = 2.0 * (q0 * q3 + q1 * q2);
        let cosy_cosp = 1.0 - 2.0 * (q2 * q2 + q3 * q3);
        let yaw = siny_cosp.simd_atan2(cosy_cosp);

        Vector3::new(roll, pitch, yaw)
    }

}

