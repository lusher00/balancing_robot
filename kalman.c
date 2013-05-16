/* Kalman filter variables and constants */
const double Q_angle = 0.01; // Process noise covariance for the accelerometer - Sw
const double Q_gyro = 0.003; // Process noise covariance for the gyro - Sw
const double R_angle = 0.3; // Measurement noise covariance - Sv

double angle = 0.0; // The angle output from the Kalman filter
double bias = 0.0; // The gyro bias calculated by the Kalman filter
double P_00 = 0.0, P_01 = 0.0, P_10 = 0.0, P_11 = 0.0;
double y, S;
double K_0, K_1;
double kalman(double newAngle, double newRate, double dt) 
{

	// Update xhat - Project the state ahead
	angle += dt * (newRate - bias);

	// Update estimation error covariance - Project the error covariance ahead
	//P_00 += dt * (dt*P_11 - P_01 - P_10 + Q_angle);
	P_00 += -dt * (P_10 + P_01) + Q_angle * dt;
	P_01 += -dt * P_11;
	P_10 += -dt * P_11;
	P_11 += Q_gyro * dt;

	// Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
	// Calculate Kalman gain - Compute the Kalman gain
	S = P_00 + R_angle;
	K_0 = P_00 / S;
	K_1 = P_10 / S;

	// Calculate angle and resting rate - Update estimate with measurement zk
	y = newAngle - angle;
	angle += K_0 * y;
	bias += K_1 * y;

	// Calculate estimation error covariance - Update the error covariance
	P_00 -= K_0 * P_00;
	P_01 -= K_0 * P_01;
	P_10 -= K_1 * P_00;
	P_11 -= K_1 * P_01;


	return angle;
}
