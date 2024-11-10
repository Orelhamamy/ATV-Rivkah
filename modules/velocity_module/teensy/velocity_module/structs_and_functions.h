struct KalmanFilter{
    double X_k;                 // Estimated State
    double F_k;                 // State transition model
    double B_k;                 // Control-input model
    double P_k;                 // State covariance 
    double P_k_;
    double Q_k;                 // Process noise covariance
    double H_k;                 // Observation model
    double R_k;                 // Observation noise covariance
    double K_k;                 // Kalman gain
};

struct PID
{
    float Kp;              // Proportional gain constant
    float Ki;              // Integral gain constant
    float Kd;              // Derivative gain constant
    float Kaw;             // Anti-windup gain constant
    float T_C;             // Time constant for derivative filtering
    float max;             // Max command
    float min;             // Min command
    float max_change;        // Max change of the command
    float integral = 0;        // Integral term
    float err_prev = 0;        // Previous error
    float deriv_prev = 0;      // Previous derivative
    float command_sat_prev = 0;// Previous saturated command
    float command_prev = 0;    // Previous command
    float delta_cmd = 0;           // control output change
};


void KF_predict(KalmanFilter *KF, double speedErr){
    KF->X_k = KF->F_k * KF->X_k + KF->B_k * speedErr;
    KF->P_k = KF->F_k * KF->P_k_ * KF->F_k + KF->Q_k;
    KF->P_k_ = KF->P_k;
}

void KF_update(KalmanFilter *KF, double vel){
    
    double y_res = vel - KF->H_k * KF->X_k;
    KF->K_k = KF->P_k * KF->H_k / (KF->H_k * KF->P_k * KF->H_k + KF->R_k); // Kalman Gain
    KF->X_k = KF->X_k + KF->K_k * y_res;
    KF->P_k = (1 - KF->K_k * KF->H_k) * KF->P_k;
    KF->P_k_ = KF->P_k;
}

double PID_Step(PID *pid, double state, double target, double dt){
    
    double err = target - state;
    double cmd; 
    double cmd_sat;
    double deriv_filt;
    // Integral term calculation with anti-windup
    pid->integral += pid->Ki * err * dt + pid->Kaw * (pid->command_sat_prev - pid->command_prev)*dt;
    // Derivative term calculation using filtered derivative method
    deriv_filt = (dt * (err - pid->err_prev) + pid->T_C * pid->deriv_prev) / (dt + pid->T_C);

    pid->err_prev = err;
    pid->deriv_prev = deriv_filt;

    cmd = pid->Kp * err + pid->integral + pid->Kd * deriv_filt;
    pid->command_prev = cmd;
    // Clamping cmd_sat [pid->min, pid->max]
    cmd_sat = cmd > pid->max ? pid->max : cmd < pid->min ? pid->min : cmd;
    // Check the rate
    cmd_sat = cmd_sat > pid->command_sat_prev + pid->max_change ? pid->command_sat_prev + pid->max_change : cmd_sat;
    cmd_sat = cmd_sat < pid->command_sat_prev - pid->max_change ? pid->command_sat_prev - pid->max_change : cmd_sat;
    pid->delta_cmd = cmd_sat - pid-> command_sat_prev;
    pid->command_sat_prev = cmd_sat;
    return cmd_sat;
}

PID pidCreate(float Kp, float Ki, float Kd, float Kaw, float T_C, float max, float min, float max_change){
    return PID {Kp, Ki, Kd, Kaw, T_C, max, min, max_change};
}

KalmanFilter KF_Create(double F_k, double B_k, double Q_k, double H_k, double R_k)  {
    return KalmanFilter{0, F_k, B_k, 0, Q_k, Q_k, H_k, R_k, 0};
    }