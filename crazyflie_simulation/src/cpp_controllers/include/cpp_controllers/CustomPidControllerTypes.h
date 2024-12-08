

typedef struct current_state_s {
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
    double d_x;
    double d_y;
    double d_z;
    double d_roll;
    double d_pitch;
    double d_yaw;
} current_state_t;

typedef struct desired_state_s {
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
    double d_x;
    double d_y;
    double d_z;
    double d_roll;
    double d_pitch;
    double d_yaw;
} desired_state_t;

typedef struct pid_gains_s{
    float kp_x;
    float ki_x;
    float kd_x;
    float integral_max_x;
    float output_max_x;
    float kp_y;
    float ki_y;
    float kd_y;
    float integral_max_y;
    float output_max_y;                                                             
    float kp_z;
    float ki_z;
    float kd_z;
    float integral_max_z;
    float output_max_z;
    float kp_roll;
    float ki_roll;
    float kd_roll;
    float integral_max_roll;
    float output_max_roll;
    float kp_pitch;
    float ki_pitch;
    float kd_pitch;
    float integral_max_pitch;
    float output_max_pitch;
    float kp_yaw;
    float ki_yaw;
    float kd_yaw;
    float integral_max_yaw;
    float output_max_yaw;
} pid_gains_t;


// typedef struct current_state_s {
//     double x;
//     double y;
//     double z;
//     double roll;
//     double pitch;
//     double yaw;
// } current_state_t;

// typedef struct desired_state_s {
//     double x;
//     double y;
//     double z;
//     double yaw;
// } desired_state_t;

