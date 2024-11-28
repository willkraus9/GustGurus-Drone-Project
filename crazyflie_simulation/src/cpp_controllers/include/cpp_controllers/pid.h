#include <Eigen/Dense>

class pid {
    public:
        float error_integral;
        float error_previous;
        float kp;
        float ki;
        float kd;
        float integral_max;

        pid(float kp, float ki, float kd, float integral_max);
        float update(float error, float dt);
};