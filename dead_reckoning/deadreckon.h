#include <math.h>

// include OpenCV header file
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

typedef struct {
    float x;
    float y;
    float th;
} POSE2D;

typedef struct {
    float pi = 3.141592;
    float rad_to_deg = 180.0/pi;
    float deg_to_rad = pi / 180.0;
    float pulse_per_rev = 400000.0;
    float radius_wheel  = 199.597/2.0 / 1000.0;  // (m)
    float dist_bet_wheels = 420.7 / 1000.0;      // (m)
    float dist_OnePulse = (2.0*pi*radius_wheel)/pulse_per_rev;
} DR_PARAM;

enum {ENCODER_GYRO, ENCODER_ONLY};

class DeadReckon {
private:
    /* data */
    POSE2D pose;
    int mode;
    DR_PARAM param;

public:
    DeadReckon();
    ~DeadReckon();

    void init(int m);
    POSE2D update(int encoder_l, int encoder_r, int gyro);
};

DeadReckon::DeadReckon()
{
    
}

DeadReckon::~DeadReckon()
{

}

void DeadReckon::init(int m=ENCODER_ONLY) {
    mode = m;
    pose.x = 0;
    pose.y = 0;
    pose.th = 0;
}

POSE2D DeadReckon::update(int enc_l, int enc_r, int gyro) {
    float gyro_rad = gyro * param.deg_to_rad;

    // convert pulses to distance(m)
    float dist_wL     = enc_l*param.dist_OnePulse;
    float dist_wR     = enc_r*param.dist_OnePulse;
    float dist_w_mean = 0.5*(dist_wL + dist_wR);    // (m)


    float dX = dist_w_mean*cos( pose.th);    // th at previous step
    float dY = dist_w_mean*sin( pose.th);
    pose.x = pose.x + dX;
    pose.y = pose.y + dY;
    
    if (mode==ENCODER_GYRO) {        // %%%% type 1 (encoder + gyro)
        pose.th = gyro_rad;
    } else {        // %%%% type 2 (encoder only)
        float dth = (1.0/param.dist_bet_wheels)*(dist_wR - dist_wL);  // (rad)
    
        pose.th = pose.th + dth;
        
    }
    return pose;
}


