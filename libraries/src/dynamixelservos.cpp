#include "../include/dynamixelservos.h"

/////////////////////////////////////////////////////////////////////////////////////////////////
DynamixelServos::DynamixelServos(){
    // Referencias medidas na IMU
    deg_bottom_tilt = 36.4;
    deg_up_tilt = -37.8;
    raw_bottom_tilt = 640;
    raw_up_tilt = 378;

    raw_deg_tilt = (raw_up_tilt - raw_bottom_tilt)/(deg_up_tilt - deg_bottom_tilt);
    deg_raw_tilt = 1.0/raw_deg_tilt;

    // Limites em raw e deg para os servos de pan e tilt
    raw_min_pan = 35;
    raw_max_pan = 4077;
    raw_min_tilt = 690;
    raw_hor_tilt = 511;
    raw_max_tilt = 350;

    raw_deg_pan  = 11.37777777, deg_raw_pan  = 0.08789062;

    deg_min_pan  = raw_min_pan*deg_raw_pan;
    deg_max_pan  = raw_max_pan*deg_raw_pan;
    deg_min_tilt = this->raw2deg(raw_min_tilt, "tilt");
    deg_hor_tilt = 0;
    deg_max_tilt = this->raw2deg(raw_max_tilt, "tilt");
}
/////////////////////////////////////////////////////////////////////////////////////////////////
DynamixelServos::~DynamixelServos(){
    ros::shutdown();
    ros::waitForShutdown();
}
/////////////////////////////////////////////////////////////////////////////////////////////////
int DynamixelServos::deg2raw(float deg, string motor){
    if(motor == "pan")
        return int(deg*raw_deg_pan);
    else
        return int((deg - deg_bottom_tilt)*raw_deg_tilt + raw_bottom_tilt);
}
/////////////////////////////////////////////////////////////////////////////////////////////////
float DynamixelServos::raw2deg(int raw, string motor){
    if(motor == "pan")
        return float(raw)*deg_raw_pan;
    else
        return (float(raw) - raw_bottom_tilt)*deg_raw_tilt + deg_bottom_tilt;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
