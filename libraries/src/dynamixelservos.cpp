#include "../include/dynamixelservos.h"

/////////////////////////////////////////////////////////////////////////////////////////////////
DynamixelServos::DynamixelServos(){
    // Lendo arquivo onde esta a calibracao de tilt
    char* home;
    home = getenv("HOME");
    ifstream calib_file(string(home)+"/calib_servos.txt");
    string params_line;
    vector<float> params;
    if(calib_file.is_open()){
        getline(calib_file, params_line);
        stringstream iss1(params_line);
        for(string s; iss1 >> s; )
            params.push_back(stof(s));
        // Referencias medidas na IMU
        deg_bottom_tilt = params[0];//36.4;
        deg_up_tilt     = params[1];//-37.8;
        raw_bottom_tilt = params[2];//640;
        raw_up_tilt     = params[3];//378;

        params.clear(); params_line.clear();
        getline(calib_file, params_line);
        stringstream iss2(params_line);
        for(string s; iss2 >> s; )
            params.push_back(stof(s));
        // Parametros de limite e horizontal
        raw_min_pan  = params[0];//35;
        raw_max_pan  = params[1];//4077;
        raw_min_tilt = params[2];//690;
        raw_hor_tilt = params[3];//511;
        raw_max_tilt = params[4];//350;
    }
    calib_file.close();

    raw_deg_tilt = (raw_up_tilt - raw_bottom_tilt)/(deg_up_tilt - deg_bottom_tilt);
    deg_raw_tilt = 1.0/raw_deg_tilt;

    // Limites em raw e deg para os servos de pan
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
