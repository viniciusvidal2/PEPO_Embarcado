#include "ros/ros.h"
#include "communication/state.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Bool.h"

using namespace std;

bool state_global = false;

void camCallback(const sensor_msgs::ImageConstPtr& msg){

}

bool switch_state(communication::state::Request &req, communication::state::Response &res){
    // Avaliando mudanca de sistema dita na entrada
    switch(req.state){
    case 0:
//        state_global = false;
        res.result = 1;
    case 1:
//        state_global = true;
        res.result = 1;
    default:
        res.result = 0;
    }
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "system_state");
    ros::NodeHandle nh;
    ROS_INFO("Ligando monitoramento do sistema.");

    ros::Subscriber    sub_cam = nh.subscribe("/camera/image_raw", 10, camCallback);
    ros::ServiceServer service = nh.advertiseService("switch_state", switch_state);
    ros::Publisher     pub     = nh.advertise<std_msgs::Bool>("system_state", 10);

    ros::Rate r(2);
    while(ros::ok()){
        std_msgs::Bool msg;
        msg.data = state_global;
        pub.publish(msg);

        state_global = (sub_cam.getNumPublishers() == 0) ? false : true;

        r.sleep();
        ros::spinOnce();
    }

    return 0;
}
