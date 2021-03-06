#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

using namespace std;

bool state_cam_global = false, state_scn_global = false;

void camCallback(const sensor_msgs::ImageConstPtr& msg){

}

void feedbackCallback(const std_msgs::Float32ConstPtr& msg){

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "system_state");
    ros::NodeHandle nh;
    ROS_INFO("Ligando monitoramento do sistema.");

    ros::Subscriber    sub_cam = nh.subscribe("/camera/image_raw", 10, camCallback);
    ros::Subscriber    sub_scn = nh.subscribe("/feedback_scan", 10, feedbackCallback);
    ros::Publisher     pub_cam = nh.advertise<std_msgs::Bool>("camera_state", 10);
    ros::Publisher     pub_scn = nh.advertise<std_msgs::Bool>("scan_state"  , 10);

    ros::Rate r(2);
    std_msgs::Bool msg;
    while(ros::ok()){
        msg.data = state_cam_global;
        pub_cam.publish(msg);
        msg.data = state_scn_global;
        pub_scn.publish(msg);

        state_cam_global = (sub_cam.getNumPublishers() == 0) ? false : true;
        state_scn_global = (sub_scn.getNumPublishers() == 0) ? false : true;

        r.sleep();
        ros::spinOnce();
    }

    return 0;
}
