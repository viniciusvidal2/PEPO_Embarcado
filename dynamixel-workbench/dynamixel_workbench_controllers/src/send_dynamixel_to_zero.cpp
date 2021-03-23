#include "ros/ros.h"
#include <cstdlib>

#include <dynamixel_workbench_msgs/DynamixelCommand.h>
#include <dynamixel_workbench_msgs/DynamixelInfo.h>
#include <dynamixel_workbench_msgs/JointCommand.h>
#include <dynamixel_workbench_msgs/DynamixelState.h>
#include "../../../libraries/include/dynamixelservos.h"

ros::Subscriber sub_dyn;
DynamixelServos *ds;
ros::ServiceClient comando_motor;

/// Callback dos servos
///
void servosCallback(const nav_msgs::OdometryConstPtr& msg){

    dynamixel_workbench_msgs::JointCommand cmd;
    cmd.request.pan_pos  = ds->raw_min_pan;
    cmd.request.tilt_pos = ds->raw_hor_tilt;

    if(abs(msg->pose.pose.position.x - ds->raw_min_pan) > 20 || abs(msg->pose.pose.position.y - ds->raw_hor_tilt) > 35){
        if(comando_motor.call(cmd))
            ros::Duration(0.5).sleep();
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "send_dynamixel_to_zero");

    ros::NodeHandle nh;
    // Servico para mover os servos
    comando_motor = nh.serviceClient<dynamixel_workbench_msgs::JointCommand>("/joint_command");

    // Classe com valores dos servos
    ds = new DynamixelServos();

    // Subscriber para saber onde os servos estao e continuar a enviar para a origem
    sub_dyn = nh.subscribe("/dynamixel_angulos_sincronizados", 2, servosCallback);

    dynamixel_workbench_msgs::JointCommand cmd;
    cmd.request.pan_pos  = ds->raw_min_pan;
    cmd.request.tilt_pos = ds->raw_hor_tilt;
    if(comando_motor.call(cmd))
        ROS_INFO("Indo para a posicao 0 ...");

    ros::spin();
    return 0;
}
