
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>

//#include <vant/ToOffboard.h> //Antes
#include "/home/laic/catkin_ws/src/vant/include/vant/ToOffboard.h" //RODRIGO ALTEROU ISSO AQUI

#include <iostream>

using namespace std;

//**********************************************************************************
// Codigo para manter o fluxo de pelo menos 2Hz.
//**********************************************************************************

#define TAXA_FLUXO 10 // Defina qual taxa de atualização do tópico de atuação.

vant::ToOffboard thisMsg;

void chatterCallbackToOffboard(const vant::ToOffboard::ConstPtr& msg){
    thisMsg = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "toPublish");
    ros::NodeHandle n;
    ros::Rate loop_rate(TAXA_FLUXO);

    ros::Subscriber sub = n.subscribe("/setpoint_offboard", 1, chatterCallbackToOffboard);

    ros::Publisher chatter_pub_vel = n.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel",1);  // Para velocidade
    geometry_msgs::TwistStamped msgVel;
    msgVel.header.frame_id = "base_link";
    msgVel.header.stamp = ros::Time::now();
    msgVel.header.seq=1;

    ros::Publisher chatter_pub_pose = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",1); // Para deslocamento
    geometry_msgs::PoseStamped msgPose;
    msgPose.header.frame_id = "base_link";
    msgPose.header.stamp = ros::Time::now();
    msgPose.header.seq=1;

    while(ros::ok())
    {
        ros::spinOnce();
        cout << "Valor: " << thisMsg.index << endl;

        if(thisMsg.index == 1){ // Tipo Position
            cout << "Position Mode" << endl;
            msgPose.pose = thisMsg.PoseStamped.pose;
            chatter_pub_pose.publish(msgPose);

        }
        if (thisMsg.index == 0){ // Tipo Velocity
            cout << "Velocity Mode" << endl;
            msgVel.twist = thisMsg.TwistStamped.twist;
            chatter_pub_vel.publish(msgVel);
        }

        loop_rate.sleep();

    }

    return 0;
}








