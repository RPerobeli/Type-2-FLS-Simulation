
// SISTEMAS //
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <fstream>
#include <math.h>
#include <algorithm>
#include <ctime>
#include <string.h>
#include <sys/stat.h>

#define LOGNAME_FORMAT "%d-%b-%y_%H:%M:%S"
#define LOGNAME_SIZE 20
//////////////

/// ROS //////
#include <ros/ros.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_listener.h>
//rodrigo alterou aqui
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
//fim mudanças rodrigo
//////////////

//#include <vaant.h> //ANTES ESTAVA ASSIM
#include "/home/laic/catkin_ws/src/vant/include/vant/vaant.h"    //RODRIGO ALTEROU ISSO AQUI
//#include <ToOffboard.h> //ANTES
#include "/home/laic/catkin_ws/src/vant/include/vant/ToOffboard.h" //RODRIGO ALTEROU ISSO AQUI



int main(int argc, char **argv)
{

    ros::init(argc, argv, "send_position");
    ros::NodeHandle n;
    vant::ToOffboard pubMsg;
    pubMsg.index = 1;

    ros::Rate loop_rate(10);
    ros::Publisher chatter_pub = n.advertise<vant::ToOffboard>("/setpoint_offboard",1);

    if(argc == 1){
        ROS_WARN("Nenhuma posicao informada. Utilizando o DEFAULT {x,y,z,Yaw(graus) } = {0,0,5,0}");
        pubMsg.PoseStamped.pose.position.z = 5;
    }

    else{

        cout << "Enviando {x,y,z,Yaw(graus)}: "<< atof(argv[1])<< " " << atof(argv[2]) << " " << atof(argv[3]) << " " << atof(argv[4]) << endl;
        pubMsg.PoseStamped.pose.position.x = atof(argv[1]);
        pubMsg.PoseStamped.pose.position.y = atof(argv[2]);
        pubMsg.PoseStamped.pose.position.z = atof(argv[3]);
        tf::quaternionTFToMsg(tf::createQuaternionFromYaw(DEG2RAD(atof(argv[4]))),pubMsg.PoseStamped.pose.orientation);

    }

    int cont =0;
    while(ros::ok() && cont < 10)
    {
        chatter_pub.publish(pubMsg);
        loop_rate.sleep();
        cont++;
    }

    return 0;
}

