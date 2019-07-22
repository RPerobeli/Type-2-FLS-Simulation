#ifndef MISSION_H
#define MISSION_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/CommandCode.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointPushRequest.h>

/// #### SISTEMA #### //
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>

// #### MATRICIAL ### //
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>

//#include <vant/vaant.h>
//#include "../rapidjson/document.h" //ESTAVA ASSIM ANTES
#include "/home/laic/catkin_ws/src/vant/include/rapidjson/document.h" //RODRIGO ALTEROU

using namespace std;
//using namespace rapidjson;

class mission
{
public:
    mission(char end[]);
    ~mission();


    // JSON
    void setWpTolerance(int);
    void readJson(char[]);
    void printJson();
    int getJsonSize() const;
    // Sem argumento, vetor dinâmico
    mavros_msgs::Waypoint* getJsonWp() const;
    // Com argumento int, pega um índice específico
    mavros_msgs::Waypoint getJsonWp(int) const;

//    void sendWaypoint(vaant::rosProperties);
private:
    // JSON
    rapidjson::Document j;
    int waypointTolerance;
    int numWp;
    mavros_msgs::Waypoint* wp;
};

#endif // MISSION_H
