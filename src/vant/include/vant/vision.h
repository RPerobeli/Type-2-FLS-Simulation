#ifndef VISION_H
#define VISION_H

#define PI 3.14159265
#define DEG2RAD(DEG) ((DEG)*((PI)/(180.000000)))
#define RAD2DEG(RAD) ((RAD)*((180.000000)/(PI)))
#define HT_M_C_aux 1 // Homogeneous Transformation Matrix of Marker {M} relative Camera Frame {C}
#define HT_V_I 2 // Homogeneous Transformation Matrix of Vant {V} relative Inertial Frame {I}

#define MULTI 2  // Defining the subscriber topic:
#define SINGLE 1 // Multi came from multi tag landing area. It is necessary to RUN the multiAlvar node.
                 // Single come direct from ar_track_alvar

#define LAND_1 0 // Defining the landing area pattern type
#define LAND_2 1
#define LAND_3 2

#define GET_NAME(n) #n

// ##### ROS ######## //
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>


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

using namespace std;

class vision
{
public:
    vision(ros::NodeHandle); // Overload to use after MultiTagsLanding
    vision(ros::NodeHandle, const int, int, int); // To use to create a MultiTagslanding
    ~vision();

    struct flagList{
        int flagEmpty; // Is it empty? No (False = 0) Yes (True = 1)
        int flagChooseID;
    };

    struct option{
        enum {single, multi = 2};
    };

    struct version{
        enum {V0,V1,V2,V3};// JP MUDOU PARA TESTE DA OSCILACAO DO CHAVEAMENTO DO MARCADOR
    };

    struct VISION_ARGUMENTS{
        static vision::option opt;
        static vision::version v;
    };

// Defining the landig area pattern type used in MultiAlvar. It is not used in SINGLE option (check defines)
    void defineLandingPattern(int, int);
    void landV0(int);
    void landV1(int);
    void landV2(int);
    void landV3(int);// JP MUDOU PARA TESTE DA OSCILACAO DO CHAVEAMENTO DO MARCADOR

    tf::StampedTransform getPoseAlvarHTmatrix();
    ar_track_alvar_msgs::AlvarMarker getPoseAlvar();
    vision::flagList getFlags();

    int getNMARKER();

private:
    ros::NodeHandle nodeHandle;

    const int nMarkerMax;                    // Number of the greatest marker on a Bundle;


    int VERSION,     //  Version of the landing marker (bundle marker)
        nMarkers,    // Number of markers used in multiAlvar (bundle marker)
        *markers;   //

    ros::Subscriber subMultiAlvar, // ROS subscriber used
                    subAlvar;


    ar_track_alvar_msgs::AlvarMarker *msg2,         // Used in multiAlvar
                                     msgAlvar;      // Used in singleAlvar

    geometry_msgs::PoseStamped poseAlvar;

    tf::StampedTransform TH_M_C_aux; // Homogeneous transform between Marker relative to Camera

    vision::flagList f;

    // Callbacks
        void chatterCallbackPoseMultiAlvar(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr&);
        void chatterCallbackPoseAlvar(const ar_track_alvar_msgs::AlvarMarker::ConstPtr&);

    // Relative with the SINGLE option
        void chooseTag2();
        void fillVector(int*);

};

#endif // VISION_H
