#ifndef vaant_h
#define vaant_h

#define PI 3.14159265
#define DEG2RAD(DEG) ((DEG)*((PI)/(180.000000)))
#define RAD2DEG(RAD) ((RAD)*((180.000000)/(PI)))
#define HT_M_C_aux 1 // Homogeneous Transformation Matrix of Marker {M} relative Camera Frame {C}
#define HT_V_I 2 // Homogeneous Transformation Matrix of Vant {V} relative Inertial Frame {I}

// ##### ROS ######## //
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
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
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/WaypointClearRequest.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/WaypointList.h>
#include <tf/transform_listener.h>
#include <mavros_msgs/Mavlink.h>
#include <mavros_msgs/CommandHome.h>
//#include <mavros_msgs/mavlink_convert.h>

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

// #### JSON ### //
//#include "../rapidjson/document.h"

// Nossas bibliotecas
#include <vant/mission.h>
#include <vant/vision.h>
#include <vant/ToOffboard.h> // definition of the mensagem to toPublish node

#include "/opt/ros/kinetic/include/mavlink/v2.0/common/mavlink.h"

using namespace std;
using namespace Eigen;
using namespace rapidjson;

class vaant{

public:
    vaant(ros::NodeHandle);
    vaant();
    ~vaant();

    struct displacement{
        double x, y, z;
        int maneuver;
    };

    struct velocity{
        double x, y, z;
    };

    struct coord{
        double lat, lon, alt;
    };

    struct hbParameters{
        double x, y, z,         // ENU
            harversineDistance, // Haversine Distance
            bearingAngle,       // Bearing Angle Ref to North
            euclidean;
    };


    struct greatCircle{
        double harversineDistance;
        float bearingAngle;
    };

    struct angles{
        double roll,pitch,yaw;
    };

    struct logReached{ // para aplicar no goToDepois
        //time t;
        vaant::displacement posReached;
    };

    struct MODE{
        enum modes {MANUAL,ACRO,ALTCTL,POSCTL,OFFBOARD,RATTITUDE,AUTO_MISSION,AUTO_LOITER,AUTO_RTL,AUTO_LAND,AUTO_RTGS,AUTO_READY,AUTO_TAKEOFF};
    };

    geometry_msgs::PoseStamped poseMavros;
    sensor_msgs::NavSatFix poseGPSmavros;

    //Functions
    int readSetpointListfromFile(char*,int);

    static double transformHaversine(vaant::coord,vaant::coord);
    static double transformBearing(vaant::coord,vaant::coord);
    int applyHaversineBearingToArray();
    int applyHaversineBearingToArrayRefHome(vaant::coord);

    static hbParameters applyHaversineBearing(vaant::coord, vaant::coord);
    static hbParameters applyHaversineBearing(mavros_msgs::Waypoint, mavros_msgs::Waypoint);

    static coord TransformWPinCoord(mavros_msgs::Waypoint);

    static tf::Vector3 applyENU2MavrosRef(tf::Vector3);

    int getNumSetpointsGreatCircle();
    int getNumSetpointsDelta();
    int getNumSetpointsDeltaRefHome();
    int getNumSetpoints();

    void clearSetpointList();
    void clearSetpointGreatCircleList();
    void clearSetpointDelta();
    void clearSetpointDeltaRefHome();

    vaant::displacement getSetpoint(int); // List with the setpoints in the txt. mission file
    vaant::greatCircle  getSetpointGreatCircle(int index); // List with the bearing and haversine numbers between 2 coordinates
    vaant::displacement getSetpointDelta(int index); // List with the X,Y,Z delta between 2 coordinates
    vaant::displacement getSetpointDeltaRefHome(int index); // List with the X,Y,Z delta between 2 coordinates relative to home

    double goToPosition(tf::Vector3, float);
    double goToPosition(tf::Vector3, float, vision &);

    void chatterCallback_PoseMavros(const geometry_msgs::PoseStamped::ConstPtr&);
    void chatterCallback_GPSMavros(const sensor_msgs::NavSatFix::ConstPtr&);
    void chatterCallback_GPSMavrosAltRel(const std_msgs::Float64::ConstPtr&);
    void chatterCallback_State(const mavros_msgs::State::ConstPtr&);
    void chatterCallback_WPList(const mavros_msgs::WaypointList::ConstPtr&);
    void chatterCallback_MissionFinished(const mavros_msgs::MavlinkConstPtr&);

    void arrived(vaant::displacement, geometry_msgs::PoseStamped, float);
    bool compareGPS(mavros_msgs::Waypoint,mavros_msgs::Waypoint, float);
    bool arrivedGPS(mavros_msgs::Waypoint,float);

    static tf::StampedTransform createHT_Matrix(double, double, double, double, double, double);
    MatrixXd TF2THEigen(tf::StampedTransform m);
    void printMatrix(string,tf::StampedTransform);
    void printMatrix(string,tf::Matrix3x3);

    void getFlags(int *);
    void getDone(int &);
    void getCurrentGlobalPosition(mavros_msgs::Waypoint &, int);
    void getStateMode(mavros_msgs::State &);
    void getWPList(mavros_msgs::WaypointList &);
//    void getMissionFinished(int,bool &);

    tf::StampedTransform getHTmatrix(int HTmatrix);
    void setDone(int);

    bool sendMission(const mission &);
    bool clearMission();
    bool changeMode(int);
    void caseMode(int, string &);    
    bool changeHome(mavros_msgs::Waypoint);
    bool changeHome(mavros_msgs::WaypointList);

    // Para o mediana movel
    static double calcMedian(std::vector<double>);
    static std::vector<double> addShift(std::vector<double>,double);

private:

    mavros_msgs::State current_state;
    int flagError, numSetpoints,numSetpointsGreatCirles, numSetpointsDelta, numSetpointsDeltaRefHome, done,rate = 10; // The max value of ID from a specific bundle of Markers
    displacement *setPointList; // dynamic
    greatCircle *setPointGreatCircleList; // dynamic
    displacement *setPointDelta;
    displacement *setPointDeltaRefHome;
    tf::StampedTransform TH_M_C_aux,TH_V_I; //homogeneous transform variables -Ex: TH_M_C -> homogeneous transform of frame {M} relative frame {C}
    ros::NodeHandle nodeHandle;
    ros::Subscriber subPosGPS,subPosGPSAltRel, state_sub,subWPList,subPos,subMissionFinished;
    std_msgs::Float64 altRel;
    ros::ServiceClient set_mode_client;
    mavros_msgs::WaypointList wpList;
    ros::Publisher chatter_offboard; // Defining the publisher to toPublishV2 node
    bool missionFinished = false;
    mavros_msgs::Mavlink MISSION_ITEM_REACHED;
    mavlink_message_t mavlinkMSG;
    mavlink_mission_item_reached_t mavN46;

};

#endif // VAANT_H
