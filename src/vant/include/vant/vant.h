#ifndef VANT_H
#define VANT_H

#include <iostream>
#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "tf/tf.h"

#include <cstdlib>


#include "Eigen/Core"
#include "Eigen/Geometry"
#include <Eigen/LU>


using namespace std;

class vant{
    public:
        vant();

        struct dataPos{
            double x,y,z;
        };

        struct dataOrient{
            double x,y,z,w;
        };

        struct RPY{
            double roll,pitch,yaw;
        };

        void setVarCallback(const geometry_msgs::PoseStamped*);
        //void getVarCallback(dataPos*,dataOrient*);
        void getVarCallback(geometry_msgs::PoseStamped*);
       //void chatterCallback(const geometry_msgs::PoseStamped::ConstPtr&,geometry_msgs::PoseStamped::ConstPtr&);

    private:
       dataPos position;
       dataOrient orientation;
       geometry_msgs::PoseStamped poseVant;
       Eigen::MatrixXf m;
};


#endif // VANT_H
