#include "vant.h"

#define pi 3.1415926

vant::vant(){

}

/*##############################################################################################################
 *
 * Essas duas funcoes estao funcionando, porem implementadas de forma diferentes
    vant::dataPos pos;
    vant::dataOrient orient;
    -> forma de chamar no algoritmo principal: //quadrotor.getVarCallback(&pos,&orient); // passando o endereco de pos e orient
 * #############################################################################################################
 *
void vant::setVarCallback(const geometry_msgs::PoseStamped::ConstPtr* msg){

    position.x = msg->get()->pose.position.x;
    position.y = msg->get()->pose.position.y;
    position.z = msg->get()->pose.position.z;

    orientation.x = msg->get()->pose.orientation.x;
    orientation.y = msg->get()->pose.orientation.y;
    orientation.z = msg->get()->pose.orientation.z;
    orientation.w = msg->get()->pose.orientation.w;
}

void vant::getVarCallback(dataPos *pos, dataOrient *orient){
    pos->x = position.x;
    pos->y = position.y;
    pos->z = position.z;

    orient->x = orientation.x;
    orient->y = orientation.y;
    orient->z = orientation.z;
    orient->w = orientation.w;
}
*/

//void chatterCallback(const geometry_msgs::PoseStamped::ConstPtr& msg,geometry_msgs::PoseStamped::ConstPtr& pose){
//    pose = msg;
//}

//quadrotor.setVarCallback(&msg); //passa o endereco de msg
 //quadrotor.getVarCallback(&pose);

void vant::setVarCallback(const geometry_msgs::PoseStamped* msg){
    poseVant.pose.position.x = msg->pose.position.x;
    poseVant.pose.position.y = msg->pose.position.y;
    poseVant.pose.position.z = msg->pose.position.z;

    poseVant.pose.orientation.x = msg->pose.orientation.x;
    poseVant.pose.orientation.y = msg->pose.orientation.y;
    poseVant.pose.orientation.z = msg->pose.orientation.z;
    poseVant.pose.orientation.w = msg->pose.orientation.w;
}

void vant::getVarCallback(geometry_msgs::PoseStamped* msg){
    msg->pose.position.x = poseVant.pose.position.x ;
    msg->pose.position.y = poseVant.pose.position.y ;
    msg->pose.position.z = poseVant.pose.position.z ;

    msg->pose.orientation.x = poseVant.pose.orientation.x;
    msg->pose.orientation.y = poseVant.pose.orientation.y;
    msg->pose.orientation.z = poseVant.pose.orientation.z;
    msg->pose.orientation.w = poseVant.pose.orientation.w;
}

