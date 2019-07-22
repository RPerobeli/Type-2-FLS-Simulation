
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
//////////////
#include <vant/vaant.h>
#include <vant/vision.h>

// #### MATRICIAL ### //
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>

// ========================================================================================================================================= //
// Segunda versao do multiTagsLanding. O metodo chooseTag2 foi Modificado
// O centro de pouso e o marcador 15 !!! VERSAO TAG = VERSAO ANTIGA MAS COM OUTROS MARCADORES AO REDOR
// ========================================================================================================================================= //

using namespace std;

int main(int argc, char **argv)
{
    int flagID ;
    double tempX, tempY, tempZ;

    ros::init(argc, argv, "multiTagsLandingV2");
    ros::NodeHandle n;
    vision visao(n,100,vision::VISION_ARGUMENTS::opt.single,vision::VISION_ARGUMENTS::v.V3);

    ros::Publisher chatter_pub = n.advertise<ar_track_alvar_msgs::AlvarMarker>("/vision/main_tag_pose",1);
    ar_track_alvar_msgs::AlvarMarker toPub;
    ros::Rate loop_rate(10);


    vision::flagList tempFlag;

    static tf::TransformBroadcaster br;
    tf::TransformListener listener;
    tf::StampedTransform TH_M_C, TH_CENTER_C, TH_M0_M34, TH_M1_M34,
            TH_M2_M34, TH_M4_M34, TH_M5_M34, TH_M6_M34, TH_M8_M34, TH_M9_M34, TH_M11_M34, TH_M12_M34,TH_M13_M34,
            TH_M14_M34,TH_M15_M34, TH_M16_M34, TH_M18_M34, TH_M25_M34, TH_M27_M34, TH_M31_M34, TH_M32_M34,
            TH_M34_M34, TH_M38_M34, TH_M40_M34, TH_M43_M34, TH_M45_M34, TH_M54_M34, TH_M59_M34, TH_M60_M34,
            TH_M61_M34, TH_M63_M34, TH_M64_M34, TH_M68_M34, TH_M73_M34, TH_M75_M34, TH_M79_M34, TH_M80_M34,
            TH_M91_M34, TH_M95_M34, TH_M96_M34, TH_M98_M34, TH_M100_M34;

    TH_M0_M34  = vaant::createHT_Matrix(-0.000,-0.865,0,0,0,0);       /// Cria a Matriz Marker 0 em relacao ao Marker 34.
    TH_M1_M34 = vaant::createHT_Matrix( 0.112,-0.866,0,0,0,0);   /// Cria a Matriz Marker 1 em relacao ao Marker 34.
    TH_M2_M34 = vaant::createHT_Matrix( -0.530,0.801,0,0,0,0);   /// Cria a Matriz Marker 2 em relacao ao Marker 34.
    TH_M4_M34 = vaant::createHT_Matrix( -0.265,0.801,0,0,0,0);   /// Cria a Matriz Marker 4 em relacao ao Marker 34.
    TH_M5_M34 = vaant::createHT_Matrix(  0.000,0.801,0,0,0,0);
    TH_M6_M34 = vaant::createHT_Matrix(  0.265,0.801,0,0,0,0);
    TH_M8_M34 = vaant::createHT_Matrix(  0.530,0.803,0,0,0,0);
    TH_M9_M34 = vaant::createHT_Matrix( -0.530,0.535,0,0,0,0);
    TH_M11_M34 = vaant::createHT_Matrix( -0.265,0.535,0,0,0,0);
    TH_M12_M34 = vaant::createHT_Matrix(  0.000,0.535,0,0,0,0);
    TH_M13_M34 = vaant::createHT_Matrix( 0.265,0.535,0,0,0,0);
    TH_M14_M34 = vaant::createHT_Matrix( 0.530,-0.535,0,0,0,0);
    TH_M15_M34 = vaant::createHT_Matrix( -0.530,0.269,0,0,0,0);
    TH_M16_M34 = vaant::createHT_Matrix( -0.265,0.269,0,0,0,0);
    TH_M18_M34 = vaant::createHT_Matrix(  0.0,0.269,0,0,0,0);
    TH_M25_M34 = vaant::createHT_Matrix(  0.265,0.269,0,0,0,0);
    TH_M27_M34 = vaant::createHT_Matrix(  0.530,0.271,0,0,0,0);
    TH_M31_M34 = vaant::createHT_Matrix( -0.530,0.0,0,0,0,0);
    TH_M32_M34 = vaant::createHT_Matrix( -0.265,0.0,0,0,0,0);
    TH_M34_M34 = vaant::createHT_Matrix( 0.,0.0,0,0,0,0);
    TH_M38_M34 = vaant::createHT_Matrix( 0.265,0.0,0,0,0,0);
    TH_M40_M34 = vaant::createHT_Matrix( 0.530,0.0,0,0,0,0);
    TH_M43_M34 = vaant::createHT_Matrix( -0.530,-0.265,0,0,0,0);
    TH_M45_M34 = vaant::createHT_Matrix( -0.265,-0.265,0,0,0,0);
    TH_M54_M34 = vaant::createHT_Matrix(  0.000,-0.265,0,0,0,0);
    TH_M59_M34 = vaant::createHT_Matrix(  0.265,-0.265,0,0,0,0);
    TH_M60_M34 = vaant::createHT_Matrix(  0.530, -0.264,0,0,0,0);
    TH_M61_M34 = vaant::createHT_Matrix( -0.625,-0.515,0,0,0,0);
    TH_M63_M34 = vaant::createHT_Matrix(  0.632,-0.515,0,0,0,0);
    TH_M64_M34 = vaant::createHT_Matrix( -0.625,-0.775,0,0,0,0);   /// Cria a Matriz Marker 64 em relacao ao Marker 34.
    TH_M68_M34 = vaant::createHT_Matrix( -0.632,-0.775,0,0,0,0);   /// Cria a Matriz Marker 68 em relacao ao Marker 34.
    TH_M73_M34 = vaant::createHT_Matrix( -0.625,-1.035,0,0,0,0);   /// Cria a Matriz Marker 73 em relacao ao Marker 34.
    TH_M75_M34 = vaant::createHT_Matrix(  0.632,-1.035,0,0,0,0);   /// Cria a Matriz Marker 75 em relacao ao Marker 34.
    TH_M79_M34 = vaant::createHT_Matrix( -0.625,-1.295,0,0,0,0);   /// Cria a Matriz Marker 79 em relacao ao Marker 34.
    TH_M80_M34 = vaant::createHT_Matrix(  0.632,-1.295,0,0,0,0);   /// Cria a Matriz Marker 80 em relacao ao Marker 34.
    TH_M91_M34 = vaant::createHT_Matrix( -0.530,-1.465,0,0,0,0);   /// Cria a Matriz Marker 91 em relacao ao Marker 34.
    TH_M95_M34 = vaant::createHT_Matrix( -0.265,-1.465,0,0,0,0);   /// Cria a Matriz Marker 95 em relacao ao Marker 34.
    TH_M96_M34 = vaant::createHT_Matrix(  0.000,-1.465,0,0,0,0);   /// Cria a Matriz Marker 96 em relacao ao Marker 34.
    TH_M98_M34 = vaant::createHT_Matrix(  0.265,-1.465,0,0,0,0);   /// Cria a Matriz Marker 98 em relacao ao Marker 34.
    TH_M100_M34 = vaant::createHT_Matrix( 0.530,-1.465,0,0,0,0);   /// Cria a Matriz Marker 100 em relacao ao Marker 34.


    while(ros::ok()){

        tempFlag = visao.getFlags(); // Para verificar se viu algum marcador e assim não repetir TFs antigos
        cout<< tempFlag.flagEmpty << endl;
        if(tempFlag.flagEmpty == 1 ){
            cout << "Nao vejo nada" << endl;
            toPub.id = 0; // Nao tem marcador
            toPub.pose.pose.position.x = 0;
            toPub.pose.pose.position.y = 0;
            toPub.pose.pose.position.z = 0;
            toPub.pose.pose.orientation.w = 0;
            toPub.pose.pose.orientation.x = 0;
            toPub.pose.pose.orientation.y = 0;
            toPub.pose.pose.orientation.z = 0;

        }
        else{
            TH_M_C = visao.getPoseAlvarHTmatrix(); //monta mat trans. homogenea do currentTag (ou marcador visto) em relacao a camera
            tempFlag = visao.getFlags();      // NUNCA ISNERIR UM SPIN ENTRE ESSA INSTRUCAO E ANTERIOR
            br.sendTransform(tf::StampedTransform(TH_M_C,ros::Time::now(),"camera","currentTag"));//envia transformacao para o TF. Camera eh o parent frame e marcador child frame

            flagID = tempFlag.flagChooseID;
            cout <<"marcador visto " <<flagID <<endl;
            switch (flagID){ // Switch necessario para determinar quem que o currentTag na arvore de TF !

            case 0:
                br.sendTransform(tf::StampedTransform(TH_M0_M34.inverse(),ros::Time::now(),"currentTag","center"));//envia transformacao para o TF. Camera eh o parent frame e marcador child frame
                break;

            case 1:
                br.sendTransform(tf::StampedTransform(TH_M1_M34.inverse(),ros::Time::now(),"currentTag","center"));//envia transformacao para o TF. Camera eh o parent frame e marcador child frame
                break;

            case 2:
                br.sendTransform(tf::StampedTransform(TH_M2_M34.inverse(),ros::Time::now(),"currentTag","center"));//envia transformacao para o TF. Camera eh o parent frame e marcador child frame
                break;

            case 4:
                br.sendTransform(tf::StampedTransform(TH_M4_M34.inverse(),ros::Time::now(),"currentTag","center"));//envia transformacao para o TF. Camera eh o parent frame e marcador child frame
                break;

            case 5:
                br.sendTransform(tf::StampedTransform(TH_M5_M34.inverse(),ros::Time::now(),"currentTag","center"));//envia transformacao para o TF. Camera eh o parent frame e marcador child frame
                break;

            case 6:
                br.sendTransform(tf::StampedTransform(TH_M6_M34.inverse(),ros::Time::now(),"currentTag","center"));//envia transformacao para o TF. Camera eh o parent frame e marcador child frame
                break;

            case 8:
                br.sendTransform(tf::StampedTransform(TH_M8_M34.inverse(),ros::Time::now(),"currentTag","center"));//envia transformacao para o TF. Camera eh o parent frame e marcador child frame
                break;

            case 9:
                br.sendTransform(tf::StampedTransform(TH_M9_M34.inverse(),ros::Time::now(),"currentTag","center"));//envia transformacao para o TF. Camera eh o parent frame e marcador child frame
                break;

            case 11:
                br.sendTransform(tf::StampedTransform(TH_M11_M34.inverse(),ros::Time::now(),"currentTag","center"));//envia transformacao para o TF. Camera eh o parent frame e marcador child frame
                break;

            case 12:
                br.sendTransform(tf::StampedTransform(TH_M12_M34.inverse(),ros::Time::now(),"currentTag","center"));//envia transformacao para o TF. Camera eh o parent frame e marcador child frame
                break;

            case 13:
                br.sendTransform(tf::StampedTransform(TH_M13_M34.inverse(),ros::Time::now(),"currentTag","center"));//envia transformacao para o TF. Camera eh o parent frame e marcador child frame
                break;

            case 14:
                br.sendTransform(tf::StampedTransform(TH_M14_M34.inverse(),ros::Time::now(),"currentTag","center"));//envia transformacao para o TF. Camera eh o parent frame e marcador child frame
                break;

            case 15:
                br.sendTransform(tf::StampedTransform(TH_M15_M34.inverse(),ros::Time::now(),"currentTag","center"));//envia transformacao para o TF. Camera eh o parent frame e marcador child frame
                break;

            case 16:
                br.sendTransform(tf::StampedTransform(TH_M16_M34.inverse(),ros::Time::now(),"currentTag","center"));//envia transformacao para o TF. Camera eh o parent frame e marcador child frame
                break;

            case 18:
                br.sendTransform(tf::StampedTransform(TH_M18_M34.inverse(),ros::Time::now(),"currentTag","center"));//envia transformacao para o TF. Camera eh o parent frame e marcador child frame
                break;

            case 25:
                br.sendTransform(tf::StampedTransform(TH_M25_M34.inverse(),ros::Time::now(),"currentTag","center"));//envia transformacao para o TF. Camera eh o parent frame e marcador child frame
                break;

            case 27:
                br.sendTransform(tf::StampedTransform(TH_M27_M34.inverse(),ros::Time::now(),"currentTag","center"));//envia transformacao para o TF. Camera eh o parent frame e marcador child frame
                break;

            case 31:
                br.sendTransform(tf::StampedTransform(TH_M31_M34.inverse(),ros::Time::now(),"currentTag","center"));//envia transformacao para o TF. Camera eh o parent frame e marcador child frame
                break;

            case 32:
                br.sendTransform(tf::StampedTransform(TH_M32_M34.inverse(),ros::Time::now(),"currentTag","center"));//envia transformacao para o TF. Camera eh o parent frame e marcador child frame
                break;

            case 34:
                br.sendTransform(tf::StampedTransform(TH_M34_M34.inverse(),ros::Time::now(),"currentTag","center"));//envia transformacao para o TF. Camera eh o parent frame e marcador child frame
                break;

            case 38:
                br.sendTransform(tf::StampedTransform(TH_M38_M34.inverse(),ros::Time::now(),"currentTag","center"));//envia transformacao para o TF. Camera eh o parent frame e marcador child frame
                break;

            case 40:
                br.sendTransform(tf::StampedTransform(TH_M40_M34.inverse(),ros::Time::now(),"currentTag","center"));//envia transformacao para o TF. Camera eh o parent frame e marcador child frame
                break;

            case 43:
                br.sendTransform(tf::StampedTransform(TH_M43_M34.inverse(),ros::Time::now(),"currentTag","center"));//envia transformacao para o TF. Camera eh o parent frame e marcador child frame
                break;

            case 45:
                br.sendTransform(tf::StampedTransform(TH_M45_M34.inverse(),ros::Time::now(),"currentTag","center"));//envia transformacao para o TF. Camera eh o parent frame e marcador child frame
                break;

            case 54:
                br.sendTransform(tf::StampedTransform(TH_M54_M34.inverse(),ros::Time::now(),"currentTag","center"));//envia transformacao para o TF. Camera eh o parent frame e marcador child frame
                break;

            case 59:
                br.sendTransform(tf::StampedTransform(TH_M59_M34.inverse(),ros::Time::now(),"currentTag","center"));//envia transformacao para o TF. Camera eh o parent frame e marcador child frame
                break;

            case 60:
                br.sendTransform(tf::StampedTransform(TH_M60_M34.inverse(),ros::Time::now(),"currentTag","center"));//envia transformacao para o TF. Camera eh o parent frame e marcador child frame
                break;

            case 61:
                br.sendTransform(tf::StampedTransform(TH_M61_M34.inverse(),ros::Time::now(),"currentTag","center"));//envia transformacao para o TF. Camera eh o parent frame e marcador child frame
                break;

            case 63:
                br.sendTransform(tf::StampedTransform(TH_M63_M34.inverse(),ros::Time::now(),"currentTag","center"));//envia transformacao para o TF. Camera eh o parent frame e marcador child frame
                break;

            case 64:
                br.sendTransform(tf::StampedTransform(TH_M64_M34.inverse(),ros::Time::now(),"currentTag","center"));//envia transformacao para o TF. Camera eh o parent frame e marcador child frame
                break;

            case 68:
                br.sendTransform(tf::StampedTransform(TH_M68_M34.inverse(),ros::Time::now(),"currentTag","center"));//envia transformacao para o TF. Camera eh o parent frame e marcador child frame
                break;

            case 73:
                br.sendTransform(tf::StampedTransform(TH_M73_M34.inverse(),ros::Time::now(),"currentTag","center"));//envia transformacao para o TF. Camera eh o parent frame e marcador child frame
                break;

            case 75:
                br.sendTransform(tf::StampedTransform(TH_M75_M34.inverse(),ros::Time::now(),"currentTag","center"));//envia transformacao para o TF. Camera eh o parent frame e marcador child frame
                break;

            case 79:
                br.sendTransform(tf::StampedTransform(TH_M79_M34.inverse(),ros::Time::now(),"currentTag","center"));//envia transformacao para o TF. Camera eh o parent frame e marcador child frame
                break;

            case 80:
                br.sendTransform(tf::StampedTransform(TH_M80_M34.inverse(),ros::Time::now(),"currentTag","center"));//envia transformacao para o TF. Camera eh o parent frame e marcador child frame
                break;

            case 91:
                br.sendTransform(tf::StampedTransform(TH_M91_M34.inverse(),ros::Time::now(),"currentTag","center"));//envia transformacao para o TF. Camera eh o parent frame e marcador child frame
                break;

            case 95:
                br.sendTransform(tf::StampedTransform(TH_M95_M34.inverse(),ros::Time::now(),"currentTag","center"));//envia transformacao para o TF. Camera eh o parent frame e marcador child frame
                break;

            case 96:
                br.sendTransform(tf::StampedTransform(TH_M96_M34.inverse(),ros::Time::now(),"currentTag","center"));//envia transformacao para o TF. Camera eh o parent frame e marcador child frame
                break;

            case 98:
                br.sendTransform(tf::StampedTransform(TH_M98_M34.inverse(),ros::Time::now(),"currentTag","center"));//envia transformacao para o TF. Camera eh o parent frame e marcador child frame
                break;

            case 100:
                br.sendTransform(tf::StampedTransform(TH_M100_M34.inverse(),ros::Time::now(),"currentTag","center"));//envia transformacao para o TF. Camera eh o parent frame e marcador child frame
                break;
            }

            try{
                listener.lookupTransform("/camera","/center",ros::Time(0),TH_CENTER_C); //obtem matriz de transf. homogena do marcador0 em relacao a camera
            }
            catch (tf::TransformException &ex) {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }

            tempX = TH_CENTER_C.getOrigin().getX();
            tempY = TH_CENTER_C.getOrigin().getY();
            tempZ = TH_CENTER_C.getOrigin().getZ();

            toPub.pose.pose.position.x = tempX;
            toPub.pose.pose.position.y = tempY;
            toPub.pose.pose.position.z = tempZ;
            toPub.pose.pose.orientation.w = TH_CENTER_C.getRotation().getW();
            toPub.pose.pose.orientation.x = TH_CENTER_C.getRotation().getX();
            toPub.pose.pose.orientation.y = TH_CENTER_C.getRotation().getY();
            toPub.pose.pose.orientation.z = TH_CENTER_C.getRotation().getZ();


            toPub.id = 1; // Tem marcador

            cout <<" Posicao Central: "<< setiosflags(ios::fixed) << setprecision(3)<< setw(6) <<tempX << setw(8) << tempY <<setw(8) << tempZ <<endl;

        }

        chatter_pub.publish(toPub);

        loop_rate.sleep();


        ros::spinOnce();
    }
    ros::spin();
    return 0;
}
