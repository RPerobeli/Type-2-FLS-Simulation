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

//#include <vaant.h> //ANTES
#include "/home/laic/catkin_ws/src/vant/include/vant/vaant.h"    //RODRIGO ALTEROU ISSO AQUI
//#include <vant/vision.h>  //ANTES
#include "/home/laic/catkin_ws/src/vant/include/vant/vision.h"  //RODRIGO ALTEROU ISSO AQUI

// #### MATRICIAL ### //
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>

// ========================================================================================================================================= //
// Codigo que busca pelo Marcador.
// Passar a altura maxima de busca pelo marcador como parametro
// Melhorar a rota de busca
// ========================================================================================================================================= //

# define BIGGER_MARKER 100 // Definindo o maior ID presente
# define BANNER 1          // Definindo qual banner utilizado

using namespace std;

void timer(int timeTolerance) // Tolerance time to start action. Parameters: Time tolerance in seconds.
{
    ros::Rate tol_rate(1);

    ROS_WARN("Tolerance Timer has been started. Tolerance Time set: %d", timeTolerance+1);

    int t = 1;
    while(ros::ok() && t< timeTolerance+1){
        ros::spinOnce();
        cout << t << "s already passed (total of "<< timeTolerance+1 << ")"<<endl;
        tol_rate.sleep();
        t++;
    }
}

double defineGoToZ(vaant quad,double maxHigh) // maxHigh that the marker is visible
{
    double goToThisZ;
    ros::spinOnce();

    if((quad.poseMavros.pose.position.z >= maxHigh) || (quad.poseMavros.pose.position.z < 0))
         goToThisZ = -(quad.poseMavros.pose.position.z - maxHigh);

    else if(quad.poseMavros.pose.position.z < maxHigh)
         goToThisZ = -(quad.poseMavros.pose.position.z - maxHigh);

    return goToThisZ;
}

// SetAlexandre
//void defineOrientation(tf::Vector3 route[], double a, double z, double dir)
//{
//    Eigen::MatrixXd rot(4,4),
//            p(4,8),
//            pRot(4,8);

//    //cout << "Antes p" << endl;

//    p << 0, a, 0, -2*a,    0, 2*a, 0, -a,
//         0, 0, a,    0, -2*a,   0, a,  0,
//         z, 0, 0,    0,    0,   0, 0,  0,
//         1, 1, 1,    1,    1,   1, 1,  1;

//    //cout << p << endl;

//    rot << cos(dir), -1*sin(dir), 0, 0,  // Conferir se e essa matriz mesmo
//           sin(dir),    cos(dir), 0, 0,
//           0,           0,        1, 0,
//           0,           0,        0, 1;

//    //cout << rot << endl;

//    pRot = rot * p;

//    //cout << endl << pRot << endl;

//    // Defining the route of seeking the marker
//    for (int i = 0; i < 8; i++)
//    {
//        //cout << pRot(0, i) << endl;
//        route[i] = {(double)pRot(0,i), (double)pRot(1,i), (double)pRot(2,i)};
//    }
//    //cout << route << endl;
//}

int main(int argc, char **argv)
{
    system("rosnode kill /lastWaypoint ");

    ros::init(argc, argv, "seekMarker");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    tf::TransformListener listener;
    tf::StampedTransform TH_CENTER_I;

    tf::Vector3 squareRoute[8];

    vision v(n, BIGGER_MARKER, MULTI, BANNER);
    vaant quad(n);

    double a = 1, firsta,                // displacement in meters
           tolConvergence = 0.2, // In meters
           goToThisZ = 6,
           maxHigh = 6,
           posOld[2],
           multiplier = 2,
           interaction = 3;

    int aux = 0,counter = 1;

    if(argc == 1){
        cout <<" Default Values. {Maximum High, Tolerance Convergence, Square Displacement, Multiplier, Interaction, Timer of tolerance of not seeing} {6, 0.2, 1, 2x, 4x, 3s}  " << endl;
    }

    else{
        maxHigh = atof(argv[1]);
        tolConvergence = atof(argv[2]);
        a = atof(argv[3]);
        cout <<" \n Maximum High Set "<< maxHigh << endl;
        cout <<" \n Tolerance Convergence "<< tolConvergence << endl;
        cout <<" \n Square Displacement "<< a << endl;
        cout <<" \n Multiplier "<< multiplier << endl;
        cout <<" \n Interactions "<< interaction << endl;
        cout <<" \n Timer of Tolerance "<< interaction << endl;
        sleep(1);
    }

   firsta = a;

   while(ros::ok() && !quad.poseMavros.pose.position.z){ // Melhorar aqui
       ros::spinOnce();
       loop_rate.sleep();
   }

   goToThisZ = defineGoToZ(quad,maxHigh);

   //////////////////////////////////////// Defining the route of seeking the marker ////////////////////////////////////////
   squareRoute[0] = {0,0, goToThisZ};
   squareRoute[1] = {a, 0 ,0};
   squareRoute[2] = {0, a ,0};
   squareRoute[3] = {-2*a, 0 ,0};
   squareRoute[4] = {0, -2*a ,0};
   squareRoute[5] = {2*a, 0 ,0};
   squareRoute[6] = {0, a ,0};
   squareRoute[7] = {-a, 0 ,0};
   //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    while(ros::ok()){

        ros::spinOnce();
        loop_rate.sleep();

        if (v.getPoseAlvar().id == 0 ) // There is no visible marker
        {
            timer(2); // Give a second chance after a time

            if (v.getPoseAlvar().id == 0 ){ // There is no visible marker after the time tolerance, so start the seeking.

                //defineOrientation(squareRoute, a, goToThisZ, dir);
                cout << " " << TH_CENTER_I.getOrigin().getX()<<" , " << TH_CENTER_I.getOrigin().getY() <<endl;

                // Rotina de busca de rota quadrada.
                cout <<" Seeking the Marker " << endl;
                cout<<"passou 1"<<endl;
                quad.goToPosition(squareRoute[aux], tolConvergence,v);
                aux ++;
                cout<<"passou 2"<<endl;
                if(aux == 8){

                    if (counter == interaction){
                        quad.changeMode(vaant::MODE::AUTO_RTL);
                    }
                    cout<<"passou 3"<<endl;
                    counter++;
                    aux = 0;

                    goToThisZ = defineGoToZ(quad,maxHigh);

                    a = multiplier*a;

                    //////////////////////////////////////// Defining the route of seeking the marker ////////////////////////////////////////
                    squareRoute[0] = {0,0, goToThisZ};
                    squareRoute[1] = {a, 0 ,0};
                    squareRoute[2] = {0, a ,0};
                    squareRoute[3] = {-2*a, 0 ,0};
                    squareRoute[4] = {0, -2*a ,0};
                    squareRoute[5] = {2*a, 0 ,0};
                    squareRoute[6] = {0, a ,0};
                    squareRoute[7] = {-a, 0 ,0};
                    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            }

        }

        }
        if (v.getPoseAlvar().id == 1 ) // There is visible marker, so let the landing algorithm act.
        {
            aux = 0;
            a = firsta;
            ROS_WARN("There is visible marker. Not seeking");

            // Implementando a rotina de memoria. Coleta a ultima posicao registrada do marcador e coloca como proimo ponto da rota de procura, com a altura desejada.

            try{
                listener.lookupTransform("/vant","/center",ros::Time(0),TH_CENTER_I); //obtem matriz de transf. homogena do centro do marcador em relacao a camera
            }
            catch (tf::TransformException &ex) {
                ROS_ERROR("%s",ex.what());
            }

            goToThisZ = defineGoToZ(quad,maxHigh);
            posOld[0] = squareRoute[0][0];
            posOld[1] = squareRoute[0][1];
            squareRoute[0] = { (double) TH_CENTER_I.getOrigin().getX(), (double) TH_CENTER_I.getOrigin().getY(), goToThisZ};
            cout << "Ultima incidencia o marcador (REF VANT)" << TH_CENTER_I.getOrigin().getX()<<" , " << TH_CENTER_I.getOrigin().getY() <<endl;

            // Fim da rotina de memoria

        }

    }

    system("rosnode kill /logGenerator ");
    return 0;

}
