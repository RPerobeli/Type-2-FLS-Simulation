

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
#include <time.h>

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
//#include <vant/vaant.h> //ANTES
#include "/home/laic/catkin_ws/src/vant/include/vant/vaant.h" //RODRIGO ALTEROU ISSO AQUI
#include <vant/vision.h>
//#include <vant/ToOffboard.h> //ANTES
#include "/home/laic/catkin_ws/src/vant/include/vant/ToOffboard.h" //RODRIGO ALTEROU ISSO AQUI

/// FUZZY ////
//#include "fl/Headers.h" //Usado para fuzzy tipo 1
//////////////

// ========================================================================================================================================= //
// ============================================ CODIGO QUE INTEGRA PID, Multiplos Tags e MAVROS ========================================== //
// =============================================== CONTROLE POR VELOCIDADE - fuzzyVel_v3.fis =============================================== //
// ========================================================================================================================================= //

using namespace std;
//using namespace fl; //namespace do fuzzylite

// Default Parameters

///////////////////////////////////////////////////////// VARIAVEIS DE OPERACAO DO VAANT //////////////////////////////////////////////////////////
double H_MAX = 10.0,                                            // altura maxima de operacao (metros)
       ABERTURA_DA_CAMERA_X = DEG2RAD(33.0),                    // Eixo X da CAMERA! (Graus)
       ABERTURA_DA_CAMERA_Y = DEG2RAD(20.0),                    // Eixo Y da CAMERA! (Graus)
       MAX_X = H_MAX*tan(ABERTURA_DA_CAMERA_Y),                 // Eixo X DO VANT !
       MAX_Y = H_MAX*tan(ABERTURA_DA_CAMERA_X);                 // Eixo Y do VANT !
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float gainX = 0.019,
      gainY = 0.019,
      gainZ = 0.019,
      offsetZ = 0,
      vFinal = -0.5,
      tolZ = 0.8;     // Limite pra considerar que pousou riginal tolZ=0.1, 0.5 é melhor!


double kp = 0.35, //constante proporcional
       ki = 0.5, //constante integrativa
       kd = 1; //constante derivativa


// PID things

ofstream PIDParameters, debugPosition, debugPositionArPose, debugTimeToProc, FuzzyEntrances,TAGposition;                         // para gerar o LOG;

////////////////////////////////////////////////////////////// INICIO PID //////////////////////////////////////////////////////////////////

double PID_vx(double erro, double prev_erro)
{
double i=0,p=0,d=0,vx = 0;

p = kp*erro;

if (abs(erro) < 0.5)
	{
       i = i + ki*erro;
  }

d = kd*(erro-prev_erro);

vx = p+i+d;
return(vx);
}

double PID_vy(double erro, double prev_erro)
{
double i=0,p=0,d=0,vy = 0;

p = kp*erro;

if (abs(erro) < 0.5)
	{
       i = i + ki*erro;
  }

d = kd*(erro-prev_erro);

vy = p+i+d;
return(vy);
}

double PID_vz(double erro, double prev_erro)
{
double i=0,p=0,d=0,vz = 0;

p = kp*erro;

if (abs(erro) < 1)
	{
       i = i + ki*erro;
  }

d = kd*(erro-prev_erro);

vz = p+i+d;
return(vz);
}
////////////////////////////////////////////////////////////// FIM PID //////////////////////////////////////////////////////////////////


std::string get_date(void)
{
    static char name[LOGNAME_SIZE];
    time_t now = time(0);
    strftime(name, sizeof(name), LOGNAME_FORMAT, localtime(&now));
    return name;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "Landing_Control");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
cout<<"passou aki 2"<<endl;
    //ros::Subscriber thrash; // There is some overlap memory so it is necessary;
		
    timeval t_start, t_end; //variáveis ue receberão tempo universal
    gettimeofday(&t_start, NULL);
    int iteracoes = 0;


    ros::Publisher chatter_pub = n.advertise<vant::ToOffboard>("/setpoint_offboard",1);
    vant::ToOffboard pubMsg;

    static tf::TransformBroadcaster br;
    tf::TransformListener listener;
    tf::StampedTransform TH_M_C,TH_C_V,TH_M_V,TH_V_I, TH_M_I;

    vision eye(n);
    vaant quadrotor(n);

    vector<double> myFuzzy(4);
    tf::Vector3 vVel, vVelInertial;
    double tempX=0, tempY=0, tempZ=0, auxX=0, auxY=0;

    string nome("/home/laic/catkin_ws/src/vant/LOGS/"); // Para o PC
   // string nome("/home/odroid/catkin_ws/src/vant/LOGS_Vision2/"); // Para o Odroid

    string nome2, nome3, nome4, nome5, nome6, nome7;
    nome += get_date();
    const int dir_err = mkdir(nome.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH); // Para criar a pasta

    if (-1 == dir_err)
    {
        printf("Error creating directory!\n");
        return(-1);
    }

    float Vx, Vy, Vz;
		
    if(argc == 1){
        Vx = 1.0;
        Vy = 1.0;
        Vz = 1.0;
        cout <<" Default Values. {Vx, Vy, Vz} = { 1 1 1 (m/s)} , Z Tolerance = 0.1, Offset Vz = 0, ForcingVelocity = -0.5 " << endl;
    }

    else{
        Vx = atof(argv[1]);
        Vy = atof(argv[2]);
        Vz = atof(argv[3]);
        tolZ = atof(argv[4]);
        offsetZ = atof(argv[5]);
        vFinal = atof(argv[6]);
        cout <<"\n Maximum velocities values "<< Vx <<" "<< Vy <<" "<< Vz << endl;
        cout <<" Vertical tolerance value "<< tolZ << endl;
        cout <<" Offset value "<< offsetZ << endl;
        cout <<" Value of Forcing Velocity "<< vFinal << endl;
        sleep(3);

    }

    //defineFuzzy(); // Define o Fuzzy

    TH_C_V = vaant::createHT_Matrix(0.17,0,-0.05,PI,0,-PI/2);///Cria a Matriz TH da camera em relacao ao vant. OBS: Ordem: Pitch, Yaw and Roll. << COLOCAR O XYZ REAL DA CAMERA ANEXADA !!!!>>

    pubMsg.TwistStamped.header.frame_id = "";

    pubMsg.index = 0; // Para velocidade

    nome2= nome+"/PIDParameters.txt";
    PIDParameters.open(nome2.c_str());
    PIDParameters << "[ENTRADA e SAIDA do Fuzzy] [X Y Z dVx dVy dVz]" << "\n\n";
    PIDParameters << "[" << "\n";

    nome3= nome+"/debugPosition.txt";
    debugPosition.open(nome3.c_str());
    debugPosition << "Posicoes do VANT [X Y Z qX qY qZ qW]"<< "\n\n";
    debugPosition << "[" << "\n";

    nome4= nome+"/debugPositionArPose.txt";
    debugPositionArPose.open(nome4.c_str());
    debugPositionArPose << "Posicoes do TAG [X Y Z qX qY qZ qW]"<< "\n\n";
    debugPositionArPose << "[" << "\n";

    nome5 = nome+"/timeDecision.txt";
    debugTimeToProc.open(nome5.c_str());
    debugTimeToProc << " Tempo de Decisão "<< "\n\n";
    debugTimeToProc << "[" << "\n";

    nome6= nome+"/FuzzyEntrances.txt";
    FuzzyEntrances.open(nome6.c_str());
    FuzzyEntrances << "[ENTRADA e SAIDA do Fuzzy do EDUARDO] [X Y  Vx Vy Vz]" << "\n\n";
    FuzzyEntrances << "[" << "\n";

    nome7= nome+"/TAGposition.txt";
    TAGposition.open(nome7.c_str());
    TAGposition << "[posiçõe do marcador] [X Y Z]" << "\n\n";
    TAGposition << "[" << "\n";

    while(!quadrotor.poseMavros.pose.position.z && ros::ok()){ // fica nesse loop ate o valor de position[2] ser alterado, ou seja, so desbloqueia quando um valor inicial for lido do position
        cout << "Posicao inicial nao encontrada" << endl;
        ros::spinOnce(); // atualiza o callback
    }

    cout << "Posicao Inicial" << "\n" << quadrotor.poseMavros.pose <<endl;
    tempZ = tolZ + 0.1; // so para ignorar os primeiros erros de TF que da

    while(ros::ok() && (abs(tempZ) > tolZ)){ 					
iteracoes = iteracoes+1;

        if(eye.getPoseAlvar().id == 0){

//            /vVelInertial.setValue(0,0,offsetZ);

            pubMsg.TwistStamped.header.stamp = ros::Time::now();
            pubMsg.TwistStamped.twist.linear.x = 0;
            pubMsg.TwistStamped.twist.linear.y = 0;
            pubMsg.TwistStamped.twist.linear.z = offsetZ;

            chatter_pub.publish(pubMsg);

            while(ros::ok() && eye.getPoseAlvar().id == 0){ // Dessa forma, eh publicado apenas uma vez a velocidade Zero, permitindo que outro NO paralelo publique.

                ROS_ERROR("I am not seeing any Marker");
                loop_rate.sleep();
                ros::spinOnce();
            }

            ros::spinOnce(); // So para atualizar a sua posicao
            TH_V_I = quadrotor.getHTmatrix(HT_V_I);//obtem mat trans. homogenea do vant em relacao ao inercial
        }

         if(eye.getPoseAlvar().id == 1){

            TH_M_C = vaant::createHT_Matrix(eye.getPoseAlvar().pose.pose.position.x,eye.getPoseAlvar().pose.pose.position.y,eye.getPoseAlvar().pose.pose.position.z,0,0,tf::getYaw(eye.getPoseAlvar().pose.pose.orientation));//monta mat trans. homogenea do marcador em relacao a camera

            br.sendTransform(tf::StampedTransform(TH_C_V,ros::Time::now(),"vant","camera"));//envia transformacao para o TF. Vant eh o parent frame e camera child frame
            pubMsg.index = 0; // Para velocidade

            ros::spinOnce();

            try{
                listener.lookupTransform("/vant","/center",ros::Time(0),TH_M_V); //obtem matriz de transf. homogena do marcador em relacao ao vant
                listener.lookupTransform("/world","/center",ros::Time(0),TH_M_I);
            }
            catch (tf::TransformException &ex) {
                ROS_ERROR("%s",ex.what());
                continue;
            }

            tempX = TH_M_V.getOrigin().getX();
            tempY = TH_M_V.getOrigin().getY();
            tempZ = TH_M_V.getOrigin().getZ();
            TAGposition << " "<< tempX <<" "<< tempY <<" "<< tempZ <<";"<< endl;

            cout << "TAG Position (x,y,z)(Based on Inertial):" << tempX << " " << tempY <<" " << tempZ << endl;
            debugPositionArPose << " " << TH_M_I.getOrigin().getX() << " " << TH_M_I.getOrigin().getY()<< " " << TH_M_I.getOrigin().getZ() << " " << TH_M_I.getRotation().getX()<< " " << TH_M_I.getRotation().getY()<< " " << TH_M_I.getRotation().getW()<< " " << TH_M_I.getRotation().getZ() <<";"<<endl;
            debugPosition << " " << TH_V_I.getOrigin().getX() << " " << TH_V_I.getOrigin().getY()<< " " << TH_V_I.getOrigin().getZ() << " " << TH_V_I.getRotation().getX()<< " " << TH_V_I.getRotation().getY()<< " " << TH_V_I.getRotation().getW()<< " " << TH_V_I.getRotation().getZ() <<";"<<endl;

            //transforma valores absolutos em porcentagem para o cálculo do fuzzy
            /*if(tempX < 0)
                auxX = -1*min(100.0*abs(tempX/MAX_X), 100.0);
            if(tempY < 0)
                auxY = -1*min(100.0*abs(tempY/MAX_Y), 100.0);
            if(tempX > 0)
                auxX = min(100.0*(tempX/MAX_X), 100.0);
            if(tempY > 0)
                auxY = min(100.0*(tempY/MAX_Y), 100.0);*/

            timeval oldCount,newCount;
            gettimeofday(&oldCount, NULL);		
				
						//Variáveis para ajudar no cálculo do PID
						double tempXprev=0, tempYprev=0, tempZprev=0; //Posições anteriores auxiliares ao calculo derivativo
						//CHAMADA DO PID
            cout << "funçao VY" << endl;
						double resultVY = PID_vy (tempY,tempYprev);            
						cout << "funçao VX" << endl;
						double resultVX = PID_vx (tempX,tempXprev);
                                                double resultVZ;
                                                cout << "funçao VZ" << endl;
                                                resultVZ = PID_vz (tempZ,tempZprev);

						//Atualização dos erros
						tempXprev = tempX;
						tempYprev = tempY;
						tempZprev = tempZ;

						//impedimento para que os valores de velocidade emitidos pelo PID não ultrapassem a velocidade máxima do VANT          
            

            gettimeofday(&newCount, NULL);
            double t = double(newCount.tv_sec -oldCount.tv_sec )
                    + double(newCount.tv_usec-oldCount.tv_usec) * 1.e-8;// tava 1e-6
				
            debugTimeToProc << t << endl;

            //cout << "Fuzzy In (x,y): " << auxX << " " << auxY << endl;

            vVel.setValue(resultVX,resultVY,0); // Tirando o Z da transformacao

            ros::spinOnce();
            TH_V_I = quadrotor.getHTmatrix(HT_V_I); //obtem mat trans. homogenea do vant em relacao ao inercial

            vVelInertial = tf::quatRotate(TH_V_I.getRotation(), vVel);
            vVelInertial.setZ(resultVZ);
        }

        cout << "Sended Velocities (Vx,Vy,Vz)(UAV Frame):" << vVel.getX() << " " << vVel.getY()<<" " << vVelInertial.getZ() << endl;

        PIDParameters << " "<< tempX <<" "<< tempY <<" "<< tempZ <<" "<<vVel.getX() << " " << vVel.getY()<<" " << vVelInertial.getZ() << ";"<< endl;

        

        pubMsg.TwistStamped.twist.linear.x = vVelInertial.getX();
        pubMsg.TwistStamped.twist.linear.y = vVelInertial.getY();
        pubMsg.TwistStamped.twist.linear.z = vVelInertial.getZ();

        cout << "Sended Velocities (Vx,Vy,Vz)(Inertial Frame):" <<  pubMsg.TwistStamped.twist.linear.x << " " <<  pubMsg.TwistStamped.twist.linear.y <<" " <<  pubMsg.TwistStamped.twist.linear.z << endl;
        cout << "Marker Position (x,y,z)(Based on UAV):" <<  eye.getPoseAlvar().pose.pose.position.x<< " " <<  eye.getPoseAlvar().pose.pose.position.y <<" " <<  eye.getPoseAlvar().pose.pose.position.z << endl;
        cout << " " << endl;

        pubMsg.TwistStamped.header.stamp = ros::Time::now();
        chatter_pub.publish(pubMsg);
        loop_rate.sleep();
        ros::spinOnce();

    }

    ROS_WARN(" I am forcing the Landing with the velocity %.2f m/s", vFinal);

		gettimeofday(&t_end, NULL);
		double tempo = double(t_end.tv_sec - t_start.tv_sec);

		cout << "TEMPO DE DESCIDA " << tempo << " " << endl;
		
    system("rosnode kill /seekMarker ");

    ros::Time t0;
    t0 = ros::Time::now();
		

    while((ros::Time::now()-t0) < ros::Duration(3)) // Forcing
    {
        vVelInertial.setValue(0,0,vFinal);
        pubMsg.TwistStamped.twist.linear.x = vVelInertial.getX();
        pubMsg.TwistStamped.twist.linear.y = vVelInertial.getY();
        pubMsg.TwistStamped.twist.linear.z = vVelInertial.getZ();
        pubMsg.TwistStamped.header.stamp = ros::Time::now();
        chatter_pub.publish(pubMsg);
    }

    PIDParameters << "]" << endl;
    PIDParameters << "Tempo de descida: "<< tempo << endl;
    PIDParameters.close();

    debugPosition << "]" << endl;
    debugPosition.close();

    debugPositionArPose << "]" << endl;
    debugPositionArPose.close();

    debugTimeToProc << "]" << endl;
    debugTimeToProc.close();
    
    FuzzyEntrances << "]" << endl;
    FuzzyEntrances.close();

    TAGposition << "]" << endl;
    TAGposition.close();

    ros::spin();
    return 0;
}
