#include "/home/laic/catkin_ws/src/vant/include/vant/vaant.h"


vaant::vaant(ros::NodeHandle n){ // Constructor
   numSetpoints = -1;
   numSetpointsGreatCirles = -1;
   numSetpointsDelta = -1;
   numSetpointsDeltaRefHome = -1;

   done = 0;

   nodeHandle = n;
   subPos = nodeHandle.subscribe("/mavros/local_position/pose",1, &vaant::chatterCallback_PoseMavros, this);
   subPosGPS = nodeHandle.subscribe("/mavros/global_position/global", 1, &vaant::chatterCallback_GPSMavros, this);
   subPosGPSAltRel = nodeHandle.subscribe("/mavros/global_position/rel_alt", 1, &vaant::chatterCallback_GPSMavrosAltRel, this);
   subWPList = nodeHandle.subscribe("/mavros/mission/waypoints",100, &vaant::chatterCallback_WPList,this);
   state_sub = nodeHandle.subscribe<mavros_msgs::State>("mavros/state", 10, &vaant::chatterCallback_State,this);
   subMissionFinished = nodeHandle.subscribe<mavros_msgs::Mavlink>("/mavlink/from", 10, &vaant::chatterCallback_MissionFinished,this);
   set_mode_client = nodeHandle.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

   chatter_offboard = nodeHandle.advertise<vant::ToOffboard>("/setpoint_offboard",1);
   ROS_WARN("Construtor COM NodeHandle");
}

vaant::vaant(){ // Constructor
   numSetpoints = -1;
   numSetpointsGreatCirles = -1;
   numSetpointsDelta = -1;
   numSetpointsDeltaRefHome = -1;
   done = 0;
   ROS_WARN("Construtor SEM NodeHandle");
}

vaant::~vaant(){ // to delete the object and free the memory;
    //if(flagError != 101 && flagError !=102)
        //clearSetpointList();
}

//####################################################################################################################################################

int vaant::readSetpointListfromFile(char* pathToFile, int deltaOrGPS){
// Function: Read waypoint list from a text file
// Parameters:
//            Path to mission.txt file
//            deltaOrGPS: 1 = Lat Lon and Alt setpoints
//                        0 = X,Y,Z setpoints
// Return: 100 - OK
//         101 - Error: Failed to open file
//         102 - Error: Data Error

    int indexM = 0, temp;
    string line;
    ifstream myfile;

    if(numSetpoints!=-1){ // only to delete all memory allocation before a new one is created
       clearSetpointList();
    }

    myfile.open(pathToFile);

    if(!myfile){
        flagError = 101;
        return flagError;
    }

    ++numSetpoints;
    while (std::getline(myfile, line)) // check the number of setpoints
        ++numSetpoints;

    if(numSetpoints == 0){
        flagError = 102;
        return flagError;
    }

   setPointList = new displacement[numSetpoints]; // dynamic array

    myfile.close();
    myfile.open(pathToFile);

    while(!myfile.eof()){

        if(indexM<numSetpoints){
            myfile >> setPointList[indexM].x >> setPointList[indexM].y >> setPointList[indexM].z >> setPointList[indexM].maneuver;
        }
        indexM++;
    }

    myfile.close();

    if (deltaOrGPS == 1)
        temp = applyHaversineBearingToArray();

    flagError = 100;
    return flagError;
}

//####################################################################################################################################################

vaant::coord vaant::TransformWPinCoord(mavros_msgs::Waypoint wp){
    // To transform waypoint mavros standard in coord lib standard

    vaant::coord temp;
    temp.lat = wp.x_lat;
    temp.lon = wp.y_long;
    temp.alt = wp.z_alt;

    return temp;

}

//####################################################################################################################################################
// Methods of GreatCircle theory

double vaant::transformHaversine(vaant::coord first, vaant::coord second){
    // Return the linear distance between two coordinates //
    // Parameters:
    //              - First Coord
    //              - Second Coordinate
    // Return:
    //              - The distance in meters

    double r, phi1, phi2, dPhi, dLamb, a, c, d;

    r = 6371000;

    phi1  = DEG2RAD(first.lat);
    phi2  = DEG2RAD(second.lat);
    dPhi  = DEG2RAD((second.lat-first.lat));
    dLamb = DEG2RAD((second.lon-first.lon));

    a = sin(dPhi/2)*sin(dPhi/2) + cos(phi1)*cos(phi2)*sin(dLamb/2)*sin(dLamb/2);
    c = 2*atan2((sqrt(a)),(sqrt(1-a)));
    d = r * c;

    return(d);

}

double vaant::transformBearing(vaant::coord first, vaant::coord second){
    // Return the angle of the Haversine based on North //
    // Parameters:
    //              - First Coord
    //              - Second Coordinate
    // Return:
    //              - The angle in Degress

    double phi1, phi2, lamb1, lamb2, x, y, b;
    phi1 = DEG2RAD(first.lat);
    phi2 = DEG2RAD(second.lat);
    lamb1 = DEG2RAD(first.lon);
    lamb2 = DEG2RAD(second.lon);

    y = sin((lamb2-lamb1))*cos(phi2);
    x = cos(phi1)*sin(phi2)- sin(phi1)*cos(phi2)*cos((lamb2-lamb1));

    b = RAD2DEG(atan2((y),(x)));

    return(b);

}

vaant::hbParameters vaant::applyHaversineBearing(vaant::coord p1, vaant::coord p2){
// Return an array with the haversine distance, bearing angle, x,y,z in ENU

    hbParameters temp;

    temp.harversineDistance = transformHaversine(p1,p2);
    temp.bearingAngle = transformBearing(p1,p2);
    temp.x = temp.harversineDistance*sin(DEG2RAD(temp.bearingAngle)); // ENU
    temp.y = temp.harversineDistance*cos(DEG2RAD(temp.bearingAngle));
    temp.z = p2.alt - p1.alt;
    temp.euclidean = sqrt(pow(temp.x,2)+pow(temp.y,2)+pow(temp.z,2));

    return temp;

}

vaant::hbParameters vaant::applyHaversineBearing(mavros_msgs::Waypoint wp1, mavros_msgs::Waypoint wp2){
// Return an array with the haversine distance, bearing angle, x,y,z in ENU


    vaant::coord p1,p2;

    p1 = vaant::TransformWPinCoord(wp1);
    p2 = vaant::TransformWPinCoord(wp2);

    hbParameters temp;

    temp.harversineDistance = transformHaversine(p1,p2);
    temp.bearingAngle = transformBearing(p1,p2);
    temp.x = temp.harversineDistance*sin(DEG2RAD(temp.bearingAngle)); // ENU
    temp.y = temp.harversineDistance*cos(DEG2RAD(temp.bearingAngle));
    temp.z = p2.alt - p1.alt;
    temp.euclidean = sqrt(pow(temp.x,2)+pow(temp.y,2)+pow(temp.z,2));

    return temp;

}

int vaant::applyHaversineBearingToArray(){
//  Apply Haversine and Bearing formula to all coordinates in setpointList
//      - Return:
//              101: erros for empty setpoint list
double h, beta;

    if(numSetpoints == -1)
        return 101; // erro de setpoint vazio

    else{
        if(numSetpointsGreatCirles != -1)
            clearSetpointGreatCircleList();
        if(numSetpointsDelta !=-1)
            clearSetpointDelta();

        numSetpointsGreatCirles = numSetpoints-1;
        numSetpointsDelta = numSetpointsGreatCirles;

        vaant::hbParameters temp;
        setPointGreatCircleList = new greatCircle[numSetpointsGreatCirles]; // dynamic array // Temos numSetpoint - 1 deslocamentos
        setPointDelta = new displacement[numSetpointsDelta];

        for (int i=1;i< numSetpoints; i++){

            coord p1 = {setPointList[i-1].x, setPointList[i-1].y, setPointList[i-1].z };
            coord p2 = {setPointList[i].x, setPointList[i].y, setPointList[i].z };

             temp = applyHaversineBearing(p1,p2);
//            h = transformHaversine(p1,p2);
//            beta = transformBearing(p1,p2);

            setPointGreatCircleList[i-1].harversineDistance = temp.harversineDistance;
            setPointGreatCircleList[i-1].bearingAngle = temp.bearingAngle;

            setPointDelta[i-1].x = temp.x; // ENU
            setPointDelta[i-1].y = temp.y;
            setPointDelta[i-1].z = temp.z;

            cout << "x: " << setPointDelta[i].x << " y: " << setPointDelta[i].y << " z: " << setPointDelta[i].z << endl;

        }


    }

}

int vaant::applyHaversineBearingToArrayRefHome(vaant::coord home){
//  Apply Haversine and Bearing formula to all coordinates in setpointList with reference in the home location
//      - Return:
//              101: erros for empty setpoint list

    if(numSetpoints == -1)
        return 101; // erro de setpoint vazio

    else{
        if(numSetpointsGreatCirles != -1)
            clearSetpointGreatCircleList();
        if(numSetpointsDeltaRefHome != -1)
            clearSetpointDeltaRefHome();

        numSetpointsGreatCirles = numSetpoints;
        numSetpointsDeltaRefHome = numSetpointsGreatCirles;

        vaant::hbParameters temp;
        setPointGreatCircleList = new greatCircle[numSetpointsGreatCirles];
        setPointDeltaRefHome = new displacement[numSetpointsDeltaRefHome];

        for (int i=0; i < numSetpoints; i++){

            coord p2 = {setPointList[i].x, setPointList[i].y, setPointList[i].z};

            temp = applyHaversineBearing(home,p2);

            setPointGreatCircleList[i].harversineDistance = temp.harversineDistance;
            setPointGreatCircleList[i].bearingAngle = temp.bearingAngle;

            setPointDeltaRefHome[i].x = temp.x;
            setPointDeltaRefHome[i].y = temp.y;
            setPointDeltaRefHome[i].z = temp.z;

            cout << "x: " << setPointDeltaRefHome[i].x << " y: " << setPointDeltaRefHome[i].y << " z: " << setPointDeltaRefHome[i].z << endl;
        }
    }
}

//####################################################################################################################################################
// Clear a specific array

void vaant::clearSetpointList(){
// Function: Delete the SetpointList already created and remove the previous memory allocation
        delete [] setPointList;
        setPointList = NULL; 
        numSetpoints = -1;
}

void vaant::clearSetpointGreatCircleList(){
// Function: Delete the SetpointList already created and remove the previous memory allocation
        delete [] setPointGreatCircleList;
        setPointGreatCircleList = NULL;
        numSetpointsGreatCirles = -1;
}

void vaant::clearSetpointDelta(){
// Function: Delete the SetpointList already created and remove the previous memory allocation
        delete [] setPointDelta;
        setPointDelta = NULL;
        numSetpointsDelta = -1;
}

void vaant::clearSetpointDeltaRefHome(){
// Function: Delete the SetpointList already created and remove the previous memory allocation
        delete [] setPointDeltaRefHome;
        setPointDeltaRefHome = NULL;
        numSetpointsDeltaRefHome = -1;
}

//####################################################################################################################################################
// Get a element of a specific array

vaant::displacement vaant::getSetpoint(int index){
// Function: get the Setpoint of a specific index
// Parameters: index from the specific Sepoint
// Return: Erro msg
//         The Setpoint

    if(index > numSetpoints-1){
        cout << "Index exceed the number of Setpoint from the list" <<endl;
    }
    else
        return(setPointList[index]);
}

vaant::greatCircle vaant::getSetpointGreatCircle(int index){
// Function: get the Setpoint of a specific index
// Parameters: index from the specific Sepoint
// Return: Erro msg
//         The Setpoint

    if(index > numSetpointsGreatCirles-1){
        cout << "Index exceed the number of Setpoint from the list" <<endl;
    }
    else
        return(setPointGreatCircleList[index]);
}

vaant::displacement vaant::getSetpointDelta(int index){
// Function: get the Setpoint of a specific index
// Parameters: index from the specific Sepoint
// Return: Erro msg
//         The Setpoint

    if(index > numSetpointsDelta-1){
        cout << "Index exceed the number of Setpoint from the list" <<endl;
    }
    else
        return(setPointDelta[index]);
}

vaant::displacement vaant::getSetpointDeltaRefHome(int index){
// Function: get the Setpoint of a specific index
// Parameters: index from the specific Sepoint
// Return: Erro msg
//         The Setpoint

    if(index > numSetpointsDeltaRefHome-1){
        cout << "Index exceed the number of Setpoint from the list" <<endl;
    }
    else
        return(setPointDeltaRefHome[index]);
}

//####################################################################################################################################################
// Get numbers of elements of a specific array

int vaant::getNumSetpoints(){
// Function: Return the number of setpoints on the list
// parameters:
// Return: -1 - Error
//          Else - Number of Setpoints
    return numSetpoints;
}

int vaant::getNumSetpointsGreatCircle(){
// Function: Return the number of setpoints GreatCircle on the list
// parameters:
// Return: -1 - Error
//          Else - Number of Setpoints
    return numSetpointsGreatCirles;
}

int vaant::getNumSetpointsDelta(){
// Function: Return the number of setpoints delta from 2 latitude on the list
// parameters:
// Return: -1 - Error
//          Else - Number of Setpoints
    return numSetpointsDelta;
}

int vaant::getNumSetpointsDeltaRefHome(){
// Function: Return the number of setpoints delta from 2 latitude on the list
// parameters:
// Return: -1 - Error
//          Else - Number of Setpoints
    return numSetpointsDeltaRefHome;
}

//####################################################################################################################################################
// Callbacks

void vaant::chatterCallback_PoseMavros(const geometry_msgs::PoseStamped::ConstPtr& msg3){
    // Function: Callback /mavros/local_position/pose. Get pose of Vant relative Inertial Frame {I}
    poseMavros = *msg3; //get Vant pose {V} relative inercial frame {I}
    tf::poseMsgToTF(msg3->pose, TH_V_I); // get homogeneous transform of vant {V} relative inertial frame {I}
    static tf::TransformBroadcaster br;
    br.sendTransform(tf::StampedTransform(TH_V_I,ros::Time::now(),"world","vant"));
    //  cout << TH_V_I.getRotation().getX() << " "<< TH_V_I.getRotation().getY() << " "<< TH_V_I.getRotation().getZ() << " "<< TH_V_I.getRotation().getW() << " "<< endl;
}

void vaant::chatterCallback_GPSMavros(const sensor_msgs::NavSatFix::ConstPtr& msg4){
    poseGPSmavros = *msg4;
}

void vaant::chatterCallback_GPSMavrosAltRel(const std_msgs::Float64::ConstPtr& msg){
    altRel = *msg;
}

void vaant::chatterCallback_State(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void vaant::chatterCallback_WPList(const mavros_msgs::WaypointList::ConstPtr& msg){
    wpList = *msg;
}

void vaant::chatterCallback_MissionFinished(const mavros_msgs::MavlinkConstPtr& msg){
    if(msg->msgid == MAVLINK_MSG_ID_MISSION_ITEM_REACHED){
        MISSION_ITEM_REACHED = *msg;
        //missionFinished = true;
    }
    //cout<<"FlagCall:"<<missionFinished<<endl;
}

//####################################################################################################################################################

void vaant::getCurrentGlobalPosition(mavros_msgs::Waypoint &wp, int frame){
    while(!this->poseGPSmavros.latitude && ros::ok()){ // fica nesse loop ate o valor de position.z ser alterado
        ros::spinOnce(); // atualiza o callback
    }
    switch (frame) {
    case mavros_msgs::Waypoint::FRAME_GLOBAL:
        wp.x_lat = poseGPSmavros.latitude;
        wp.y_long = poseGPSmavros.longitude;
        wp.z_alt = poseGPSmavros.altitude;
        break;
    case mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT:
        wp.x_lat = poseGPSmavros.latitude;
        wp.y_long = poseGPSmavros.longitude;
        wp.z_alt = altRel.data;
        break;
    default:
        break;
    }

}

void vaant::getWPList(mavros_msgs::WaypointList &wp){
    wp = wpList;
}

//####################################################################################################################################################

void vaant::arrived(vaant::displacement wp, geometry_msgs::PoseStamped poseVant, float tol){
    /* Function: Verify if vant reached the setpoint
       Parameters: wp - setpoints
                   poseVant
                   tol - error tolerance
    */
    if(abs(wp.x - poseVant.pose.position.x) < tol && abs(wp.y - poseVant.pose.position.y) < tol && abs(wp.z - poseVant.pose.position.z) < tol){
        done = 1; // quando atingir o objetivo, seta para DONE = 1, avisando que ja chegou;
    }
    else done = 0;
}

bool vaant::compareGPS(mavros_msgs::Waypoint wp1,mavros_msgs::Waypoint wp2, float tol){
    vaant::hbParameters hb;
    hb = applyHaversineBearing(wp1,wp2);
    if(hb.euclidean <= tol)
        return true;
    else return false;
}

bool vaant::arrivedGPS(mavros_msgs::Waypoint wpGoal,float tolerancia){

    //ros::Subscriber subPos = nodeHandle.subscribe("/mavros/global_position/global", 1, &vaant::chatterCallback_GPSMavros, this);

    while(!(this->poseGPSmavros.latitude && this->altRel.data) && ros::ok()){ // fica nesse loop ate o valor de position.z ser alterado
        ros::spinOnce(); // atualiza o callback
    }

    //utilizar filtro media movel
    mavros_msgs::Waypoint wp;
    wp.x_lat = this->poseGPSmavros.latitude;
    wp.y_long = this->poseGPSmavros.longitude;
    wp.z_alt = this->altRel.data;
//    cout<<"GPS\n"<<setprecision(9)<<wp<<endl;

    bool res = compareGPS(wp,wpGoal,tolerancia);

    return res;
}

//####################################################################################################################################################

double vaant::goToPosition(tf::Vector3 toGo, float tol){
// Function: send the UAV to a specific position in frame body reference
// Parameters: - toGo: the disclacement to send x, y and z (meters)
//             - tol: tolerance to check if the position was reached

    ros::Rate loop_rate(rate);
    vant::ToOffboard msg;
    double tempX, tempY, tempZ, tempYaw;

    displacement toDesloc;

    while(!poseMavros.pose.position.z)
        ros::spinOnce();

    MatrixXd TH_v_i(4,4), // TF Matrix of the vehicle based on inertial
                P_v(4,1),    // Point to go based on vehicle
                P_i(4,1);    // Point to go based on inertial

    tempX = poseMavros.pose.position.x;
    tempY = poseMavros.pose.position.y;
    tempZ = poseMavros.pose.position.z;

    tempYaw = tf::getYaw(poseMavros.pose.orientation); // In radians

    TH_v_i        << cos(tempYaw), -1*sin(tempYaw), 0, tempX,  // Conferir se e essa matriz mesmo
                     sin(tempYaw),    cos(tempYaw), 0, tempY,
                     0,                   0,        1, tempZ,
                     0,                   0,        0,     1;

    P_v << toGo.getX(),
         toGo.getY(),
         toGo.getZ(),
              1;

    P_i = TH_v_i*P_v;

    toDesloc.x = P_i(0,0); // Only to use in the arrived method
    toDesloc.y = P_i(1,0);
    toDesloc.z = P_i(2,0);

    msg.index = 1; // To Position Mode // Used in new version of toPublish
    msg.PoseStamped.pose.position.x = P_i(0,0);
    msg.PoseStamped.pose.position.y = P_i(1,0);
    msg.PoseStamped.pose.position.z = P_i(2,0);
    msg.PoseStamped.pose.orientation = poseMavros.pose.orientation;

    done = 0;

    timeval t1,t2;
    gettimeofday(&t1, NULL);

    while(ros::ok() && done==0){

        ros::spinOnce();
        chatter_offboard.publish(msg);
        arrived(toDesloc,this->poseMavros,tol);
        loop_rate.sleep();

    }

    gettimeofday(&t2, NULL);

    double deltaT = double(t2.tv_sec -t1.tv_sec ) + double(t2.tv_usec-t1.tv_usec) * 1.e-8;// time of displacement

    return deltaT;

}

double vaant::goToPosition(tf::Vector3 toGo, float tol, vision &v){
// Function: send the UAV to a specific position in frame body reference.
//           Shutdown when see any marker of vision object
// Parameters: - toGo: the disclacement to send x, y and z (meters)
//             - tol: tolerance to check if the position was reached
//             - vision object

    ros::Rate loop_rate(rate);
    vant::ToOffboard msg;
    double tempX, tempY, tempZ, tempYaw;

    displacement toDesloc;

    while(!poseMavros.pose.position.z)
        ros::spinOnce();

    MatrixXd TH_v_i(4,4), // TF Matrix of the vehicle based on inertial
                P_v(4,1),    // Point to go based on vehicle
                P_i(4,1);    // Point to go based on inertial

    tempX = poseMavros.pose.position.x;
    tempY = poseMavros.pose.position.y;
    tempZ = poseMavros.pose.position.z;

    tempYaw = tf::getYaw(poseMavros.pose.orientation); // In radians

    TH_v_i        << cos(tempYaw), -1*sin(tempYaw), 0, tempX,  // Conferir se e essa matriz mesmo
                     sin(tempYaw),    cos(tempYaw), 0, tempY,
                     0,                   0,        1, tempZ,
                     0,                   0,        0,     1;

    P_v << toGo.getX(),
         toGo.getY(),
         toGo.getZ(),
              1;

    P_i = TH_v_i*P_v;

    toDesloc.x = P_i(0,0); // Only to use in the arrived method
    toDesloc.y = P_i(1,0);
    toDesloc.z = P_i(2,0);

    msg.index = 1; // To Position Mode // Used in new version of toPublish
    msg.PoseStamped.pose.position.x = P_i(0,0);
    msg.PoseStamped.pose.position.y = P_i(1,0);
    msg.PoseStamped.pose.position.z = P_i(2,0);
    msg.PoseStamped.pose.orientation = poseMavros.pose.orientation;

    done = 0;

    timeval t1,t2;
    gettimeofday(&t1, NULL);

    while(ros::ok() && done==0 && v.getPoseAlvar().id == 0 /* or if view no marker */){
        ros::spinOnce();
        chatter_offboard.publish(msg);
        arrived(toDesloc,this->poseMavros,tol);
        loop_rate.sleep();

    }

    gettimeofday(&t2, NULL);

    double deltaT = double(t2.tv_sec -t1.tv_sec ) + double(t2.tv_usec-t1.tv_usec) * 1.e-8;// time of displacement

    return deltaT;

}


//####################################################################################################################################################

void vaant::getDone(int &x){
    /* Function: Verify if vant reached the setpoint
       Parameters: x - get done value. When vant reached the setpoint, Done = 1. Else Done = 0
    */
     x = done;
}

//####################################################################################################################################################

void vaant::setDone(int x){
    /* Function: set Done = x
    */
     done = x;
}

//####################################################################################################################################################

tf::StampedTransform vaant::createHT_Matrix(double x, double y,double z, double row, double pitch, double yaw){
    tf::Vector3 trans;
    tf::Quaternion quat;
    tf::StampedTransform matrix;

    trans.setValue(x,y,z);
    matrix.setOrigin(trans);

    quat.setRPY(row,pitch,yaw);
    matrix.setRotation(quat);

    return matrix;
}

//####################################################################################################################################################

void vaant::printMatrix(string nome,tf::StampedTransform m){
    /* Function: print Homogeneous Transformation Matrix
       Parameters: name - Matrix name will be print
                   m - Homogeneous Transformation Matrix
    */

    MatrixXd matEigen(4,4); // Variavel criada a partir da biblioteca Eigen que tbm oferece suporta a algebra linear

    matEigen << m.getBasis().getRow(0).getX(), m.getBasis().getRow(0).getY(), m.getBasis().getRow(0).getZ(),m.getOrigin().getX(),
                m.getBasis().getRow(1).getX(), m.getBasis().getRow(1).getY(), m.getBasis().getRow(1).getZ(),m.getOrigin().getY(),
                m.getBasis().getRow(2).getX(), m.getBasis().getRow(2).getY(), m.getBasis().getRow(2).getZ(),m.getOrigin().getZ(),
                0,0,0,1;

    cout<<"\n"<<nome<<" = \n"<<matEigen<<endl;

    //OBS: criei uma variavel utilizando a biblioteca Eigen so pq ela imprime de forma mais elegante hahaha.
}

//####################################################################################################################################################

void vaant::printMatrix(string nome,tf::Matrix3x3 m){
    /* Function: print Homogeneous Transformation Matrix
       Parameters: name - Matrix name will be print
                   m - Homogeneous Transformation Matrix
    */

    MatrixXd matEigen(3,3); // Variavel criada a partir da biblioteca Eigen que tbm oferece suporta a algebra linear

    matEigen << m.getRow(0).getX(), m.getRow(0).getY(), m.getRow(0).getZ(),
                m.getRow(1).getX(), m.getRow(1).getY(), m.getRow(1).getZ(),
                m.getRow(2).getX(), m.getRow(2).getY(), m.getRow(2).getZ();

    cout<<"\n"<<nome<<" = \r\n"<<matEigen<<"\r"<<endl;

    //OBS: criei uma variavel utilizando a biblioteca Eigen so pq ela imprime de forma mais elegante hahaha.
}

//####################################################################################################################################################

tf::StampedTransform vaant::getHTmatrix(int HTmatrix){
    switch(HTmatrix){
        case HT_M_C_aux:
            return TH_M_C_aux;
            break;

        case HT_V_I:
            return TH_V_I;
            break;

        default:
            cout<<"\nThere is no variable with this name."<<endl;

    }
}

//####################################################################################################################################################

MatrixXd vaant::TF2THEigen(tf::StampedTransform m){
    MatrixXd eigen(4,4);
    eigen << m.getBasis().getRow(0).getX(), m.getBasis().getRow(0).getY(), m.getBasis().getRow(0).getZ(),m.getOrigin().getX(),
            m.getBasis().getRow(1).getX(), m.getBasis().getRow(1).getY(), m.getBasis().getRow(1).getZ(),m.getOrigin().getY(),
            m.getBasis().getRow(2).getX(), m.getBasis().getRow(2).getY(), m.getBasis().getRow(2).getZ(),m.getOrigin().getZ(),
            0,0,0,1;

    return eigen;
}

//####################################################################################################################################################

tf::Vector3 vaant::applyENU2MavrosRef(tf::Vector3 enuDesloc){
// Function: Apply ENU vector to MAVROS reference without displacement - Actual MAVROS reference is NWU
    vaant q;
    tf::StampedTransform TH_ENU_I;
    tf::Vector3 nwuDesloc;

    TH_ENU_I = q.createHT_Matrix(0,0,0,0,0,DEG2RAD(90.000));//monta mat trans. homogenea do frame ENU em relacao ao world(ROS usa o NWU)
    nwuDesloc = tf::quatRotate(TH_ENU_I.getRotation(),enuDesloc); // Transformando para referencia de atuacao NWU do ROS

    return nwuDesloc;
}

//####################################################################################################################################################
bool vaant::sendMission(const mission &m){
    clearMission();
    ros::ServiceClient set_waypoint = nodeHandle.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");

    mavros_msgs::WaypointPush waypoints;
    mavros_msgs::Waypoint* wp;

    wp = new mavros_msgs::Waypoint[m.getJsonSize()];
    wp = m.getJsonWp();

    for(int i=0;i<m.getJsonSize();i++){
        waypoints.request.waypoints.push_back(wp[i]);
    }
    while(waypoints.response.success == 0)
        set_waypoint.call(waypoints);

    if(waypoints.response.success == 1) //envia para a pixhawk
        ROS_INFO("MISSION WAS SENDED SUCCESSFULLY");
    else ROS_ERROR("SEND MISSION ERROR");

    return waypoints.response.success;
}

//####################################################################################################################################################
bool vaant::clearMission(){

    ros::ServiceClient clear = nodeHandle.serviceClient<mavros_msgs::WaypointClear>("mavros/mission/clear");
    mavros_msgs::WaypointClear srv;

    while(srv.response.success == 0)
        clear.call(srv);

    if(srv.response.success == 1) //envia para a pixhawk
        ROS_INFO("WAYPOINT LIST WAS ERASED SUCCESSFULLY");
    else ROS_ERROR("ERROR WHILE ERASING WAYPOINT LIST");
    return srv.response.success;
}

//#####################################################################################################################################################

void vaant::getStateMode(mavros_msgs::State &state){
    state = current_state;
}

bool vaant::changeMode(int mode){
    string nameMode;
    vaant::caseMode(mode,nameMode);
    mavros_msgs::SetMode offb_set_mode;
    ros::Rate rate(20.0);
    mavros_msgs::State state;

    offb_set_mode.request.custom_mode = nameMode;
    set_mode_client.call(offb_set_mode);

    ros::spinOnce();
    this->getStateMode(state);
    while(state.mode != nameMode && ros::ok()){
        this->getStateMode(state);
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("Flight Mode Rejected");
    }

    ROS_INFO("Flight Mode was changed successfully");
    cout<<nameMode<<endl;

    return offb_set_mode.response.mode_sent;
}

void vaant::caseMode(int mode,string &nameMode){
    switch (mode) {
    case vaant::MODE::MANUAL:
        nameMode = "MANUAL";
        break;
    case vaant::MODE::ACRO:
        nameMode = "ACRO";
        break;
    case vaant::MODE::ALTCTL:
        nameMode = "ALTCTL";
        break;
    case vaant::MODE::POSCTL:
        nameMode = "POSCTL";
        break;
    case vaant::MODE::OFFBOARD:
        nameMode = "OFFBOARD";
        break;
    case vaant::MODE::RATTITUDE:
        nameMode = "RATTITUDE";
        break;
    case vaant::MODE::AUTO_MISSION:
        nameMode = "AUTO.MISSION";
        break;
    case vaant::MODE::AUTO_LOITER:
        nameMode = "AUTO.LOITER";
        break;
    case vaant::MODE::AUTO_RTL:
        nameMode = "AUTO.RTL";
        break;
    case vaant::MODE::AUTO_LAND:
        nameMode = "AUTO.LAND";
        break;
    case vaant::MODE::AUTO_RTGS:
        nameMode = "AUTO.RTGS";
        break;
    case vaant::MODE::AUTO_READY:
        nameMode = "AUTO.READY";
        break;
    case vaant::MODE::AUTO_TAKEOFF:
        nameMode = "AUTO.TAKEOFF";
        break;
    default:
        nameMode = "error";
        ROS_ERROR("Modo Invalido");
        cout<<"Os modos Disponiveis são:"<<endl;
        cout<<"   0 - Manual"<<endl;
        cout<<"   1 - ACRO"<<endl;
        cout<<"   2 - ALTCTL"<<endl;
        cout<<"   3 - POSCTL"<<endl;
        cout<<"   4 - OFFBOARD"<<endl;
        cout<<"   5 - RATTITUDE"<<endl;
        cout<<"   6 - AUTO.MISSION"<<endl;
        cout<<"   7 - AUTO.LOITER"<<endl;
        cout<<"   8 - AUTO.RTL"<<endl;
        cout<<"   9 - AUTO.LAND"<<endl;
        cout<<"   10 - AUTO.RTGS"<<endl;
        cout<<"   11 - AUTO.READY"<<endl;
        cout<<"   12 - AUTO.TAKEOFF"<<endl;
    }
}

//void vaant::getMissionFinished(int size,bool &flag){

//    mavros_msgs::mavlink::convert(MISSION_ITEM_REACHED,mavlinkMSG);//Essa funcao converte msg do tipo ROS para Mavlink e do Mavlink para ROS. Neste caso ela esta convertendo do ROS para mavlink_t
//    mavlink_msg_mission_item_reached_decode(&mavlinkMSG,&mavN46);//Extrai o payload da mensagem mavlink para uma mensagem especifica mavlink, no caso a mensagem 46

//    cout<<"Payload da msg Mavlink n46: "<<mavN46.seq<<endl;//Imprime o payload da mensagem mavlink
//    if(mavN46.seq == (size-1)){
//        flag = true;
//    }
//}

bool vaant::changeHome(mavros_msgs::Waypoint wp){
    ros::ServiceClient service = nodeHandle.serviceClient<mavros_msgs::CommandHome>("/mavros/cmd/set_home");
    mavros_msgs::CommandHome setHome;

    setHome.request.latitude = wp.x_lat;
    setHome.request.longitude = wp.y_long;
    setHome.request.altitude = wp.z_alt;

    service.call(setHome);
    return setHome.response.success;
}

bool vaant::changeHome(mavros_msgs::WaypointList wpList){
    ros::ServiceClient service = nodeHandle.serviceClient<mavros_msgs::CommandHome>("/mavros/cmd/set_home");
    mavros_msgs::CommandHome setHome;
    mavros_msgs::Waypoint wp;
    int size = wpList.waypoints.size();
    wp = wpList.waypoints[size-1];

    setHome.request.latitude = wp.x_lat;
    setHome.request.longitude = wp.y_long;
    this->getCurrentGlobalPosition(wp,mavros_msgs::Waypoint::FRAME_GLOBAL);
//    cout<<"Altura: "<<wp.z_alt<<endl;
    setHome.request.altitude = wp.z_alt;

    service.call(setHome);
    return setHome.response.success;
}

double vaant::calcMedian(std::vector<double> v){
    // Function: calculate a median value of a vector
    // Parameters:
    //            v: vector to median

    std::sort(v.begin(),v.end());

    if(v.size()%2==1){// Impar
        return (v[(v.size()-1)/2]);
    }
    else{//Par
        return (v[v.size()/2-1] + v[v.size()/2])/2;
    }
}

std::vector<double> vaant::addShift(std::vector<double> v,double c){
     // Function: shift a vector by 1 value ans insert a new one in the beginning
     // Parameters:
     //             v: vector to shift
     //             c: new value to inser

    for(int i=v.size()-1; i>0;i--)
        v[i]=v[i-1];

    v[0] = c;

    return v;
}
