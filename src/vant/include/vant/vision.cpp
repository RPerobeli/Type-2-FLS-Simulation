#include "vision.h"

vision::vision(ros::NodeHandle n) : nMarkerMax(1)
{
    subMultiAlvar = nodeHandle.subscribe("/vision/main_tag_pose", 1, &vision::chatterCallbackPoseAlvar, this);
    cout << " Utilizando Topico /vision/main_tag_pose para o objeto: " << endl ;
}

vision::vision(ros::NodeHandle n ,int i, int OPTION, int V) : nMarkerMax(i)
{   
// Function:   Constructor of vision object
// Parameters: NodeHandle: Node handle of the node
//             i: number of the max id that exists on the bundle marker
//             OPTION: MULTI or SINGLE ar pose topic to subscribe OBS: Another node multitag bundles is necessary for MULTI
//                     MULTI = 2; SINGLE = any number
//             V: versao do marcador de pouso
//                      V = 0 para banner com 8 marcadores de papel colado no banner com marcadores 0 e 15
//                      V = 1 para banner de 1,4m x 1,4m com 10 marcadores
//                      V = 2 para banner de 1,5m x 2,5m com 39 marcadores

    f.flagEmpty = 1;
    f.flagChooseID = -1;
    i = i+1;

//    markers[nMarkerMax]; // JP: desse jeito pode existir problema de superposição.
//    msg2[nMarkerMax];    //

    markers = new int[nMarkerMax];                           // JP: Logo declarei como dinâmico
    msg2 = new ar_track_alvar_msgs::AlvarMarker[nMarkerMax]; // JP: Nao achei conveniente dar free nesses vetores durate a execução

    VERSION = V;
    nodeHandle = n;

    if (OPTION == vision::VISION_ARGUMENTS::opt.multi){

          subMultiAlvar = nodeHandle.subscribe("/vision/main_tag_pose", 1, &vision::chatterCallbackPoseAlvar, this);
          cout << " Utilizando Topico /vision/main_tag_pose para o objeto: " << GET_NAME(this) << endl ;
    }
    else{

         subAlvar = nodeHandle.subscribe("/ar_pose_marker", 1, &vision::chatterCallbackPoseMultiAlvar, this);
         cout << " Utilizando Topico /ar_pose_marker para o objeto:  " << GET_NAME(this) << endl ;
    }

}

int vision::getNMARKER()
{
    return nMarkerMax;
}

vision::~vision(){

    delete [] msg2; //free the allocated memory
    msg2 = NULL;

    delete [] markers; //free the allocated memory
    markers = NULL;

}

// ############################################################################# //
// ################### Definindo as Versoes dos Lands ########################## //
// ############################################################################# //

void vision::landV0(int i){
    // Function: Defining the version V0 of land marker. V0 eh o banner com 8 marcadores de papel colado no banner com marcadores 0 e 15

    if(msg2[markers[i]].id == 1 || msg2[markers[i]].id == 2 || msg2[markers[i]].id == 3 || msg2[markers[i]].id == 4 || msg2[markers[i]].id == 5 || msg2[markers[i]].id == 6 || msg2[markers[i]].id == 7 || msg2[markers[i]].id == 8 || msg2[markers[i]].id == 15){
        poseAlvar.pose.position.x = msg2[markers[i]].pose.pose.position.x/9;
        poseAlvar.pose.position.y = msg2[markers[i]].pose.pose.position.y/9;
        poseAlvar.pose.position.z = msg2[markers[i]].pose.pose.position.z/9;
        poseAlvar.pose.orientation = msg2[markers[i]].pose.pose.orientation;
    }

    if(msg2[markers[i]].id == 9 || msg2[markers[i]].id ==10 || msg2[markers[i]].id == 11 || msg2[markers[i]].id == 12) {
        poseAlvar.pose.position.x = msg2[markers[i]].pose.pose.position.x/16;
        poseAlvar.pose.position.y = msg2[markers[i]].pose.pose.position.y/16;
        poseAlvar.pose.position.z = msg2[markers[i]].pose.pose.position.z/16;
        poseAlvar.pose.orientation = msg2[markers[i]].pose.pose.orientation;
    }

}

void vision::landV1(int i){
    // Function: Defining the version V1 of land marker. V1 eh o banner com 10 marcadores 1,4 x 1,4
    //Markers's IDs [0,2,5,6,8,9,11,12,15,16] and marker 15 is the reference

    if(msg2[markers[i]].id == 0)
        poseAlvar.pose = msg2[markers[i]].pose.pose;

    if(msg2[markers[i]].id == 2 || msg2[markers[i]].id == 5 || msg2[markers[i]].id == 6 || msg2[markers[i]].id == 8 || msg2[markers[i]].id == 9 || msg2[markers[i]].id == 11 || msg2[markers[i]].id == 12 || msg2[markers[i]].id == 15 || msg2[markers[i]].id == 16){
        poseAlvar.pose.position.x = msg2[markers[i]].pose.pose.position.x/9;
        poseAlvar.pose.position.y = msg2[markers[i]].pose.pose.position.y/9;
        poseAlvar.pose.position.z = msg2[markers[i]].pose.pose.position.z/9;
        poseAlvar.pose.orientation = msg2[markers[i]].pose.pose.orientation;
    }

}

void vision::landV2(int i){
    // Function: Defining the version V1 of land marker. V1 is the banner with 10 markers and dimension 1,4 x 1,4 m
    // Markers's IDs [0,2,5,6,8,9,11,12,15,16] and the marker 15 is the reference on multiTagLandingV1.cpp
    if(msg2[markers[i]].id == 0)
        poseAlvar.pose = msg2[markers[i]].pose.pose;

    if(msg2[markers[i]].id == 2 || msg2[markers[i]].id == 5 || msg2[markers[i]].id == 8 || msg2[markers[i]].id == 11 || msg2[markers[i]].id == 13 || msg2[markers[i]].id == 15 || msg2[markers[i]].id == 18 || msg2[markers[i]].id == 27 || msg2[markers[i]].id == 32 || msg2[markers[i]].id == 38 || msg2[markers[i]].id == 43 || msg2[markers[i]].id == 54 || msg2[markers[i]].id == 60 || msg2[markers[i]].id == 64 || msg2[markers[i]].id == 68 || msg2[markers[i]].id == 79 || msg2[markers[i]].id == 80 || msg2[markers[i]].id == 95 || msg2[markers[i]].id == 98){
        poseAlvar.pose.position.x = msg2[markers[i]].pose.pose.position.x/9;
        poseAlvar.pose.position.y = msg2[markers[i]].pose.pose.position.y/9;
        poseAlvar.pose.position.z = msg2[markers[i]].pose.pose.position.z/9;
        poseAlvar.pose.orientation = msg2[markers[i]].pose.pose.orientation;
    }

    if(msg2[markers[i]].id == 1 || msg2[markers[i]].id == 4 || msg2[markers[i]].id == 6 || msg2[markers[i]].id == 9 || msg2[markers[i]].id == 12 || msg2[markers[i]].id == 14 || msg2[markers[i]].id == 16 || msg2[markers[i]].id == 25 || msg2[markers[i]].id == 31 || msg2[markers[i]].id == 34 || msg2[markers[i]].id == 40 || msg2[markers[i]].id == 45 || msg2[markers[i]].id == 59 || msg2[markers[i]].id == 61 || msg2[markers[i]].id == 63 || msg2[markers[i]].id == 73 || msg2[markers[i]].id == 75 || msg2[markers[i]].id == 91 || msg2[markers[i]].id == 96 || msg2[markers[i]].id == 100){
        poseAlvar.pose.position.x = msg2[markers[i]].pose.pose.position.x/16;
        poseAlvar.pose.position.y = msg2[markers[i]].pose.pose.position.y/16;
        poseAlvar.pose.position.z = msg2[markers[i]].pose.pose.position.z/16;
        poseAlvar.pose.orientation = msg2[markers[i]].pose.pose.orientation;
    }

}

// ############################################################################# //

// JP MUDOU PARA TESTE DA OSCILACAO DO CHAVEAMENTO DO MARCADOR
void vision::landV3(int i){
    // Function: Defining the version V1 of land marker. V1 is the banner with 10 markers and dimension 1,4 x 1,4 m
    // Markers's IDs [0,2,5,6,8,9,11,12,15,16] and the marker 15 is the reference on multiTagLandingV1.cpp
    // É O LANDV2 INVERTIDO, OU SEJA, DEVE SER USADO COM MARCADOR MENOR CALIBRADO.
    if(msg2[markers[i]].id == 0)
        poseAlvar.pose.position.x = msg2[markers[i]].pose.pose.position.x*16.67;
        poseAlvar.pose.position.y = msg2[markers[i]].pose.pose.position.y*16.67;
        poseAlvar.pose.position.z = msg2[markers[i]].pose.pose.position.z*16.67;
        poseAlvar.pose.orientation = msg2[markers[i]].pose.pose.orientation;

    if(msg2[markers[i]].id == 2 || msg2[markers[i]].id == 5 || msg2[markers[i]].id == 8 || msg2[markers[i]].id == 11 || msg2[markers[i]].id == 13 || msg2[markers[i]].id == 15 || msg2[markers[i]].id == 18 || msg2[markers[i]].id == 27 || msg2[markers[i]].id == 32 || msg2[markers[i]].id == 38 || msg2[markers[i]].id == 43 || msg2[markers[i]].id == 54 || msg2[markers[i]].id == 60 || msg2[markers[i]].id == 64 || msg2[markers[i]].id == 68 || msg2[markers[i]].id == 79 || msg2[markers[i]].id == 80 || msg2[markers[i]].id == 95 || msg2[markers[i]].id == 98){
        poseAlvar.pose.position.x = msg2[markers[i]].pose.pose.position.x*1.83;
        poseAlvar.pose.position.y = msg2[markers[i]].pose.pose.position.y*1.83;
        poseAlvar.pose.position.z = msg2[markers[i]].pose.pose.position.z*1.83;
        poseAlvar.pose.orientation = msg2[markers[i]].pose.pose.orientation;
    }

    if(msg2[markers[i]].id == 1 || msg2[markers[i]].id == 4 || msg2[markers[i]].id == 6 || msg2[markers[i]].id == 9 || msg2[markers[i]].id == 12 || msg2[markers[i]].id == 14 || msg2[markers[i]].id == 16 || msg2[markers[i]].id == 25 || msg2[markers[i]].id == 31 || msg2[markers[i]].id == 34 || msg2[markers[i]].id == 40 || msg2[markers[i]].id == 45 || msg2[markers[i]].id == 59 || msg2[markers[i]].id == 61 || msg2[markers[i]].id == 63 || msg2[markers[i]].id == 73 || msg2[markers[i]].id == 75 || msg2[markers[i]].id == 91 || msg2[markers[i]].id == 96 || msg2[markers[i]].id == 100){
        poseAlvar.pose = msg2[markers[i]].pose.pose;
    }

}

// ############################################################################# //

void vision::defineLandingPattern(int VERSION, int i){

    switch(VERSION){
        case 0:
            landV0(i);
            break;

        case 1:
            landV1(i);
            break;

        case 2:
            landV2(i);
            break;

        case 3: // JP MUDOU PARA TESTE DA OSCILACAO DO CHAVEAMENTO DO MARCADOR
            landV3(i);
            break;

        default: cout << "Versao de MARCADOR não definida" << endl;
    }

}

void vision::chatterCallbackPoseMultiAlvar(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg){
    // Function: Callback ar_pose_marker. Get pose of markers using MULTIMARKERS
    // The central position of the bundle

    nMarkers = msg->markers.size(); // Number of visible markers

    if(nMarkers > nMarkerMax)      // If there is more marker, It does not count as possible marker
        nMarkers = nMarkerMax;

    if(nMarkers > 0){//If there is markers
        int *aux = new int[nMarkers];


        for(int i=0;i<nMarkers;i++){

            msg2[i] = msg->markers[i];
            aux[i] = msg2[i].id+1;

        }
        fillVector(aux); //set poseAlvar with the closer marker or with only the visible marker
        chooseTag2();
        f.flagEmpty = 0; // Is it empty? No (False)

    }
    else{

        f.flagEmpty = 1; // Is it empty? Yes (True)
        for(int i = 0;i<nMarkerMax;i++){
            markers[i] = -1;
        }
    }

}

void vision::chatterCallbackPoseAlvar(const ar_track_alvar_msgs::AlvarMarker::ConstPtr& msg1){
// CallBack
    msgAlvar = *msg1;
    tf::poseMsgToTF(msgAlvar.pose.pose, TH_M_C_aux);// Armazena a Matriz de Transformacao Homogenea do marcador {M} em relacao a camera {C} na variavel TH_M_C
}

void vision::chooseTag2(){
// Function: set poseAlvar with Markers and these repectivily proportion

    int i = 0;

    while(markers[i]==-1){
        i++;
    }

    if(i>=nMarkerMax)
        f.flagEmpty = 1;

    else{

        f.flagChooseID = msg2[markers[i]].id;
        poseAlvar.pose = msg2[markers[i]].pose.pose;
        defineLandingPattern(VERSION, i);
        tf::poseMsgToTF(poseAlvar.pose, TH_M_C_aux);// Store the Homogeneous Transform MAtriz between the Marker {M} relative to the camera {C} at TH_M_C
        msgAlvar = msg2[markers[i]];
    }

}

void vision::fillVector(int *aux){
// Function: set poseAlvar with the closer marker or with only the visible marker

    int d,* p;

    for(int i=1;i<=nMarkerMax;i++){

        p = find(aux, aux + nMarkers, i); // Find the position of 'i' inside the vector aux // If there is no 'i', return last+1 position
        if(p!=(aux + nMarkers)){// If P exist
            d = distance(aux, p);
            markers[i-1] = d;
        }
        else markers[i-1] = -1;
    }

    delete [] aux; //free the allocated memory
    aux = NULL;

}

vision::flagList vision::getFlags(){
// Function: return the flag struct, with flagEmpty and flagIDchoose
    return f;
}

ar_track_alvar_msgs::AlvarMarker vision::getPoseAlvar(){
// Return the unique marker or the central position of the marker used in the vision. Works for both: Multi or Single.
    return msgAlvar;
}

tf::StampedTransform vision::getPoseAlvarHTmatrix(){
//    Function: return the HT matrix between the Marker relative to Camera
    return TH_M_C_aux;
}
