

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
#include <vant/vision.h>
//#include <vant/ToOffboard.h> //ANTES
#include "/home/laic/catkin_ws/src/vant/include/vant/ToOffboard.h" //RODRIGO ALTEROU ISSO AQUI

/// FUZZY ////
//#include "fl/Headers.h" //utilizado para o fuzzy tipo 1
//////////////

// ========================================================================================================================================= //
// ============================================ CODIGO QUE INTEGRA FUZZY, Multiplos Tags e MAVROS ========================================== //
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


// Fuzzy things

ofstream debugFuzzyParameters, debugPosition, debugPositionArPose, debugTimeToProc, FuzzyEntrances,TAGposition;                         // para gerar o LOG;

////////////////////////////////////////////////////////////// INICIO FUNÇAO FUZZY //////////////////////////////////////////////////////////////////

double FuzzyTipo2VX (double X, double Y)
{
    // DECLARAÇÃO DAS VARIÁVEIS DE INPUT
//cout << " "<< X<< " " << Y << " "<< endl;
    double NEGATIVOLONGE_INFERIOR_POSICAOX=0, NEGATIVOLONGE_SUPERIOR_POSICAOX=0;
    double PERTO_INFERIOR_POSICAOX=0, PERTO_SUPERIOR_POSICAOX=0;
    double POSITIVOLONGE_INFERIOR_POSICAOX=0, POSITIVOLONGE_SUPERIOR_POSICAOX=0;
    double NEGATIVOLONGE_INFERIOR_POSICAOY=0, NEGATIVOLONGE_SUPERIOR_POSICAOY=0;
    double PERTO_INFERIOR_POSICAOY=0, PERTO_SUPERIOR_POSICAOY=0;
    double POSITIVOLONGE_INFERIOR_POSICAOY=0, POSITIVOLONGE_SUPERIOR_POSICAOY=0;

    // DEFINIÇÃO DAS FUNÇÕES DE PERTINÊNCIA SUPERIOR E INFERIOR DAS VARIÁVEIS DE INPUT

    if (X <= -15) {
        NEGATIVOLONGE_INFERIOR_POSICAOX = 0.5;
    }
    else {
        if (X <= -10) {
            NEGATIVOLONGE_INFERIOR_POSICAOX =  -0.1 * X - 1;
        }
        else {
            NEGATIVOLONGE_INFERIOR_POSICAOX = 0;
        }
    }

    if (X <= -10) {
        NEGATIVOLONGE_SUPERIOR_POSICAOX = 1;
    }
    else {
        if (X <= 0) {
            NEGATIVOLONGE_SUPERIOR_POSICAOX =  -0.1 * X;
        }
        else {
            NEGATIVOLONGE_SUPERIOR_POSICAOX = 0;
        }
    }

    if (X <= -5) {
        PERTO_INFERIOR_POSICAOX = 0;
    }
    else {
        if (X <= 0) {
            PERTO_INFERIOR_POSICAOX =  0.1 * X + 0.5;
        }
        else {
                if (X <= 5) {
                    PERTO_INFERIOR_POSICAOX = -0.1*X+0.5;
                }
                else {
		
                    	PERTO_INFERIOR_POSICAOX = 0;
			                }
                }
        }
    

    if (X <= -15) {
        PERTO_SUPERIOR_POSICAOX = 0;
    }
    else {
        if (X <= -5) {
            PERTO_SUPERIOR_POSICAOX =  0.1 * X + 1.5;
        }
        else {
                if (X <= 5) {
                    PERTO_SUPERIOR_POSICAOX = 1;
                }
                else {
                        if (X <= 15) {
                            PERTO_SUPERIOR_POSICAOX = -0.1 * X + 1.5;
                        }
                        else {
                            PERTO_SUPERIOR_POSICAOX = 0;
                        }
                }
        }
    }

    if (X <= 10) {
        POSITIVOLONGE_INFERIOR_POSICAOX = 0;
    }
    else {
        if (X <= 15) {
            POSITIVOLONGE_INFERIOR_POSICAOX =  0.1 * X - 1;
        }
        else {
            POSITIVOLONGE_INFERIOR_POSICAOX = 0.5;
        }
    }

    if (X <= 0) {
        POSITIVOLONGE_SUPERIOR_POSICAOX = 0;
    }
    else {
        if (X <= 10) {
            POSITIVOLONGE_SUPERIOR_POSICAOX =  0.1 * X;
        }
        else {
            POSITIVOLONGE_SUPERIOR_POSICAOX = 1;
        }
    }

    if (Y <= -15) {
        NEGATIVOLONGE_INFERIOR_POSICAOY = 0.5;
    }
    else {
        if (Y <= -10) {
            NEGATIVOLONGE_INFERIOR_POSICAOY =  -0.1 * Y - 1;
        }
        else {
            NEGATIVOLONGE_INFERIOR_POSICAOY = 0;
        }
    }

    if (Y <= -10) {
        NEGATIVOLONGE_SUPERIOR_POSICAOY = 1;
    }
    else {
        if (Y <= 0) {
            NEGATIVOLONGE_SUPERIOR_POSICAOY =  -0.1 * Y;
        }
        else {
            NEGATIVOLONGE_SUPERIOR_POSICAOY = 0;
        }
    }

      if (Y <= -5) {
        PERTO_INFERIOR_POSICAOY = 0;
    }
    else {
        if (Y <= 0) {
            PERTO_INFERIOR_POSICAOY =  0.1 * Y + 0.5;
        }
        else {
                if (Y <= 5) {
                    PERTO_INFERIOR_POSICAOY = -0.1 * Y + 0.5;
                }
                else {
		
                    		PERTO_INFERIOR_POSICAOY = 0;
			                }
                }
        }
    

    if (Y <= -15) {
        PERTO_SUPERIOR_POSICAOY = 0;
    }
    else {
        if (Y <= -5) {
            PERTO_SUPERIOR_POSICAOY =  0.1 * Y + 1.5;
        }
        else {
                if (Y <= 5) {
                    PERTO_SUPERIOR_POSICAOY = 1;
                }
                else {
                        if (Y <= 15) {
                            PERTO_SUPERIOR_POSICAOY = -0.1 * Y + 1.5;
                        }
                        else {
                            PERTO_SUPERIOR_POSICAOY = 0;
                        }
                }
        }
    }

    if (Y <= 10) {
        POSITIVOLONGE_INFERIOR_POSICAOY = 0;
    }
    else {
        if (Y <= 15) {
            POSITIVOLONGE_INFERIOR_POSICAOY =  0.1 * Y - 1;
        }
        else {
            POSITIVOLONGE_INFERIOR_POSICAOY = 0.5;
        }
    }

    if (Y <= 0) {
        POSITIVOLONGE_SUPERIOR_POSICAOY = 0;
    }
    else {
        if (Y <= 10) {
            POSITIVOLONGE_SUPERIOR_POSICAOY =  0.1 * Y;
        }
        else {
            POSITIVOLONGE_SUPERIOR_POSICAOY = 1;
        }
    }

    // IMPRESSÃO DAS VARIÁVEIS DE INPUT PARA OS VALORES DE X E Y CAPTADOS PELA CÂMERA DO DRONE

    //printf ("IMPRESSÃO DAS VARIÁVEIS DE INPUT PARA OS VALORES DE X E Y CAPTADOS PELA CÂMERA DO DRONE");
    //printf ("\n NEGATIVO LONGE INFERIOR NA POSICAO X = %.2f         NEGATIVO LONGE SUPERIOR NA POSICAO X = %.2f", NEGATIVOLONGE_INFERIOR_POSICAOX, NEGATIVOLONGE_SUPERIOR_POSICAOX);
    //printf ("\n PERTO INFERIOR NA POSICAO X = %.2f                  PERTO SUPERIOR NA POSICAO X = %.2f", PERTO_INFERIOR_POSICAOX, PERTO_SUPERIOR_POSICAOX);
    //printf ("\n POSITIVO LONGE INFERIOR NA POSICAO X = %.2f         POSITIVO LONGE SUPERIOR NA POSICAO X = %.2f", POSITIVOLONGE_INFERIOR_POSICAOX, POSITIVOLONGE_SUPERIOR_POSICAOX);
    //printf ("\n NEGATIVO LONGE INFERIOR NA POSICAO Y = %.2f         NEGATIVO LONGE SUPERIOR NA POSICAO Y = %.2f", NEGATIVOLONGE_INFERIOR_POSICAOY, NEGATIVOLONGE_SUPERIOR_POSICAOY);
    //printf ("\n PERTO INFERIOR NA POSICAO Y = %.2f                  PERTO SUPERIOR NA POSICAO Y = %.2f", PERTO_INFERIOR_POSICAOY, PERTO_SUPERIOR_POSICAOY);
    //printf ("\n POSITIVO LONGE INFERIOR NA POSICAO Y = %.2f         POSITIVO LONGE SUPERIOR NA POSICAO Y = %.2f", POSITIVOLONGE_INFERIOR_POSICAOY, POSITIVOLONGE_SUPERIOR_POSICAOY);

    // DECLARAÇÃO DAS VARIÁVEIS DE OUTPUT

		//printf ("DECLARAÇÃO DAS VARIÁVEIS DE OUTPUT");
    double NEGATIVO_INFERIOR_VELOCIDADE=0, NEGATIVO_SUPERIOR_VELOCIDADE=0;
    double NULO_INFERIOR_VELOCIDADE=0, NULO_SUPERIOR_VELOCIDADE=0;
    double POSITIVO_INFERIOR_VELOCIDADE=0, POSITIVO_SUPERIOR_VELOCIDADE=0;

    // DECLARAÇÃO DOS CONSEQUENTES DE SBRF DO TIPO 2 COM SAÍDA INTERVALAR CALCULADOS VIA EXCEL

		//printf ("DECLARAÇÃO DOS CONSEQUENTES DE SBRF DO TIPO 2 COM SAÍDA INTERVALAR CALCULADOS VIA EXCEL");
    NEGATIVO_INFERIOR_VELOCIDADE = -13.33;
    NEGATIVO_SUPERIOR_VELOCIDADE = -9.58;
    NULO_INFERIOR_VELOCIDADE = 0;
    NULO_SUPERIOR_VELOCIDADE = 0;
    POSITIVO_INFERIOR_VELOCIDADE = 9.58;
    POSITIVO_SUPERIOR_VELOCIDADE = 13.33;

		/*NEGATIVO_INFERIOR_VELOCIDADE = -56.24;
    NEGATIVO_SUPERIOR_VELOCIDADE = -52.46;
    NULO_INFERIOR_VELOCIDADE = 0;
    NULO_SUPERIOR_VELOCIDADE = 0;
    POSITIVO_INFERIOR_VELOCIDADE = 52.46;
    POSITIVO_SUPERIOR_VELOCIDADE = 56.24;*/

    // IMPRESSÃO DOS CONSEQUENTES DE SBRF DO TIPO 2 COM SAÍDA INTERVALAR

    //printf("IMPRESSÃO DOS CONSEQUENTES DE SBRF DO TIPO 2 COM SAÍDA INTERVALAR");
    //printf ("\n NEGATIVO_INFERIOR_VELOCIDADE = %.2f     NEGATIVO_SUPERIOR_VELOCIDADE = %.2f", NEGATIVO_INFERIOR_VELOCIDADE, NEGATIVO_SUPERIOR_VELOCIDADE);
    //printf ("\n NULO_INFERIOR_VELOCIDADE = %.2f         NULO_SUPERIOR_VELOCIDADE = %.2f", NULO_INFERIOR_VELOCIDADE, NULO_SUPERIOR_VELOCIDADE);
    //printf ("\n POSITIVO_INFERIOR_VELOCIDADE = %.2f     POSITIVO_SUPERIOR_VELOCIDADE = %.2f", POSITIVO_INFERIOR_VELOCIDADE, POSITIVO_SUPERIOR_VELOCIDADE);

    //printf ("\n");

    // DECLARAÇÃO DAS VARIÁVEIS PARA CÁLCULO DO VX

    double f1_INF_VX=0, f2_INF_VX=0, f3_INF_VX=0, f4_INF_VX=0, f5_INF_VX=0, f6_INF_VX=0, f7_INF_VX=0, f8_INF_VX=0,f9_INF_VX=0;
    double f1_SUP_VX=0, f2_SUP_VX=0, f3_SUP_VX=0, f4_SUP_VX=0, f5_SUP_VX=0, f6_SUP_VX=0, f7_SUP_VX=0, f8_SUP_VX=0, f9_SUP_VX=0;
    double f1_VX, f2_VX=0, f3_VX=0, f4_VX=0, f5_VX=0, f6_VX=0, f7_VX=0, f8_VX=0, f9_VX=0;
    double y_INF_VX=0, y_INF34_VX=0, y_INF67_VX=0, yL_VX=0, y_SUP_VX=0, y_SUP34_VX=0, y_SUP67_VX=0, yR_VX=0, y_VX=0;
    int condL_VX=0, condR_VX=0;

    // DECLARAÇÃO DA BASE DE REGRAS PARA VX (ORDENADO DE FORMA CRESCENTE)

    //printf("DECLARAÇÃO DA BASE DE REGRAS PARA VX (ORDENADO DE FORMA CRESCENTE)");
    //printf ("\n REGRA 1: SE POSICAO X FOR NEGATIVO LONGE E POSICAO Y FOR POSITIVO LONGE  -> VX = NEGATIVO");
    //printf ("\n REGRA 2: SE POSICAO X FOR NEGATIVO LONGE E POSICAO Y FOR PERTO           -> VX = NEGATIVO");
    //printf ("\n REGRA 3: SE POSICAO X FOR NEGATIVO LONGE E POSICAO Y FOR NEGATIVO LONGE  -> VX = NEGATIVO");
    //printf ("\n REGRA 4: SE POSICAO X FOR PERTO E POSICAO Y FOR POSITIVO LONGE           -> VX = NULO");
    //printf ("\n REGRA 5: SE POSICAO X FOR PERTO E POSICAO Y FOR PERTO                    -> VX = NULO");
    //printf ("\n REGRA 6: SE POSICAO X FOR PERTO E POSICAO Y FOR NEGATIVO LONGE           -> VX = NULO");
    //printf ("\n REGRA 7: SE POSICAO X FOR POSITIVO LONGE E POSICAO Y FOR POSITIVO LONGE  -> VX = POSITIVO");
    //printf ("\n REGRA 8: SE POSICAO X FOR POSITIVO LONGE E POSICAO Y FOR PERTO           -> VX = POSITIVO");
    //printf ("\n REGRA 9: SE POSICAO X FOR POSITIVO LONGE E POSICAO Y FOR NEGATIVO LONGE  -> VX = POSITIVO");

    // CÁLCULO DO F INFERIOR E F SUPERIOR UTLIZANDO A REGRA DO MÍNIMO (SEMELHANTE AO EXEMPLO DO MENDEL)

    /*if (NEGATIVOLONGE_INFERIOR_POSICAOX < POSITIVOLONGE_INFERIOR_POSICAOY) {
        f1_INF_VX = NEGATIVOLONGE_INFERIOR_POSICAOX;
    }
    else {
        f1_INF_VX = POSITIVOLONGE_INFERIOR_POSICAOY;
    }
    if (NEGATIVOLONGE_SUPERIOR_POSICAOX < POSITIVOLONGE_SUPERIOR_POSICAOY) {
        f1_SUP_VX = NEGATIVOLONGE_SUPERIOR_POSICAOX;
    }
    else {
        f1_SUP_VX = POSITIVOLONGE_SUPERIOR_POSICAOY;
    }
    f1_VX = (f1_SUP_VX + f1_INF_VX)/2;

    if (NEGATIVOLONGE_INFERIOR_POSICAOX < PERTO_INFERIOR_POSICAOY) {
        f2_INF_VX = NEGATIVOLONGE_INFERIOR_POSICAOX;
    }
    else {
        f2_INF_VX = PERTO_INFERIOR_POSICAOY;
    }
    if (NEGATIVOLONGE_SUPERIOR_POSICAOX < PERTO_SUPERIOR_POSICAOY) {
        f2_SUP_VX = NEGATIVOLONGE_SUPERIOR_POSICAOX;
    }
    else {
        f2_SUP_VX = PERTO_SUPERIOR_POSICAOY;
    }
    f2_VX = (f2_SUP_VX + f2_INF_VX)/2;

    if (NEGATIVOLONGE_INFERIOR_POSICAOX < NEGATIVOLONGE_INFERIOR_POSICAOY) {
        f3_INF_VX = NEGATIVOLONGE_INFERIOR_POSICAOX;
    }
    else {
        f3_INF_VX = NEGATIVOLONGE_INFERIOR_POSICAOY;
    }
    if (NEGATIVOLONGE_SUPERIOR_POSICAOX < NEGATIVOLONGE_SUPERIOR_POSICAOY) {
        f3_SUP_VX = NEGATIVOLONGE_SUPERIOR_POSICAOX;
    }
    else {
        f3_SUP_VX = NEGATIVOLONGE_SUPERIOR_POSICAOY;
    }
    f3_VX = (f3_SUP_VX + f3_INF_VX)/2;

    if (PERTO_INFERIOR_POSICAOX < POSITIVOLONGE_INFERIOR_POSICAOY) {
        f4_INF_VX = PERTO_INFERIOR_POSICAOX;
    }
    else {
        f4_INF_VX = POSITIVOLONGE_INFERIOR_POSICAOY;
    }
    if (PERTO_SUPERIOR_POSICAOX < POSITIVOLONGE_SUPERIOR_POSICAOY) {
        f4_SUP_VX = PERTO_SUPERIOR_POSICAOX;
    }
    else {
        f4_SUP_VX = POSITIVOLONGE_SUPERIOR_POSICAOY;
    }
    f4_VX = (f4_SUP_VX + f4_INF_VX)/2;

    if (PERTO_INFERIOR_POSICAOX < PERTO_INFERIOR_POSICAOY) {
        f5_INF_VX = PERTO_INFERIOR_POSICAOX;
    }
    else {
        f5_INF_VX = PERTO_INFERIOR_POSICAOY;
    }
    if (PERTO_SUPERIOR_POSICAOX < PERTO_SUPERIOR_POSICAOY) {
        f5_SUP_VX = PERTO_SUPERIOR_POSICAOX;
    }
    else {
        f5_SUP_VX = PERTO_SUPERIOR_POSICAOY;
    }
    f5_VX = (f5_SUP_VX + f5_INF_VX)/2;

    if (PERTO_INFERIOR_POSICAOX < NEGATIVOLONGE_INFERIOR_POSICAOY) {
        f6_INF_VX = PERTO_INFERIOR_POSICAOX;
    }
    else {
        f6_INF_VX = NEGATIVOLONGE_INFERIOR_POSICAOY;
    }
    if (PERTO_SUPERIOR_POSICAOX < NEGATIVOLONGE_SUPERIOR_POSICAOY) {
        f6_SUP_VX = PERTO_SUPERIOR_POSICAOX;
    }
    else {
        f6_SUP_VX = NEGATIVOLONGE_SUPERIOR_POSICAOY;
    }
    f6_VX = (f6_SUP_VX + f6_INF_VX)/2;

    if (POSITIVOLONGE_INFERIOR_POSICAOX < POSITIVOLONGE_INFERIOR_POSICAOY) {
        f7_INF_VX = POSITIVOLONGE_INFERIOR_POSICAOX;
    }
    else {
        f7_INF_VX = POSITIVOLONGE_INFERIOR_POSICAOY;
    }
    if (POSITIVOLONGE_SUPERIOR_POSICAOX < POSITIVOLONGE_SUPERIOR_POSICAOY) {
        f7_SUP_VX = POSITIVOLONGE_SUPERIOR_POSICAOX;
    }
    else {
        f7_SUP_VX = POSITIVOLONGE_SUPERIOR_POSICAOY;
    }
    f7_VX = (f7_SUP_VX + f7_INF_VX)/2;

    if (POSITIVOLONGE_INFERIOR_POSICAOX < PERTO_INFERIOR_POSICAOY) {
        f8_INF_VX = POSITIVOLONGE_INFERIOR_POSICAOX;
    }
    else {
        f8_INF_VX = PERTO_INFERIOR_POSICAOY;
    }
    if (POSITIVOLONGE_SUPERIOR_POSICAOX < PERTO_SUPERIOR_POSICAOY) {
        f8_SUP_VX = POSITIVOLONGE_SUPERIOR_POSICAOX;
    }
    else {
        f8_SUP_VX = PERTO_SUPERIOR_POSICAOY;
    }
    f8_VX = (f8_SUP_VX + f8_INF_VX)/2;

    if (POSITIVOLONGE_INFERIOR_POSICAOX < NEGATIVOLONGE_INFERIOR_POSICAOY) {
        f9_INF_VX = POSITIVOLONGE_INFERIOR_POSICAOX;
    }
    else {
        f9_INF_VX = NEGATIVOLONGE_INFERIOR_POSICAOY;
    }
    if (POSITIVOLONGE_SUPERIOR_POSICAOX < NEGATIVOLONGE_SUPERIOR_POSICAOY) {
        f9_SUP_VX = POSITIVOLONGE_SUPERIOR_POSICAOX;
    }
    else {
        f9_SUP_VX = NEGATIVOLONGE_SUPERIOR_POSICAOY;
    }
    f9_VX = (f9_SUP_VX + f9_INF_VX)/2;*/

    f1_INF_VX = NEGATIVOLONGE_INFERIOR_POSICAOX * POSITIVOLONGE_INFERIOR_POSICAOY;
    f1_SUP_VX = NEGATIVOLONGE_SUPERIOR_POSICAOX * POSITIVOLONGE_SUPERIOR_POSICAOY;
    f1_VX = (f1_SUP_VX + f1_INF_VX)/2;

    f2_INF_VX = NEGATIVOLONGE_INFERIOR_POSICAOX * PERTO_INFERIOR_POSICAOY;
    f2_SUP_VX = NEGATIVOLONGE_SUPERIOR_POSICAOX * PERTO_SUPERIOR_POSICAOY;
    f2_VX = (f2_SUP_VX + f2_INF_VX)/2;

    f3_INF_VX = NEGATIVOLONGE_INFERIOR_POSICAOX * NEGATIVOLONGE_INFERIOR_POSICAOY;
    f3_SUP_VX = NEGATIVOLONGE_SUPERIOR_POSICAOX * NEGATIVOLONGE_SUPERIOR_POSICAOY;
    f3_VX = (f3_SUP_VX + f3_INF_VX)/2;

    f4_INF_VX = PERTO_INFERIOR_POSICAOX * POSITIVOLONGE_INFERIOR_POSICAOY;
    f4_SUP_VX = PERTO_SUPERIOR_POSICAOX * POSITIVOLONGE_SUPERIOR_POSICAOY;
    f4_VX = (f4_SUP_VX + f4_INF_VX)/2;

    f5_INF_VX = PERTO_INFERIOR_POSICAOX * PERTO_INFERIOR_POSICAOY;
    f5_SUP_VX = PERTO_SUPERIOR_POSICAOX * PERTO_SUPERIOR_POSICAOY;
    f5_VX = (f5_SUP_VX + f5_INF_VX)/2;

    f6_INF_VX = PERTO_INFERIOR_POSICAOX * NEGATIVOLONGE_INFERIOR_POSICAOY;
    f6_SUP_VX = PERTO_SUPERIOR_POSICAOX * NEGATIVOLONGE_SUPERIOR_POSICAOY;
    f6_VX = (f6_SUP_VX + f6_INF_VX)/2;

    f7_INF_VX = POSITIVOLONGE_INFERIOR_POSICAOX * POSITIVOLONGE_INFERIOR_POSICAOY;
    f7_SUP_VX = POSITIVOLONGE_SUPERIOR_POSICAOX * POSITIVOLONGE_SUPERIOR_POSICAOY;
    f7_VX = (f7_SUP_VX + f7_INF_VX)/2;

    f8_INF_VX = POSITIVOLONGE_INFERIOR_POSICAOX * PERTO_INFERIOR_POSICAOY;
    f8_SUP_VX = POSITIVOLONGE_SUPERIOR_POSICAOX * PERTO_SUPERIOR_POSICAOY;
    f8_VX = (f8_SUP_VX + f8_INF_VX)/2;

    f9_INF_VX = POSITIVOLONGE_INFERIOR_POSICAOX * NEGATIVOLONGE_INFERIOR_POSICAOY;
    f9_SUP_VX = POSITIVOLONGE_SUPERIOR_POSICAOX * NEGATIVOLONGE_SUPERIOR_POSICAOY;
    f9_VX = (f9_SUP_VX + f9_INF_VX)/2;

    //printf ("\n f1_INF_VX (min) = %.2f       f1_SUP_VX (min) = %.2f       f1_VX = %.2f", f1_INF_VX, f1_SUP_VX, f1_VX);
    //printf ("\n f2_INF_VX (min) = %.2f       f2_SUP_VX (min) = %.2f       f2_VX = %.2f", f2_INF_VX, f2_SUP_VX, f2_VX);
    //printf ("\n f3_INF_VX (min) = %.2f       f3_SUP_VX (min) = %.2f       f3_VX = %.2f", f3_INF_VX, f3_SUP_VX, f3_VX);
    //printf ("\n f4_INF_VX (min) = %.2f       f4_SUP_VX (min) = %.2f       f4_VX = %.2f", f4_INF_VX, f4_SUP_VX, f4_VX);
    //printf ("\n f5_INF_VX (min) = %.2f       f5_SUP_VX (min) = %.2f       f5_VX = %.2f", f5_INF_VX, f5_SUP_VX, f5_VX);
    //printf ("\n f6_INF_VX (min) = %.2f       f6_SUP_VX (min) = %.2f       f6_VX = %.2f", f6_INF_VX, f6_SUP_VX, f6_VX);
    //printf ("\n f7_INF_VX (min) = %.2f       f7_SUP_VX (min) = %.2f       f7_VX = %.2f", f7_INF_VX, f7_SUP_VX, f7_VX);
    //printf ("\n f8_INF_VX (min) = %.2f       f8_SUP_VX (min) = %.2f       f8_VX = %.2f", f8_INF_VX, f8_SUP_VX, f8_VX);
    //printf ("\n f9_INF_VX (min) = %.2f       f9_SUP_VX (min) = %.2f       f9_VX = %.2f", f9_INF_VX, f9_SUP_VX, f9_VX);

    //printf("calculo do yL Vx");

    // CÁLCULO DO yL DO VX

	if (f1_SUP_VX + f2_SUP_VX + f3_SUP_VX + f4_INF_VX + f5_INF_VX + f6_INF_VX + f7_INF_VX + f8_INF_VX + f9_INF_VX == 0) {
        y_INF34_VX = 0;
    	}
	else {y_INF34_VX = (f1_SUP_VX * NEGATIVO_INFERIOR_VELOCIDADE + f2_SUP_VX * NEGATIVO_INFERIOR_VELOCIDADE + f3_SUP_VX * NEGATIVO_INFERIOR_VELOCIDADE + f4_INF_VX * NULO_INFERIOR_VELOCIDADE + f5_INF_VX * NULO_INFERIOR_VELOCIDADE + f6_INF_VX * NULO_INFERIOR_VELOCIDADE + f7_INF_VX * POSITIVO_INFERIOR_VELOCIDADE + f8_INF_VX * POSITIVO_INFERIOR_VELOCIDADE + f9_INF_VX * POSITIVO_INFERIOR_VELOCIDADE)/(f1_SUP_VX + f2_SUP_VX + f3_SUP_VX + f4_INF_VX + f5_INF_VX + f6_INF_VX + f7_INF_VX + f8_INF_VX + f9_INF_VX);
	}
    
    //printf ("\n y_INF34_VX = %.2f (ENTRE %.2f e %.2f)", y_INF34_VX, NEGATIVO_INFERIOR_VELOCIDADE, NULO_INFERIOR_VELOCIDADE);

if (f1_SUP_VX + f2_SUP_VX + f3_SUP_VX + f4_SUP_VX + f5_SUP_VX + f6_SUP_VX + f7_INF_VX + f8_INF_VX + f9_INF_VX == 0) {
        y_INF67_VX = 0;
    }
    else {y_INF67_VX = (f1_SUP_VX * NEGATIVO_INFERIOR_VELOCIDADE + f2_SUP_VX * NEGATIVO_INFERIOR_VELOCIDADE + f3_SUP_VX * NEGATIVO_INFERIOR_VELOCIDADE + f4_SUP_VX * NULO_INFERIOR_VELOCIDADE + f5_SUP_VX * NULO_INFERIOR_VELOCIDADE + f6_SUP_VX * NULO_INFERIOR_VELOCIDADE + f7_INF_VX * POSITIVO_INFERIOR_VELOCIDADE + f8_INF_VX * POSITIVO_INFERIOR_VELOCIDADE + f9_INF_VX * POSITIVO_INFERIOR_VELOCIDADE)/(f1_SUP_VX + f2_SUP_VX + f3_SUP_VX + f4_SUP_VX + f5_SUP_VX + f6_SUP_VX + f7_INF_VX + f8_INF_VX + f9_INF_VX);}
    
    //printf ("\n y_INF67_VX = %.2f (ENTRE %.2f e %.2f)", y_INF67_VX, NULO_INFERIOR_VELOCIDADE, POSITIVO_INFERIOR_VELOCIDADE);

if (f1_VX + f2_VX + f3_VX + f4_VX + f5_VX + f6_VX + f7_VX + f8_VX + f9_VX == 0) {
        y_INF_VX = 0;
    }
else {
    y_INF_VX = (f1_VX * NEGATIVO_INFERIOR_VELOCIDADE + f2_VX * NEGATIVO_INFERIOR_VELOCIDADE + f3_VX * NEGATIVO_INFERIOR_VELOCIDADE + f4_VX * NULO_INFERIOR_VELOCIDADE + f5_VX * NULO_INFERIOR_VELOCIDADE + f6_VX * NULO_INFERIOR_VELOCIDADE + f7_VX * POSITIVO_INFERIOR_VELOCIDADE + f8_VX * POSITIVO_INFERIOR_VELOCIDADE + f9_VX * POSITIVO_INFERIOR_VELOCIDADE)/(f1_VX + f2_VX + f3_VX + f4_VX + f5_VX + f6_VX + f7_VX + f8_VX + f9_VX);
    }
    //printf ("\n y_INF_VX = %.2f", y_INF_VX);

    //printf("\n\n ITERACAO DO yL DO-WHILE");

    do {

        condL_VX = 0;

        if (y_INF_VX >= NEGATIVO_INFERIOR_VELOCIDADE-0.1 && y_INF_VX <= NULO_INFERIOR_VELOCIDADE) {
            y_INF_VX = y_INF34_VX;
            //printf ("\n y_INF34_VX = %.2f", y_INF_VX);
        }

        if (y_INF_VX > NULO_INFERIOR_VELOCIDADE && y_INF_VX <= POSITIVO_INFERIOR_VELOCIDADE+0.1) {
            y_INF_VX = y_INF67_VX;
            //printf ("\n y_INF67_VX = %.2f", y_INF_VX);
        }

        if (y_INF_VX == y_INF34_VX || y_INF_VX == y_INF67_VX) {
            condL_VX = 1;
            yL_VX = y_INF_VX;
            //printf ("\n yL_VX = %.2f", yL_VX);
        }

    } while (condL_VX == 0);
    //printf ("SAIU DO DO-WHILE DO YL\n");
    

    // CÁLCULO DO yR DO VX

		//printf ("calculo do yR do VX\n");


		if (f1_VX + f2_VX + f3_VX + f4_VX + f5_VX + f6_VX + f7_VX + f8_VX + f9_VX == 0) {
				    y_SUP_VX = 0;
				}
		else {
				y_SUP_VX = (f1_VX * NEGATIVO_SUPERIOR_VELOCIDADE + f2_VX * NEGATIVO_SUPERIOR_VELOCIDADE + f3_VX * NEGATIVO_SUPERIOR_VELOCIDADE + f4_VX * NULO_SUPERIOR_VELOCIDADE + f5_VX * NULO_SUPERIOR_VELOCIDADE + f6_VX * NULO_SUPERIOR_VELOCIDADE + f7_VX * POSITIVO_SUPERIOR_VELOCIDADE + f8_VX * POSITIVO_SUPERIOR_VELOCIDADE + f9_VX * POSITIVO_SUPERIOR_VELOCIDADE)/(f1_VX + f2_VX + f3_VX + f4_VX + f5_VX + f6_VX + f7_VX + f8_VX + f9_VX);
        }		

printf ("\n y_SUP_VX = %.2f", y_SUP_VX);

		if (f1_INF_VX + f2_INF_VX + f3_INF_VX + f4_SUP_VX + f5_SUP_VX + f6_SUP_VX + f7_SUP_VX + f8_SUP_VX + f9_SUP_VX == 0) {
				    y_SUP34_VX = 0; 

				}
		else {
				y_SUP34_VX = (f1_INF_VX * NEGATIVO_SUPERIOR_VELOCIDADE + f2_INF_VX * NEGATIVO_SUPERIOR_VELOCIDADE + f3_INF_VX * NEGATIVO_SUPERIOR_VELOCIDADE + f4_SUP_VX * NULO_SUPERIOR_VELOCIDADE + f5_SUP_VX * NULO_SUPERIOR_VELOCIDADE + f6_SUP_VX * NULO_SUPERIOR_VELOCIDADE + f7_SUP_VX * POSITIVO_SUPERIOR_VELOCIDADE + f8_SUP_VX * POSITIVO_SUPERIOR_VELOCIDADE + f9_SUP_VX * POSITIVO_SUPERIOR_VELOCIDADE)/(f1_INF_VX + f2_INF_VX + f3_INF_VX + f4_SUP_VX + f5_SUP_VX + f6_SUP_VX + f7_SUP_VX + f8_SUP_VX + f9_SUP_VX);
			
}
			
printf ("\n y_SUP34_VX = %.2f (ENTRE %.2f e %.2f)", y_SUP34_VX, NEGATIVO_SUPERIOR_VELOCIDADE, NULO_SUPERIOR_VELOCIDADE);
			if (f1_INF_VX + f2_INF_VX + f3_INF_VX + f4_INF_VX + f5_INF_VX + f6_INF_VX + f7_SUP_VX + f8_SUP_VX + f9_SUP_VX == 0) {
				    y_SUP67_VX = 0;
				}
		else {
				y_SUP67_VX = (f1_INF_VX * NEGATIVO_SUPERIOR_VELOCIDADE + f2_INF_VX * NEGATIVO_SUPERIOR_VELOCIDADE + f3_INF_VX * NEGATIVO_SUPERIOR_VELOCIDADE + f4_INF_VX * NULO_SUPERIOR_VELOCIDADE + f5_INF_VX * NULO_SUPERIOR_VELOCIDADE + f6_INF_VX * NULO_SUPERIOR_VELOCIDADE + f7_SUP_VX * POSITIVO_SUPERIOR_VELOCIDADE + f8_SUP_VX * POSITIVO_SUPERIOR_VELOCIDADE + f9_SUP_VX * POSITIVO_SUPERIOR_VELOCIDADE)/(f1_INF_VX + f2_INF_VX + f3_INF_VX + f4_INF_VX + f5_INF_VX + f6_INF_VX + f7_SUP_VX + f8_SUP_VX + f9_SUP_VX);
					
}
		
printf ("\n y_SUP67_VX = %.2f (ENTRE %.2f e %.2f)\n", y_SUP67_VX, NULO_SUPERIOR_VELOCIDADE, POSITIVO_SUPERIOR_VELOCIDADE);

				//cout << "ITERACAO DO yR DO-WHILE" << endl;
      do {

        condR_VX = 0;
				
				//cout << " "<< y_SUP_VX << " dentro do DO" << endl;

        if (y_SUP_VX >= NEGATIVO_SUPERIOR_VELOCIDADE-0.1/*-9.58*/ && y_SUP_VX <= NULO_SUPERIOR_VELOCIDADE/*0*/) {
            y_SUP_VX = y_SUP34_VX;
            //printf ("\n y_SUP34_VX = %.2f", y_SUP_VX);
						cout << " "<< y_SUP_VX << " dentro do DO 34" << endl;
        }

        if (y_SUP_VX > NULO_SUPERIOR_VELOCIDADE/*0*/ && y_SUP_VX <= POSITIVO_SUPERIOR_VELOCIDADE+0.1/*13.33*/) {
            y_SUP_VX = y_SUP67_VX;
						cout << " "<< y_SUP_VX << " dentro do DO 67" << endl;
            //printf ("\n y_SUP67_VX = %.2f", y_SUP_VX);
						
        }

        if (y_SUP_VX == y_SUP34_VX || y_SUP_VX == y_SUP67_VX) {
            condR_VX = 1;
            yR_VX = y_SUP_VX;
						cout << " "<< yR_VX << " dentro do DO FINAL" << endl;
            //printf ("\n yR_VX = %.2f", yR_VX);
        }

    } while (condR_VX == 0);

printf ("\n\n SAIU DO DO-WHILE YR");
    //printf ("Vx");

    // CÁLCULO DO y_VX
    y_VX = (yL_VX + yR_VX)/2;
    //printf ("\n VALOR DEFUZZIFICADO DA VELOCIDADE NA DIRECAO X: %.2f \n", y_VX);
    return (y_VX);
}

double FuzzyTipo2VY (double X, double Y)
{
    // DECLARAÇÃO DAS VARIÁVEIS DE INPUT

    double NEGATIVOLONGE_INFERIOR_POSICAOX=0, NEGATIVOLONGE_SUPERIOR_POSICAOX=0;
    double PERTO_INFERIOR_POSICAOX=0, PERTO_SUPERIOR_POSICAOX=0;
    double POSITIVOLONGE_INFERIOR_POSICAOX=0, POSITIVOLONGE_SUPERIOR_POSICAOX=0;
    double NEGATIVOLONGE_INFERIOR_POSICAOY=0, NEGATIVOLONGE_SUPERIOR_POSICAOY=0;
    double PERTO_INFERIOR_POSICAOY=0, PERTO_SUPERIOR_POSICAOY=0;
    double POSITIVOLONGE_INFERIOR_POSICAOY=0, POSITIVOLONGE_SUPERIOR_POSICAOY=0;

    // DEFINIÇÃO DAS FUNÇÕES DE PERTINÊNCIA SUPERIOR E INFERIOR DAS VARIÁVEIS DE INPUT

   if (X <= -15) {
        NEGATIVOLONGE_INFERIOR_POSICAOX = 0.5;
    }
    else {
        if (X <= -10) {
            NEGATIVOLONGE_INFERIOR_POSICAOX =  -0.1 * X - 1;
        }
        else {
            NEGATIVOLONGE_INFERIOR_POSICAOX = 0;
        }
    }

    if (X <= -10) {
        NEGATIVOLONGE_SUPERIOR_POSICAOX = 1;
    }
    else {
        if (X <= 0) {
            NEGATIVOLONGE_SUPERIOR_POSICAOX =  -0.1 * X;
        }
        else {
            NEGATIVOLONGE_SUPERIOR_POSICAOX = 0;
        }
    }

    if (X <= -5) {
        PERTO_INFERIOR_POSICAOX = 0;
    }
    else {
        if (X <= 0) {
            PERTO_INFERIOR_POSICAOX =  0.1 * X + 0.5;
        }
        else {
                if (X <= 5) {
                    PERTO_INFERIOR_POSICAOX = -0.1*X+0.5;
                }
                else {
		
                    	PERTO_INFERIOR_POSICAOX = 0;
			                }
                }
        }
    

    if (X <= -15) {
        PERTO_SUPERIOR_POSICAOX = 0;
    }
    else {
        if (X <= -5) {
            PERTO_SUPERIOR_POSICAOX =  0.1 * X + 1.5;
        }
        else {
                if (X <= 5) {
                    PERTO_SUPERIOR_POSICAOX = 1;
                }
                else {
                        if (X <= 15) {
                            PERTO_SUPERIOR_POSICAOX = -0.1 * X + 1.5;
                        }
                        else {
                            PERTO_SUPERIOR_POSICAOX = 0;
                        }
                }
        }
    }

    if (X <= 10) {
        POSITIVOLONGE_INFERIOR_POSICAOX = 0;
    }
    else {
        if (X <= 15) {
            POSITIVOLONGE_INFERIOR_POSICAOX =  0.1 * X - 1;
        }
        else {
            POSITIVOLONGE_INFERIOR_POSICAOX = 0.5;
        }
    }

    if (X <= 0) {
        POSITIVOLONGE_SUPERIOR_POSICAOX = 0;
    }
    else {
        if (X <= 10) {
            POSITIVOLONGE_SUPERIOR_POSICAOX =  0.1 * X;
        }
        else {
            POSITIVOLONGE_SUPERIOR_POSICAOX = 1;
        }
    }

    if (Y <= -15) {
        NEGATIVOLONGE_INFERIOR_POSICAOY = 0.5;
    }
    else {
        if (Y <= -10) {
            NEGATIVOLONGE_INFERIOR_POSICAOY =  -0.1 * Y - 1;
        }
        else {
            NEGATIVOLONGE_INFERIOR_POSICAOY = 0;
        }
    }

    if (Y <= -10) {
        NEGATIVOLONGE_SUPERIOR_POSICAOY = 1;
    }
    else {
        if (Y <= 0) {
            NEGATIVOLONGE_SUPERIOR_POSICAOY =  -0.1 * Y;
        }
        else {
            NEGATIVOLONGE_SUPERIOR_POSICAOY = 0;
        }
    }

      if (Y <= -5) {
        PERTO_INFERIOR_POSICAOY = 0;
    }
    else {
        if (Y <= 0) {
            PERTO_INFERIOR_POSICAOY =  0.1 * Y + 0.5;
        }
        else {
                if (Y <= 5) {
                    PERTO_INFERIOR_POSICAOY = -0.1 * Y + 0.5;
                }
                else {
		
                    		PERTO_INFERIOR_POSICAOY = 0;
			                }
                }
        }
    

    if (Y <= -15) {
        PERTO_SUPERIOR_POSICAOY = 0;
    }
    else {
        if (Y <= -5) {
            PERTO_SUPERIOR_POSICAOY =  0.1 * Y + 1.5;
        }
        else {
                if (Y <= 5) {
                    PERTO_SUPERIOR_POSICAOY = 1;
                }
                else {
                        if (Y <= 15) {
                            PERTO_SUPERIOR_POSICAOY = -0.1 * Y + 1.5;
                        }
                        else {
                            PERTO_SUPERIOR_POSICAOY = 0;
                        }
                }
        }
    }

    if (Y <= 10) {
        POSITIVOLONGE_INFERIOR_POSICAOY = 0;
    }
    else {
        if (Y <= 15) {
            POSITIVOLONGE_INFERIOR_POSICAOY =  0.1 * Y - 1;
        }
        else {
            POSITIVOLONGE_INFERIOR_POSICAOY = 0.5;
        }
    }

    if (Y <= 0) {
        POSITIVOLONGE_SUPERIOR_POSICAOY = 0;
    }
    else {
        if (Y <= 10) {
            POSITIVOLONGE_SUPERIOR_POSICAOY =  0.1 * Y;
        }
        else {
            POSITIVOLONGE_SUPERIOR_POSICAOY = 1;
        }
    }
    // IMPRESSÃO DAS VARIÁVEIS DE INPUT PARA OS VALORES DE X E Y CAPTADOS PELA CÂMERA DO DRONE

    //printf ("\n");
    //printf ("\n NEGATIVO LONGE INFERIOR NA POSICAO X = %.2f         NEGATIVO LONGE SUPERIOR NA POSICAO X = %.2f", NEGATIVOLONGE_INFERIOR_POSICAOX, NEGATIVOLONGE_SUPERIOR_POSICAOX);
    //printf ("\n PERTO INFERIOR NA POSICAO X = %.2f                  PERTO SUPERIOR NA POSICAO X = %.2f", PERTO_INFERIOR_POSICAOX, PERTO_SUPERIOR_POSICAOX);
    //printf ("\n POSITIVO LONGE INFERIOR NA POSICAO X = %.2f         POSITIVO LONGE SUPERIOR NA POSICAO X = %.2f", POSITIVOLONGE_INFERIOR_POSICAOX, POSITIVOLONGE_SUPERIOR_POSICAOX);
    //printf ("\n NEGATIVO LONGE INFERIOR NA POSICAO Y = %.2f         NEGATIVO LONGE SUPERIOR NA POSICAO Y = %.2f", NEGATIVOLONGE_INFERIOR_POSICAOY, NEGATIVOLONGE_SUPERIOR_POSICAOY);
    //printf ("\n PERTO INFERIOR NA POSICAO Y = %.2f                  PERTO SUPERIOR NA POSICAO Y = %.2f", PERTO_INFERIOR_POSICAOY, PERTO_SUPERIOR_POSICAOY);
    //printf ("\n POSITIVO LONGE INFERIOR NA POSICAO Y = %.2f         POSITIVO LONGE SUPERIOR NA POSICAO Y = %.2f", POSITIVOLONGE_INFERIOR_POSICAOY, POSITIVOLONGE_SUPERIOR_POSICAOY);

    // DECLARAÇÃO DAS VARIÁVEIS DE OUTPUT

    float NEGATIVO_INFERIOR_VELOCIDADE=0, NEGATIVO_SUPERIOR_VELOCIDADE=0;
    float NULO_INFERIOR_VELOCIDADE=0, NULO_SUPERIOR_VELOCIDADE=0;
    float POSITIVO_INFERIOR_VELOCIDADE=0, POSITIVO_SUPERIOR_VELOCIDADE=0;

    // DECLARAÇÃO DOS CONSEQUENTES DE SBRF DO TIPO 2 COM SAÍDA INTERVALAR CALCULADOS VIA EXCEL

    NEGATIVO_INFERIOR_VELOCIDADE = -13.33;
    NEGATIVO_SUPERIOR_VELOCIDADE = -9.58;
    NULO_INFERIOR_VELOCIDADE = 0;
    NULO_SUPERIOR_VELOCIDADE = 0;
    POSITIVO_INFERIOR_VELOCIDADE = 9.58;
    POSITIVO_SUPERIOR_VELOCIDADE = 13.33; 

		/*NEGATIVO_INFERIOR_VELOCIDADE = -56.24;
    NEGATIVO_SUPERIOR_VELOCIDADE = -52.46;
    NULO_INFERIOR_VELOCIDADE = 0;
    NULO_SUPERIOR_VELOCIDADE = 0;
    POSITIVO_INFERIOR_VELOCIDADE = 52.46;
    POSITIVO_SUPERIOR_VELOCIDADE = 56.24;*/

    // IMPRESSÃO DOS CONSEQUENTES DE SBRF DO TIPO 2 COM SAÍDA INTERVALAR

    //printf("\n");
    //printf ("\n NEGATIVO_INFERIOR_VELOCIDADE = %.2f     NEGATIVO_SUPERIOR_VELOCIDADE = %.2f", NEGATIVO_INFERIOR_VELOCIDADE, NEGATIVO_SUPERIOR_VELOCIDADE);
    //printf ("\n NULO_INFERIOR_VELOCIDADE = %.2f         NULO_SUPERIOR_VELOCIDADE = %.2f", NULO_INFERIOR_VELOCIDADE, NULO_SUPERIOR_VELOCIDADE);
    //printf ("\n POSITIVO_INFERIOR_VELOCIDADE = %.2f     POSITIVO_SUPERIOR_VELOCIDADE = %.2f", POSITIVO_INFERIOR_VELOCIDADE, POSITIVO_SUPERIOR_VELOCIDADE);

    //printf ("\n");

    // DECLARAÇÃO DAS VARIÁVEIS PARA CÁLCULO DO VY

    double f1_INF_VY=0, f2_INF_VY=0, f3_INF_VY=0, f4_INF_VY=0, f5_INF_VY=0, f6_INF_VY=0, f7_INF_VY=0, f8_INF_VY=0, f9_INF_VY=0;
    double f1_SUP_VY=0, f2_SUP_VY=0, f3_SUP_VY=0, f4_SUP_VY=0, f5_SUP_VY=0, f6_SUP_VY=0, f7_SUP_VY=0, f8_SUP_VY=0, f9_SUP_VY=0;
    double f1_VY=0, f2_VY=0, f3_VY=0, f4_VY=0, f5_VY=0, f6_VY=0, f7_VY=0, f8_VY=0, f9_VY=0;
    double y_INF_VY=0, y_INF34_VY=0, y_INF67_VY=0, yL_VY=0, y_SUP_VY=0, y_SUP34_VY=0, y_SUP67_VY=0, yR_VY=0, y_VY=0;
    int condL_VY=0, condR_VY=0;

    // DECLARAÇÃO DA BASE DE REGRAS PARA VY (ORDENADO DE FORMA CRESCENTE)

    //printf("\n");
    //printf ("\n REGRA 1: SE POSICAO X FOR POSITIVO LONGE E POSICAO Y FOR NEGATIVO LONGE  -> VY = NEGATIVO");
    //printf ("\n REGRA 2: SE POSICAO X FOR PERTO E POSICAO Y FOR NEGATIVO LONGE           -> VY = NEGATIVO");
    //printf ("\n REGRA 3: SE POSICAO X FOR NEGATIVO LONGE E POSICAO Y FOR NEGATIVO LONGE  -> VY = NEGATIVO");
    //printf ("\n REGRA 4: SE POSICAO X FOR POSITIVO LONGE E POSICAO Y FOR PERTO           -> VY = NULO");
    //printf ("\n REGRA 5: SE POSICAO X FOR PERTO E POSICAO Y FOR PERTO                    -> VY = NULO");
    //printf ("\n REGRA 6: SE POSICAO X FOR NEGATIVO LONGE E POSICAO Y FOR PERTO           -> VY = NULO");
    //printf ("\n REGRA 7: SE POSICAO X FOR POSITIVO LONGE E POSICAO Y FOR POSITIVO LONGE  -> VY = POSITIVO");
    //printf ("\n REGRA 8: SE POSICAO X FOR PERTO E POSICAO Y FOR POSITIVO LONGE           -> VY = POSITIVO");
    //printf ("\n REGRA 9: SE POSICAO X FOR NEGATIVO LONGE E POSICAO Y FOR POSITIVO LONGE  -> VY = POSITIVO");

    // CÁLCULO DO F INFERIOR E F SUPERIOR UTLIZANDO A REGRA DO MÍNIMO (SEMELHANTE AO EXEMPLO DO MENDEL)

    /*if (POSITIVOLONGE_INFERIOR_POSICAOX < NEGATIVOLONGE_INFERIOR_POSICAOY) {
        f1_INF_VY = POSITIVOLONGE_INFERIOR_POSICAOX;
    }
    else {
        f1_INF_VY = NEGATIVOLONGE_INFERIOR_POSICAOY;
    }
    if (POSITIVOLONGE_SUPERIOR_POSICAOX < NEGATIVOLONGE_SUPERIOR_POSICAOY) {
        f1_SUP_VY = POSITIVOLONGE_SUPERIOR_POSICAOX;
    }
    else {
        f1_SUP_VY = NEGATIVOLONGE_SUPERIOR_POSICAOY;
    }
    f1_VY = (f1_SUP_VY + f1_INF_VY)/2;

    if (PERTO_INFERIOR_POSICAOX < NEGATIVOLONGE_INFERIOR_POSICAOY) {
        f2_INF_VY = PERTO_INFERIOR_POSICAOX;
    }
    else {
        f2_INF_VY = NEGATIVOLONGE_INFERIOR_POSICAOY;
    }
    if (PERTO_SUPERIOR_POSICAOX < NEGATIVOLONGE_SUPERIOR_POSICAOY) {
        f2_SUP_VY = PERTO_SUPERIOR_POSICAOX;
    }
    else {
        f2_SUP_VY = NEGATIVOLONGE_SUPERIOR_POSICAOY;
    }
    f2_VY = (f2_SUP_VY + f2_INF_VY)/2;

    if (NEGATIVOLONGE_INFERIOR_POSICAOX < NEGATIVOLONGE_INFERIOR_POSICAOY) {
        f3_INF_VY = NEGATIVOLONGE_INFERIOR_POSICAOX;
    }
    else {
        f3_INF_VY = NEGATIVOLONGE_INFERIOR_POSICAOY;
    }
    if (NEGATIVOLONGE_SUPERIOR_POSICAOX < NEGATIVOLONGE_SUPERIOR_POSICAOY) {
        f3_SUP_VY = NEGATIVOLONGE_SUPERIOR_POSICAOX;
    }
    else {
        f3_SUP_VY = NEGATIVOLONGE_SUPERIOR_POSICAOY;
    }
    f3_VY = (f3_SUP_VY + f3_INF_VY)/2;

    if (POSITIVOLONGE_INFERIOR_POSICAOX < PERTO_INFERIOR_POSICAOY) {
        f4_INF_VY = POSITIVOLONGE_INFERIOR_POSICAOX;
    }
    else {
        f4_INF_VY = PERTO_INFERIOR_POSICAOY;
    }
    if (POSITIVOLONGE_SUPERIOR_POSICAOX < PERTO_SUPERIOR_POSICAOY) {
        f4_SUP_VY = POSITIVOLONGE_SUPERIOR_POSICAOX;
    }
    else {
        f4_SUP_VY = PERTO_SUPERIOR_POSICAOY;
    }
    f4_VY = (f4_SUP_VY + f4_INF_VY)/2;

    if (PERTO_INFERIOR_POSICAOX < PERTO_INFERIOR_POSICAOY) {
        f5_INF_VY = PERTO_INFERIOR_POSICAOX;
    }
    else {
        f5_INF_VY = PERTO_INFERIOR_POSICAOY;
    }
    if (PERTO_SUPERIOR_POSICAOX < PERTO_SUPERIOR_POSICAOY) {
        f5_SUP_VY = PERTO_SUPERIOR_POSICAOX;
    }
    else {
        f5_SUP_VY = PERTO_SUPERIOR_POSICAOY;
    }
    f5_VY = (f5_SUP_VY + f5_INF_VY)/2;

    if (NEGATIVOLONGE_INFERIOR_POSICAOX < PERTO_INFERIOR_POSICAOY) {
        f6_INF_VY = NEGATIVOLONGE_INFERIOR_POSICAOX;
    }
    else {
        f6_INF_VY = PERTO_INFERIOR_POSICAOY;
    }
    if (NEGATIVOLONGE_SUPERIOR_POSICAOX < PERTO_SUPERIOR_POSICAOY) {
        f6_SUP_VY = NEGATIVOLONGE_SUPERIOR_POSICAOX;
    }
    else {
        f6_SUP_VY = PERTO_SUPERIOR_POSICAOY;
    }
    f6_VY = (f6_SUP_VY + f6_INF_VY)/2;

    if (POSITIVOLONGE_INFERIOR_POSICAOX < POSITIVOLONGE_INFERIOR_POSICAOY) {
        f7_INF_VY = POSITIVOLONGE_INFERIOR_POSICAOX;
    }
    else {
        f7_INF_VY = POSITIVOLONGE_INFERIOR_POSICAOY;
    }
    if (POSITIVOLONGE_SUPERIOR_POSICAOX < POSITIVOLONGE_SUPERIOR_POSICAOY) {
        f7_SUP_VY = POSITIVOLONGE_SUPERIOR_POSICAOX;
    }
    else {
        f7_SUP_VY = POSITIVOLONGE_SUPERIOR_POSICAOY;
    }
    f7_VY = (f7_SUP_VY + f7_INF_VY)/2;

    if (PERTO_INFERIOR_POSICAOX < POSITIVOLONGE_INFERIOR_POSICAOY) {
        f8_INF_VY = PERTO_INFERIOR_POSICAOX;
    }
    else {
        f8_INF_VY = POSITIVOLONGE_INFERIOR_POSICAOY;
    }
    if (PERTO_SUPERIOR_POSICAOX < POSITIVOLONGE_SUPERIOR_POSICAOY) {
        f8_SUP_VY = PERTO_SUPERIOR_POSICAOX;
    }
    else {
        f8_SUP_VY = POSITIVOLONGE_SUPERIOR_POSICAOY;
    }
    f8_VY = (f8_SUP_VY + f8_INF_VY)/2;

    if (NEGATIVOLONGE_INFERIOR_POSICAOX < POSITIVOLONGE_INFERIOR_POSICAOY) {
        f9_INF_VY = NEGATIVOLONGE_INFERIOR_POSICAOX;
    }
    else {
        f9_INF_VY = POSITIVOLONGE_INFERIOR_POSICAOY;
    }
    if (NEGATIVOLONGE_SUPERIOR_POSICAOX < POSITIVOLONGE_SUPERIOR_POSICAOY) {
        f9_SUP_VY = NEGATIVOLONGE_SUPERIOR_POSICAOX;
    }
    else {
        f9_SUP_VY = POSITIVOLONGE_SUPERIOR_POSICAOY;
    }
    f9_VY = (f9_SUP_VY + f9_INF_VY)/2;*/

		f1_INF_VY = POSITIVOLONGE_INFERIOR_POSICAOX * NEGATIVOLONGE_INFERIOR_POSICAOY;
    f1_SUP_VY = POSITIVOLONGE_SUPERIOR_POSICAOX * NEGATIVOLONGE_SUPERIOR_POSICAOY;
    f1_VY = (f1_SUP_VY + f1_INF_VY)/2;

    f2_INF_VY = PERTO_INFERIOR_POSICAOX * NEGATIVOLONGE_INFERIOR_POSICAOY;
    f2_SUP_VY = PERTO_SUPERIOR_POSICAOX * NEGATIVOLONGE_SUPERIOR_POSICAOY;
    f2_VY = (f2_SUP_VY + f2_INF_VY)/2;

    f3_INF_VY = NEGATIVOLONGE_INFERIOR_POSICAOX * NEGATIVOLONGE_INFERIOR_POSICAOY;
    f3_SUP_VY = NEGATIVOLONGE_SUPERIOR_POSICAOX * NEGATIVOLONGE_SUPERIOR_POSICAOY;
    f3_VY = (f3_SUP_VY + f3_INF_VY)/2;

    f4_INF_VY = POSITIVOLONGE_INFERIOR_POSICAOX * PERTO_INFERIOR_POSICAOY;
    f4_SUP_VY = POSITIVOLONGE_SUPERIOR_POSICAOX * PERTO_SUPERIOR_POSICAOY;
    f4_VY = (f4_SUP_VY + f4_INF_VY)/2;

    f5_INF_VY = PERTO_INFERIOR_POSICAOX * PERTO_INFERIOR_POSICAOY;
    f5_SUP_VY = PERTO_SUPERIOR_POSICAOX * PERTO_SUPERIOR_POSICAOY;
    f5_VY = (f5_SUP_VY + f5_INF_VY)/2;

    f6_INF_VY = NEGATIVOLONGE_INFERIOR_POSICAOX * PERTO_INFERIOR_POSICAOY;
    f6_SUP_VY = NEGATIVOLONGE_SUPERIOR_POSICAOX * PERTO_SUPERIOR_POSICAOY;
    f6_VY = (f6_SUP_VY + f6_INF_VY)/2;

    f7_INF_VY = POSITIVOLONGE_INFERIOR_POSICAOX * POSITIVOLONGE_INFERIOR_POSICAOY;
    f7_SUP_VY = POSITIVOLONGE_SUPERIOR_POSICAOX * POSITIVOLONGE_SUPERIOR_POSICAOY;
    f7_VY = (f7_SUP_VY + f7_INF_VY)/2;

    f8_INF_VY = PERTO_INFERIOR_POSICAOX * POSITIVOLONGE_INFERIOR_POSICAOY;
    f8_SUP_VY = PERTO_SUPERIOR_POSICAOX * POSITIVOLONGE_SUPERIOR_POSICAOY;
    f8_VY = (f8_SUP_VY + f8_INF_VY)/2;

    f9_INF_VY = NEGATIVOLONGE_INFERIOR_POSICAOX * POSITIVOLONGE_INFERIOR_POSICAOY;
    f9_SUP_VY = NEGATIVOLONGE_SUPERIOR_POSICAOX * POSITIVOLONGE_SUPERIOR_POSICAOY;
    f9_VY = (f9_SUP_VY + f9_INF_VY)/2;

    //printf ("\n");
    //printf ("\n f1_INF_VY (min) = %.2f       f1_SUP_VY (min) = %.2f       f1_VY = %.2f", f1_INF_VY, f1_SUP_VY, f1_VY);
    //printf ("\n f2_INF_VY (min) = %.2f       f2_SUP_VY (min) = %.2f       f2_VY = %.2f", f2_INF_VY, f2_SUP_VY, f2_VY);
    //printf ("\n f3_INF_VY (min) = %.2f       f3_SUP_VY (min) = %.2f       f3_VY = %.2f", f3_INF_VY, f3_SUP_VY, f3_VY);
    //printf ("\n f4_INF_VY (min) = %.2f       f4_SUP_VY (min) = %.2f       f4_VY = %.2f", f4_INF_VY, f4_SUP_VY, f4_VY);
    //printf ("\n f5_INF_VY (min) = %.2f       f5_SUP_VY (min) = %.2f       f5_VY = %.2f", f5_INF_VY, f5_SUP_VY, f5_VY);
    //printf ("\n f6_INF_VY (min) = %.2f       f6_SUP_VY (min) = %.2f       f6_VY = %.2f", f6_INF_VY, f6_SUP_VY, f6_VY);
    //printf ("\n f7_INF_VY (min) = %.2f       f7_SUP_VY (min) = %.2f       f7_VY = %.2f", f7_INF_VY, f7_SUP_VY, f7_VY);
    //printf ("\n f8_INF_VY (min) = %.2f       f8_SUP_VY (min) = %.2f       f8_VY = %.2f", f8_INF_VY, f8_SUP_VY, f8_VY);
    //printf ("\n f9_INF_VY (min) = %.2f       f9_SUP_VY (min) = %.2f       f9_VY = %.2f", f9_INF_VY, f9_SUP_VY, f9_VY);

    //printf ("\n");

    // CÁLCULO DO yL DO VY
if (f1_SUP_VY + f2_SUP_VY + f3_SUP_VY + f4_INF_VY + f5_INF_VY + f6_INF_VY + f7_INF_VY + f8_INF_VY + f9_INF_VY == 0) {
        y_INF34_VY = 0;
    }
else{
    y_INF34_VY = (f1_SUP_VY * NEGATIVO_INFERIOR_VELOCIDADE + f2_SUP_VY * NEGATIVO_INFERIOR_VELOCIDADE + f3_SUP_VY * NEGATIVO_INFERIOR_VELOCIDADE + f4_INF_VY * NULO_INFERIOR_VELOCIDADE + f5_INF_VY * NULO_INFERIOR_VELOCIDADE + f6_INF_VY * NULO_INFERIOR_VELOCIDADE + f7_INF_VY * POSITIVO_INFERIOR_VELOCIDADE + f8_INF_VY * POSITIVO_INFERIOR_VELOCIDADE + f9_INF_VY * POSITIVO_INFERIOR_VELOCIDADE)/(f1_SUP_VY + f2_SUP_VY + f3_SUP_VY + f4_INF_VY + f5_INF_VY + f6_INF_VY + f7_INF_VY + f8_INF_VY + f9_INF_VY);}
    
    //printf ("\n y_INF34_VY = %.2f (ENTRE %.2f e %.2f)", y_INF34_VY, NEGATIVO_INFERIOR_VELOCIDADE, NULO_INFERIOR_VELOCIDADE);

if (f1_SUP_VY + f2_SUP_VY + f3_SUP_VY + f4_SUP_VY + f5_SUP_VY + f6_SUP_VY + f7_INF_VY + f8_INF_VY + f9_INF_VY == 0) {
        y_INF67_VY = 0;
    }
else {
    y_INF67_VY = (f1_SUP_VY * NEGATIVO_INFERIOR_VELOCIDADE + f2_SUP_VY * NEGATIVO_INFERIOR_VELOCIDADE + f3_SUP_VY * NEGATIVO_INFERIOR_VELOCIDADE + f4_SUP_VY * NULO_INFERIOR_VELOCIDADE + f5_SUP_VY * NULO_INFERIOR_VELOCIDADE + f6_SUP_VY * NULO_INFERIOR_VELOCIDADE + f7_INF_VY * POSITIVO_INFERIOR_VELOCIDADE + f8_INF_VY * POSITIVO_INFERIOR_VELOCIDADE + f9_INF_VY * POSITIVO_INFERIOR_VELOCIDADE)/(f1_SUP_VY + f2_SUP_VY + f3_SUP_VY + f4_SUP_VY + f5_SUP_VY + f6_SUP_VY + f7_INF_VY + f8_INF_VY + f9_INF_VY);}
    
    //printf ("\n y_INF67_VY = %.2f (ENTRE %.2f e %.2f)", y_INF67_VY, NULO_INFERIOR_VELOCIDADE, POSITIVO_INFERIOR_VELOCIDADE);
 if (f1_VY + f2_VY + f3_VY + f4_VY + f5_VY + f6_VY + f7_VY + f8_VY + f9_VY == 0) {
        y_INF_VY = 0;
    }
else {

    y_INF_VY = (f1_VY * NEGATIVO_INFERIOR_VELOCIDADE + f2_VY * NEGATIVO_INFERIOR_VELOCIDADE + f3_VY * NEGATIVO_INFERIOR_VELOCIDADE + f4_VY * NULO_INFERIOR_VELOCIDADE + f5_VY * NULO_INFERIOR_VELOCIDADE + f6_VY * NULO_INFERIOR_VELOCIDADE + f7_VY * POSITIVO_INFERIOR_VELOCIDADE + f8_VY * POSITIVO_INFERIOR_VELOCIDADE + f9_VY * POSITIVO_INFERIOR_VELOCIDADE)/(f1_VY + f2_VY + f3_VY + f4_VY + f5_VY + f6_VY + f7_VY + f8_VY + f9_VY);
   }
    //printf ("\n y_INF_VY = %.2f", y_INF_VY);

    //printf("\n\n ITERACAO DO yL");

    do {

        condL_VY = 0;

        if (y_INF_VY >= NEGATIVO_INFERIOR_VELOCIDADE-0.1 && y_INF_VY <= NULO_INFERIOR_VELOCIDADE) {
            y_INF_VY = y_INF34_VY;
            //printf ("\n y_INF34_VY = %.2f", y_INF_VY);
        }

        if (y_INF_VY >= NULO_INFERIOR_VELOCIDADE && y_INF_VY <= POSITIVO_INFERIOR_VELOCIDADE+0.1) {
            y_INF_VY = y_INF67_VY;
            //printf ("\n y_INF67_VY = %.2f", y_INF_VY);
        }

        if (y_INF_VY == y_INF34_VY|| y_INF_VY == y_INF67_VY) {
            condL_VY = 1;
            yL_VY = y_INF_VY;
            //printf ("\n yL_VY = %.2f", yL_VY);
        }

    } while (condL_VY == 0);

    //printf ("\n");

    // CÁLCULO DO yR DO VY

 if (f1_INF_VY + f2_INF_VY + f3_INF_VY + f4_SUP_VY + f5_SUP_VY + f6_SUP_VY + f7_SUP_VY + f8_SUP_VY + f9_SUP_VY == 0) {
        y_SUP34_VY = 0;
    }
else {
    y_SUP34_VY = (f1_INF_VY * NEGATIVO_SUPERIOR_VELOCIDADE + f2_INF_VY * NEGATIVO_SUPERIOR_VELOCIDADE + f3_INF_VY * NEGATIVO_SUPERIOR_VELOCIDADE + f4_SUP_VY * NULO_SUPERIOR_VELOCIDADE + f5_SUP_VY * NULO_SUPERIOR_VELOCIDADE + f6_SUP_VY * NULO_SUPERIOR_VELOCIDADE + f7_SUP_VY * POSITIVO_SUPERIOR_VELOCIDADE + f8_SUP_VY * POSITIVO_SUPERIOR_VELOCIDADE + f9_SUP_VY * POSITIVO_SUPERIOR_VELOCIDADE)/(f1_INF_VY + f2_INF_VY + f3_INF_VY + f4_SUP_VY + f5_SUP_VY + f6_SUP_VY + f7_SUP_VY + f8_SUP_VY + f9_SUP_VY);
}
   
    //printf ("\n y_SUP34_VY = %.2f (ENTRE %.2f e %.2f)", y_SUP34_VY, NEGATIVO_SUPERIOR_VELOCIDADE, NULO_SUPERIOR_VELOCIDADE);
 if (f1_INF_VY + f2_INF_VY + f3_INF_VY + f4_INF_VY + f5_INF_VY + f6_INF_VY + f7_SUP_VY + f8_SUP_VY + f9_SUP_VY == 0) {
        y_SUP67_VY = 0;
    }
else {
    y_SUP67_VY = (f1_INF_VY * NEGATIVO_SUPERIOR_VELOCIDADE + f2_INF_VY * NEGATIVO_SUPERIOR_VELOCIDADE + f3_INF_VY * NEGATIVO_SUPERIOR_VELOCIDADE + f4_INF_VY * NULO_SUPERIOR_VELOCIDADE + f5_INF_VY * NULO_SUPERIOR_VELOCIDADE + f6_INF_VY * NULO_SUPERIOR_VELOCIDADE + f7_SUP_VY * POSITIVO_SUPERIOR_VELOCIDADE + f8_SUP_VY * POSITIVO_SUPERIOR_VELOCIDADE + f9_SUP_VY * POSITIVO_SUPERIOR_VELOCIDADE)/(f1_INF_VY + f2_INF_VY + f3_INF_VY + f4_INF_VY + f5_INF_VY + f6_INF_VY + f7_SUP_VY + f8_SUP_VY + f9_SUP_VY);}
   
    //printf ("\n y_SUP67_VY = %.2f (ENTRE %.2f e %.2f)", y_SUP67_VY, NEGATIVO_SUPERIOR_VELOCIDADE, NULO_SUPERIOR_VELOCIDADE);
if (f1_VY + f2_VY + f3_VY + f4_VY + f5_VY + f6_VY + f7_VY + f8_VY + f9_VY == 0) {
        y_SUP_VY = 0;
    }
else {
    y_SUP_VY = (f1_VY * NEGATIVO_SUPERIOR_VELOCIDADE + f2_VY * NEGATIVO_SUPERIOR_VELOCIDADE + f3_VY * NEGATIVO_SUPERIOR_VELOCIDADE + f4_VY * NULO_SUPERIOR_VELOCIDADE + f5_VY * NULO_SUPERIOR_VELOCIDADE + f6_VY * NULO_SUPERIOR_VELOCIDADE + f7_VY * POSITIVO_SUPERIOR_VELOCIDADE + f8_VY * POSITIVO_SUPERIOR_VELOCIDADE + f9_VY * POSITIVO_SUPERIOR_VELOCIDADE)/(f1_VY + f2_VY + f3_VY + f4_VY + f5_VY + f6_VY + f7_VY + f8_VY + f9_VY);}
    
    //printf ("\n y_SUP_VY = %.2f", y_SUP_VY);

    //printf ("\n\n ITERACAO DO yR");

    do {

        condR_VY = 0;

        if (y_SUP_VY >= NEGATIVO_SUPERIOR_VELOCIDADE-0.1 && y_SUP_VY <= NULO_SUPERIOR_VELOCIDADE) {
            y_SUP_VY = y_SUP34_VY;
            //printf ("\n y_SUP34_VY = %.2f", y_SUP_VY);
        }

        if (y_SUP_VY > NULO_SUPERIOR_VELOCIDADE && y_SUP_VY <= POSITIVO_SUPERIOR_VELOCIDADE+0.1) {
            y_SUP_VY = y_SUP67_VY;
            //printf ("\n y_SUP67_VY = %.2f", y_SUP_VY);
        }

        if (y_SUP_VY == y_SUP34_VY || y_SUP_VY == y_SUP67_VY) {
            condR_VY = 1;
            yR_VY = y_SUP_VY;
            //printf ("\n yR_VY = %.2f", yR_VY);
        }

    } while (condR_VY == 0);

    //printf ("\n");

    // CÁLCULO DO y_VY
    y_VY = (yL_VY + yR_VY)/2;
    //printf ("\n VALOR DEFUZZIFICADO DA VELOCIDADE NA DIRECAO Y: %.2f", y_VY);
    return (y_VY);
}

    double FuzzyTipo2VZ (double X, double Y)
{
    // DECLARAÇÃO DAS VARIÁVEIS DE INPUT

    double NEGATIVOLONGE_INFERIOR_POSICAOX=0, NEGATIVOLONGE_SUPERIOR_POSICAOX=0;
    double PERTO_INFERIOR_POSICAOX=0, PERTO_SUPERIOR_POSICAOX=0;
    double POSITIVOLONGE_INFERIOR_POSICAOX=0, POSITIVOLONGE_SUPERIOR_POSICAOX=0;
    double NEGATIVOLONGE_INFERIOR_POSICAOY=0, NEGATIVOLONGE_SUPERIOR_POSICAOY=0;
    double PERTO_INFERIOR_POSICAOY=0, PERTO_SUPERIOR_POSICAOY=0;
    double POSITIVOLONGE_INFERIOR_POSICAOY=0, POSITIVOLONGE_SUPERIOR_POSICAOY=0;

    // DEFINIÇÃO DAS FUNÇÕES DE PERTINÊNCIA SUPERIOR E INFERIOR DAS VARIÁVEIS DE INPUT
if (X <= -15) {
        NEGATIVOLONGE_INFERIOR_POSICAOX = 0.5;
    }
    else {
        if (X <= -10) {
            NEGATIVOLONGE_INFERIOR_POSICAOX =  -0.1 * X - 1;
        }
        else {
            NEGATIVOLONGE_INFERIOR_POSICAOX = 0;
        }
    }

    if (X <= -10) {
        NEGATIVOLONGE_SUPERIOR_POSICAOX = 1;
    }
    else {
        if (X <= 0) {
            NEGATIVOLONGE_SUPERIOR_POSICAOX =  -0.1 * X;
        }
        else {
            NEGATIVOLONGE_SUPERIOR_POSICAOX = 0;
        }
    }

    if (X <= -5) {
        PERTO_INFERIOR_POSICAOX = 0;
    }
    else {
        if (X <= 0) {
            PERTO_INFERIOR_POSICAOX =  0.1 * X + 0.5;
        }
        else {
                if (X <= 5) {
                    PERTO_INFERIOR_POSICAOX = -0.1*X+0.5;
                }
                else {
		
                    	PERTO_INFERIOR_POSICAOX = 0;
			                }
                }
        }
    

    if (X <= -15) {
        PERTO_SUPERIOR_POSICAOX = 0;
    }
    else {
        if (X <= -5) {
            PERTO_SUPERIOR_POSICAOX =  0.1 * X + 1.5;
        }
        else {
                if (X <= 5) {
                    PERTO_SUPERIOR_POSICAOX = 1;
                }
                else {
                        if (X <= 15) {
                            PERTO_SUPERIOR_POSICAOX = -0.1 * X + 1.5;
                        }
                        else {
                            PERTO_SUPERIOR_POSICAOX = 0;
                        }
                }
        }
    }

    if (X <= 10) {
        POSITIVOLONGE_INFERIOR_POSICAOX = 0;
    }
    else {
        if (X <= 15) {
            POSITIVOLONGE_INFERIOR_POSICAOX =  0.1 * X - 1;
        }
        else {
            POSITIVOLONGE_INFERIOR_POSICAOX = 0.5;
        }
    }

    if (X <= 0) {
        POSITIVOLONGE_SUPERIOR_POSICAOX = 0;
    }
    else {
        if (X <= 10) {
            POSITIVOLONGE_SUPERIOR_POSICAOX =  0.1 * X;
        }
        else {
            POSITIVOLONGE_SUPERIOR_POSICAOX = 1;
        }
    }

    if (Y <= -15) {
        NEGATIVOLONGE_INFERIOR_POSICAOY = 0.5;
    }
    else {
        if (Y <= -10) {
            NEGATIVOLONGE_INFERIOR_POSICAOY =  -0.1 * Y - 1;
        }
        else {
            NEGATIVOLONGE_INFERIOR_POSICAOY = 0;
        }
    }

    if (Y <= -10) {
        NEGATIVOLONGE_SUPERIOR_POSICAOY = 1;
    }
    else {
        if (Y <= 0) {
            NEGATIVOLONGE_SUPERIOR_POSICAOY =  -0.1 * Y;
        }
        else {
            NEGATIVOLONGE_SUPERIOR_POSICAOY = 0;
        }
    }

      if (Y <= -5) {
        PERTO_INFERIOR_POSICAOY = 0;
    }
    else {
        if (Y <= 0) {
            PERTO_INFERIOR_POSICAOY =  0.1 * Y + 0.5;
        }
        else {
                if (Y <= 5) {
                    PERTO_INFERIOR_POSICAOY = -0.1 * Y + 0.5;
                }
                else {
		
                    		PERTO_INFERIOR_POSICAOY = 0;
			                }
                }
        }
    

    if (Y <= -15) {
        PERTO_SUPERIOR_POSICAOY = 0;
    }
    else {
        if (Y <= -5) {
            PERTO_SUPERIOR_POSICAOY =  0.1 * Y + 1.5;
        }
        else {
                if (Y <= 5) {
                    PERTO_SUPERIOR_POSICAOY = 1;
                }
                else {
                        if (Y <= 15) {
                            PERTO_SUPERIOR_POSICAOY = -0.1 * Y + 1.5;
                        }
                        else {
                            PERTO_SUPERIOR_POSICAOY = 0;
                        }
                }
        }
    }

    if (Y <= 10) {
        POSITIVOLONGE_INFERIOR_POSICAOY = 0;
    }
    else {
        if (Y <= 15) {
            POSITIVOLONGE_INFERIOR_POSICAOY =  0.1 * Y - 1;
        }
        else {
            POSITIVOLONGE_INFERIOR_POSICAOY = 0.5;
        }
    }

    if (Y <= 0) {
        POSITIVOLONGE_SUPERIOR_POSICAOY = 0;
    }
    else {
        if (Y <= 10) {
            POSITIVOLONGE_SUPERIOR_POSICAOY =  0.1 * Y;
        }
        else {
            POSITIVOLONGE_SUPERIOR_POSICAOY = 1;
        }
    }
    // IMPRESSÃO DAS VARIÁVEIS DE INPUT PARA OS VALORES DE X E Y CAPTADOS PELA CÂMERA DO DRONE

    //printf ("\n");
    //printf ("\n NEGATIVO LONGE INFERIOR NA POSICAO X = %.2f         NEGATIVO LONGE SUPERIOR NA POSICAO X = %.2f", NEGATIVOLONGE_INFERIOR_POSICAOX, NEGATIVOLONGE_SUPERIOR_POSICAOX);
    //printf ("\n PERTO INFERIOR NA POSICAO X = %.2f                  PERTO SUPERIOR NA POSICAO X = %.2f", PERTO_INFERIOR_POSICAOX, PERTO_SUPERIOR_POSICAOX);
    //printf ("\n POSITIVO LONGE INFERIOR NA POSICAO X = %.2f         POSITIVO LONGE SUPERIOR NA POSICAO X = %.2f", POSITIVOLONGE_INFERIOR_POSICAOX, POSITIVOLONGE_SUPERIOR_POSICAOX);
    //printf ("\n NEGATIVO LONGE INFERIOR NA POSICAO Y = %.2f         NEGATIVO LONGE SUPERIOR NA POSICAO Y = %.2f", NEGATIVOLONGE_INFERIOR_POSICAOY, NEGATIVOLONGE_SUPERIOR_POSICAOY);
    //printf ("\n PERTO INFERIOR NA POSICAO Y = %.2f                  PERTO SUPERIOR NA POSICAO Y = %.2f", PERTO_INFERIOR_POSICAOY, PERTO_SUPERIOR_POSICAOY);
    //printf ("\n POSITIVO LONGE INFERIOR NA POSICAO Y = %.2f         POSITIVO LONGE SUPERIOR NA POSICAO Y = %.2f", POSITIVOLONGE_INFERIOR_POSICAOY, POSITIVOLONGE_SUPERIOR_POSICAOY);

    // DECLARAÇÃO DAS VARIÁVEIS DE OUTPUT

    double NEGATIVO_INFERIOR_VELOCIDADE=0, NEGATIVO_SUPERIOR_VELOCIDADE=0;
    double NULO_INFERIOR_VELOCIDADE=0, NULO_SUPERIOR_VELOCIDADE=0;
    double POSITIVO_INFERIOR_VELOCIDADE=0, POSITIVO_SUPERIOR_VELOCIDADE=0;

    // DECLARAÇÃO DOS CONSEQUENTES DE SBRF DO TIPO 2 COM SAÍDA INTERVALAR CALCULADOS VIA EXCEL

    /*NEGATIVO_INFERIOR_VELOCIDADE = -13.33;
    NEGATIVO_SUPERIOR_VELOCIDADE = -9.58;
    NULO_INFERIOR_VELOCIDADE = 0;
    NULO_SUPERIOR_VELOCIDADE = 0;
    POSITIVO_INFERIOR_VELOCIDADE = 9.58;
    POSITIVO_SUPERIOR_VELOCIDADE = 13.33;*/

		NEGATIVO_INFERIOR_VELOCIDADE = -56.24;
    NEGATIVO_SUPERIOR_VELOCIDADE = -52.46;
    NULO_INFERIOR_VELOCIDADE = 0;
    NULO_SUPERIOR_VELOCIDADE = 0;
    POSITIVO_INFERIOR_VELOCIDADE = 52.46;
    POSITIVO_SUPERIOR_VELOCIDADE = 56.24;

    // IMPRESSÃO DOS CONSEQUENTES DE SBRF DO TIPO 2 COM SAÍDA INTERVALAR

    //printf("\n");
    //printf ("\n NEGATIVO_INFERIOR_VELOCIDADE = %.2f     NEGATIVO_SUPERIOR_VELOCIDADE = %.2f", NEGATIVO_INFERIOR_VELOCIDADE, NEGATIVO_SUPERIOR_VELOCIDADE);
    //printf ("\n NULO_INFERIOR_VELOCIDADE = %.2f         NULO_SUPERIOR_VELOCIDADE = %.2f", NULO_INFERIOR_VELOCIDADE, NULO_SUPERIOR_VELOCIDADE);
    //printf ("\n POSITIVO_INFERIOR_VELOCIDADE = %.2f     POSITIVO_SUPERIOR_VELOCIDADE = %.2f", POSITIVO_INFERIOR_VELOCIDADE, POSITIVO_SUPERIOR_VELOCIDADE);

    //printf ("\n");

    // DECLARAÇÃO DAS VARIÁVEIS PARA CÁLCULO DO VZ

    double f1_INF_VZ=0, f2_INF_VZ=0, f3_INF_VZ=0, f4_INF_VZ=0, f5_INF_VZ=0, f6_INF_VZ=0, f7_INF_VZ=0, f8_INF_VZ=0, f9_INF_VZ=0;
    double f1_SUP_VZ=0, f2_SUP_VZ=0, f3_SUP_VZ=0, f4_SUP_VZ=0, f5_SUP_VZ=0, f6_SUP_VZ=0, f7_SUP_VZ=0, f8_SUP_VZ=0, f9_SUP_VZ=0;
    double f1_VZ=0, f2_VZ=0, f3_VZ=0, f4_VZ=0, f5_VZ=0, f6_VZ=0, f7_VZ=0, f8_VZ=0, f9_VZ=0;
    double y_INF12_VZ=0, yL_VZ=0, y_SUP12_VZ=0, yR_VZ=0, y_VZ=0;

    // DECLARAÇÃO DA BASE DE REGRAS PARA VZ (ORDENADO DE FORMA CRESCENTE)

    //printf("\n");
    //printf ("\n REGRA 1: SE POSICAO X FOR PERTO E POSICAO Y FOR PERTO                    -> VZ = NEGATIVO");
    //printf ("\n REGRA 2: SE POSICAO X FOR PERTO E POSICAO Y FOR NEGATIVO LONGE           -> VZ = NULO");
    //printf ("\n REGRA 3: SE POSICAO X FOR PERTO E POSICAO Y FOR POSITIVO LONGE           -> VZ = NULO");
    //printf ("\n REGRA 4: SE POSICAO X FOR POSITIVO LONGE E POSICAO Y FOR NEGATIVO LONGE  -> VZ = NULO");
    //printf ("\n REGRA 5: SE POSICAO X FOR POSITIVO LONGE E POSICAO Y FOR PERTO           -> VZ = NULO");
    //printf ("\n REGRA 6: SE POSICAO X FOR POSITIVO LONGE E POSICAO Y FOR POSITIVO LONGE  -> VZ = NULO");
    //printf ("\n REGRA 7: SE POSICAO X FOR NEGATIVO LONGE E POSICAO Y FOR NEGATIVO LONGE  -> VZ = NULO");
    //printf ("\n REGRA 8: SE POSICAO X FOR NEGATIVO LONGE E POSICAO Y FOR PERTO           -> VZ = NULO");
    //printf ("\n REGRA 9: SE POSICAO X FOR NEGATIVO LONGE E POSICAO Y FOR POSITIVO LONGE  -> VZ = NULO");

    /*if (PERTO_INFERIOR_POSICAOX < PERTO_INFERIOR_POSICAOY) {
        f1_INF_VZ = PERTO_INFERIOR_POSICAOX;
    }
    else {
        f1_INF_VZ = PERTO_INFERIOR_POSICAOY;
    }
    if (PERTO_SUPERIOR_POSICAOX < PERTO_SUPERIOR_POSICAOY) {
        f1_SUP_VZ = PERTO_SUPERIOR_POSICAOX;
    }
    else {
        f1_SUP_VZ = PERTO_SUPERIOR_POSICAOY;
    }
    f1_VZ = (f1_SUP_VZ + f1_INF_VZ)/2;

    if (PERTO_INFERIOR_POSICAOX < NEGATIVOLONGE_INFERIOR_POSICAOY) {
        f2_INF_VZ = PERTO_INFERIOR_POSICAOX;
    }
    else {
        f2_INF_VZ = NEGATIVOLONGE_INFERIOR_POSICAOY;
    }
    if (PERTO_SUPERIOR_POSICAOX < NEGATIVOLONGE_SUPERIOR_POSICAOY) {
        f2_SUP_VZ = PERTO_SUPERIOR_POSICAOX;
    }
    else {
        f2_SUP_VZ = NEGATIVOLONGE_SUPERIOR_POSICAOY;
    }
    f2_VZ = (f2_SUP_VZ + f2_INF_VZ)/2;

    if (PERTO_INFERIOR_POSICAOX < POSITIVOLONGE_INFERIOR_POSICAOY) {
        f3_INF_VZ = PERTO_INFERIOR_POSICAOX;
    }
    else {
        f3_INF_VZ = POSITIVOLONGE_INFERIOR_POSICAOY;
    }
    if (PERTO_SUPERIOR_POSICAOX < POSITIVOLONGE_SUPERIOR_POSICAOY) {
        f3_SUP_VZ = PERTO_SUPERIOR_POSICAOX;
    }
    else {
        f3_SUP_VZ = POSITIVOLONGE_SUPERIOR_POSICAOY;
    }
    f3_VZ = (f3_SUP_VZ + f3_INF_VZ)/2;

    if (POSITIVOLONGE_INFERIOR_POSICAOX < NEGATIVOLONGE_INFERIOR_POSICAOY) {
        f4_INF_VZ = POSITIVOLONGE_INFERIOR_POSICAOX;
    }
    else {
        f4_INF_VZ = NEGATIVOLONGE_INFERIOR_POSICAOY;
    }
    if (POSITIVOLONGE_SUPERIOR_POSICAOX < NEGATIVOLONGE_SUPERIOR_POSICAOY) {
        f4_SUP_VZ = POSITIVOLONGE_SUPERIOR_POSICAOX;
    }
    else {
        f4_SUP_VZ = NEGATIVOLONGE_SUPERIOR_POSICAOY;
    }
    f4_VZ = (f4_SUP_VZ + f4_INF_VZ)/2;

    if (POSITIVOLONGE_INFERIOR_POSICAOX < PERTO_INFERIOR_POSICAOY) {
        f5_INF_VZ = POSITIVOLONGE_INFERIOR_POSICAOX;
    }
    else {
        f5_INF_VZ = PERTO_INFERIOR_POSICAOY;
    }
    if (POSITIVOLONGE_SUPERIOR_POSICAOX < PERTO_SUPERIOR_POSICAOY) {
        f5_SUP_VZ = POSITIVOLONGE_SUPERIOR_POSICAOX;
    }
    else {
        f5_SUP_VZ = PERTO_SUPERIOR_POSICAOY;
    }
    f5_VZ = (f5_SUP_VZ + f5_INF_VZ)/2;

    if (POSITIVOLONGE_INFERIOR_POSICAOX < POSITIVOLONGE_INFERIOR_POSICAOY) {
        f6_INF_VZ = POSITIVOLONGE_INFERIOR_POSICAOX;
    }
    else {
        f6_INF_VZ = POSITIVOLONGE_INFERIOR_POSICAOY;
    }
    if (POSITIVOLONGE_SUPERIOR_POSICAOX < POSITIVOLONGE_SUPERIOR_POSICAOY) {
        f6_SUP_VZ = POSITIVOLONGE_SUPERIOR_POSICAOX;
    }
    else {
        f6_SUP_VZ = POSITIVOLONGE_SUPERIOR_POSICAOY;
    }
    f6_VZ = (f6_SUP_VZ + f6_INF_VZ)/2;

    if (NEGATIVOLONGE_INFERIOR_POSICAOX < NEGATIVOLONGE_INFERIOR_POSICAOY) {
        f7_INF_VZ = NEGATIVOLONGE_INFERIOR_POSICAOX;
    }
    else {
        f7_INF_VZ = NEGATIVOLONGE_INFERIOR_POSICAOY;
    }
    if (NEGATIVOLONGE_SUPERIOR_POSICAOX < NEGATIVOLONGE_SUPERIOR_POSICAOY) {
        f7_SUP_VZ = NEGATIVOLONGE_SUPERIOR_POSICAOX;
    }
    else {
        f7_SUP_VZ = NEGATIVOLONGE_SUPERIOR_POSICAOY;
    }
    f7_VZ = (f7_SUP_VZ + f7_INF_VZ)/2;

    if (NEGATIVOLONGE_INFERIOR_POSICAOX < PERTO_INFERIOR_POSICAOY) {
        f8_INF_VZ = NEGATIVOLONGE_INFERIOR_POSICAOX;
    }
    else {
        f8_INF_VZ = PERTO_INFERIOR_POSICAOY;
    }
    if (NEGATIVOLONGE_SUPERIOR_POSICAOX < PERTO_SUPERIOR_POSICAOY) {
        f8_SUP_VZ = NEGATIVOLONGE_SUPERIOR_POSICAOX;
    }
    else {
        f8_SUP_VZ = PERTO_SUPERIOR_POSICAOY;
    }
    f8_VZ = (f8_SUP_VZ + f8_INF_VZ)/2;

    if (NEGATIVOLONGE_INFERIOR_POSICAOX < POSITIVOLONGE_INFERIOR_POSICAOY) {
        f9_INF_VZ = NEGATIVOLONGE_INFERIOR_POSICAOX;
    }
    else {
        f9_INF_VZ = POSITIVOLONGE_INFERIOR_POSICAOY;
    }
    if (NEGATIVOLONGE_SUPERIOR_POSICAOX < POSITIVOLONGE_SUPERIOR_POSICAOY) {
        f9_SUP_VZ = NEGATIVOLONGE_SUPERIOR_POSICAOX;
    }
    else {
        f9_SUP_VZ = POSITIVOLONGE_SUPERIOR_POSICAOY;
    }
    f9_VZ = (f9_SUP_VZ + f9_INF_VZ)/2;*/

		f1_INF_VZ = PERTO_INFERIOR_POSICAOX * PERTO_INFERIOR_POSICAOY;
    f1_SUP_VZ = PERTO_SUPERIOR_POSICAOX * PERTO_SUPERIOR_POSICAOY;
    f1_VZ = (f1_SUP_VZ + f1_INF_VZ)/2;

    f2_INF_VZ = PERTO_INFERIOR_POSICAOX * NEGATIVOLONGE_INFERIOR_POSICAOY;
    f2_SUP_VZ = PERTO_SUPERIOR_POSICAOX * NEGATIVOLONGE_SUPERIOR_POSICAOY;
    f2_VZ = (f2_SUP_VZ + f2_INF_VZ)/2;

    f3_INF_VZ = PERTO_INFERIOR_POSICAOX * POSITIVOLONGE_INFERIOR_POSICAOY;
    f3_SUP_VZ = PERTO_SUPERIOR_POSICAOX * POSITIVOLONGE_SUPERIOR_POSICAOY;
    f3_VZ = (f3_SUP_VZ + f3_INF_VZ)/2;

    f4_INF_VZ = POSITIVOLONGE_INFERIOR_POSICAOX * NEGATIVOLONGE_INFERIOR_POSICAOY;
    f4_SUP_VZ = POSITIVOLONGE_SUPERIOR_POSICAOX * NEGATIVOLONGE_SUPERIOR_POSICAOY;
    f4_VZ = (f4_SUP_VZ + f4_INF_VZ)/2;

    f5_INF_VZ = POSITIVOLONGE_INFERIOR_POSICAOX * PERTO_INFERIOR_POSICAOY;
    f5_SUP_VZ = POSITIVOLONGE_SUPERIOR_POSICAOX * PERTO_SUPERIOR_POSICAOY;
    f5_VZ = (f5_SUP_VZ + f5_INF_VZ)/2;

    f6_INF_VZ = POSITIVOLONGE_INFERIOR_POSICAOX * POSITIVOLONGE_INFERIOR_POSICAOY;
    f6_SUP_VZ = POSITIVOLONGE_SUPERIOR_POSICAOX * POSITIVOLONGE_SUPERIOR_POSICAOY;
    f6_VZ = (f6_SUP_VZ + f6_INF_VZ)/2;

    f7_INF_VZ = NEGATIVOLONGE_INFERIOR_POSICAOX * NEGATIVOLONGE_INFERIOR_POSICAOY;
    f7_SUP_VZ = NEGATIVOLONGE_SUPERIOR_POSICAOX * NEGATIVOLONGE_SUPERIOR_POSICAOY;
    f7_VZ = (f7_SUP_VZ + f7_INF_VZ)/2;

    f8_INF_VZ = NEGATIVOLONGE_INFERIOR_POSICAOX * PERTO_INFERIOR_POSICAOY;
    f8_SUP_VZ = NEGATIVOLONGE_SUPERIOR_POSICAOX * PERTO_SUPERIOR_POSICAOY;
    f8_VZ = (f8_SUP_VZ + f8_INF_VZ)/2;

    f9_INF_VZ = NEGATIVOLONGE_INFERIOR_POSICAOX * POSITIVOLONGE_INFERIOR_POSICAOY;
    f9_SUP_VZ = NEGATIVOLONGE_SUPERIOR_POSICAOX * POSITIVOLONGE_SUPERIOR_POSICAOY;
    f9_VZ = (f9_SUP_VZ + f9_INF_VZ)/2;

    //printf ("\n");
    //printf ("\n f1_INF_VZ (min) = %.2f       f1_SUP_VZ (min) = %.2f       f1_VZ = %.2f", f1_INF_VZ, f1_SUP_VZ, f1_VZ);
    //printf ("\n f2_INF_VZ (min) = %.2f       f2_SUP_VZ (min) = %.2f       f2_VZ = %.2f", f2_INF_VZ, f2_SUP_VZ, f2_VZ);
    //printf ("\n f3_INF_VZ (min) = %.2f       f3_SUP_VZ (min) = %.2f       f3_VZ = %.2f", f3_INF_VZ, f3_SUP_VZ, f3_VZ);
    //printf ("\n f4_INF_VZ (min) = %.2f       f4_SUP_VZ (min) = %.2f       f4_VZ = %.2f", f4_INF_VZ, f4_SUP_VZ, f4_VZ);
    //printf ("\n f5_INF_VZ (min) = %.2f       f5_SUP_VZ (min) = %.2f       f5_VZ = %.2f", f5_INF_VZ, f5_SUP_VZ, f5_VZ);
    //printf ("\n f6_INF_VZ (min) = %.2f       f6_SUP_VZ (min) = %.2f       f6_VZ = %.2f", f6_INF_VZ, f6_SUP_VZ, f6_VZ);
    //printf ("\n f7_INF_VZ (min) = %.2f       f7_SUP_VZ (min) = %.2f       f7_VZ = %.2f", f7_INF_VZ, f7_SUP_VZ, f7_VZ);
    //printf ("\n f8_INF_VZ (min) = %.2f       f8_SUP_VZ (min) = %.2f       f8_VZ = %.2f", f8_INF_VZ, f8_SUP_VZ, f8_VZ);
    //printf ("\n f9_INF_VZ (min) = %.2f       f9_SUP_VZ (min) = %.2f       f9_VZ = %.2f", f9_INF_VZ, f9_SUP_VZ, f9_VZ);

    //printf ("\n");

    // CÁLCULO DO yL DO VZ
if (f1_SUP_VZ + f2_INF_VZ + f3_INF_VZ + f4_INF_VZ + f5_INF_VZ + f6_INF_VZ + f7_INF_VZ + f8_INF_VZ + f9_INF_VZ == 0) {
        y_INF12_VZ = 0;
    }
else{

    y_INF12_VZ = (f1_SUP_VZ * NEGATIVO_INFERIOR_VELOCIDADE + f2_INF_VZ * NULO_INFERIOR_VELOCIDADE + f3_INF_VZ * NULO_INFERIOR_VELOCIDADE + f4_INF_VZ * NULO_INFERIOR_VELOCIDADE + f5_INF_VZ * NULO_INFERIOR_VELOCIDADE + f6_INF_VZ * NULO_INFERIOR_VELOCIDADE + f7_INF_VZ * NULO_INFERIOR_VELOCIDADE + f8_INF_VZ * NULO_INFERIOR_VELOCIDADE + f9_INF_VZ * NULO_INFERIOR_VELOCIDADE)/(f1_SUP_VZ + f2_INF_VZ + f3_INF_VZ + f4_INF_VZ + f5_INF_VZ + f6_INF_VZ + f7_INF_VZ + f8_INF_VZ + f9_INF_VZ);}
    
    //printf ("\n y_INF12_VZ = %.2f (ENTRE %.2f e %.2f)", y_INF12_VZ, NEGATIVO_INFERIOR_VELOCIDADE, NULO_INFERIOR_VELOCIDADE);
    yL_VZ = y_INF12_VZ;
    //printf ("\n yL_VZ = %.2f", yL_VZ);

    //printf ("\n");

   // CÁLCULO DO yR DO VZ
  if (f1_INF_VZ + f2_SUP_VZ + f3_SUP_VZ + f4_SUP_VZ + f5_SUP_VZ + f6_SUP_VZ + f7_SUP_VZ + f8_SUP_VZ + f9_SUP_VZ == 0) {
        y_SUP12_VZ = 0;
    }
else {
    y_SUP12_VZ = (f1_INF_VZ * NEGATIVO_SUPERIOR_VELOCIDADE + f2_SUP_VZ * NULO_SUPERIOR_VELOCIDADE + f3_SUP_VZ * NULO_SUPERIOR_VELOCIDADE + f4_SUP_VZ * NULO_SUPERIOR_VELOCIDADE + f5_SUP_VZ * NULO_SUPERIOR_VELOCIDADE + f6_SUP_VZ * NULO_SUPERIOR_VELOCIDADE + f7_SUP_VZ * NULO_SUPERIOR_VELOCIDADE + f8_SUP_VZ * NULO_SUPERIOR_VELOCIDADE + f9_SUP_VZ * NULO_SUPERIOR_VELOCIDADE)/(f1_INF_VZ + f2_SUP_VZ + f3_SUP_VZ + f4_SUP_VZ + f5_SUP_VZ + f6_SUP_VZ + f7_SUP_VZ + f8_SUP_VZ + f9_SUP_VZ);}
  
    //printf ("\n y_SUP12_VZ = %.2f (ENTRE %.2f e %.2f)", y_SUP12_VZ, NEGATIVO_SUPERIOR_VELOCIDADE, NULO_SUPERIOR_VELOCIDADE);
    yR_VZ = y_SUP12_VZ;
    //printf ("\n yR_VZ = %.2f", yR_VZ);

    // CÁLCULO DO y_VZ
    y_VZ = (yL_VZ + yR_VZ)/2;
    //printf ("\n\n VALOR DEFUZZIFICADO DA VELOCIDADE NA DIRECAO Z: %.2f", y_VZ);
    return (y_VZ);
}



////////////////////////////////////////////////////////////// FIM FUNÇÃO FUZZY //////////////////////////////////////////////////////////////////


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
        printf("Error creating directory!n");
        return(-1);
    }

    float Vx, Vy, Vz;
		double radial=0; //declaração da variavel q verifica o erro radial

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

    nome2= nome+"/debugFuzzyParameters.txt";
    debugFuzzyParameters.open(nome2.c_str());
    debugFuzzyParameters << "[ENTRADA e SAIDA do Fuzzy] [X Y Z dVx dVy dVz]" << "\n\n";
    debugFuzzyParameters << "[" << "\n";

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

    while(ros::ok() && (abs(tempZ) > tolZ)){ 					//ORIGINALMENTE N TINHA NDA MULTIPLICANDO tolZ
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

            if(tempX < 0)
                auxX = -1*min(100.0*abs(tempX/MAX_X), 100.0);
            if(tempY < 0)
                auxY = -1*min(100.0*abs(tempY/MAX_Y), 100.0);
            if(tempX > 0)
                auxX = min(100.0*(tempX/MAX_X), 100.0);
            if(tempY > 0)
                auxY = min(100.0*(tempY/MAX_Y), 100.0);

            timeval oldCount,newCount;
            gettimeofday(&oldCount, NULL);						

						FuzzyEntrances << " "<< auxX <<" "<< auxY <<";"<< endl;
            //myFuzzy = fuzzy(auxX, auxY);
            
						

            cout << "funçao VY" << endl;
						double prcentVY = FuzzyTipo2VY (auxX,auxY);
            cout << "funçao VZ" << endl;
						double prcentVZ = FuzzyTipo2VZ (auxX,auxY);
						cout << "funçao VX" << endl;
						double prcentVX = FuzzyTipo2VX (auxX,auxY);
						          
            //FuzzyEntrances << " "<< auxX <<" "<< auxY <<" "<< prcentVX <<" "<< prcentVY << " " << prcentVZ<<";"<< endl;

            gettimeofday(&newCount, NULL);
            double t = double(newCount.tv_sec -oldCount.tv_sec )
                    + double(newCount.tv_usec-oldCount.tv_usec) * 1.e-8;// tava 1e-6
				
            debugTimeToProc << t << endl;

            cout << "Fuzzy In (x,y): " << auxX << " " << auxY << endl;

            vVel.setValue(Vx*gainX*prcentVX,Vy*gainY*prcentVY,0); // Tirando o Z da transformacao

            ros::spinOnce();
            TH_V_I = quadrotor.getHTmatrix(HT_V_I); //obtem mat trans. homogenea do vant em relacao ao inercial

            vVelInertial = tf::quatRotate(TH_V_I.getRotation(), vVel);
            vVelInertial.setZ(Vz*gainZ*prcentVZ+offsetZ);
        }

        cout << "Sended Velocities (Vx,Vy,Vz)(UAV Frame):" << vVel.getX() << " " << vVel.getY()<<" " << vVelInertial.getZ() << endl;

        debugFuzzyParameters << " "<< tempX <<" "<< tempY <<" "<< tempZ <<" "<<vVel.getX() << " " << vVel.getY()<<" " << vVelInertial.getZ() << ";"<< endl;

        

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

    debugFuzzyParameters << "]" << endl;
    debugFuzzyParameters.close();

    debugPosition << "]" << endl;
    debugPosition.close();

    debugPositionArPose << "]" << endl;
    debugPositionArPose.close();

    debugTimeToProc << "]" << endl;
    debugTimeToProc.close();
    
    FuzzyEntrances << "]" << endl;
    FuzzyEntrances << "Tempo de descida: "<< tempo << endl;
    FuzzyEntrances.close();

		TAGposition << "]" << endl;
    TAGposition.close();

    ros::spin();
    return 0;
}
