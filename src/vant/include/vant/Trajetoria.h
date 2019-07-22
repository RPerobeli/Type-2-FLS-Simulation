#ifndef TRAJETORIA_H
#define TRAJETORIA_H

#include <vector>
using std::vector;

class Trajetoria
{
public:
	Trajetoria();
	Trajetoria(int);	// Construtor da classe, argumento de entrada fornece o número de pontos entre as referências para interpolação

    void addReferencia(double,double,double);		// Adiciona novo ponto ao trajeto

    void interpola();						// Acrescenta o número de pontos escolhido entre as referências. Interpolação linear

	void setNumeroPontosEntreRef(int);		// Configura número de pontos entre referências.

    void setNumeroCiclos(int);

	int getNumeroPontosEntreRef() const;	// Retorna número de pontos entre referências iniciais.

    int getNumeroCiclos(); // Retorna número de ciclos de caminho.

	int getNumeroWaypoints() const;			// Retorna número total de waypoints

	void getWaypoint(double *,int);			// Retorna determinado waypoint

    void setCicloAtual(int ciclo);

    int getCicloAtual();

	void proximoWaypoint(double *);			// Retorna waypoint seguindo uma ordem

	int getUltimoWaypoint() const;

    int refazCaminho();

    void set_limitesCaminho(int,int);

    int nTotalWaypoints() const;

private:

    int numeroPontosEntreRef;				// Número de pontos entre as referências usado para interpolação
    int numeroCiclos;
    int cicloAtual;
    float dx,dy;

    vector<double> referenciasX;				// Vetor que armazena a coordenada x dos pontos de referência do trajeto
    vector<double> referenciasY;				// Vetor que armazena a coordenada y dos pontos de referência do trajeto
    vector<double> referenciasZ;				// Vetor que armazena a coordenada z dos pontos de referência do trajeto
    int nReferencias;						// número total de pontos de referência

    double waypointsX[10000];				// Vetor que armazena as coordenadas x dos waypoints da trajetória
    double waypointsY[10000];				// Vetor que armazena as coordenadas y dos waypoints da trajetória
    double waypointsZ[10000];				// Vetor que armazena as coordenadas z dos waypoints da trajetória
	int nWaypoints;							// número total de waypoints

	int ultimoWaypointUsado;				// 

	void linspace(double *,double,double,int);	// Função que retorna valores linearmente espaçados entre dois valores
};

#endif
