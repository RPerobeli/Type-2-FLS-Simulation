#include "Trajetoria.h"
#include <stdio.h>
// Construtor da classe, 'npontos' fornece o número de pontos
// entre cada referência, possibilitando assim a interpolação
Trajetoria::Trajetoria(int npontos)
{
	setNumeroPontosEntreRef(npontos);
	nReferencias = 0;
	ultimoWaypointUsado = 0;
}

Trajetoria::Trajetoria()
{
	setNumeroPontosEntreRef(0);
	nReferencias = 0;
	ultimoWaypointUsado = 0;
}

// Retorna o waypoint de número 'indice' da trajetória
void Trajetoria::getWaypoint(double *waypoint,int indice)
{
    waypoint[0] = referenciasX[indice-1];
    waypoint[1] = referenciasY[indice-1];
    waypoint[2] = referenciasZ[indice-1];
}

// Acrescenta um ponto de referência à trajetoria
void Trajetoria::addReferencia(double x, double y, double z)
{
	referenciasX.push_back(x);
	referenciasY.push_back(y);
    referenciasZ.push_back(z);
	nReferencias++;
}

int Trajetoria::nTotalWaypoints() const
{
    return nReferencias;
}

// Retorna o índice do ultimo waypoint usado como referência
int Trajetoria::getUltimoWaypoint() const
{
	return ultimoWaypointUsado;
}

int Trajetoria::refazCaminho()
{
    ultimoWaypointUsado = 0;
    return ultimoWaypointUsado;
}

// Altera o waypoint para o próximo na lista do trajeto
void Trajetoria::proximoWaypoint(double *waypoint)
{
	getWaypoint(waypoint,ultimoWaypointUsado);
	ultimoWaypointUsado++;
}

// Realiza interpolação linear adicionando 'numeroPontosEntreRef'
// pontos entre cada referência adicionada ao trajeto
void Trajetoria::interpola()
{
    // número total de waypoints após a interpolação é determinado.

    // vetor armazena as coordenadas dos novos pontos gerados
    //double *novosWaypoints = new double[numeroPontosEntreRef];
    double novosWaypoints[numeroPontosEntreRef+nReferencias-1];
    int cont = 0;
    for(int i = 0; i < nReferencias -1; i++)
    {
        // determinação dos novos pontos
        linspace(novosWaypoints,referenciasX[i],referenciasX[i+1],numeroPontosEntreRef);
        for(int j = 0; j <= numeroPontosEntreRef; j++)
        {
            // acrescenta novo ponto ao conjunto de waypoints
                waypointsX[cont] = novosWaypoints[j];
                cont++;
        }
    }
    waypointsX[(nReferencias-1)*numeroPontosEntreRef+(nReferencias-1)] = referenciasX[nReferencias-1];

    cont = 0;
    for(int i = 0; i < nReferencias - 1; i++)
    {
        linspace(novosWaypoints,referenciasY[i],referenciasY[i+1],numeroPontosEntreRef);
        for(int j = 0; j <= numeroPontosEntreRef; j++)
        {
            waypointsY[cont] = novosWaypoints[j];
            cont++;
        }
    }
    waypointsY[(nReferencias-1)*numeroPontosEntreRef+(nReferencias-1)] = referenciasY[nReferencias-1];

    nWaypoints = numeroPontosEntreRef *(nReferencias - 1) + nReferencias;
}


// cria um vetor de 'npontos' pontos espaçados linearmente entre 'menor' e 'maior'
void Trajetoria::linspace(double *valores, double menor, double maior,int npontos)
{
	double intervalo = maior - menor;
	double passo = (double)(intervalo/(npontos+1));

	valores[0] = menor;

	for(int i = 1; i <= npontos; i++)
	{
		valores[i] = menor + i * passo;
	}
}

void Trajetoria::setNumeroPontosEntreRef(int pontos)
{
	numeroPontosEntreRef = pontos;
}

void Trajetoria::setNumeroCiclos(int ciclos)
{
    numeroCiclos = ciclos;
}

void Trajetoria::setCicloAtual(int ciclo)
{
    cicloAtual = ciclo;
}

int Trajetoria::getNumeroCiclos()
{
    return numeroCiclos;
}

int Trajetoria::getCicloAtual()
{
    return cicloAtual;
}

int Trajetoria::getNumeroPontosEntreRef() const
{
	return numeroPontosEntreRef;
}

int Trajetoria::getNumeroWaypoints() const
{
    return nWaypoints;
}

void Trajetoria::set_limitesCaminho(int x, int y){
    dx = x;
    dy = y;

    float disc = 0.1, eixoX[100000],eixoY[100000],aux=0;
    aux = (x/disc)+1;
    eixoX[0] = 0;
    for(int i=1;i<=aux;i++){
        eixoX[i] = eixoX[i-1] + disc;
        printf("\n\nMapa X = %.2f",eixoX[i]);
    }

    aux = (y/disc)+1;
    eixoY[0] = 0;
    for(int i=1;i<=aux;i++){
        eixoY[i] = eixoY[i-1] + disc;
        printf("\n\nMapa Y = %.2f",eixoY[i]);
    }
}
