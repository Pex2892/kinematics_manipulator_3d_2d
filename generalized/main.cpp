//============================================================================
// Name        : SR_project.cpp
// Author      : Giuseppe Giliberto, Giuseppe Puglisi, Giuseppe Sgroi
// Version     : 2.0
// Copyright   : Universita' degli Studi di Catania
// Description : Generalized Kinematics Forward and Inverse
//============================================================================

#define _USE_MATH_DEFINES
#include <iostream>
#include <math.h>
//#include "Kinematics.h"
using namespace std;

//definizione della classe Kinematics
class Kinematics {
	//parte public accessibile a tutti
	public:
		//costruttore
		Kinematics();
		
		//calcola cinematica diretta
		void calcForwardKinematics(int iter, double *returnBracci, double *inputThetaRadiant, double *returnResult);

		//calcola cinematica inversa
		void calcInverseKinematics(int direction, int iter, double *returnBracci, double *inputThetaRadiant, double *returnResult);

		void calcTheta_0(double *returnResult); //calcolo il theta0

		void calcTheta_1(double *returnBracci, double *returnResult); //calcolo il theta1

		void calcTheta_i(int i, int direction, double *returnBracci, double *returnResult); //calcolo il theta i-esimo

		void calcTheta_last(int iter, double *returnResult); //calcolo l'ultimo theta
};

//definizione del costruttore
Kinematics::Kinematics() {}

//definizione funzione per calcolare la cinematica diretta
void Kinematics::calcForwardKinematics(int iter, double *returnBracci, double *inputThetaRadiant, double *returnResult) {
	double calc_x_t, calc_y_t, calc_z_t, calc_alpha;

	for(int i = 0; i < iter; i++) {
		double calcTmpTheta = 0;
             
		for(int j = 0; j <= i; j++)
			calcTmpTheta += inputThetaRadiant[j];
             
		calc_x_t += returnBracci[i] * cos(calcTmpTheta);
		calc_y_t += returnBracci[i] * sin(calcTmpTheta);
		//calc_z_t += 0; // ??
             
		calc_alpha = calcTmpTheta+inputThetaRadiant[iter]; //all'ultima iterazione avr� calcolato l'alfa che mi interessa
	}
     
	returnResult[0] = calc_x_t;
	returnResult[1] = calc_y_t;
	returnResult[2] = 0; //calc_z_t;
	returnResult[3] = calc_alpha;
}

//definizione funzione per calcolare la cinematica inversa
void Kinematics::calcInverseKinematics(int direction, int iter, double *returnBracci, double *inputThetaRadiant, double *returnResult) {
	calcTheta_0(returnResult);

	for(int i = 2; i <= iter-1; i++)
		calcTheta_i(i, direction, returnBracci, returnResult);

	calcTheta_1(returnBracci, returnResult);

	calcTheta_last(iter, returnResult);
}

void Kinematics::calcTheta_0(double *returnResult) {
	returnResult[4] = 0;
}

void Kinematics::calcTheta_1(double *returnBracci, double *returnResult) {
	returnResult[4+1] = (atan2(returnResult[1], returnResult[0]) - atan2((returnBracci[2] * sin(returnResult[4+2])), (returnBracci[1] + returnBracci[2] * cos(returnResult[4+2]))));
}

void Kinematics::calcTheta_i(int i, int direction, double *returnBracci, double *returnResult) {
	double x_2 = pow(returnResult[0], 2.0); //pow(calc_x_t, 2.0);
	double y_2 = pow(returnResult[1], 2.0);//pow(calc_y_t, 2.0);
	double l1_2 = pow(returnBracci[i-1], 2.0);
	double l2_2 = pow(returnBracci[i], 2.0);

	cout << "braccio (L" << i-1 << ") : " << returnBracci[i-1] << endl;
	cout << "braccio (L" << i << ") : " << returnBracci[i] << endl;

	double tmp2 = (x_2 + y_2 - l1_2 - l2_2)/(2 * returnBracci[i-1] * returnBracci[i]);
	double tmp1 = sqrt(1 - pow(tmp2, 2.0)) * direction;
	
	cout << "theta_i : " << i << endl;
	cout << "tmp2 : " << tmp2 << endl;
	cout << "tmp1 : " << tmp1 << endl;

	returnResult[4+i] = atan2(tmp1, tmp2); //salvo dalle 5a cella in poi... theta2 verrà salvato in 4+2 = 6 cella
}

void Kinematics::calcTheta_last(int iter, double *returnResult) {
	returnResult[4+iter] = (returnResult[3] - returnResult[4+iter-2] - returnResult[4+iter-1]);
}

//============================================================================

//gli angoli per essere calcolati devono essere in radianti
double DegreeToRadiant(double degree) {
	return degree * (M_PI / 180.0);
}

double RadiantToDegree(double radiant) {
	return radiant * (180.0 / M_PI);
}

//============================================================================

int main() {
	int num;
    cout << "Quanti bracci ci sono ? ";
    cin >> num;
     
    double inputBracci[num], inputTheta[num+1], inputThetaRadiant[num+1];

    for(int i = 0; i < num; i++) {
		cout << "Inserisci la lunghezza del braccio " << i << " : ";
		cin >> inputBracci[i];
    }
    
    for(int i = 0; i < num+1; i++) {
		double input;
		cout << "Inserisci l'angolo theta " << i << " (in gradi) : ";
		cin >> input;
		inputTheta[i] = input;
		inputThetaRadiant[i] = DegreeToRadiant(input);
    }
    
    int direction = 1;
    
    double result[num+1+4]; //num(num di bracci)+1(numero di angoli)+4(alfa, x, y, z)
    
    cout << " ============================================================================ " << endl;
    cout << "Parametri utente:" << endl;
    for(int i = 0; i < num; i++) {
    	cout << "L" << i << " : " << inputBracci[i] << endl;
    }
    
    for(int i = 0; i < num+1; i++) {
    	cout << "theta " << i << " : " << inputTheta[i] << " Gradi --> " << inputThetaRadiant[i] << " Radianti" << endl;
    }
	
	cout << "Direzione: " << direction << endl;
	
	cout << " ============================================================================ " << endl;    
    
   	//============================================================================
    
    /* istanziazione di un oggetto di tipo kinematics */
	Kinematics k;

    //============================================================================
    
    //============================================================================
	
    k.calcForwardKinematics(num, inputBracci, inputThetaRadiant, result);
	cout << "Calcolo la Cinematica Diretta" << endl;
	cout << "x_t: " << result[0] << endl;
	cout << "y_t: " << result[1] << endl;
	cout << "z_t: " << result[2] << endl;
	cout << "alpha: " << result[3] << endl;
	
    //============================================================================
    
	cout << " ============================================================================ " << endl;

    //============================================================================
	k.calcInverseKinematics(direction, num, inputBracci, inputThetaRadiant, result);
	cout << "Calcolo la Cinematica Inversa" << endl;
	for(int i = 4; i < num+4+1; i++) {
		cout << "theta " << i-4 << " : " << RadiantToDegree(result[i]) << " Gradi" << endl;
	}

	//============================================================================
	
    cout << " ============================================================================ " << endl;
    
    //system("PAUSE");
    //return EXIT_SUCCESS;
    return 0;
}
