//============================================================================
// Name        : SR_project.cpp
// Author      : Giuseppe Giliberto, Giuseppe Puglisi, Giuseppe Sgroi
// Version     : 1.0
// Copyright   : Università degli Studi di Catania
// Description : Kinematics Forward and Inverse
//============================================================================

#define _USE_MATH_DEFINES
#include <iostream>
#include <math.h>
#include "Kinematics.h"
using namespace std;

//============================================================================

//gli angoli per essere calcolati devono essere in radianti
double DegreeToRadiant(double degree) {
       return degree * (M_PI / 180.0);
}

double RadiantToDegree(double radiant) {
       return radiant * (180.0 / M_PI);
}

//============================================================================

int main(int argc, char *argv[]) {
    //============================================================================
    // Parametri in input
    double angle1 = 0, angle2 = 23, angle3 = 35, angle4 = 10; //angoli in gradi
   	double L1 = 35, L2 = (L1/2); //bracci
	int direction = 1; //la direzione mi indica il verso dell'angolo, se == 1 a destra, altrimenti == -1 a sinistra
	//============================================================================
	
	double theta1 = DegreeToRadiant(angle1), theta2 = DegreeToRadiant(angle2), theta3 = DegreeToRadiant(angle3), theta4 = DegreeToRadiant(angle4); //angoli in radianti

    cout << " ============================================================================ " << endl;
    
	cout << "Parametri utente:" << endl;
	cout << "theta1: " << angle1 << " Gradi --> " << theta1 << " Radianti" << endl;
	cout << "theta2: " << angle2 << " Gradi --> " << theta2 << " Radianti" << endl;
	cout << "theta3: " << angle3 << " Gradi --> " << theta3 << " Radianti" << endl;
	cout << "theta4: " << angle4 << " Gradi --> " << theta4 << " Radianti" << endl;
	cout << "L1: " << L1 << endl;
	cout << "L2: " << L2 << endl;
	cout << "Direzione: " << direction << endl;
	
	cout << " ============================================================================ " << endl;
    
	//============================================================================
    
    /* istanziazione di un oggetto di tipo kinematics */
	Kinematics k(theta1, theta2, theta3, theta4, L1, L2);

    //============================================================================
	
    k.calcForwardKinematics();
	cout << "Calcolo la Cinematica Diretta" << endl;
	cout << "x_t: " << k.calc_x_t << endl;
	cout << "y_t: " << k.calc_y_t << endl;
	cout << "z_t: " << k.calc_z_t << endl;
	cout << "alpha: " << k.calc_alpha << endl;
	
    //============================================================================
    
	cout << " ============================================================================ " << endl;

    //============================================================================
	k.calcInverseKinematics(direction);
	cout << "Calcolo la Cinematica Inversa" << endl;
	cout << "theta1: " << RadiantToDegree(k.calc_theta1) << " Gradi" << endl;
	cout << "theta2: " << RadiantToDegree(k.calc_theta2) << " Gradi" << endl;
	cout << "theta3: " << RadiantToDegree(k.calc_theta3) << " Gradi" << endl;
	cout << "theta4: " << RadiantToDegree(k.calc_theta4) << " Gradi" << endl;
	//============================================================================
	
    cout << " ============================================================================ " << endl;
    
    system("PAUSE");
    return EXIT_SUCCESS;
}
