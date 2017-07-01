//============================================================================
// Name        : SR_project.cpp
// Author      : Giuseppe Giliberto, Giuseppe Puglisi, Giuseppe Sgroi
// Version     : 2.0
// Copyright   : Università degli Studi di Catania
// Description : Kinematics Forward and Inverse
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
		//void calcInverseKinematics(int direction);

		//ritorna theta2
		//void calcTheta2();

		//calcola il theta3
		//void calcTheta3(int direction);

		//calcola il theta4
		//void calcTheta4();	

		//double theta1, theta2, theta3, theta4; //angoli
		//double L1, L2; //bracci
	 
        //double calc_alpha;
		//double calc_x_t, calc_y_t, calc_z_t; //coordinate calcolate
		//double calc_theta1, calc_theta2, calc_theta3, calc_theta4; //angoli calcolati
};

//definizione del costruttore
Kinematics::Kinematics() {}

//definizione funzione per calcolare la cinematica diretta
void Kinematics::calcForwardKinematics(int iter, double *returnBracci, double *inputThetaRadiant, double *returnResult) {
     double calc_x_t, calc_y_t, calc_z_t, calc_alpha;
     
     for(int i = 0; i < iter; i++) {
             double calcTmpTheta = 0;
             
             for(int j = 0; j <= i; j++) {
                     cout << "iter : " << i << " - j : " << j << endl;
                     calcTmpTheta += inputThetaRadiant[j];
             }
             
             calc_x_t += returnBracci[i] * cos(calcTmpTheta);
             calc_y_t += returnBracci[i] * sin(calcTmpTheta);
             //calc_z_t += 0; // ??
             
             calc_alpha = calcTmpTheta+inputThetaRadiant[iter]; //all'ultima iterazione avrò calcolato l'alfa che mi interessa
     }
     
     returnResult[0] = calc_x_t;
     returnResult[1] = calc_y_t;
     returnResult[2] = 0; //calc_z_t;
     returnResult[3] = calc_alpha;
}

/*//definizione funzione per calcolare la cinematica inversa
void Kinematics::calcInverseKinematics(int direction){
     calc_theta1 = 0;

	 calcTheta3(direction);
	
	 calcTheta2();
	
	 calcTheta4();
}

//calcola theta2
void Kinematics::calcTheta2(){
     calc_theta2 = (atan2(calc_y_t, calc_x_t) - atan2((L2 * sin(calc_theta3)), (L1 + L2 * cos(calc_theta3))));
}

//calcola theta3
void Kinematics::calcTheta3(int direction) {
     double x_2 = pow(calc_x_t, 2.0);
	 double y_2 = pow(calc_y_t, 2.0);
	 double l1_2 = pow(L1, 2.0);
	 double l2_2 = pow(L2, 2.0);

     double tmp2 = (x_2 + y_2 - l1_2 - l2_2)/(2 * L1 * L2);
	 double tmp1 = sqrt(1 - pow(tmp2, 2.0)) * direction;
	
	 calc_theta3 = (atan2(tmp1, tmp2));
}

//calcola theta4
void Kinematics::calcTheta4() {
     calc_theta4 = (calc_alpha - calc_theta2 - calc_theta3);
}*/

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
    int num;
    cout << "Quanti bracci ci sono ? ";
    cin >> num;
     
    double inputBracci[num];

    for(int i=0; i < num; i++) {
            cout << "Inserisci la lunghezza del braccio " << i << " : ";
            cin >> inputBracci[i];
    }
    
    double inputTheta[num+1], inputThetaRadiant[num+1];
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
    for(int i=0; i < num; i++) {
            cout << "L" << i << " : " << inputBracci[i] << endl;
    }
    
    for(int i=0; i < num+1; i++) {
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

    /*//============================================================================
	k.calcInverseKinematics(direction);
	cout << "Calcolo la Cinematica Inversa" << endl;
	cout << "theta1: " << RadiantToDegree(k.calc_theta1) << " Gradi" << endl;
	cout << "theta2: " << RadiantToDegree(k.calc_theta2) << " Gradi" << endl;
	cout << "theta3: " << RadiantToDegree(k.calc_theta3) << " Gradi" << endl;
	cout << "theta4: " << RadiantToDegree(k.calc_theta4) << " Gradi" << endl;
	//============================================================================
	
    cout << " ============================================================================ " << endl;*/
    
    system("PAUSE");
    return EXIT_SUCCESS;
}
