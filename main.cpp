//============================================================================
// Name        : main.cpp
// Author      : Giuseppe Giliberto, Giuseppe Puglisi, Giuseppe Sgroi
// Version     : 2.0
// Copyright   : Universita' degli Studi di Catania
// Description : Forward and Inverse Kinematics 
//============================================================================

#include <cstdlib>
#include <iostream>
#include <math.h>
#define _USE_MATH_DEFINES
#include "Kinematics.h"

using namespace std;

//============================================================================

main() {
       system("COLOR 0A");
       int scelta;
       struct inputForward inF; 
       struct inputInverse inI; 
       struct geometryArms geoA; 
       geoA.L1 = 0;
       geoA.L2 = 0;
       
       while (scelta != 0) {
             about();
             cout<<"+-------------------------------+            +-------------------------+\n"
                 <<"|  0: Chiudi                    |            |  L1: " << geoA.L1 << "            \n"
                 <<"+-------------------------------+            +-------------------------+\n"
                 <<"|  1: Geometria del braccio     |            |  L2: " << geoA.L2 << "            \n"
                 <<"+-------------------------------+            +-------------------------+\n"
                 <<"|  2: Cinematica Diretta        |\n"
                 <<"+-------------------------------+\n"
                 <<"|  3: Cinematica Inversa        |\n"
                 <<"+-------------------------------+\n";
             cout<< "\nCosa vuoi fare? : ";
             cin>>scelta;
                 
             system("cls");
                 
             switch(scelta) {
                 case 0:
                      break;
                 case 1:
                      setGeometryArms(geoA);
                      break;
                 case 2:
                      setInputForward(inF, geoA);
                      break;
                 case 3:
                      setInputInverse(inI, geoA);
                      break;
             } 
       }
       system("PAUSE");
}

//============================================================================

void setGeometryArms(struct geometryArms &input) {
     cout<<"+------------------------------------------------+ \n";
     cout<<"| HAI scelto \"Inserimento Geometria del Braccio\" | \n";
     cout<<"+------------------------------------------------+ \n\n";
     cout<<"+---------------------------------------+ \n";
     cout<<"|  Inserisci lunghezza del braccio 1    | \n";
     cout<<"+---------------------------------------+ \n";
     cin>>input.L1;
     cout<<"+---------------------------------------+ \n";
     cout<<"|  Inserisci lunghezza del braccio 2    | \n";
     cout<<"+---------------------------------------+ \n";
     cin>>input.L2;
     
     system("PAUSE");
     system("cls");
}

//============================================================================

void setInputForward(struct inputForward &input, struct geometryArms &input2) {
     cout<<"+------------------------------------------------+ \n";
     cout<<"| HAI scelto \"Calcolo della Cinematica Diretta\"  | \n";
     cout<<"+------------------------------------------------+ \n\n";
     cout<<"+--------------------------------------+ \n";
     cout<<"|  Inserisci l'angolo theta 1          | \n";
     cout<<"+--------------------------------------+ \n";
     cin>>input.theta1;
     cout<<"+--------------------------------------+ \n";
     cout<<"|  Inserisci l'angolo theta 2          | \n";
     cout<<"+--------------------------------------+ \n";
     cin>>input.theta2;
     cout<<"+--------------------------------------+ \n";
     cout<<"|  Inserisci l'angolo theta 3          | \n";
     cout<<"+--------------------------------------+ \n";
     cin>>input.theta3;
     cout<<"+--------------------------------------+ \n";
     cout<<"|  Inserisci l'angolo gamma (Gradi)    | \n";
     cout<<"+--------------------------------------+ \n";
     cin>>input.gamma;
     
     input.theta1 = DegreeToRadiant(input.theta1);
     input.theta2 = DegreeToRadiant(input.theta2);
     input.theta3 = DegreeToRadiant(input.theta3);
     input.gamma = DegreeToRadiant(input.gamma);
     
     calculateForwardKinematics(input, input2);
  
     system("PAUSE");
     system("cls");
}

//============================================================================

void setInputInverse(struct inputInverse &input, struct geometryArms &input2) {
     cout<<"+------------------------------------------------+ \n";
     cout<<"| HAI scelto \"Calcolo della Cinematica Inversa\"  | \n";
     cout<<"+------------------------------------------------+ \n\n";
     cout<<"+-----------------------------------+ \n";
     cout<<"|  Inserisci il punto X             | \n";
     cout<<"+-----------------------------------+ \n";
     cin>>input.x;
     cout<<"+-----------------------------------+ \n";
     cout<<"|  Inserisci il punto Y             | \n";
     cout<<"+-----------------------------------+ \n";
     cin>>input.y;
     cout<<"+-----------------------------------+ \n";
     cout<<"|  Inserisci il punto Z             | \n";
     cout<<"+-----------------------------------+ \n";
     cin>>input.z;
     cout<<"+-----------------------------------+ \n";
     cout<<"|  Inserisci il punto Alfa (Gradi)  | \n";
     cout<<"+-----------------------------------+ \n";
     cin>>input.alpha;
     
     input.alpha = DegreeToRadiant(input.alpha);
     
     calculateInverseKinematics(input, input2);
         
     system("PAUSE");
     system("cls");
}

//============================================================================

void calculateForwardKinematics(struct inputForward &input, struct geometryArms &input2) {
     double x_primo = input2.L1 * cos(input.theta1) + input2.L2 * cos(input.theta1 + input.theta2);
     double y_primo = input2.L1 * sin(input.theta1) + input2.L2 * sin(input.theta1 + input.theta2);
     double alpha = input.theta1 + input.theta2 + input.theta3;
     
     double x = cos(input.gamma) * x_primo;
     double y = y_primo;
     double z = sin(input.gamma) * x_primo;
     
     cout<<"\n+---------------------------------------------------------------------+ \n\n";
     cout<<"+--------------------------------------+ \n";
     cout<<"|  Punto X : " << x << "\n";
     cout<<"+--------------------------------------+ \n";
     cout<<"|  Punto Y : " << y << "\n";
     cout<<"+--------------------------------------+ \n";
     cout<<"|  Punto Z : " << z << "\n";
     cout<<"+--------------------------------------+ \n";
     cout<<"|  Alfa : " << RadiantToDegree(alpha) << " (Gradi)\n";
     cout<<"+--------------------------------------+ \n";
}

//============================================================================

void calculateInverseKinematics(struct inputInverse &input, struct geometryArms &input2) {
     double gamma = atan2(input.z, input.x);
     double x_primo = input.x * cos(gamma) + input.z * sin(gamma);
     double y_primo = input.y;

   	 /* Cacolo Theta 2 */
   	 int direction = 1;
     double x_primo_2 = pow(x_primo, 2.0);
	 double y_primo_2 = pow(y_primo, 2.0);
	 double l1_2 = pow(input2.L1, 2.0);
	 double l2_2 = pow(input2.L2, 2.0);
	 double tmp2 = (x_primo_2 + y_primo_2 - l1_2 - l2_2)/(2 * input2.L1 * input2.L2);
	 double tmp1 = sqrt(1 - pow(tmp2, 2.0)) * direction;
	 double theta2 = atan2(tmp1, tmp2);
	 
	 /* Calcolo Theta 1 */
	 double theta1 = (atan2(y_primo, x_primo) - atan2((input2.L2 * sin(theta2)), (input2.L1 + input2.L2 * cos(theta2))));
	 
	 /* Calcolo Theta 3 */
	 double theta3 = (input.alpha - theta1 - theta2);

	 cout<<"\n+---------------------------------------------------------------------+ \n\n";
     cout<<"+------------------------------------------------------------+ \n";
     cout<<"|  Theta 1 : " << theta1 << " (Radianti) , " << RadiantToDegree(theta1) << " (Gradi) \n";
     cout<<"+------------------------------------------------------------+ \n";
     cout<<"|  Theta 2 : " << theta2 << " (Radianti) , " << RadiantToDegree(theta2) << " (Gradi) \n";
     cout<<"+------------------------------------------------------------+ \n";
     cout<<"|  Theta 3 : " << theta3 << " (Radianti) , " << RadiantToDegree(theta3) << " (Gradi) \n";
     cout<<"+------------------------------------------------------------+ \n";
     cout<<"|  Gamma : " << gamma << " (Radianti) , " << RadiantToDegree(gamma) << " (Gradi) \n";
     cout<<"+------------------------------------------------------------+ \n";
}

//============================================================================

double DegreeToRadiant(double degree) {
       return degree * (M_PI / 180.0);
}

//============================================================================

double RadiantToDegree(double radiant) {
       return radiant * (180.0 / M_PI);
}

//============================================================================

void about() {
     cout<<"######                                                        #####   #####  \n";
     cout<<"#     #  ####  #####   ####  ##### #  ####    ##             #     # #     # \n";
     cout<<"#     # #    # #    # #    #   #   # #    #  #  #                  # #       \n";
     cout<<"######  #    # #####  #    #   #   # #      #    #   #####    #####  #  #### \n";
     cout<<"#   #   #    # #    # #    #   #   # #      ######                 # #     # \n";
     cout<<"#    #  #    # #    # #    #   #   # #    # #    #           #     # #     # \n";
     cout<<"#     #  ####  #####   ####    #   #  ####  #    #            #####   #####  \n\n";
}
