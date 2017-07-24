#include <cstdlib>
#include <iostream>
#include <math.h>
#define _USE_MATH_DEFINES
#include "Kinematics3d.h"
#include "Utility.h"

using namespace std;

//============================================================================

void Kinematics3d::setInputForward(GeometryArms g) {
	cout << "+-------------------------------------+ \n";
	cout << "| Selected: \"3D Forward Kinematics\"   | \n";
	cout << "+-------------------------------------+ \n\n";

	if(g.sGeoA.L1 != 0 && g.sGeoA.L2 != 0) {
		cout << "+-------------------------------+ \n";
		cout << "|  Enter theta 1 angle (Degree) | \n";
		cout << "+-------------------------------+ \n";
		cin >> inF_3d.theta1;
		cout << "+-------------------------------+ \n";
		cout << "|  Enter theta 2 angle (Degree) | \n";
		cout << "+-------------------------------+ \n";
		cin >> inF_3d.theta2;
		cout << "+-------------------------------+ \n";
		cout << "|  Enter theta 3 angle (Degree) | \n";
		cout << "+-------------------------------+ \n";
		cin >> inF_3d.theta3;
		cout << "+-------------------------------+ \n";
		cout << "|  Enter gamma angle (Degree)   | \n";
		cout << "+-------------------------------+ \n";
		cin >>inF_3d.gamma;

		inF_3d.theta1 = DegreeToRadiant(inF_3d.theta1);
		inF_3d.theta2 = DegreeToRadiant(inF_3d.theta2);
		inF_3d.theta3 = DegreeToRadiant(inF_3d.theta3);
		inF_3d.gamma = DegreeToRadiant(inF_3d.gamma);

		calculateForwardKinematics(g);
	} else {
		cout << "You should first set Arm Geometry!\n\n";
	}

	//system("PAUSE");
	//system("cls");
	cls(); //clear screen
}

//============================================================================

void Kinematics3d::calculateForwardKinematics(GeometryArms g) {
	double x_primo = g.sGeoA.L1 * cos(inF_3d.theta1) + g.sGeoA.L2 * cos(inF_3d.theta1 + inF_3d.theta2);
	double y_primo = g.sGeoA.L1 * sin(inF_3d.theta1) + g.sGeoA.L2 * sin(inF_3d.theta1 + inF_3d.theta2);
	double alpha = inF_3d.theta1 + inF_3d.theta2 + inF_3d.theta3;

	double x = cos(inF_3d.gamma) * x_primo;
	double y = y_primo;
	double z = sin(inF_3d.gamma) * x_primo;

	cout << "\n+---------------------------------------------------------------------+ \n\n";
	cout << "+--------------------------------------+ \n";
	cout << "|  Point X : " << x << "\n";
	cout << "+--------------------------------------+ \n";
	cout << "|  Point Y : " << y << "\n";
	cout << "+--------------------------------------+ \n";
	cout << "|  Point Z : " << z << "\n";
	cout << "+--------------------------------------+ \n";
	cout << "|  Alpha : " << RadiantToDegree(alpha) << " (Degree)\n";
	cout << "+--------------------------------------+ \n\n";
}

//============================================================================

void Kinematics3d::setInputInverse(GeometryArms g) {
	cout << "+-------------------------------------+ \n";
	cout << "| Selected: \"3D Inverse Kinematics\"   | \n";
	cout << "+-------------------------------------+ \n\n";

	if(g.sGeoA.L1 != 0 && g.sGeoA.L2 != 0) {
		cout << "+-----------------------------+ \n";
		cout << "|  Enter point X              | \n";
		cout << "+-----------------------------+ \n";
		cin >> inI_3d.x;
		cout << "+-----------------------------+ \n";
		cout << "|  Enter point Y              | \n";
		cout << "+-----------------------------+ \n";
		cin >> inI_3d.y;
		cout << "+-----------------------------+ \n";
		cout << "|  Enter point Z              | \n";
		cout << "+-----------------------------+ \n";
		cin >> inI_3d.z;
		cout << "+-----------------------------+ \n";
		cout << "|  Enter Alpha angle (Degree) | \n";
		cout << "+-----------------------------+ \n";
		cin >> inI_3d.alpha;

		inI_3d.alpha = DegreeToRadiant(inI_3d.alpha);

		calculateInverseKinematics(g);
	} else {
		cout << "You should first set Arm Geometry!\n\n";
	}

	//system("PAUSE");
	//system("cls");
	cls(); //clear screen
}

//============================================================================

void Kinematics3d::calculateInverseKinematics(GeometryArms g) {
	double gamma = atan2(inI_3d.z, inI_3d.x);
	double x_primo = inI_3d.x * cos(gamma) + inI_3d.z * sin(gamma);
	double y_primo = inI_3d.y;

	/* Cacolo Theta 2 */
	int direction = 1;
	double x_primo_2 = pow(x_primo, 2.0);
	double y_primo_2 = pow(y_primo, 2.0);
	double l1_2 = pow(g.sGeoA.L1, 2.0);
	double l2_2 = pow(g.sGeoA.L2, 2.0);
	double tmp2 = (x_primo_2 + y_primo_2 - l1_2 - l2_2)/(2 * g.sGeoA.L1 * g.sGeoA.L2);
	double tmp2_2 = pow(tmp2, 2.0);

	if(tmp2_2 <= 1) {
		double tmp1 = sqrt(1 - tmp2_2) * direction;
		double theta2 = atan2(tmp1, tmp2);

		/* Calcolo Theta 1 */
		double theta1 = (atan2(y_primo, x_primo) - atan2((g.sGeoA.L2 * sin(theta2)), (g.sGeoA.L1 + g.sGeoA.L2 * cos(theta2))));

		/* Calcolo Theta 3 */
		double theta3 = (inI_3d.alpha - theta1 - theta2);

		cout << "\n+---------------------------------------------------------------------+ \n\n";
		cout << "+------------------------------------------------------------+ \n";
		cout << "|  Theta 1 : " << theta1 << " (Radiant) , " << RadiantToDegree(theta1) << " (Degree) \n";
		cout << "+------------------------------------------------------------+ \n";
		cout << "|  Theta 2 : " << theta2 << " (Radiant) , " << RadiantToDegree(theta2) << " (Degree) \n";
		cout << "+------------------------------------------------------------+ \n";
		cout << "|  Theta 3 : " << theta3 << " (Radiant) , " << RadiantToDegree(theta3) << " (Degree) \n";
		cout << "+------------------------------------------------------------+ \n";
		cout << "|  Gamma : " << gamma << " (Radiant) , " << RadiantToDegree(gamma) << " (Degree) \n";
		cout << "+------------------------------------------------------------+ \n\n";
	} else {
		cout << "\nPoint unreachable !" << endl;
	}
}
