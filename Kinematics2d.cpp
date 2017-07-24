#include <cstdlib>
#include <iostream>
#include <math.h>
#define _USE_MATH_DEFINES
#include "Kinematics2d.h"
#include "Utility.h"

using namespace std;

//============================================================================

void Kinematics2d::setInputForward(GeometryArms g) {
	cout << "+-------------------------------------+ \n";
	cout << "| Selected: \"2D Forward Kinematics\"   | \n";
	cout << "+-------------------------------------+ \n\n";

	if(g.sGeoA.L1 != 0 && g.sGeoA.L2 != 0) {
		cout << "+-------------------------------+ \n";
		cout << "|  Enter theta 1 angle (Degree) | \n";
		cout << "+-------------------------------+ \n";
		cin >> inF_2d.theta1;
		cout << "+-------------------------------+ \n";
		cout << "|  Enter theta 2 angle (Degree) | \n";
		cout << "+-------------------------------+ \n";
		cin >> inF_2d.theta2;
		cout << "+-------------------------------+ \n";
		cout << "|  Enter theta 3 angle (Degree) | \n";
		cout << "+-------------------------------+ \n";
		cin >> inF_2d.theta3;

		inF_2d.theta1 = DegreeToRadiant(inF_2d.theta1);
		inF_2d.theta2 = DegreeToRadiant(inF_2d.theta2);
		inF_2d.theta3 = DegreeToRadiant(inF_2d.theta3);

		calculateForwardKinematics(g);
	} else {
		cout << "You should first set Arm Geometry!\n\n";
	}

	//system("PAUSE");
	//system("cls");
	cls(); //clear screen
}

//============================================================================

void Kinematics2d::calculateForwardKinematics(GeometryArms g) {
	double x = g.sGeoA.L1 * cos(inF_2d.theta1) + g.sGeoA.L2 * cos(inF_2d.theta1 + inF_2d.theta2);
	double y = g.sGeoA.L1 * sin(inF_2d.theta1) + g.sGeoA.L2 * sin(inF_2d.theta1 + inF_2d.theta2);
	double alpha = inF_2d.theta1 + inF_2d.theta2 + inF_2d.theta3;

	cout << "\n+---------------------------------------------------------------------+ \n\n";
	cout << "+--------------------------------------+ \n";
	cout << "|  Point X : " << x << "\n";
	cout << "+--------------------------------------+ \n";
	cout << "|  Point Y : " << y << "\n";
	cout << "+--------------------------------------+ \n";
	cout << "|  Alpha : " << RadiantToDegree(alpha) << " (Gradi)\n";
	cout << "+--------------------------------------+ \n\n";
}

//============================================================================

void Kinematics2d::setInputInverse(GeometryArms g) {
	cout << "+-------------------------------------+ \n";
	cout << "| Selected: \"2D Inverse Kinematics\"   | \n";
	cout << "+-------------------------------------+ \n\n";

	if(g.sGeoA.L1 != 0 && g.sGeoA.L2 != 0) {
		cout << "+-----------------------------+ \n";
		cout << "|  Enter point X              | \n";
		cout << "+-----------------------------+ \n";
		cin >> inI_2d.x;
		cout << "+-----------------------------+ \n";
		cout << "|  Enter point Y              | \n";
		cout << "+-----------------------------+ \n";
		cin >> inI_2d.y;
		cout << "+-----------------------------+ \n";
		cout << "|  Enter Alpha angle (Degree) | \n";
		cout << "+-----------------------------+ \n";
		cin >> inI_2d.alpha;

		inI_2d.alpha = DegreeToRadiant(inI_2d.alpha);

		calculateInverseKinematics(g);
	} else {
		cout << "You should first set Arm Geometry!\n\n";
	}

	//system("PAUSE");
	//system("cls");
	cls(); //clear screen
}

//============================================================================

void Kinematics2d::calculateInverseKinematics(GeometryArms g) {
	/* Cacolo Theta 2 */
	int direction = 1;
	double x_2 = pow(inI_2d.x, 2.0);
	double y_2 = pow(inI_2d.y, 2.0);
	double l1_2 = pow(g.sGeoA.L1, 2.0);
	double l2_2 = pow(g.sGeoA.L2, 2.0);
	double tmp2 = (x_2 + y_2 - l1_2 - l2_2)/(2 * g.sGeoA.L1 * g.sGeoA.L2);
	double tmp2_2 = pow(tmp2, 2.0);

	if(tmp2_2 <= 1) {
		double tmp1 = sqrt(1 - tmp2_2) * direction;
		double theta2 = atan2(tmp1, tmp2);

		/* Calcolo Theta 1 */
		double theta1 = (atan2(inI_2d.y, inI_2d.x) - atan2((g.sGeoA.L2 * sin(theta2)), (g.sGeoA.L1 + g.sGeoA.L2 * cos(theta2))));

		/* Calcolo Theta 3 */
		double theta3 = (inI_2d.alpha - theta1 - theta2);

		cout << "\n+---------------------------------------------------------------------+ \n\n";
		cout << "+------------------------------------------------------------+ \n";
		cout << "|  Theta 1 : " << theta1 << " (Radiant) , " << RadiantToDegree(theta1) << " (Degree) \n";
		cout << "+------------------------------------------------------------+ \n";
		cout << "|  Theta 2 : " << theta2 << " (Radiant) , " << RadiantToDegree(theta2) << " (Degree) \n";
		cout << "+------------------------------------------------------------+ \n";
		cout << "|  Theta 3 : " << theta3 << " (Radiant) , " << RadiantToDegree(theta3) << " (Degree) \n";
		cout << "+------------------------------------------------------------+ \n\n";
	} else {
		cout << "\nPoint unreachable !" << endl;
	}
}
