#include <cstdlib>
#include <iostream>
#include <math.h>
#define _USE_MATH_DEFINES
#include "GeometryArms.h"

using namespace std;

void menu(GeometryArms g) {
	cout << "  #####   #####            ######                                             \n";
	cout << " #     # #     #           #     #  ####  #####   ####  ##### #  ####   ####  \n";
	cout << "       # #                 #     # #    # #    # #    #   #   # #    # #      \n";
	cout << "  #####  #  ####   #####   ######  #    # #####  #    #   #   # #       ####  \n";
	cout << "       # #     #           #   #   #    # #    # #    #   #   # #           # \n";
	cout << " #     # #     #           #    #  #    # #    # #    #   #   # #    # #    # \n";
	cout << "  #####   #####            #     #  ####  #####   ####    #   #  ####   ####  \n\n";
     
     
	cout << "+------------ Menu ------------+            +-------------------------+\n"
		 << "|  0: Exit                     |            |  L1: " << g.sGeoA.L1 << "            \n"
		 << "+------------------------------+            +-------------------------+\n"
		 << "|  1: Arm Geometry             |            |  L2: " << g.sGeoA.L2 << "            \n"
		 << "+------------------------------+            +-------------------------+\n"
		 << "|  2: 3D Forward Kinematics    |\n"
		 << "+------------------------------+\n"
		 << "|  3: 3D Inverse Kinematics    |\n"
		 << "+------------------------------+\n"
		 << "|  4: 2D Forward Kinematics    |\n"
		 << "+------------------------------+\n"
		 << "|  5: 2D Inverse Kinematics    |\n"
		 << "+------------------------------+\n";
	cout << "\nSelect an option : ";

}

//============================================================================

void cls() {
	cout << string(5, '\n');
}

//============================================================================

double DegreeToRadiant(double degree) {
	return degree * (M_PI / 180.0);
}

//============================================================================

double RadiantToDegree(double radiant) {
	return radiant * (180.0 / M_PI);
}
