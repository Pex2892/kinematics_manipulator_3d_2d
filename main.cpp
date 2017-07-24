//============================================================================
// Name        : main.cpp
// Author      : Giuseppe Giliberto, Giuseppe Puglisi, Giuseppe Sgroi
// Version     : 2.0
// Copyright   : Universita' degli Studi di Catania
// Description : Forward and Inverse Kinematics 3D / 2D
//============================================================================

#include <cstdlib>
#include <iostream>
#include <math.h>
#define _USE_MATH_DEFINES
#include "GeometryArms.h"
#include "Kinematics3d.h"
#include "Kinematics2d.h"
#include "Utility.h"

using namespace std;

//============================================================================

int main(int argc, char *argv[]) {
	//system("COLOR 0A");
	int scelta;

	GeometryArms g;
	g.sGeoA.L1 = 0;
	g.sGeoA.L2 = 0;

	Kinematics3d K3d;
	Kinematics2d K2d;

	while (scelta != 0) {
		menu(g);
		cin >> scelta;

		//system("cls");
		cls(); //clear screen

		switch(scelta) {
			case 0:
				break;
			case 1:
				g.setGeometryArms();
				break;
			case 2:
				K3d.setInputForward(g);
				break;
			case 3:
				K3d.setInputInverse(g);
				break;
			case 4:
				K2d.setInputForward(g);
				break;
			case 5:
				K2d.setInputInverse(g);
				break;
		}
	}

	//system("PAUSE");
	return EXIT_SUCCESS;
}
