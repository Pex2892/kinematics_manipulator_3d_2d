#include <cstdlib>
#include <iostream>
#include "GeometryArms.h"

using namespace std;

void GeometryArms::setGeometryArms() {
     cout<<"+----------------------------------+ \n";
     cout<<"| Selected: \"Enter Arm Geometry\"   | \n";
     cout<<"+----------------------------------+ \n\n";
     cout<<"+----------------------+ \n";
     cout<<"|  Enter Arm 1 length  | \n";
     cout<<"+----------------------+ \n";
     cin>>sGeoA.L1;
     cout<<"+----------------------+ \n";
     cout<<"|  Enter Arm 2 length  | \n";
     cout<<"+----------------------+ \n";
     cin>>sGeoA.L2;
     
     system("PAUSE");
     system("cls");
}



