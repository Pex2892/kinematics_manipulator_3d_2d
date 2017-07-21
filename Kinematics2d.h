#ifndef Kinematics2d_H
#define Kinematics2d_H
#include "GeometryArms.h"

class Kinematics2d : public GeometryArms {
      public:
             struct inputForward {
                    double theta1;
                    double theta2;
                    double theta3;
             };

             struct inputInverse {
                    double x;
                    double y;
                    double alpha;
             };
             
             void setInputForward(GeometryArms g);
             void setInputInverse(GeometryArms g);

             void calculateForwardKinematics(GeometryArms g);
             void calculateInverseKinematics(GeometryArms g);
            
             inputForward inF_2d;
             inputInverse inI_2d;
};

#endif
