// Kinematics.h
#ifndef Kinematics_H
#define Kinematics_H

//============================================================================

struct geometryArms
{
       double L1;
       double L2;
};

struct inputForward
{
       double theta1;
       double theta2;
       double theta3;
       double gamma;
};

struct inputInverse
{
       double x;
       double y;
       double z;
       double alpha;
};

//============================================================================

void setGeometryArms(struct geometryArms &input);

void setInputForward(struct inputForward &input, struct geometryArms &input2);
void setInputInverse(struct inputInverse &input, struct geometryArms &input2);

void calculateForwardKinematics(struct inputForward &input, struct geometryArms &input2);
void calculateInverseKinematics(struct inputInverse &input, struct geometryArms &input2);

double DegreeToRadiant(double degree);
double RadiantToDegree(double radiant);
void about();

#endif
