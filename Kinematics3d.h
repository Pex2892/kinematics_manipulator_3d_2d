#ifndef Kinematics3d_H
#define Kinematics3d_H
#include "GeometryArms.h"

class Kinematics3d : public GeometryArms {
	public:
		struct inputForward {
			double theta1;
			double theta2;
			double theta3;
			double gamma;
		};

		struct inputInverse {
			double x;
			double y;
			double z;
			double alpha;
		};

		void setInputForward(GeometryArms g);
		void setInputInverse(GeometryArms g);

		void calculateForwardKinematics(GeometryArms g);
		void calculateInverseKinematics(GeometryArms g);

		inputForward inF_3d;
		inputInverse inI_3d;
};

#endif
