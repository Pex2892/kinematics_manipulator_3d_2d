// Kinematics.h
#ifndef Kinematics_H
#define Kinematics_H

//definizione della classe Kinematics
class Kinematics {
	//parte public accessibile a tutti
	public:
    	//costruttore
		Kinematics(double t1, double t2, double t3, double t4, double l1, double l2);

		//calcola cinematica diretta
		void calcForwardKinematics();

		//calcola cinematica inversa
		void calcInverseKinematics(int direction);

		//ritorna theta2
		void calcTheta2();

		//calcola il theta3
		void calcTheta3(int direction);

		//calcola il theta4
		void calcTheta4();

		double theta1, theta2, theta3, theta4; //angoli
		double L1, L2; //bracci
	 
        double calc_alpha;
		double calc_x_t, calc_y_t, calc_z_t; //coordinate calcolate
		double calc_theta1, calc_theta2, calc_theta3, calc_theta4; //angoli calcolati
};

//definizione del costruttore
Kinematics::Kinematics(double t1, double t2, double t3, double t4, double l1, double l2) {
	theta1 = t1; theta2 = t2; theta3 = t3; theta4 = t4;
	L1 = l1; L2 = l2;
}

//definizione funzione per calcolare la cinematica diretta
void Kinematics::calcForwardKinematics() {
     calc_x_t = L1 * cos((theta2)) + L2 * cos((theta2) + (theta3));
     
     calc_y_t = L1 * sin((theta2)) + L2 * sin((theta2) + (theta3)); 
	 
     calc_z_t = 0;

	 calc_alpha = theta2 + theta3 + theta4;
}

//definizione funzione per calcolare la cinematica inversa
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
}

#endif
