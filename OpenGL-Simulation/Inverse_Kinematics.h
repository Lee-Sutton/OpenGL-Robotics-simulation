#pragma once



// Define global constants
#define PI 3.14159265358979324

// Create a structure to hold the joint variables
struct joint
{
	// Store all the theta values as part of the joint
	float t1;
	float t2;
	float t3;
	float t4;
	float t5;
	float t6;

	// Include a boolean value indicating if the theta values are within the joint restrictions
};

// Create a structre to hold the components of the transformation matrix
struct transform
{
	float px;
	float py;
	float pz;
	float r11;
	float r12;
	float r13;
	float r21;
	float r22;
	float r23;
	float r31;
	float r32;
	float r33;
};



// Inverse kinematics Function
// This function will use the inverse kinematics equations to find the joint variables
// Input will be a transformation matrix including all the rotation components and translation components
// Output will include be a struct of type joint containing all the joint variables theta_1 to theta_6
joint InvKin(transform matrix) {
	// Define link parameters
	float a0 = 0;
	float alpha0 = 0;
	float d1 = 0;
	float a1 = 0;
	float alpha1 = -PI / 2;
	float d2 = 0;
	float a2 = 10;
	float alpha2 = 0;
	float d3 = 0.5;
	float a3 = 3.5;
	float alpha3 = -PI / 2;
	float d4 = 5.5;

	// Define the transformation matrix parameters
	float px = matrix.px;
	float py = matrix.py;
	float pz = matrix.pz;
	float r11 = matrix.r11;
	float r12 = matrix.r12;
	float r13 = matrix.r13;
	float r21 = matrix.r21;
	float r22 = matrix.r22;
	float r23 = matrix.r23;
	float r31 = matrix.r31;
	float r32 = matrix.r32;
	float r33 = matrix.r33;

	// Initialize the joint angles variable to return from this function containing the inverse kinematic solutions
	joint joint_angles;

	// Initialize boolean values to check if all the joint angles have been assigned as valid solutions
	bool t1_valid = FALSE;
	bool t2_valid = FALSE;
	bool t3_valid = FALSE;
	bool t4_valid = FALSE;
	bool t5_valid = FALSE;
	bool t6_valid = FALSE;

	// Solve for the joint variable theta 1
	// There will be 2 solutions based on the +/- in the square root
	float t1_1 = atan2(py, px) - atan2(d3, sqrt(pow(px, 2) + pow(py, 2) - pow(d3, 2)));
	float t1_2 = atan2(py, px) - atan2(d3, -sqrt(pow(px, 2) + pow(py, 2) - pow(d3, 2)));

	// Next solve for joint variable, theta 3
	// First we need to calculate K
	float K = (pow(px, 2) + pow(py, 2) + pow(pz, 2) - pow(a2, 2) - pow(a3, 2) - pow(d3, 2) - pow(d4, 2)) / (2 * a2);

	// First solution for t3
	// There will be 2 solutions based on the +/- in the square root
	float t3_1 = atan2(a3, d4) - atan2(K, sqrt(pow(a3, 2) + pow(d4, 2) - pow(K, 2)));
	float t3_2 = atan2(a3, d4) - atan2(K, -sqrt(pow(a3, 2) + pow(d4, 2) - pow(K, 2)));

	// Next solve for the joint variable theta 2
	// First we need to find theta_23
	// There are four possible values based on the combinations of theta 1 and theta 2
	// First for t1_1 and t3_1
	float c1 = cos(t1_1);
	float s1 = sin(t1_1);
	float c3 = cos(t3_1);
	float s3 = sin(t3_1);
	float t23_1 = atan2f((-a3 - a2*c3)*pz - (c1*px + s1*py)*(d4 - a2*s3), (a2*s3 - d4)*pz + (a3 + a2*c3)*(c1*px + s1*py));	

	// Second for t1_2 and t3_1
	float t23_2 = atan2((-a3 - a2*cos(t3_1))*pz - (cos(t1_2)*px + sin(t1_2)*py)*(d4 - a2*sin(t3_1)), (a2*sin(t3_1) - d4)*pz + (a3 + a2*cos(t3_1))*(cos(t1_2)*px + sin(t1_2)*py));
	// Third solution for t1_2 and t3_2
	float t23_3 = atan2((-a3 - a2*cos(t3_2))*pz - (cos(t1_1)*px + sin(t1_1)*py)*(d4 - a2*sin(t3_2)), (a2*sin(t3_2) - d4)*pz + (a3 + a2*cos(t3_2))*(cos(t1_1)*px + sin(t1_1)*py));
	// Fourth solution for t1_1 and t3_2
	float t23_4 = atan2((-a3 - a2*cos(t3_2))*pz - (cos(t1_2)*px + sin(t1_2)*py)*(d4 - a2*sin(t3_2)), (a2*sin(t3_2) - d4)*pz + (a3 + a2*cos(t3_2))*(cos(t1_2)*px + sin(t1_2)*py));

	// Now caclulate t2 from t23
	// There will be 4 different answers based on the values for t23
	// The same t3 value must be used to find t2 after calculating t23
	float t2_1 = t23_1 - t3_1;
	float t2_2 = t23_2 - t3_1;
	float t2_3 = t23_3 - t3_2;
	float t2_4 = t23_4 - t3_2;

	// Next calclulate the joint variables theta 4, theta 5, and theta 6
	// There will be 4 different solutions 1 for each of the values of t23
	// 4 more solutions can be found by adding 180 degrees
	// Define s1 = sin(t1) etc. to make notation easier
	// First solution for t4
	s1 = sin(t1_1);
	c1 = cos(t1_1);
	float c23 = cos(t23_1);
	float s23 = sin(t23_1);
	float t4_1 = atan2(-r13*s1 + r23*c1, -r13*c1*c23 - r23*s1*c23 + r33*s23);
	// First solution for theta 5
	float s4 = sin(t4_1);
	float c4 = cos(t4_1);
	float s5 = -(r13*(c1*c23*c4 + s1*s4) + r23*(s1*c23*c4 - c1*s4) - r33*s23*c4);
	float c5 = r13*(-c1*s23) + r23*(-s1*s23) + r33*(-c23);
	float t5_1 = atan2(s5, c5);
	// First solution for theta 6
	float s6 = -r11*(c1*c23*s4 - s1*c4) - r21*(s1*c23*s4 + c1*c4) + r31*(s23*s4);
	float c6 = r11*((c1*c23*c4 + s1*s4)*c5 - c1*s23*s5) + r21*((s1*c23*c4 - c1*s4)*c5 - s1*s23*s5) - r31*(s23*c4*c5 + c23*s5);
	float t6_1 = atan2(s6, c6);
	// Alternate solutions
	float t4_1_alt = t4_1 + PI;
	float t5_1_alt = -t5_1;
	float t6_1_alt = t6_1 + PI;

	// Second solution for t4
	s1 = sin(t1_2);
	c1 = cos(t1_2);
	c23 = cos(t23_2);
	s23 = sin(t23_2);
	float t4_2 = atan2(-r13*s1 + r23*c1, -r13*c1*c23 - r23*s1*c23 + r33*s23);
	// Second solution for theta 5
	s4 = sin(t4_2);
	c4 = cos(t4_2);
	s5 = -(r13*(c1*c23*c4 + s1*s4) + r23*(s1*c23*c4 - c1*s4) - r33*s23*c4);
	c5 = r13*(-c1*s23) + r23*(-s1*s23) + r33*(-c23);
	float t5_2 = atan2(s5, c5);
	// First solution for theta 6
	s6 = -r11*(c1*c23*s4 - s1*c4) - r21*(s1*c23*s4 + c1*c4) + r31*(s23*s4);
	c6 = r11*((c1*c23*c4 + s1*s4)*c5 - c1*s23*s5) + r21*((s1*c23*c4 - c1*s4)*c5 - s1*s23*s5) - r31*(s23*c4*c5 + c23*s5);
	float t6_2 = atan2(s6, c6);
	// Alternate solutions
	float t4_2_alt = t4_2 + PI;
	float t5_2_alt = -t5_2;
	float t6_2_alt = t6_2 + PI;

	// Third solution for t4
	s1 = sin(t1_1);
	c1 = cos(t1_1);
	c23 = cos(t23_3);
	s23 = sin(t23_3);
	float t4_3 = atan2(-r13*s1 + r23*c1, -r13*c1*c23 - r23*s1*c23 + r33*s23);
	// Third solution for theta 5
	s4 = sin(t4_3);
	c4 = cos(t4_3);
	s5 = -(r13*(c1*c23*c4 + s1*s4) + r23*(s1*c23*c4 - c1*s4) - r33*s23*c4);
	c5 = r13*(-c1*s23) + r23*(-s1*s23) + r33*(-c23);
	float t5_3 = atan2(s5, c5);
	// Third solution for theta 6
	s6 = -r11*(c1*c23*s4 - s1*c4) - r21*(s1*c23*s4 + c1*c4) + r31*(s23*s4);
	c6 = r11*((c1*c23*c4 + s1*s4)*c5 - c1*s23*s5) + r21*((s1*c23*c4 - c1*s4)*c5 - s1*s23*s5) - r31*(s23*c4*c5 + c23*s5);
	float t6_3 = atan2(s6, c6);
	// Alternate solutions
	float t4_3_alt = t4_3 + PI;
	float t5_3_alt = -t5_3;
	float t6_3_alt = t6_3 + PI;

	// Fourth solution for t4
	s1 = sin(t1_2);
	c1 = cos(t1_2);
	c23 = cos(t23_4);
	s23 = sin(t23_4);
	float t4_4 = atan2(-r13*s1 + r23*c1, -r13*c1*c23 - r23*s1*c23 + r33*s23);
	// Fourth solution for theta 5
	s4 = sin(t4_4);
	c4 = cos(t4_4);
	s5 = -(r13*(c1*c23*c4 + s1*s4) + r23*(s1*c23*c4 - c1*s4) - r33*s23*c4);
	c5 = r13*(-c1*s23) + r23*(-s1*s23) + r33*(-c23);
	float t5_4 = atan2(s5, c5);
	// First solution for theta 6
	s6 = -r11*(c1*c23*s4 - s1*c4) - r21*(s1*c23*s4 + c1*c4) + r31*(s23*s4);
	c6 = r11*((c1*c23*c4 + s1*s4)*c5 - c1*s23*s5) + r21*((s1*c23*c4 - c1*s4)*c5 - s1*s23*s5) - r31*(s23*c4*c5 + c23*s5);
	float t6_4 = atan2(s6, c6);
	// Alternate solutions
	float t4_4_alt = t4_4 + PI;
	float t5_4_alt = -t5_4;
	float t6_4_alt = t6_4 + PI;

	/*// Check if the solutions are valid
	// To check this we make sure all of the theta values are within the joint restrictions
	// Check if the first solution is valid
	if (0 <= t1_1 && t1_1 <= PI / 2 &&						// Theta 1 should be in between 0 and 90 degrees
		0 <= t2_1 && t2_1 <= PI / 2 &&						// Theta 2 should be between 0 and 90 degrees
		20 * PI / 180 <= t3_1 && t3_1 <= PI / 2 &&			// Theta 3 should be between 20 and 90 degrees
		0 <= t4_1 && t4_1 <= 3 * PI / 2 &&					// Theta 4 should be between 0 and 270 degrees
		PI / 6 <= t5_1 && t5_1 <= 130 * PI / 180 &&			// Theta 5 should be between 30 and 180 degrees
		0 <= t6_1 && t6_1 <= 5 * PI / 6) {

		// If all the theta values are within the joint angles, this solution is valid
		// return the theta values for this solution
		// Print solution 1 is valid on the screen for testing purposes
		joint_angles.t1 = t1_1;
		joint_angles.t2 = t2_1;
		joint_angles.t3 = t3_1;
		joint_angles.t4 = t4_1;
		joint_angles.t5 = t5_1;
		joint_angles.t6 = t6_1;
		return joint_angles;
	}

	// Check if the second solution is valid
	if (0 <= t1_2 && t1_2 <= PI / 2 &&						// Theta 1 should be in between 0 and 90 degrees
		0 <= t2_2 && t2_2 <= PI / 2 &&						// Theta 2 should be between 0 and 90 degrees
		20 * PI / 180 <= t3_1 && t3_1 <= PI / 2 &&			// Theta 3 should be between 20 and 90 degrees
		0 <= t4_2 && t4_2 <= 3 * PI / 2 &&					// Theta 4 should be between 0 and 270 degrees
		PI / 6 <= t5_2 && t5_2 <= 130 * PI / 180 &&			// Theta 5 should be between 30 and 180 degrees
		0 <= t6_2 && t6_2 <= 5 * PI / 6) {

		// If all the theta values are within the joint angles, this solution is valid
		// return the theta values for this solution
		// Print solution 2 is valid on the screen for testing purposes
		joint_angles.t1 = t1_2;
		joint_angles.t2 = t2_2;
		joint_angles.t3 = t3_1;
		joint_angles.t4 = t4_2;
		joint_angles.t5 = t5_2;
		joint_angles.t6 = t6_2;
		return joint_angles;
	}

	// Check if the third solution is valid
	if (0 <= t1_1 && t1_1 <= PI / 2 &&						// Theta 1 should be in between 0 and 90 degrees
		0 <= t2_3 && t2_3 <= PI / 2 &&						// Theta 2 should be between 0 and 90 degrees
		20 * PI / 180 <= t3_2 && t3_2 <= PI / 2 &&			// Theta 3 should be between 20 and 90 degrees
		0 <= t4_3 && t4_3 <= 3 * PI / 2 &&					// Theta 4 should be between 0 and 270 degrees
		PI / 6 <= t5_3 && t5_3 <= 130 * PI / 180 &&			// Theta 5 should be between 30 and 180 degrees
		0 <= t6_3 && t6_3 <= 5 * PI / 6) {

		// If all the theta values are within the joint angles, this solution is valid
		// return the theta values for this solution
		// Print solution 3 is valid on the screen for testing purposes
		joint_angles.t1 = t1_2;
		joint_angles.t2 = t2_3;
		joint_angles.t3 = t3_1;
		joint_angles.t4 = t4_3;
		joint_angles.t5 = t5_3;
		joint_angles.t6 = t6_3;
		return joint_angles;
	}

	// Check if the fourth solution is valid
	if (0 <= t1_2 && t1_2 <= PI / 2 &&						// Theta 1 should be in between 0 and 90 degrees
		0 <= t2_4 && t2_4 <= PI / 2 &&						// Theta 2 should be between 0 and 90 degrees
		20 * PI / 180 <= t3_2 && t3_2 <= PI / 2 &&			// Theta 3 should be between 20 and 90 degrees
		0 <= t4_4 && t4_4 <= 3 * PI / 2 &&					// Theta 4 should be between 0 and 270 degrees
		PI / 6 <= t5_4 && t5_4 <= 130 * PI / 180 &&			// Theta 5 should be between 30 and 180 degrees
		0 <= t6_4 && t6_4 <= 5 * PI / 6) {
		// If all the theta values are within the joint angles, this solution is valid
		// return the theta values for this solution
		// Print solution 4 is valid on the screen for testing purposes
		joint_angles.t1 = t1_2;
		joint_angles.t2 = t2_3;
		joint_angles.t3 = t3_2;
		joint_angles.t4 = t4_4;
		joint_angles.t5 = t5_4;
		joint_angles.t6 = t6_4;
		return joint_angles;
	}

	// Check if the fifth solution is valid
	if (0 <= t1_1 && t1_1 <= PI / 2 &&								// Theta 1 should be in between 0 and 90 degrees
		0 <= t2_1 && t2_1 <= PI / 2 &&								// Theta 2 should be between 0 and 90 degrees
		20 * PI / 180 <= t3_1 && t3_1 <= PI / 2 &&					// Theta 3 should be between 20 and 90 degrees
		0 <= t4_1_alt && t4_1_alt <= 3 * PI / 2 &&					// Theta 4 should be between 0 and 270 degrees
		PI / 6 <= t5_1_alt && t5_1_alt <= 130 * PI / 180 &&			// Theta 5 should be between 30 and 180 degrees
		0 <= t6_1_alt && t6_1_alt <= 5 * PI / 6) {

		// If all the theta values are within the joint angles, this solution is valid
		// return the theta values for this solution
		// Print solution 5 is valid on the screen for testing purposes
		joint_angles.t1 = t1_1;
		joint_angles.t2 = t2_1;
		joint_angles.t3 = t3_1;
		joint_angles.t4 = t4_1_alt;
		joint_angles.t5 = t5_1_alt;
		joint_angles.t6 = t6_1_alt;
		return joint_angles;
	}

	// Check if the sixth solution is valid
	if (0 <= t1_2 && t1_2 <= PI / 2 &&									// Theta 1 should be in between 0 and 90 degrees
		0 <= t2_2 && t2_2 <= PI / 2 &&									// Theta 2 should be between 0 and 90 degrees
		20 * PI / 180 <= t3_1 && t3_1 <= PI / 2 &&						// Theta 3 should be between 20 and 90 degrees
		0 <= t4_2_alt && t4_2_alt <= 3 * PI / 2 &&						// Theta 4 should be between 0 and 270 degrees
		PI / 6 <= t5_2_alt && t5_2_alt <= 130 * PI / 180 &&				// Theta 5 should be between 30 and 180 degrees
		0 <= t6_2 && t6_2 <= 5 * PI / 6) {
		// If all the theta values are within the joint angles, this solution is valid
		// return the theta values for this solution
		// Print solution 6 is valid on the screen for testing purposes
		joint_angles.t1 = t1_2;
		joint_angles.t2 = t2_2;
		joint_angles.t3 = t3_1;
		joint_angles.t4 = t4_2_alt;
		joint_angles.t5 = t5_2_alt;
		joint_angles.t6 = t6_2_alt;
		return joint_angles;
	}

	// Check if the seventh solution is valid
	if (0 <= t1_1 && t1_1 <= PI / 2 &&								// Theta 1 should be in between 0 and 90 degrees
		0 <= t2_3 && t2_3 <= PI / 2 &&								// Theta 2 should be between 0 and 90 degrees
		20 * PI / 180 <= t3_2 && t3_2 <= PI / 2 &&					// Theta 3 should be between 20 and 90 degrees
		0 <= t4_3_alt && t4_3_alt <= 3 * PI / 2 &&					// Theta 4 should be between 0 and 270 degrees
		PI / 6 <= t5_3_alt && t5_3_alt <= 130 * PI / 180 &&			// Theta 5 should be between 30 and 180 degrees
		0 <= t6_3_alt && t6_3_alt <= 5 * PI / 6) {
		// If all the theta values are within the joint angles, this solution is valid
		// return the theta values for this solution
		// Print solution 7 is valid on the screen for testing purposes
		joint_angles.t1 = t1_1;
		joint_angles.t2 = t2_1;
		joint_angles.t3 = t3_2;
		joint_angles.t4 = t4_3_alt;
		joint_angles.t5 = t5_3_alt;
		joint_angles.t6 = t6_3_alt;
		return joint_angles;
	}

	// Check if the eigth solution is valid
	if (0 <= t1_2 && t1_2 <= PI / 2 &&								// Theta 1 should be in between 0 and 90 degrees
		0 <= t2_4 && t2_4 <= PI / 2 &&								// Theta 2 should be between 0 and 90 degrees
		20 * PI / 180 <= t3_2 && t3_2 <= PI / 2 &&					// Theta 3 should be between 20 and 90 degrees
		0 <= t4_4_alt && t4_4_alt <= 3 * PI / 2 &&					// Theta 4 should be between 0 and 270 degrees
		PI / 6 <= t5_4_alt && t5_4_alt <= 130 * PI / 180 &&			// Theta 5 should be between 30 and 180 degrees
		0 <= t6_4_alt && t6_4_alt <= 5 * PI / 6) {
		// If all the theta values are within the joint angles, this solution is valid
		// return the theta values for this solution
		joint_angles.t1 = t1_2;
		joint_angles.t2 = t2_4;
		joint_angles.t3 = t3_2;
		joint_angles.t4 = t4_4_alt;
		joint_angles.t5 = t5_4_alt;
		joint_angles.t6 = t6_4_alt;
		return joint_angles;
	}*/

	/////////////////////////////////////////
	// Check the theta values individually
	// check for a valid solution for theta 1
	if (0 <= t1_1 && t1_1 <= 2*PI) {
		joint_angles.t1 = t1_1;
		t1_valid = TRUE;

	}

	// check the second solution for t1 if t1 has not already been assigned
	if (0 <= t1_2 && t1_2 <=2 *PI && t1_valid == FALSE) {
		joint_angles.t1 = t1_2;
		t1_valid = TRUE;

	}

	// Check the positive values this corresponds with the rotation function in the openGL code
	if (0 <= abs(t1_1) && abs(t1_1) <= 2 * PI) {
		joint_angles.t1 = t1_1;
		t1_valid = TRUE;

	}

	// check the second solution for t1 if t1 has not already been assigned
	if (0 <= abs(t1_2) && abs(t1_2) <= 2 * PI && t1_valid == FALSE) {
		joint_angles.t1 = t1_2;
		t1_valid = TRUE;

	}


	////////////////////////////////////////////////////////////////////////
	// Check first solution for t2 to see if a valid solution for t2 exists
	if (0 <= t2_1 && t2_1 <= PI / 2 && t2_valid == FALSE) {
		joint_angles.t2 = t2_1;
		t2_valid = TRUE;
	}

	// Check second solution for t2 to see if a valid solution for t2 exists
	if (0 <= t2_2 && t2_2 <= PI / 2 && t2_valid == FALSE) {
		joint_angles.t2 = t2_2;
		t2_valid = TRUE;
	}

	// Check if the third solution for t2 to see if a valid solution for t2 exists
	if (0 <= t2_3 && t2_3 <= PI / 2 && t2_valid == FALSE) {
		joint_angles.t2 = t2_3;
		t2_valid = TRUE;
	}

	// Check the fourth solution for t2 to see if a valid solution for t2 exists
	if (0 <= t2_4 && t2_4 <= PI / 2 && t2_valid == FALSE) {
		joint_angles.t2 = t2_4;
		t2_valid = TRUE;
	}

	////////////////////////////////////////////////////////////////////////
	// Check first solution for t2 to see if a valid solution for t2 exists
	if (0 <= abs(t2_1) && abs(t2_1) <= PI / 2 && t2_valid == FALSE) {
		joint_angles.t2 = t2_1;
		t2_valid = TRUE;
	}

	// Check second solution for t2 to see if a valid solution for t2 exists
	if (0 <= abs(t2_2) && abs(t2_2) <= PI / 2 && t2_valid == FALSE) {
		joint_angles.t2 = t2_2;
		t2_valid = TRUE;
	}

	// Check if the third solution for t2 to see if a valid solution for t2 exists
	if (0 <= abs(t2_3) && abs(t2_3) <= PI / 2 && t2_valid == FALSE) {
		joint_angles.t2 = t2_3;
		t2_valid = TRUE;
	}

	// Check the fourth solution for t2 to see if a valid solution for t2 exists
	if (0 <= abs(t2_4) && abs(t2_4) <= PI / 2 && t2_valid == FALSE) {
		joint_angles.t2 = t2_4;
		t2_valid = TRUE;
	}

	/////////////////////////////////////////////////
	// check if a valid solution for t3 exists
	if (20 *PI/180 <= t3_1 && t3_1 <= PI / 2 && t3_valid == FALSE) {
		joint_angles.t3 = t3_1;
		t3_valid = TRUE;

	}

	// check the second solution for t3
	if (20 * PI / 180 <= t3_2 && t3_2 <= PI / 2 && t3_valid == FALSE) {
		joint_angles.t3 = t3_1;
		t3_valid = TRUE;

	}

	// check if a valid solution for t3 exists
	if (0 <= abs(t3_1) && t3_1 <= PI && t3_valid == FALSE) {
		joint_angles.t3 = t3_1;
		t3_valid = TRUE;

	}

	// check the second solution for t3
	if (0 <= abs(t3_2) && abs(t3_2) <= PI && t3_valid == FALSE) {
		joint_angles.t3 = t3_1;
		t3_valid = TRUE;

	}

	//////////////////////////////////////////////
	// Check to see if theta 4 is a valid solution
	if (0 <= t4_1 && t4_1 <= 3 * PI / 2 && t4_valid == FALSE) {
		joint_angles.t4 = t4_1;
		t4_valid = TRUE;

	}

	// Check to see if theta 4_2 is a valid solution
	if (0 <= t4_2 && t4_2 <= 3 * PI / 2 && t4_valid == FALSE) {
		joint_angles.t4 = t4_2;
		t4_valid = TRUE;
	}

	// Check to see if theta 4_3 is a valid solution
	if (0 <= t4_3 && t4_3 <= 3 * PI / 2 && t4_valid == FALSE) {
		joint_angles.t4 = t4_3;
		t4_valid = TRUE;
	}

	// Check to see if theta 4_4 is a valid solution
	if (0 <= t4_4 && t4_4 <= 3 * PI / 2 && t4_valid == FALSE) {
		joint_angles.t4 = t4_4;
		t4_valid = TRUE;
	}

	// Check to see if theta 4_1_alt is a valid solution
	if (0 <= t4_1_alt && t4_1_alt <= 3 * PI / 2 && t4_valid == FALSE) {
		joint_angles.t4 = t4_1_alt;
		t4_valid = TRUE;
	}
	// Check to see if theta 4_2_alt is a valid solution
	if (0 <= t4_2_alt && t4_2_alt <= 3 * PI / 2 && t4_valid == FALSE) {
		joint_angles.t4 = t4_2_alt;
		t4_valid = TRUE;
	}
	// Check to see if theta 4_3_alt is a valid solution
	if (0 <= t4_3_alt && t4_3_alt <= 3 * PI / 2 && t4_valid == FALSE) {
		joint_angles.t4 = t4_3_alt;
		t4_valid = TRUE;
	}
	// Check to see if theta 4_4_alt is a valid solution
	if (0 <= t4_4_alt && t4_4_alt <= 3 * PI / 2 && t4_valid == FALSE) {
		joint_angles.t4 = t4_4_alt;
		t4_valid = TRUE;
	}

	//////////////////////////////////////////////
	// Check to see if theta 5 is a valid solution
	if (PI / 6 <= t5_1 && t5_1 <= 130 * PI / 180 && t5_valid == FALSE) {
		joint_angles.t5 = t5_1;
		t5_valid = TRUE;

	}

	// Check to see if theta 5_2 is a valid solution
	if (PI / 6 <= t5_2 && t5_2 <= 130 * PI / 180 && t5_valid == FALSE) {
		joint_angles.t5 = t5_2;
		t5_valid = TRUE;

	}
	// Check to see if theta 5_3 is a valid solution
	if (PI / 6 <= t5_3 && t5_3 <= 130 * PI / 180 && t5_valid == FALSE) {
		joint_angles.t5 = t5_3;
		t5_valid = TRUE;

	}
	// Check to see if theta 5_4 is a valid solution
	if (PI / 6 <= t5_4 && t5_4 <= 130 * PI / 180 && t5_valid == FALSE) {
		joint_angles.t5 = t5_4;
		t5_valid = TRUE;

	}
	// Check to see if theta 5_1_alt is a valid solution
	if (PI / 6 <= t5_1_alt && t5_1_alt <= 130 * PI / 180 && t5_valid == FALSE) {
		joint_angles.t5 = t5_1_alt;
		t5_valid = TRUE;

	}
	// Check to see if theta 5_2_alt is a valid solution
	if (PI / 6 <= t5_2_alt && t5_2_alt <= 130 * PI / 180 && t5_valid == FALSE) {
		joint_angles.t5 = t5_2_alt;
		t5_valid = TRUE;

	}
	// Check to see if theta 5_3_alt is a valid solution
	if (PI / 6 <= t5_3_alt && t5_3_alt <= 130 * PI / 180 && t5_valid == FALSE) {
		joint_angles.t5 = t5_3_alt;
		t5_valid = TRUE;

	}
	// Check to see if theta 5_4_alt is a valid solution
	if (PI / 6 <= t5_4_alt && t5_4_alt <= 130 * PI / 180 && t5_valid == FALSE) {
		joint_angles.t5 = t5_4_alt;
		t5_valid = TRUE;

	}

	//////////////////////////////////////////////
	// Check to see if theta 6 is a valid solution
	if (0 <= t6_1 && t6_valid == FALSE) {
		joint_angles.t6 = t6_1;
		t6_valid = TRUE;

	}

	// Check to see if theta 6_2 is a valid solution
	if (0 <= t6_2 && t6_valid == FALSE) {
		joint_angles.t6 = t6_2;
		t6_valid = TRUE;

	}


	// Check to see if theta 6_3 is a valid solution
	if (0 <= t6_3  && t6_valid == FALSE) {
		joint_angles.t6 = t6_3;
		t6_valid = TRUE;
	}


	// Check to see if theta 6_4 is a valid solution
	if (0 <= t6_4 && t6_valid == FALSE) {
		joint_angles.t6 = t6_4;
		t6_valid = TRUE;

	}

	// Check to see if theta 6_1_alt is a valid solution
	if (0 <= t6_1_alt && t6_valid == FALSE) {
		joint_angles.t6 = t6_1_alt;
		t6_valid = TRUE;

	}

	// Check to see if theta 6_2_alt is a valid solution
	if (0 <= t6_2_alt && t6_valid == FALSE) {
		joint_angles.t6 = t6_2_alt;
		t6_valid = TRUE;

	}

	// Check to see if theta 6_3_alt is a valid solution
	if (0 <= t6_3_alt && t6_valid == FALSE) {
		joint_angles.t6 = t6_3_alt;
		t5_valid = TRUE;

	}

	// Check to see if theta 6_4_alt is a valid solution
	if (0 <= t6_4_alt  && t6_valid == FALSE) {
		joint_angles.t6 = t6_4_alt;
		t5_valid = TRUE;

	}

	// Check to see if a valid solution exists if it does, return the joint angles otherwise, return all negative values for the joint angles
	if (t1_valid == TRUE && t2_valid == TRUE && t3_valid == TRUE && t4_valid == TRUE && t5_valid == TRUE && t6_valid == TRUE) {

		// convert to degrees before returning
		joint angle_degrees;
		angle_degrees.t1 = 180 / PI * joint_angles.t1;
		angle_degrees.t2 = 180 / PI * joint_angles.t2;
		angle_degrees.t3 = 180 / PI * joint_angles.t3;
		angle_degrees.t4 = 180 / PI * joint_angles.t4;
		angle_degrees.t5 = 180 / PI * joint_angles.t5;
		angle_degrees.t6 = 180 / PI * joint_angles.t6;
		return angle_degrees;
	}

	// If no valid solution exist, return all negative values for the joint angles
	else {

		// Print no solution is valid for testing purposes
		joint joint_angles;
		joint_angles.t1 = -10;
		joint_angles.t2 = -10;
		joint_angles.t3 = -10;
		joint_angles.t4 = -10;
		joint_angles.t5 = -10;
		joint_angles.t6 = -10;
		return joint_angles;
	}

}


// Forward kinematics function
// Takes in a variable of type joint which contains all of the theta values 1 throuhg 6
// Outputs a transformation matrix using the forward kinematic equations
transform ForwardKin(joint angles) {
	
	// Define link parameters
	float a0 = 0;
	float alpha0 = 0;
	float d1 = 0;
	float a1 = 0;
	float alpha1 = -PI / 2;
	float d2 = 0;
	float a2 = 10;
	float alpha2 = 0;
	float d3 = 0.5;
	float a3 = 3.5;
	float alpha3 = -PI / 2;
	float d4 = 5.5;

	// Start by assigning the angles from the joint variable to the theta values to be used in the calculations
	float t1 = PI / 180 * angles.t1;
	float t2 = PI / 180 * angles.t2;
	float t3 = PI / 180 * angles.t3;
	float t4 = PI / 180 * angles.t4;
	float t5 = PI / 180 * angles.t5;
	float t6 = PI / 180 * angles.t6;
	
	// Start by defining the angles needed
	float c1 = cos(t1);
	float s1 = sin(t1);
	float c2 = cos(t2);
	float s2 = sin(t2);
	float c3 = cos(t3);
	float s3 = sin(t3);
	float c4 = cos(t4);
	float s4 = sin(t4);
	float c5 = cos(t5);
	float s5 = sin(t5);
	float c6 = cos(t6);
	float s6 = sin(t6);
	float c23 = c2*c3 - s2*s3;
	float s23 = c2*s3 + s2*c3;

	// Test these values using the forward kinematic equations
	float r11 = c1*(c23*(c4*c5*c6 - s4*s6) - s23*s5*c6) + s1*(s4*c5*c6 + c4*s6);

	float r21 = s1*(c23*(c4*c5*c6 - s4*s6) - s23*s5*c6) - c1*(s4*c5*c6 + c4*s6);

	float r31 = -s23*(c4*c5*c6 - s4*s6) - c23*s5*c6;

	float r12 = c1*(c23*(-c4*c5*s6 - s4*c6) + s23*s5*s6) + s1*(c4*c6 - s4*c5*s6);

	float r22 = s1*(c23*(-c4*c5*s6 - s4*c6) + s23*s5*s6) - c1*(c4*c6 - s4*c5*s6);

	float r32 = -s23*(-c4*c5*s6 - s4*c6) + c23*s5*s6;

	float r13 = -c1*(c23*c4*s5 + s23*c5) - s1*s4*s5;

	float r23 = -s1*(c23*c4*s5 + s23*c5) - c1*s4*s5;

	float r33 = s23*c4*s5 - c23*c5;

	float px = c1*(a2*c2 + a3*c23 - d4*s23) - d3*s1;

	float py = s1*(a2*c2 + a3*c23 - d4*s23) + d3*c1;

	float pz = -a3*s23 - a2*s2 - d4*c23;

	// Define the transformation matrix to send to the inverse kinematics
	transform matrix;
	matrix.px = px;
	matrix.py = py;
	matrix.pz = pz;
	matrix.r11 = r11;
	matrix.r12 = r12;
	matrix.r13 = r13;
	matrix.r21 = r21;
	matrix.r22 = r22;
	matrix.r23 = r23;
	matrix.r31 = r31;
	matrix.r32 = r32;
	matrix.r33 = r33;

	// Return the matrix
	return matrix;

}




