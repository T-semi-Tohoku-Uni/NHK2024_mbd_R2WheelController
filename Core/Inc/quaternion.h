/*
 * quarternion.h
 *
 *  Created on: May 28, 2024
 *      Author: YYcri
 */

#ifndef INC_QUATERNION_H_
#define INC_QUATERNION_H_

typedef struct{
	double w;
	double x;
	double y;
	double z;
}quaternion;

quaternion quaternion_multiply(quaternion q1, quaternion q2);
quaternion quaternion_conjugate(quaternion q);
double quaternion_norm(quaternion q);
quaternion quaternion_inverse(quaternion q);
quaternion quaternion_normalize(quaternion q);
double quaternion_to_heading(quaternion q);


#endif /* INC_QUATERNION_H_ */
