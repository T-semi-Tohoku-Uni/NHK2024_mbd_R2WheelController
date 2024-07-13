/*
 * quaternion.c
 *
 *  Created on: May 28, 2024
 *      Author: YYcri
 */
#include "quaternion.h"

quaternion quaternion_multiply(quaternion q1, quaternion q2) {
    quaternion result;
    result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
    result.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
    result.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
    result.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
    return result;
}

// クォータニオンの共役
quaternion quaternion_conjugate(quaternion q) {
    quaternion result;
    result.w = q.w;
    result.x = -q.x;
    result.y = -q.y;
    result.z = -q.z;
    return result;
}

// クォータニオンのノルム
double quaternion_norm(quaternion q) {
    return sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
}

// クォータニオンの逆元
quaternion quaternion_inverse(quaternion q) {
    double norm = quaternion_norm(q);
    quaternion conjugate = quaternion_conjugate(q);
    quaternion result;
    result.w = conjugate.w / (norm * norm);
    result.x = conjugate.x / (norm * norm);
    result.y = conjugate.y / (norm * norm);
    result.z = conjugate.z / (norm * norm);
    return result;
}

// クォータニオンの正規化
quaternion quaternion_normalize(quaternion q) {
    double norm = quaternion_norm(q);
    quaternion result;
    result.w = q.w / norm;
    result.x = q.x / norm;
    result.y = q.y / norm;
    result.z = q.z / norm;
    return result;
}

double quaternion_to_heading(quaternion q) {
    // yaw (z軸回転)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    double heading = atan2(siny_cosp, cosy_cosp);

    return heading;
}

