/**
 * @file kinematics.h
 * @brief Forward and Closed-Form Inverse Kinematics for 4-DOF Planar-Articulated Manipulator
 * @author Md. Al-Imran
 */

#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Arduino.h>
#include "config.h"

struct JointAngles {
    float theta1; // Base azimuth (deg)
    float theta2; // Shoulder pitch (deg)
    float theta3; // Elbow pitch (deg)
    float theta4; // Wrist / gripper pitch (deg)
    bool reachable;
};

struct CartesianPoint {
    float x; // X position (mm)
    float y; // Y position (mm)
    float z; // Z position (mm)
    float phi; // End-effector pitch approach angle (rad)
};

class RobotKinematics {
public:
    RobotKinematics();

    // Closed-form geometric inverse kinematics solving theta1, theta2, theta3, theta4
    JointAngles computeInverseKinematics(float Px, float Py, float Pz, float phiDeg = 0.0f);

    // Forward kinematics validation checking Cartesian end-effector coordinates
    CartesianPoint computeForwardKinematics(float theta1Deg, float theta2Deg, float theta3Deg, float theta4Deg);

private:
    float d1;
    float a2;
    float a3;
    float a4;

    static float radToDeg(float rad);
    static float degToRad(float deg);
};

#endif // KINEMATICS_H
