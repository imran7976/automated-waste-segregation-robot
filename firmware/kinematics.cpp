/**
 * @file kinematics.cpp
 * @brief Implementation of D-H closed-form inverse kinematics equations (4)-(10)
 * @author Md. Al-Imran
 */

#include "kinematics.h"
#include <math.h>

RobotKinematics::RobotKinematics() 
    : d1(DH_D1), a2(DH_A2), a3(DH_A3), a4(DH_A4) {}

float RobotKinematics::radToDeg(float rad) {
    return rad * (180.0f / (float)M_PI);
}

float RobotKinematics::degToRad(float deg) {
    return deg * ((float)M_PI / 180.0f);
}

JointAngles RobotKinematics::computeInverseKinematics(float Px, float Py, float Pz, float phiDeg) {
    JointAngles solution;
    solution.reachable = false;

    float phiRad = degToRad(phiDeg);

    // Eq. (4): Base Azimuth Angle
    float theta1Rad = atan2(Py, Px);

    // Radial distance in XY ground plane
    float R_ground = sqrt(Px * Px + Py * Py);

    // Eq. (5) & Eq. (6): Planar wrist center coordinates
    float r = R_ground - a4 * cos(phiRad);
    float z_prime = Pz - d1 - a4 * sin(phiRad);

    // Law of cosines for planar two-link system (Eq. 7)
    float D_sq = r * r + z_prime * z_prime;
    float cosTheta3 = (D_sq - a2 * a2 - a3 * a3) / (2.0f * a2 * a3);

    // Workspace boundary check
    if (cosTheta3 < -1.0f || cosTheta3 > 1.0f) {
        // Target is outside reachable physical envelope
        solution.theta1 = 0;
        solution.theta2 = 0;
        solution.theta3 = 0;
        solution.theta4 = 0;
        return solution;
    }

    // Eq. (8): Elbow Joint Angle (Elbow-down configuration)
    float sinTheta3 = sqrt(1.0f - cosTheta3 * cosTheta3);
    float theta3Rad = atan2(sinTheta3, cosTheta3);

    // Eq. (9): Shoulder Joint Angle
    float alpha = atan2(z_prime, r);
    float beta = atan2(a3 * sin(theta3Rad), a2 + a3 * cos(theta3Rad));
    float theta2Rad = alpha - beta;

    // Eq. (10): Wrist Orientation Angle
    float theta4Rad = phiRad - (theta2Rad + theta3Rad);

    // Convert radians to degrees
    solution.theta1 = radToDeg(theta1Rad);
    solution.theta2 = radToDeg(theta2Rad);
    solution.theta3 = radToDeg(theta3Rad);
    solution.theta4 = radToDeg(theta4Rad);
    solution.reachable = true;

    return solution;
}

CartesianPoint RobotKinematics::computeForwardKinematics(float theta1Deg, float theta2Deg, float theta3Deg, float theta4Deg) {
    float th1 = degToRad(theta1Deg);
    float th2 = degToRad(theta2Deg);
    float th3 = degToRad(theta3Deg);
    float th4 = degToRad(theta4Deg);

    float phi = th2 + th3 + th4;

    // Planar forward kinematics projection
    float r_wrist = a2 * cos(th2) + a3 * cos(th2 + th3);
    float z_wrist = d1 + a2 * sin(th2) + a3 * sin(th2 + th3);

    float r_total = r_wrist + a4 * cos(phi);
    float z_total = z_wrist + a4 * sin(phi);

    CartesianPoint pt;
    pt.x = r_total * cos(th1);
    pt.y = r_total * sin(th1);
    pt.z = z_total;
    pt.phi = phi;

    return pt;
}
