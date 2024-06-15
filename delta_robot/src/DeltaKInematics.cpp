#include "DeltaKinematics.h"
#include <cmath>
#include <cstdio>

DeltaKinematics::DeltaKinematics(float _e, float _f, float _re, float _rf)
    : e(_e), f(_f), re(_re), rf(_rf) {}

ForwardResult DeltaKinematics::forward(float theta1, float theta2, float theta3) {
    float x0, y0, z0;
    float t = (f - e) * tan30 / 2;

    float y1 = -(t + rf * cos(theta1));
    float z1 = -rf * sin(theta1);

    float y2 = (t + rf * cos(theta2)) * sin30;
    float x2 = y2 * tan60;
    float z2 = -rf * sin(theta2);

    float y3 = (t + rf * cos(theta3)) * sin30;
    float x3 = -y3 * tan60;
    float z3 = -rf * sin(theta3);

    float dnm = (y2 - y1) * x3 - (y3 - y1) * x2;

    float w1 = y1 * y1 + z1 * z1;
    float w2 = x2 * x2 + y2 * y2 + z2 * z2;
    float w3 = x3 * x3 + y3 * y3 + z3 * z3;

    float a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1);
    float b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0;

    float a2 = -(z2 - z1) * x3 + (z3 - z1) * x2;
    float b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0;

    float a = a1 * a1 + a2 * a2 + dnm * dnm;
    float b = 2 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm * dnm);
    float c = (b2 - y1 * dnm) * (b2 - y1 * dnm) + b1 * b1 + dnm * dnm * (z1 * z1 - re * re);

    // discriminant
    float d = b * b - 4.0f * a * c;
    if (d < 0) {
        perror("Non existing point"); // non-existing point
        // Return some error code or handle it appropriately
    }

    z0 = -0.5f * (b + sqrt(d)) / a;
    x0 = (a1 * z0 + b1) / dnm;
    y0 = (a2 * z0 + b2) / dnm;

    ForwardResult result;
    result.x = x0;
    result.y = y0;
    result.z = z0;

    return result;
}

int DeltaKinematics::delta_calcAngleYZ(float x0, float y0, float z0, float* theta) {
    float y1 = -0.5 * 0.57735f * f; // f/2 * tg 30
    y0 -= 0.5 * 0.57735f * e; // shift center to edge
    // z = a + b*y
    float a = (x0 * x0 + y0 * y0 + z0 * z0 + rf * rf - re * re - y1 * y1) / (2 * z0);
    float b = (y1 - y0) / z0;
    // discriminant
    float d = -(a + b * y1) * (a + b * y1) + rf * (b * b * rf + rf);
    if (d < 0) return -1; // non-existing point
    float yj = (y1 - a * b - sqrt(d)) / (b * b + 1); // choosing outer point
    float zj = a + b * yj;
    *theta = 180.0f * atan(-zj / (y1 - yj)) / pi + ((yj > y1) ? 180.0f : 0.0f);
    return 0;
}

InverseResult DeltaKinematics::inverse(float x0, float y0, float z0) {
    float t1, t2, t3 = 0;
    float t = (f - e) * tan30 / 2;
    float dtr = pi / 180.0f;

    int status = delta_calcAngleYZ(x0, y0, z0, &t1);
    if (status == 0) {
        status = delta_calcAngleYZ(x0 * cos120 + y0 * sin120, y0 * cos120 - x0 * sin120, z0, &t2); // rotate coords to +120 deg
    }
    if (status == 0) {
        status = delta_calcAngleYZ(x0 * cos120 - y0 * sin120, y0 * cos120 + x0 * sin120, z0, &t3); // rotate coords to -120 deg
    }

    InverseResult result;
    result.theta1 = t1;
    result.theta2 = t2;
    result.theta3 = t3;

    return result;
}
