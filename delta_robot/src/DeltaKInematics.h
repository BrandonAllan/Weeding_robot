#ifndef DELTAKINEMATICS_H
#define DELTAKINEMATICS_H

struct ForwardResult {
    float x;
    float y;
    float z;
};

struct InverseResult {
    float theta1;
    float theta2;
    float theta3;
};

class DeltaKinematics {
public:
    DeltaKinematics(float _e, float _f, float _re, float _rf);

    ForwardResult forward(float theta1, float theta2, float theta3);
    InverseResult inverse(float x0, float y0, float z0);

private:
    float e; // end effector
    float f; // base
    float re;
    float rf;

    const float sqrt3 = 1.73205; // sqrt(3.0);
    const float pi = 3.141592653; // PI
    const float sin120 = 0.86602; // sqrt3/2.0;
    const float cos120 = -0.5;
    const float tan60 = 1.73205;
    const float sin30 = 0.5;
    const float tan30 = 0.57735; // 1.0/sqrt3;

    int delta_calcAngleYZ(float x0, float y0, float z0, float* theta);
};

#endif
