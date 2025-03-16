#ifndef COMMON_H_
#define COMMON_H_

struct ObjState
{
    double x;
    double y;
    double theta;
    double v;
    double accel;
    double yaw_rate;
};

struct CtrlInput
{
    double accel;
    double yaw_rate;
};

struct ObjSize
{
    double length;
    double width;
    double height;
};

#endif