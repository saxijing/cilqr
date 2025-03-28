#ifndef COMMON_H_
#define COMMON_H_

#include<iostream>
#include<vector>
#include<cmath>
#include<string>
using namespace std;

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
    double wheel_base;
    double wheel_track;
};

struct ObsInfo
{
    int id;
    string name;
    vector<ObjState> predicted_states;
    ObjSize size;
};

#endif