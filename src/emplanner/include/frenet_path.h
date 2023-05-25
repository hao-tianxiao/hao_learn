/*************************************************************************
	> File Name: frenet_path.h
	> Author: TAI Lei
	> Mail: ltai@ust.hk
	> Created Time: Tue Apr  9 17:08:22 2019
 ************************************************************************/

#ifndef _FRENET_PATH_H
#define _FRENET_PATH_H

#include<iostream>
#include<vector>
#include<array>
#include<string>
#include<iterator>

#ifndef _CPPROBOTICS_TYPES_H
#define _CPPROBOTICS_TYPES_H

namespace cpprobotics{

using Vec_f=std::vector<float>;
using Poi_f=std::array<float, 2>;
using Vec_Poi=std::vector<Poi_f>;

};

#endif

struct Point {
    double x;
    double y;
    double l;
    double r;
    double s;
    double theta;
};

struct State{
        double x = 0;          // m
        double y = 0;          // m
        double yaw = 0;        // degree
        double speed = 0;      // m/s
        double yawrate = 0;
    };

namespace cpprobotics{

class FrenetPath{
public:
  float cd = 0.0;
  float cv = 0.0;
  float cf = 0.0;

  Vec_f t;
  Vec_f d;
  Vec_f d_d;
  Vec_f d_dd;
  Vec_f d_ddd;
  Vec_f s;
  Vec_f s_d;
  Vec_f s_dd;
  Vec_f s_ddd;

  Vec_f x;
  Vec_f y;
  Vec_f yaw;
  Vec_f ds;
  Vec_f c;

  float max_speed;
  float max_accel;
  float max_curvature;
};

using Vec_Path=std::vector<FrenetPath>;
}
#endif
