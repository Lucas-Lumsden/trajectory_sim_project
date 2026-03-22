#include <iostream>
#include <cmath>
#include "objects.h"
#include "math.h"

double calc_x(double force, double angle){

    double rad = angle * 0.0174533;
    return force * std::cos(rad);

};

double calc_y(double force, double angle){
    
    double rad = angle * 0.0174533;
    return force * std::sin(rad);

};

double calc_vel(double vi, double accel, double dt){

    return vi + accel * dt;
};

double calc_pos(double xi, double vi, double dt){

    return xi + vi * dt;
};

double calc_accel(double force, double mass){
    double accel;
    return accel = force / mass;

};

double calc_ax(double fx, double mass){

    return fx / mass;

};

double calc_ay(double fy, double mass){

    return (fy / mass) - 9.8;

}

double calc_drag(double vel, double airden, double dcoef, double area){

    return 0.5 * airden * (vel * vel) * dcoef * area;

}
