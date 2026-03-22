#pragma once
double calc_x(double force, double angle);
double calc_y(double force, double angle);
double calc_accel(double mass, double force);
double calc_vel(double vi, double accel, double dt);
double calc_pos(double xi, double vi, double dt);
double calc_ax(double fx, double mass);
double calc_ay(double fy, double mass);
double calc_drag(double vel, double airden, double dcoef, double area);