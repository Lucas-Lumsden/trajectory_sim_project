#include <iostream>
#include <cmath>
#include "objects.h"
#include "math.h"
#include <fstream>

int main (){

    constexpr vec1 rocket1_start_pos{

        .x = 0,
        .y = 0

    };

    double mass;
    double force;
    double angle;
    double bt;
    double dcoef;
    double area;
    double vel;
    double airden;

    std::cout << "Input mass of object in kilograms: ";
    std::cin >> mass;
    std::cout << "Input force in newtons: ";
    std::cin >> force;
    std::cout << "Input angle of force in degrees: ";
    std::cin >> angle;
    std::cout << "Input burn time in seconds: ";
    std::cin >> bt;
    std::cout << "Input drag coefficient: ";
    std::cin >> dcoef;
    std::cout << "Input frontal area: ";
    std::cin >> area;

std::ofstream save("C:\\Users\\lucge\\physics_sim_project\\sim_params.txt");
if(!save.is_open()){
    std::cout << "Failed to save params!" << std::endl;
} else {
    save << mass << "\n" << force << "\n" << angle << "\n" 
         << bt << "\n" << dcoef << "\n" << area;
    save.close();
    std::cout << "Params saved." << std::endl;
}

    double fx = calc_x(force, angle);
    double fy = calc_y(force, angle);

    double ax = calc_ax(fx, mass);
    double ay = calc_ay(fy, mass);

    double accel = calc_accel(force, mass);

    double t = 0.0;
    double dt = 0.1;
    double dur = 600;

    double vx = 0.0, vy = 0.0;
    double px = 0.0, py = 0.0;
    
    vel = sqrt((vx * vx) + (vy * vy));


    rocket rocket1;
    rocket1.init_rocket(rocket1_start_pos.x, rocket1_start_pos.y);
    rocket rocket2;
    
    while(t <= dur){
        t += dt;

        if(t >= bt){
            ax = 0.0;
            ay = -9.8;
        }

        if(py < 1000){
            airden = 1.225;
        } else if(py >= 1000){
                airden = 1.112;
        } else if(py >= 2000){
                airden = 1.007;
        } else if(py >= 5000){
                airden = 10.736;
        } else if(py >= 10000){
                airden = 10.413;
        } else if(py >= 11000){
                airden = 0.364;
        } else if(py >= 20000){
                airden = 0.088;
        } else if(py >= 30000){
                airden = 0.018;
        } else if (py >= 84,852){
                airden = 0.000007;
        }
        
        vx += ax * dt;
        vy += ay * dt;

        if(vel > 0){
        double drag = calc_drag(vel, airden, dcoef, area);
        ax -= (drag / mass) * (vx / vel);
        ay -= (drag / mass) * (vy /vel);
        }

        px += vx * dt;
        py += vy * dt;

        rocket1.set_pos(px, py);
        vec1 pos = rocket1.get_pos();


        std::cout << "t: " << t << "s x: " << px << "m y: " << py << "m\n";

        if(py <= 0 && t > dt) break;

    }

    std::cout << "x intital: " << rocket1_start_pos.x << "m\ny intial: " << rocket1_start_pos.y << "m\n";
    std::cout << "Force x: " << fx << "N\nForce y: " << fy << "N\n";
    std::cout << "Acceleration: " << accel << "m/s^2";

    return 0;
}