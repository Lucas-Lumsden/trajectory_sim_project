#include <iostream>
#include <cmath>

struct vec1{
    float x, y;
};

class rocket{

    public:
    void init_rocket (float start_pos_x, float start_pos_y){

        pos.x = start_pos_x;
        pos.y = start_pos_y;

    }

    void set_pos(double new_x, double new_y){

        pos.x += new_x;
        pos.y += new_y;

    };

    vec1 get_pos(){
        return pos;
    }

    private:
    vec1 pos;
    vec1 vel;
    vec1 acc;

    float heading;

};