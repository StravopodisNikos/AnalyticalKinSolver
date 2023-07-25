#ifndef anthropomorphic_robot_H
#define anthropomorphic_robot_H

#include <cmath>

namespace anthropomorphic {

class anthopomorphic_robot
{
private:
    float _a[3]; // link lengths

public:
    anthopomorphic_robot(/* args */);
    ~anthopomorphic_robot();

    float q[3]; // position variables
};

anthopomorphic_robot::anthopomorphic_robot(/* args */)
{
}

anthopomorphic_robot::~anthopomorphic_robot()
{
}

anthopomorphic_robot::FKP() {

}

}
#endif //anthropomorphic_robot_H
