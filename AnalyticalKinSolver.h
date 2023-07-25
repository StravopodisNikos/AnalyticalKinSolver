#ifndef AnalyticalKinSolver_H
#define AnalyticalKinSolver_H

#include "Arduino.h"

//#include <robot_analytical_kinematics/anthropomorphic_robot.h>
#include "screws4robotics.h"
#include <robots_def/smm/example_smm.h>

namespace kinematics_ns {

    class AnalyticalKinSolver: public screws4robotics
    {
    private:
        /* data */
    public:
        AnalyticalKinSolver(/* args */);
        ~AnalyticalKinSolver();
    };
    

    
}
#endif //AnalyticalKinSolver_H