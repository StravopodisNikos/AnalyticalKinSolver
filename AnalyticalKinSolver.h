#ifndef AnalyticalKinSolver_H
#define AnalyticalKinSolver_H

#include "Arduino.h"
#include <memory>
//#include <robot_analytical_kinematics/anthropomorphic_robot.h>
#include "screws4robotics.h"                      // Basic screw theory utilities
#include <robots_def/smm/example_smm2.h>          // Polymorphism magic
#include <robots_def/smm/loaded_robot_settings.h> // this may be moved to another folder

namespace kinematics_ns {

    class AnalyticalKinSolver: public screws_utils_ns::screws4robotics
    {
    private:

    public:
        AnalyticalKinSolver(/* args */);
        ~AnalyticalKinSolver();
    };

    class AnalyticalJacobians: public screws_utils_ns::screws4robotics
    {
    private:
        //Structure2Pseudos _robot_def2;
        //Structure3Pseudos _robot_def3;
        //Structure4Pseudos _robot_def4;
        std::shared_ptr<RobotParametersBase> _robot_def;
        uint8_t _dof;

        // Screw utils 
        Eigen::Isometry3f _Pi[DOF-1];
        Eigen::Isometry3f _exp; // saves current 4x4 twist exponential matrix
        Eigen::Matrix<float, 6, 6> _ad;  // saves current 6x6 adjoint matrix
        Eigen::Matrix<float, 6, DOF> _Jsp;
        Eigen::Matrix<float, 6, DOF> _Jbd;
        Eigen::Matrix<float, DOF, DOF> _Jt;

    public:
        AnalyticalJacobians();
        AnalyticalJacobians(const std::shared_ptr<RobotParametersBase>& robot_def);

        ~AnalyticalJacobians();

        void PseudoTfs();

        void SpatialJacobian(float *q); // eq.3.54, A Mathematical Introduction to robotic Manipulation Book
        void BodyJacobian(float *q);

        void PrintJacobian(Stream& serialPort, char jacobian_char_id);
    };
    

    
}
#endif //AnalyticalKinSolver_H