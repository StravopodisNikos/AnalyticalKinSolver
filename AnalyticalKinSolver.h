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
        std::shared_ptr<RobotParametersBase> _robot_def;
        uint8_t _dof;
        Eigen::Vector3f _p_tcp_sp; // spatial position vector of the TCP
        // Screw utils 
        Eigen::Isometry3f _Pi[DOF-1];
        Eigen::Isometry3f _exp; // saves current 4x4 twist exponential matrix
        Eigen::Matrix<float, 6, 6> _ad;  // saves current 6x6 adjoint matrix 
               
    public:
        AnalyticalKinSolver(/* args */);
        AnalyticalKinSolver(const std::shared_ptr<RobotParametersBase>& robot_def);
        ~AnalyticalKinSolver();
        
        void PseudoTfs();
        Eigen::Isometry3f ForwardKinematicsTCP(float *q);

        void PrintTransform(Stream& serialPort, Eigen::Isometry3f g);
    };

    //class AnalyticalJacobians: public screws_utils_ns::screws4robotics
    class AnalyticalJacobians: public AnalyticalKinSolver
    {
    private:
        std::shared_ptr<RobotParametersBase> _robot_def;
        uint8_t _dof;
        float *_q_cur;
        Eigen::Isometry3f _g_tcp_sp; // current spatial tcp transform
        Eigen::Vector3f _p_tcp_sp; // spatial position vector of the TCP

        // Screw utils 
        Eigen::Isometry3f _Pi[DOF-1];
        Eigen::Isometry3f _exp; // saves current 4x4 twist exponential matrix
        Eigen::Matrix<float, 6, 6> _ad;  // saves current 6x6 adjoint matrix
        Eigen::Matrix<float, 6, DOF> _Jsp;
        Eigen::Matrix<float, 6, DOF> _Jbd;
        Eigen::Matrix<float, DOF, DOF> _Jtcp;

        void _update_q_rad(float *q); // auxiliary for test only
        void _update_p_tcp_sp();
    public:
        AnalyticalJacobians();
        AnalyticalJacobians(const std::shared_ptr<RobotParametersBase>& robot_def);
        ~AnalyticalJacobians();

        void SpatialJacobian(float *q); // eq.3.54, A Mathematical Introduction to robotic Manipulation Book
        void BodyJacobian(float *q);
        void ToolJacobian(float *q, char jacobian_char_id); // char sets the matrix calculation preference

        void PrintJacobian(Stream& serialPort, char jacobian_char_id);
    };
    

    
}
#endif //AnalyticalKinSolver_H