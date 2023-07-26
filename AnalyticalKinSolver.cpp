#include "AnalyticalKinSolver.h"   

using namespace kinematics_ns;

// Class AnalyticalKinSolver
AnalyticalKinSolver::AnalyticalKinSolver(/* args */)
{
}

AnalyticalKinSolver::~AnalyticalKinSolver()
{
}

// Class AnalyticalJacobians
AnalyticalJacobians::AnalyticalJacobians() {
    _dof = DOF; // Only 3 dof supported

    // Initialize screw utils
};

AnalyticalJacobians::AnalyticalJacobians(const Structure2Pseudos& robot_def) : _robot_def2(robot_def)
{
}
AnalyticalJacobians::AnalyticalJacobians(const Structure3Pseudos& robot_def) : _robot_def3(robot_def)
{
}
AnalyticalJacobians::AnalyticalJacobians(const Structure4Pseudos& robot_def) : _robot_def4(robot_def)
{
}
AnalyticalJacobians::~AnalyticalJacobians()
{
}

void AnalyticalJacobians::SpatialJacobian(float *q) {
    // 1st column
    _Jsp.col(0) = _robot_def2.active_twists[0];
    // 2nd column
    _exp = screws_utils_ns::twistExp(_robot_def2.active_twists[0], *(q+0) ); // update tf due to previous active joint rot (Joint 1)
    screws_utils_ns::ad(_ad, _exp); // update adjoint tf
    _Jsp.col(1) = _ad * _robot_def2.active_twists[1];
    // 3rd column
    _exp = _exp * screws_utils_ns::twistExp(_robot_def2.active_twists[1], *(q+1) ); // update tf due to previous active joint rot (Joint 1+2)
    screws_utils_ns::ad(_ad, _exp); // update adjoint tf
    _Jsp.col(2) = _ad * _robot_def2.active_twists[2];
}