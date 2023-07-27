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
AnalyticalJacobians::AnalyticalJacobians(const std::shared_ptr<RobotParametersBase>& robot_def)
    : _robot_def(robot_def) {

    PseudoTfs(); // Initialize the product exponentials
}

AnalyticalJacobians::~AnalyticalJacobians()
{
}

void AnalyticalJacobians::PseudoTfs() {
  _Pi[0] = twistExp(_robot_def->getPassiveTwist(0), _robot_def->getPSEUDO_ANGLE(0)); 
  _Pi[1] = twistExp(_robot_def->getPassiveTwist(1), _robot_def->getPSEUDO_ANGLE(1)); 
}

void AnalyticalJacobians::SpatialJacobian(float *q) {
   // 1st column
    _Jsp.col(0) = _robot_def->active_twists[0];

    // 2nd column
    _exp = twistExp(_robot_def->active_twists[0], *(q+0) ) * _Pi[0]; 
    ad(_ad, _exp);
    _Jsp.col(1) = _ad * _robot_def->active_twists[1];

    // 3rd column
     _exp = _exp * twistExp(_robot_def->active_twists[1], *(q+1) ) * _Pi[1];
    ad(_ad, _exp);
    _Jsp.col(2) = _ad * _robot_def->active_twists[2];

}

void AnalyticalJacobians::BodyJacobian(float *q) {
    // 1st column
    //_exp = twistExp(_robot_def2.active_twists[0], *(q+0) ) * twistExp(_robot_def2.active_twists[1], *(q+1) ) * twistExp(_robot_def2.active_twists[2], *(q+2) ) * _robot_def2.gst0;
}

void AnalyticalJacobians::PrintJacobian(Stream& serialPort, char jacobian_char_id) {
    switch (jacobian_char_id)
    {
    case 's':
        //printMatrix(_Jsp); // unknown mystery bug
        serialPort.print(_Jsp(0,0),4); serialPort.print(" "); serialPort.print(_Jsp(0,1),4); serialPort.print(" "); serialPort.println(_Jsp(0,2),4);
        serialPort.print(_Jsp(1,0),4); serialPort.print(" "); serialPort.print(_Jsp(1,1),4); serialPort.print(" "); serialPort.println(_Jsp(1,2),4);
        serialPort.print(_Jsp(2,0),4); serialPort.print(" "); serialPort.print(_Jsp(2,1),4); serialPort.print(" "); serialPort.println(_Jsp(2,2),4);
        serialPort.print(_Jsp(3,0),4); serialPort.print(" "); serialPort.print(_Jsp(3,1),4); serialPort.print(" "); serialPort.println(_Jsp(3,2),4);
        serialPort.print(_Jsp(4,0),4); serialPort.print(" "); serialPort.print(_Jsp(4,1),4); serialPort.print(" "); serialPort.println(_Jsp(4,2),4);
        serialPort.print(_Jsp(5,0),4); serialPort.print(" "); serialPort.print(_Jsp(5,1),4); serialPort.print(" "); serialPort.println(_Jsp(5,2),4);
        break;
    
    default:
        break;
    }
    
}