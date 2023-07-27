#include "AnalyticalKinSolver.h"   

using namespace kinematics_ns;

// Class AnalyticalKinSolver
AnalyticalKinSolver::AnalyticalKinSolver(/* args */)
{
    _dof = DOF; // Only 3 dof supported
}

AnalyticalKinSolver::AnalyticalKinSolver(const std::shared_ptr<RobotParametersBase>& robot_def)
    : _robot_def(robot_def) {

    PseudoTfs(); // Initialize the product exponentials
}

AnalyticalKinSolver::~AnalyticalKinSolver()
{
}

void AnalyticalKinSolver::PseudoTfs() {
  _Pi[0] = twistExp(_robot_def->getPassiveTwist(0), _robot_def->getPSEUDO_ANGLE(0)); 
  _Pi[1] = twistExp(_robot_def->getPassiveTwist(1), _robot_def->getPSEUDO_ANGLE(1)); 
}

Eigen::Isometry3f AnalyticalKinSolver::ForwardKinematicsTCP(float *q) {
    return twistExp(_robot_def->active_twists[0], *(q+0) ) * _Pi[0] * twistExp(_robot_def->active_twists[1], *(q+1) ) * _Pi[1] * twistExp(_robot_def->active_twists[2], *(q+2) ) * _robot_def->gst0;
}

void AnalyticalKinSolver::PrintTransform(Stream& serialPort, Eigen::Isometry3f g) {
    serialPort.print(g(0,0),4); serialPort.print(" "); serialPort.print(g(0,1),4); serialPort.print(" "); serialPort.print(g(0,2),4); serialPort.print(" "); serialPort.println(g(0,3),4);
    serialPort.print(g(1,0),4); serialPort.print(" "); serialPort.print(g(1,1),4); serialPort.print(" "); serialPort.print(g(1,2),4); serialPort.print(" "); serialPort.println(g(1,3),4);
    serialPort.print(g(2,0),4); serialPort.print(" "); serialPort.print(g(2,1),4); serialPort.print(" "); serialPort.print(g(2,2),4); serialPort.print(" "); serialPort.println(g(2,3),4);    
}

// Class AnalyticalJacobians
AnalyticalJacobians::AnalyticalJacobians() {
    _dof = DOF; // Only 3 dof supported

    // Initialize screw utils
}

AnalyticalJacobians::AnalyticalJacobians(const std::shared_ptr<RobotParametersBase>& robot_def)
    : _robot_def(robot_def) {

    PseudoTfs(); // Initialize the product exponentials
}

AnalyticalJacobians::~AnalyticalJacobians()
{
}

void AnalyticalJacobians::_update_q_rad(float *q) {
    _q_cur = q;
}

void AnalyticalJacobians::_update_p_tcp_sp() {
    _g_tcp_sp = ForwardKinematicsTCP(_q_cur);

    _p_tcp_sp.x() = _g_tcp_sp(0,3);
    _p_tcp_sp.y() = _g_tcp_sp(1,3);
    _p_tcp_sp.z() = _g_tcp_sp(2,3);
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

void AnalyticalJacobians::ToolJacobian(float *q, char jacobian_char_id) {
    Eigen::Vector3f row0;
    Eigen::Vector3f row1;
    Eigen::Vector3f row2;

    switch (jacobian_char_id)
    {
        case 's':
        // The spatial jacobian and the spatial TCP position vector (relative to {S) frame are used
            _update_q_rad(q);        // update active configuration angles
            SpatialJacobian(_q_cur); // update spatial jacobian
            _update_p_tcp_sp();      // update spatial tcp pos vector

            row0.x() = _Jsp(1,1) + _Jsp(5,1)*_p_tcp_sp.z() - _Jsp(6,1)*_p_tcp_sp.y() ;
            row0.y() = _Jsp(1,2) + _Jsp(5,2)*_p_tcp_sp.z() - _Jsp(6,2)*_p_tcp_sp.y() ;
            row0.z() = _Jsp(1,3) + _Jsp(5,3)*_p_tcp_sp.z() - _Jsp(6,3)*_p_tcp_sp.y() ;

            row1.x() = _Jsp(2,1) - _Jsp(4,1)*_p_tcp_sp.z() + _Jsp(6,1)*_p_tcp_sp.x() ;
            row1.y() = _Jsp(2,2) - _Jsp(4,2)*_p_tcp_sp.z() + _Jsp(6,2)*_p_tcp_sp.x() ;
            row1.z() = _Jsp(2,3) - _Jsp(4,3)*_p_tcp_sp.z() + _Jsp(6,3)*_p_tcp_sp.x() ;

            row2.x() = _Jsp(4,1)*_p_tcp_sp.y() - _Jsp(5,1)*_p_tcp_sp.x() + _Jsp(3,1) ;
            row2.y() = _Jsp(4,2)*_p_tcp_sp.y() - _Jsp(5,2)*_p_tcp_sp.x() + _Jsp(3,2) ;
            row2.z() = _Jsp(4,3)*_p_tcp_sp.y() - _Jsp(5,3)*_p_tcp_sp.x() + _Jsp(3,3) ;
            break;
        case 'b':
        // The body jacobian and the TCP position vector relative to the {T} frame are used
            row0.setZero();
            row1.setZero();
            row2.setZero();        
            break;
        default:
            row0.setZero();
            row1.setZero();
            row2.setZero();
            break;
    }
    _Jtcp.row(0) = row0;
    _Jtcp.row(1) = row1;
    _Jtcp.row(2) = row2;
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
    case 't':
        serialPort.print(_Jtcp(0,0),4); serialPort.print(" "); serialPort.print(_Jtcp(0,1),4); serialPort.print(" "); serialPort.println(_Jtcp(0,2),4);
        serialPort.print(_Jtcp(1,0),4); serialPort.print(" "); serialPort.print(_Jtcp(1,1),4); serialPort.print(" "); serialPort.println(_Jtcp(1,2),4);
        serialPort.print(_Jtcp(2,0),4); serialPort.print(" "); serialPort.print(_Jtcp(2,1),4); serialPort.print(" "); serialPort.println(_Jtcp(2,2),4);    
    default:
        break;
    }
    
}