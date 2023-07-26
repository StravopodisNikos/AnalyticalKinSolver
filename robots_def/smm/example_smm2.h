#ifndef EXAMPLE_SMM2_H
#define EXAMPLE_SMM2_H

#include <Eigen30.h>
#include <EigenAVR.h>
#include <Eigen/Dense>
#include <Eigen/Core>

class RobotParametersBase {
public:
    // DATA COMMON FOR ALL
    Eigen::Matrix<float, 6, 1> active_twists[3]; 
    Eigen::Isometry3f gst0;

    // FUNCTIONS
    virtual ~RobotParametersBase() {}
    virtual uint8_t getPSEUDOS() const = 0;

};

class Structure2Pseudos : public RobotParametersBase {
public:
    static constexpr int SMM_PSEUDOS = 2;
    Eigen::Matrix<float, SMM_PSEUDOS, 1> anatomy;
    Eigen::Matrix<float, 6, 1> passive_twists[SMM_PSEUDOS];

    uint8_t getPSEUDOS() const override { return SMM_PSEUDOS; }
};

class Structure3Pseudos : public RobotParametersBase {
public:
    static constexpr int SMM_PSEUDOS = 3;
    Eigen::Matrix<float, SMM_PSEUDOS, 1> anatomy;
    Eigen::Matrix<float, 6, 1> passive_twists[SMM_PSEUDOS];

    uint8_t getPSEUDOS() const override { return SMM_PSEUDOS; }
};

class Structure4Pseudos : public RobotParametersBase {
public:
    static constexpr int SMM_PSEUDOS = 4;
    Eigen::Matrix<float, SMM_PSEUDOS, 1> anatomy;
    Eigen::Matrix<float, 6, 1> passive_twists[SMM_PSEUDOS];

    uint8_t getPSEUDOS() const override { return SMM_PSEUDOS; }
};

#endif // EXAMPLE_SMM