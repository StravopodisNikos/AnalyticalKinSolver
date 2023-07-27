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
    virtual const float getPSEUDO_ANGLE(int index) const = 0;
    virtual const Eigen::Matrix<float, 6, 1> getPassiveTwist(int index) const = 0;
};

class Structure2Pseudos : public RobotParametersBase {
public:
    static constexpr int SMM_PSEUDOS = 2;
    float  pseudo_angles[SMM_PSEUDOS];
    Eigen::Matrix<float, 6, 1> passive_twists[SMM_PSEUDOS];

    uint8_t getPSEUDOS() const override { return SMM_PSEUDOS; }
    // Implement the virtual function to return the passive_twists for Structure2Pseudos
    const Eigen::Matrix<float, 6, 1> getPassiveTwist(int index) const override {
        // Ensure index is within bounds (0 <= index < SMM_PSEUDOS)
        if (index >= 0 && index < SMM_PSEUDOS) {
            return passive_twists[index];
        } else {
            // Return some default value or handle the error as per your requirement
            // Here, we return the first passive_twists by default
            return passive_twists[0];
        }
    }
    const float getPSEUDO_ANGLE(int index) const override {
        return pseudo_angles[index];
    }
};

class Structure3Pseudos : public RobotParametersBase {
public:
    static constexpr int SMM_PSEUDOS = 3;
    float  pseudo_angles[SMM_PSEUDOS];
    Eigen::Matrix<float, 6, 1> passive_twists[SMM_PSEUDOS];

    uint8_t getPSEUDOS() const override { return SMM_PSEUDOS; }
    // Implement the virtual function to return the passive_twists for Structure2Pseudos
    const Eigen::Matrix<float, 6, 1> getPassiveTwist(int index) const override {
        // Ensure index is within bounds (0 <= index < SMM_PSEUDOS)
        if (index >= 0 && index < SMM_PSEUDOS) {
            return passive_twists[index];
        } else {
            // Return some default value or handle the error as per your requirement
            // Here, we return the first passive_twists by default
            return passive_twists[0];
        }
    }
    const float getPSEUDO_ANGLE(int index) const override {
        return pseudo_angles[index];
    }
};

class Structure4Pseudos : public RobotParametersBase {
public:
    static constexpr int SMM_PSEUDOS = 4;
    float  pseudo_angles[SMM_PSEUDOS];
    Eigen::Matrix<float, 6, 1> passive_twists[SMM_PSEUDOS];

    uint8_t getPSEUDOS() const override { return SMM_PSEUDOS; }
    // Implement the virtual function to return the passive_twists for Structure2Pseudos
    const Eigen::Matrix<float, 6, 1> getPassiveTwist(int index) const override {
        // Ensure index is within bounds (0 <= index < SMM_PSEUDOS)
        if (index >= 0 && index < SMM_PSEUDOS) {
            return passive_twists[index];
        } else {
            // Return some default value or handle the error as per your requirement
            // Here, we return the first passive_twists by default
            return passive_twists[0];
        }
    }
    const float getPSEUDO_ANGLE(int index) const override {
        return pseudo_angles[index];
    }
};

#endif // EXAMPLE_SMM