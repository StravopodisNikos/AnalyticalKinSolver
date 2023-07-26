#ifndef EXAMPLE_SMM_H
#define EXAMPLE_SMM_H

#include <Eigen30.h>
#include <EigenAVR.h>
#include <Eigen/Dense>
#include <Eigen/Core>

struct Structure2Pseudos {
    // Active twists @reference anatomy
    Eigen::Matrix<float, 6, 1> active_twists[3];

    // Anatomy
    Eigen::Matrix<float, 2, 1> anatomy;

    // Passive twists @reference anatomy
    Eigen::Matrix<float, 6, 1> passive_twists[2];

    // gst0
    Eigen::Isometry3f gst0;
};

struct Structure3Pseudos {
    // Active twists @reference anatomy
    Eigen::Matrix<float, 6, 1> active_twists[3];

    // Anatomy
    Eigen::Matrix<float, 3, 1> anatomy;

    // Passive twists @reference anatomy
    Eigen::Matrix<float, 6, 1> passive_twists[3];

    // gst0
    Eigen::Isometry3f gst0;
};

struct Structure4Pseudos {
    // Active twists @reference anatomy
    Eigen::Matrix<float, 6, 1> active_twists[3];

    // Anatomy
    Eigen::Matrix<float, 4, 1> anatomy;

    // Passive twists @reference anatomy
    Eigen::Matrix<float, 6, 1> passive_twists[4];

    // gst0
    Eigen::Isometry3f gst0;
};

#endif // EXAMPLE_SMM