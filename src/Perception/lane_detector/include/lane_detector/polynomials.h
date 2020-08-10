/**
 * Copyright (c) 2019 Koło Naukowe Robotyków
 * This code is licensed under BSD license (see LICENSE for details)
 **/

// Fit a polynomial.
// Adapted from:
// https://github.com/patLoeber/Polyfit/blob/master/PolyfitBoost.hpp

#ifndef LANE_DETECTOR_POLYNOMIALS_H
#define LANE_DETECTOR_POLYNOMIALS_H

#include <vector>

#include <boost/numeric/ublas/lu.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <opencv2/highgui/highgui.hpp>
// #define BOOST_UBLAS_TYPE_CHECK 0

bool polyfit(const std::vector<cv::Point2f> &points, int degree, std::vector<float> &coeff);
float getPolyY(const std::vector<float> &coeff, float x);

#endif  //  LANE_DETECTOR_POLYNOMIALS_H
