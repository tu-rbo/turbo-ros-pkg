/* * IAP - Interactive Perception System - Segment visually observable environment
 * into rigid bodies and estimate type and properties of joints between them by
 * means of interaction.
 * Copyright (C) 2012 Technische Universitaet Berlin - RBO
 * <robotics@robotics.tu-berlin.de>
 * 
 * This file is part of IAP.
 * 
 * IAP is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * IAP is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with IAP.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef __MATHFUNC__
#define __MATHFUNC__
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/math/special_functions/erf.hpp>
#include <vector>
#include <iomanip>
#include <limits>
#include <numeric>
#include <cmath>
#include "feature.h"
#include "cv.h"
#include "Cout.h"
#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif

/**
 * @brief Additional methods for the class Feature which are mostly based on matlab implementations.
 * Also added mathematical methods for std::vector and boost variables.
 */

namespace math
{

/**
 * @brief Q function implemented by the error function which is defined by (1-erf(x-mean)*lambda/sqrt(2))/2
 * mean shifts the function to the appropriate position and lambda defines the shape
 * @param x
 * @param mean
 * @param lambda
 */
double qerf(double x, double mean, double lambda);

/**
 * @brief fit a line to a set of features in 3d space. returns two points on the line
 *
 * @param featureSet
 * @param lowerLineEnd
 * @param upperLineEnd
 */
double fitThreeDimensionalLine(std::vector<vision::FeaturePtr> &featureSet, vision::FeaturePtr &lowerLineEnd,
                               vision::FeaturePtr &upperLineEnd);

/**
 * @brief calculates the amount of motion between two point clouds, assuming both have the same number of features
 *
 * @param fvl
 * @param fvr
 */
double motionBetweenPointClouds(std::vector<vision::FeaturePtr> &fvl, std::vector<vision::FeaturePtr> &fvr);

/**
 * @brief for a given k, print to the console how many percent of the interval [startK,endK] is done and clear the line
 *
 * @param k
 * @param startK
 * @param endK
 */
void printProgress(double k, double startK, double endK);

/**
 * @brief returns the feature which has the greatest euclidean distance to p
 *
 * @param p
 * @param f
 * @param bestIndex
 */
double highestDistanceFeature(vision::FeaturePtr &p, std::vector<vision::FeaturePtr> &f, int &bestIndex);

/**
 * @brief calculate the distance of p from a line specified by two points in space
 *
 * @param p
 * @param lineOne
 * @param lineTwo
 */
double distanceFromLine(vision::FeaturePtr &p, vision::FeaturePtr &lineOne, vision::FeaturePtr &lineTwo);

//
/**
 * @brief calculates the feature, which has the highest motion in bodyIn
 *
 * @param bodyIn
 */
double calculateOverallMotion(std::vector<std::vector<vision::FeaturePtr> > &bodyIn);

//
/**
 * @brief the same as calculateOverallMotion, but also report the index of the feature
 *
 * @param bodyIn
 * @param index
 */
double calculateOverallMotion(std::vector<std::vector<vision::FeaturePtr> > &bodyIn, int &index);

//
/**
 * @brief calculates the cummulated distance (euclidean) between the features
 *
 * @param f
 */
double calculateFeatureMotion(std::vector<vision::FeaturePtr> &f);


//
/**
 * @brief returns the number of elements in d which are greater than threshold
 *
 * @param d
 * @param threshold
 */
int numberSmallerThan(std::vector<double> &d, double threshold);

//
/**
 * @brief calculates the mean feature of a vector of features
 *
 * @param featureVec
 */
vision::FeaturePtr mean(std::vector<vision::FeaturePtr> &featureVec);

//
/**
 * @brief calculates the variance feature of a vector of features
 *
 * @param featureVec
 */
vision::FeaturePtr var(std::vector<vision::FeaturePtr> &featureVec);

//
/**
 * @brief calculates the mean variance of the entries x,y,z
 *
 * @param featureVec
 */
double dvar(std::vector<vision::FeaturePtr> &featureVec);

//
/**
 * @brief performs fvl-fvr for every feature
 *
 * @param fvl
 * @param fvr
 */
std::vector<vision::FeaturePtr> subtract(std::vector<vision::FeaturePtr> &fvl, std::vector<vision::FeaturePtr> &fvr);

//
/**
 * @brief calculates x+y+z for all features and sum the results up
 *
 * @param featureVec
 */
double sum(std::vector<vision::FeaturePtr> &featureVec);

/**
 * @brief takes the absolute value of every x,y,z of every feature
 *
 * @param fvl
 */
std::vector<vision::FeaturePtr> abs(std::vector<vision::FeaturePtr> &fvl);

//
/**
 * @brief the norm of a feature defined as {x,y,z}/sqrt(x^2+y^2+z^2)
 *
 * @param f
 */
vision::FeaturePtr norm(vision::FeaturePtr &f);

//
/**
 * @brief cross product a x b
 *
 * @param a
 * @param b
 */
vision::FeaturePtr cross(vision::FeaturePtr &a, vision::FeaturePtr &b);

//
/**
 * @brief addition a+b
 *
 * @param a
 * @param b
 */
vision::FeaturePtr add(vision::FeaturePtr &a, vision::FeaturePtr &b);

//
/**
 * @brief subtraction a-b
 *
 * @param a
 * @param b
 */
vision::FeaturePtr sub(vision::FeaturePtr &a, vision::FeaturePtr &b);

//
/**
 * @brief f = scalar*f
 *
 * @param scalar
 * @param f
 */
vision::FeaturePtr prod(double scalar, vision::FeaturePtr &f);

//
/**
 * @brief returns a feature with the minimal x,y and z values in all the features given
 *
 * @param f
 */
vision::FeaturePtr min(std::vector<vision::FeaturePtr> &f);

//
/**
 * @brief returns a feature with the maximal x,y and z values in all the features given
 *
 * @param f
 */
vision::FeaturePtr max(std::vector<vision::FeaturePtr> &f);

//
/**
 * @brief the same as vision::FeaturePtr max, but also returns the index
 *
 * @param f
 * @param index
 */
vision::FeaturePtr max(std::vector<vision::FeaturePtr> &f, int &index);

//
/**
 * @brief dot product defined as a*b = |a|*|b|*cos(a,b)
 *
 * @param a
 * @param b
 */
double dot(vision::FeaturePtr &a, vision::FeaturePtr &b);

//
/**
 * @brief absolute value of a feature defined as sqrt(x^2+y^2+z^2)
 *
 * @param f
 */
double abs(vision::FeaturePtr &f);

//
/**
 * @brief absolute value of a two features defined as abs(f1-f2) => sqrt( (x1-x2)^2+ ... )
 *
 * @param f1
 * @param f2
 */
double abs(const vision::FeaturePtr &f1, const vision::FeaturePtr &f2);

/**
 * @brief l2 norm (or euclidean norm)
 *
 * @param f1
 * @param f2
 */
double l2norm(vision::FeaturePtr &f1, vision::FeaturePtr &f2);

/**
 * @brief infinity norm, as defined here http://en.wikipedia.org/wiki/Maximum_norm
 *
 * @param f1
 * @param f2
 */
double l_infinity_norm(vision::FeaturePtr &f1, vision::FeaturePtr &f2);

/** multiply a vision::FeaturePtr x,y,z to left of a 3x3 matrix m
 * @brief
 *
 * @param f
 * @param m
 */
vision::FeaturePtr mul33(vision::FeaturePtr f, boost::numeric::ublas::matrix<double> &m);

//
/**
 * @brief calculate the maximum value in the current column of m
 *
 * @param m
 */
double max(boost::numeric::ublas::matrix_column<boost::numeric::ublas::matrix<double> > &m);

//
/**
 * @brief calculate the minimum value in the current column of m
 *
 * @param m
 */
double min(boost::numeric::ublas::matrix_column<boost::numeric::ublas::matrix<double> > &m);

//
/**
 * @brief calculates the determinant of a 3x3 matrix
 *
 * @param m
 */
double det33(boost::numeric::ublas::matrix<double> &m);

//
/**
 * @brief transposes a 3x3 matrix with the property m^-1 = m^t
 *
 * @param m
 */
void transpose33(boost::numeric::ublas::matrix<double> &m);

//
/**
 * @brief inverse of a homogenous 4x4 matrix
 *
 * @param m
 */
boost::numeric::ublas::matrix<double> inverse44(const boost::numeric::ublas::matrix<double> &m);

//
/**
 * @brief singular value decomposition using opencv methods
 *
 * @param A
 * @param U
 * @param S
 * @param V
 */
void computeSVD(boost::numeric::ublas::matrix<double> &A, boost::numeric::ublas::matrix<double> &U,
                boost::numeric::ublas::matrix<double> &S, boost::numeric::ublas::matrix<double> &V);

//
/**
 * @brief eigenvector and eigenvalue calculation using opencv methods for a symmetric matrix SourceMatrix
 *
 * @param SourceMatrix
 * @param EigenValues
 * @param EigenVectors
 */
void computeEig(boost::numeric::ublas::matrix<double> &SourceMatrix,
                boost::numeric::ublas::matrix<double> &EigenValues, boost::numeric::ublas::matrix<double> &EigenVectors);

//
/**
 * @brief calculates the angle from a rotation matrix R
 *
 * @param R
 */
double getRotationAngleFromMatrix(boost::numeric::ublas::matrix<double> &R);

//
/**
 * @brief creates a sxs unit matrix
 *
 * @param s
 */
boost::numeric::ublas::matrix<double> eye(int s);

//
/**
 * @brief creates a sxk matrix consisting of zeros
 *
 * @param s
 * @param k
 */
boost::numeric::ublas::matrix<double> zeros(int s, int k);

//
/**
 * @brief calculates the product over a vector of 4x4 matrices, by multiplicating vec(0)*vec(1)*...*vec(N)
 *
 * @param vm
 */
boost::numeric::ublas::matrix<double> prodAll44(std::vector<boost::numeric::ublas::matrix<double> > &vm);

/*=================================================================*/
/* additional functions for use with std::vector 				   */
/*=================================================================*/

//
/**
 * @brief gives back the correlation coefficient of d1 and d2
 *
 * @param d1
 * @param d2
 */
double correlation(std::vector<double> &d1, std::vector<double> &d2);

//
/**
 * @brief calculates the covariance between d1 and d2
 *
 * @param d1
 * @param d2
 */
double cov(std::vector<double> &d1, std::vector<double> &d2);

//
/**
 * @brief calculates the mean of vector of doubles
 *
 * @param vd
 */
double mean(std::vector<double> &vd);

//
/**
 * @brief calculates the variance of vector of doubles
 *
 * @param vd
 */
double var(std::vector<double> &vd);

//
/**
 * @brief calculates the standart deviation of vector of doubles
 *
 * @param vd
 */
double stddev(std::vector<double> &vd);

//
/**
 * @brief sums up all the entries of the vector
 *
 * @param vd
 */
double sum(std::vector<double> &vd);

//
/**
 * @brief sums up all the entries of the vector
 *
 * @param vd
 */
long sum(std::vector<int> &vd);

//
/**
 * @brief calculates the maximum value in the input vector
 *
 * @param vd
 */
double max(std::vector<double> &vd);

//
/**
 * @brief calculates the maximum value in the input vector and its index
 *
 * @param vd
 * @param index
 */
double max(std::vector<double> &vd, int &index);

//
/**
 * @brief calculates the maximum value in the input vector and its index
 *
 * @param vd
 * @param index
 */
double max(const std::vector<double> &vd, int &index);

//
/**
 * @brief calculates the maximum value in the input vector
 *
 * @param vd
 */
int max(std::vector<int> &vd);

//
/**
 * @brief normalizes a vector, such that the sum equals to 1
 *
 * @param vd
 */
void norm(std::vector<double> &vd);

//
/**
 * @brief calculates the minimum value in the input vector
 *
 * @param vd
 */
double min(std::vector<double> &vd);

//
/**
 * @brief calculates the minimum value in the input vector and its index
 *
 * @param vd
 * @param index
 */
double min(std::vector<double> &vd, int &index);

/**
 * @brief returns the value corresponding to x given a gaussian with mean mu and standart deviation stddev
 *
 * @param x
 * @param mu
 * @param stddev
 */
double normal(double x, double mu, double stddev = 1.0 / sqrtf(2.0 * M_PI));

/**
 * @brief a lambda greater than 1 narrows the gaussian, while a value smaller broadens the gaussian
 *
 * @param x
 * @param mu
 * @param lambda
 * @param stddev
 */
double normalLambda(double x, double mu, double lambda, double stddev = 1.0 / sqrtf(2.0 * M_PI));

/**
 * @brief Returns a uniformly distributed random number between min and max
 *
 * @param max
 * @param min
 */
double uniform(double max = 1.0, double min = 0.0);

/**
 * @brief Returns a normally distributed random number with mean and std
 *
 * @param max
 * @param min
 */
double gaussian(double mean = 0.0, double std = 1.0);

//__attribute__ ((unused)) is used to suppress warnings, if the variable is not used
__attribute__ ((unused)) static const double dMax = std::numeric_limits<double>::max(); /**< TODO */
__attribute__ ((unused)) static const double dMin = std::numeric_limits<double>::min(); /**< TODO */
static const double dNaN = std::numeric_limits<double>::quiet_NaN(); /**< TODO */
__attribute__ ((unused)) static const int iNaN = std::numeric_limits<int>::quiet_NaN(); /**< TODO */
/**
 * @brief A feature, with not a number entries
 *
 * @param math::dNaN
 * @param math::dNaN
 * @param math::dNaN
 */
const vision::FeaturePtr fNaN(new vision::Feature(math::dNaN, math::dNaN, math::dNaN));

///minimal motion which can still be calculated

#ifdef HAVE_ODE
const double minimalMotionToCompute = 1.8; ///ode: 1.0
#else
const double minimalMotionToCompute = 0.2; ///physx: 1.6 /**< TODO */
#endif

}
#endif

