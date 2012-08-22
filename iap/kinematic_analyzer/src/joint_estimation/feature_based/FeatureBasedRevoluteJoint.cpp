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
/*
 * FeatureBasedRevoluteJoint.cpp
 *
 *  Created on: Apr 23, 2012
 *      Author: shoefer
 */

#include "feature_based/FeatureBasedRevoluteJoint.h"

#include "MathFunctions.h"

using namespace vision;
using namespace math;
using namespace std;

FeatureBasedRevoluteJoint::FeatureBasedRevoluteJoint() :
  Joint(), FeatureBasedJoint(), RevoluteJoint(), _circle_goodness(0)
{
}

FeatureBasedRevoluteJoint::FeatureBasedRevoluteJoint(const FeatureBasedRevoluteJoint& tbpj) :
  Joint(tbpj), FeatureBasedJoint(tbpj), RevoluteJoint(tbpj), _circle_goodness(tbpj._circle_goodness)
{

}

FeatureBasedRevoluteJoint::~FeatureBasedRevoluteJoint()
{
}

/*
 * JointDetector::findGoodFeatures
 *
 * Features which are near to a revolute joint do not move as much as
 * features which are far away. Because we can easier conclude the joint
 * type if we got features which move a lot, this function tries to find them.
 */
vector<int> findGoodFeatures(vector<vector<FeaturePtr> > &body)
{
  //we define good features as features, which have a large euclidean distance to each other

  //std::cout << "starting find good features..." << std::endl;
  vector<int> goodFeatures;

  //the threshold below we ignore all features
  double percent = 0.7;

  int bestFeatureIndex = 0;

  double overallDistance = math::calculateOverallMotion(body, bestFeatureIndex);

  for (unsigned int i = 0; i < body.at(0).size(); i++)
  {

    std::vector<FeaturePtr> curFeature;
    for (unsigned int j = 0; j < body.size(); j++)
    {
      curFeature.push_back(body.at(j).at(i));
    }

    double traveledDistanceCurFeature = math::calculateFeatureMotion(curFeature);

    if (traveledDistanceCurFeature > percent * overallDistance)
    {
      goodFeatures.push_back(i);
    }
  }

  return goodFeatures;
}

vector<vector<FeaturePtr> > getGoodFeatures(vector<vector<FeaturePtr> > &body)
{
  vector<int> goodFeatures = findGoodFeatures(body);

  //std::cout << "GOOD FEATURES: " << goodFeatures << std::endl;
  vector<vector<FeaturePtr> > bodyGoodFeatures;

  for (unsigned int i = 0; i < body.size(); i++)
  {
    vector<FeaturePtr> currentFrameGoodFeatures;

    for (unsigned int j = 0; j < goodFeatures.size(); j++)
    {
      currentFrameGoodFeatures.push_back(body.at(i).at(goodFeatures.at(j)));
    }
    bodyGoodFeatures.push_back(currentFrameGoodFeatures);
  }
  return bodyGoodFeatures;
}

void FeatureBasedRevoluteJoint::estimateAxis()
{
  //fit a circle

  vector<boost::numeric::ublas::matrix<double> > alignment_matrices;
  vector<vision::FeaturePtr> centerPoints;

  vector<double> circleQuality;
  vector<double> occupiedCircleAngle;

  unsigned int Npoints = this->_ct2->getNumberOfFeatures();
  unsigned int Nframes = this->_ct2->getTrajectoryLength();

  ClusterTrajectoryPtr body_local_ct = ClusterTrajectoryPtr(new ClusterTrajectory());

  for (int i = 0; i < Nframes; i++)
  {
    body_local_ct->addFeatureSet(
                                 this->_ct2->getCluster(i)->applyInverseRigidTransformation(
                                                                                            this->_btp->getBody1Trajectory()->getRigidTransformation(
                                                                                                                                                     i)));
  }

  std::vector<std::vector<FeaturePtr> > body_local_all = body_local_ct->FeatureTrajectorySet2Vector();

  vector<vector<FeaturePtr> > body_local = getGoodFeatures(body_local_all);

  Npoints = body_local.at(0).size();

  // First fit a plane through all image features and compute the alignment matrix
  // The alignment matrix rotates the estimated plane to the x-y plane
  ROS_INFO_STREAM_NAMED("FeatureBasedRevoluteJoint.estimateAxis",
                        "Number of Features: " << Npoints << " Number of Frames: " << Nframes);
  if (Nframes <= 1)
  {
    std::string error_msg("Number of frames is not enough (<=1)");
    ROS_ERROR_STREAM_NAMED("FeatureBasedRevoluteJoint.estimateAxis", error_msg);
    throw error_msg;
  }

  vector<vector<double> > planeParams;
  vector<double> mean_plane_ori;
  mean_plane_ori.push_back(0.);
  mean_plane_ori.push_back(0.);
  mean_plane_ori.push_back(0.);
  try
  {
    for (unsigned int feat_idx = 0; feat_idx < Npoints; feat_idx++)
    {
      // Extract the trajectory of each individual Feature
      vector<vision::FeaturePtr> current_feature_trajectory;
      for (unsigned int frame_idx = 0; frame_idx < Nframes; frame_idx++)
      {
        current_feature_trajectory.push_back(body_local.at(frame_idx).at(feat_idx));
      }

      // Estimate the plane parameters that minimize R in Ax+By+Cz+D = R
      // If R is zero we have found the plane that comprehend the trajectory
      planeParams.push_back(fitPlane(current_feature_trajectory));

      // Estimate the transformation matrix which aligns the plane to be coplanar to the x-y plane
      boost::numeric::ublas::matrix<double> alignment_matrix = magic_cosini(planeParams.at(feat_idx).at(0),
                                                                            planeParams.at(feat_idx).at(1),
                                                                            planeParams.at(feat_idx).at(2));
      mean_plane_ori.at(0) += planeParams.at(feat_idx).at(0);
      mean_plane_ori.at(1) += planeParams.at(feat_idx).at(1);
      mean_plane_ori.at(2) += planeParams.at(feat_idx).at(2);
      alignment_matrices.push_back(alignment_matrix);
    }
  }
  catch (string &s)
  {
    string error_msg = string("Error thrown while estimating the planes of motion of individual Features. Message: ")
        + s;
    ROS_ERROR_STREAM_NAMED("FeatureBasedRevoluteJoint.estimateAxis", error_msg);
  }

  vector<double> mean_posi;
  mean_posi.push_back(0.);
  mean_posi.push_back(0.);
  mean_posi.push_back(0.);
  for (unsigned int feat_idx = 0; feat_idx < Npoints; feat_idx++)
  {
    // Transform the Feature feat_idx in the first frame to be aligned with the x-y plane (but with offset in z)
    vision::FeaturePtr feature_rotated = math::mul33(body_local.at(0).at(feat_idx), alignment_matrices.at(feat_idx));

    // Strange, we could set z=0 directly...
    double z_alignment_offset = feature_rotated->getZ();
    feature_rotated->setZ(feature_rotated->getZ() - z_alignment_offset);

    // Do the same for the Feature feat_idx in the rest of the frames using the first estimated offset in z axis
    vector<vision::FeaturePtr> feature_trajectory_rotated;
    feature_trajectory_rotated.push_back(feature_rotated);
    //std::cout << feature_trajectory_rotated.at(0)->getX() << " " << feature_trajectory_rotated.at(0)->getY() << " " << feature_trajectory_rotated.at(0)->getZ() << std::endl;
    for (unsigned int frame_idx = 1; frame_idx < Nframes; frame_idx++)
    {
      feature_trajectory_rotated.push_back(
                                           math::mul33(body_local.at(frame_idx).at(feat_idx),
                                                       alignment_matrices.at(feat_idx)));
      feature_trajectory_rotated.at(frame_idx)->setZ(
                                                     feature_trajectory_rotated.at(frame_idx)->getZ()
                                                         - z_alignment_offset);
      //std::cout << feature_trajectory_rotated.at(frame_idx)->getX() << " " << feature_trajectory_rotated.at(frame_idx)->getY() << " " << feature_trajectory_rotated.at(frame_idx)->getZ() << std::endl;
    }

    vector<double> circle_params = circleFitByTaubin(feature_trajectory_rotated);

    // If we project the center point back to the plane, we got one point, which should lie on the axis of rotation.
    vision::FeaturePtr center(new vision::Feature(circle_params.at(0), circle_params.at(1), z_alignment_offset));
    transpose33(alignment_matrices.at(feat_idx));
    center = mul33(center, alignment_matrices.at(feat_idx));
    centerPoints.push_back(center);

    mean_posi.at(0) += center->getX();
    mean_posi.at(1) += center->getY();
    mean_posi.at(2) += center->getZ();
    circleQuality.push_back(circle_params.at(3));
    occupiedCircleAngle.push_back(circle_params.at(4));
  }

  // The average distance from the line is the number of points in a confidential interval
  double averageDistanceFromLine = fitThreeDimensionalLine(centerPoints, this->_axis.position, this->_axis.orientation);

  this->_axis.orientation = vision::FeaturePtr(
                                               new vision::Feature(mean_plane_ori.at(0), mean_plane_ori.at(1),
                                                                   mean_plane_ori.at(2)));

  this->_axis.orientation->normalize();

  this->_axis.position = vision::FeaturePtr(new vision::Feature(mean_posi.at(0), mean_posi.at(1), mean_posi.at(2)));

  this->_axis.position->normalize();

  //probability that the occupied angle is small
  double pOccupiedAngle = 0.0;//math::qerf( mean(occupiedCircleAngle), 5.0, 0.5); //20.0 0.5

  //probability of a good circle fit
  double pCircleQuality = math::mean(circleQuality);

  this->_circle_goodness = pCircleQuality * (1.0 - pOccupiedAngle) * averageDistanceFromLine;
  ROS_INFO_STREAM_NAMED(
                        "FeatureBasedRevoluteJoint.estimateAxis",
                        "Revolute Axis Circle fit: Quality = " << pCircleQuality << " AvgLine = "
                            << averageDistanceFromLine << " MeanOccupiedAngle = " << (1.0 - pOccupiedAngle)
                            << " Result = " << this->_circle_goodness);
  ROS_DEBUG_STREAM_NAMED("FeatureBasedRevoluteJoint.estimateAxis", "AxisOfRotationPVec: " << this->_axis.position);
  ROS_DEBUG_STREAM_NAMED("FeatureBasedRevoluteJoint.estimateAxis", "AxisOfRotationDirVec: " << this->_axis.orientation);

  // Check if score is not in the interval [0,1] => error in this section
  if (this->_circle_goodness > 1.0 || this->_circle_goodness < 0.0)
  {
    ROS_ERROR_STREAM_NAMED("FeatureBasedRevoluteJoint.estimateAxis", "The goodness of the circle fit is out of bounds!");
  }
}

double FeatureBasedRevoluteJoint::getGoodnessOfFit()
{
  // TODO
  return this->_circle_goodness;
}

double FeatureBasedRevoluteJoint::getJointOrientationUncertainty() const
{
  return (sqrt(1-this->_circle_goodness));
}


double FeatureBasedRevoluteJoint::getJointPositionUncertainty() const
{
  return (sqrt(1-this->_circle_goodness));
}

::iap_common::JointMsg FeatureBasedRevoluteJoint::toROSMsg()
{
  ::iap_common::JointMsg msg = RevoluteJoint::toROSMsg();
  FeatureBasedJoint::toROSMsg(msg);
  return msg;
}

std::vector<double> FeatureBasedRevoluteJoint::circleFitByTaubin(std::vector<vision::FeaturePtr> &f)
{
  //assume that the z-value is zero => we just have to fit a circle in the x-y plane
  assert(f.size() > 0);
  unsigned int Npoints = f.size();
  vision::FeaturePtr centroid = mean(f);

  //moments of the circle
  double mxx = 0.0;
  double myy = 0.0;
  double mxy = 0.0;
  double mxz = 0.0;
  double myz = 0.0;
  double mzz = 0.0;

  for (unsigned int i = 0; i < Npoints; i++)
  {
    double xi = f.at(i)->getX() - centroid->getX();
    double yi = f.at(i)->getY() - centroid->getY();
    double zi = xi * xi + yi * yi;
    mxy += xi * yi;
    mxx += xi * xi;
    myy += yi * yi;
    mxz += xi * zi;
    myz += yi * zi;
    mzz += zi * zi;
  }
  mxx /= Npoints;
  myy /= Npoints;
  mxy /= Npoints;
  mxz /= Npoints;
  myz /= Npoints;
  mzz /= Npoints;

  //computing coefficients of characteristic polynom
  double mz = mxx + myy;
  double cov_xy = mxx * myy - mxy * mxy;
  double a3 = 4 * mz;
  double a2 = -3 * mz * mz - mzz;
  double a1 = mzz * mz + 4 * cov_xy * mz - mxz * mxz - myz * myz - mz * mz * mz;
  double a0 = mxz * mxz * myy + myz * myz * mxx - mzz * cov_xy - 2 * mxz * myz * mxy + mz * mz * cov_xy;
  double a22 = a2 + a2;
  double a33 = a3 + a3 + a3;

  double xnew = 0;
  double ynew = pow(10.0, 20);

  //the precision
  double epsilon = pow(10.0, -12);
  double IterMax = 20;

  for (int iter = 0; iter < IterMax; iter++)
  {
    double yold = ynew;
    ynew = a0 + xnew * (a1 + xnew * (a2 + xnew * a3));
    if (fabs(ynew) > fabs(yold))
    {
      ROS_WARN_STREAM_NAMED("FeatureBasedRevoluteJoint.circleFitByTaubin",
                            "Newton-Taubin goes in wrong direction: |ynew| > |yold|");
      xnew = 0;
      break;
    }
    double dy = a1 + xnew * (a22 + xnew * a33);
    double xold = xnew;
    xnew = xold - ynew / dy;
    if (fabs((xnew - xold) / xnew) < epsilon)
    {
      break;
    }
    if (iter >= IterMax)
    {
      ROS_ERROR_STREAM_NAMED("FeatureBasedRevoluteJoint.circleFitByTaubin",
                             "Newton-Taubin does NOT converge! (max iterations reached)");
      xnew = 0;
    }
    if (xnew < 0)
    {
      //cout << "Newton-Taubin negative root:  x=" << xnew << endl;
      xnew = 0;
    }
  }

  double det = xnew * xnew - xnew * mz + cov_xy;

  double centerx = ((mxz * (myy - xnew) - myz * mxy) / (det)) * 0.5;
  double centery = ((myz * (mxx - xnew) - mxz * mxy) / (det)) * 0.5;
  double radius = sqrtf(centerx * centerx + centery * centery + mz);

  std::vector<double> output;
  output.push_back(centerx + centroid->getX());
  output.push_back(centery + centroid->getY());
  output.push_back(radius);

  //norm the data on the unit circle
  std::vector<vision::FeaturePtr> normCirc;
  //unitCircle=[(XY(:,1)-xc)./Par(3) (XY(:,2)-yc)./Par(3)];
  for (unsigned int i = 0; i < Npoints; i++)
  {
    vision::FeaturePtr n(new vision::Feature(0.0, 0.0, 0.0));
    n->setX((f.at(i)->getX() - output.at(0)) / output.at(2));
    n->setY((f.at(i)->getY() - output.at(1)) / output.at(2));
    normCirc.push_back(n);
  }

  //the confidence interval is a expert-designed parameter which defines
  //how much noise we want accept as a revolute joint
  double confidenceInterval = 0.1; ///changed june10,2011 from 0.1 to 0.05
  double upperConfidenceBound = 1.0 + confidenceInterval;
  double lowerConfidenceBound = 1.0 - confidenceInterval;

  unsigned int pointsInConfidenceInterval = 0;

  for (unsigned int i = 0; i < Npoints; i++)
  {
    double radiusOnUnitCircle = sqrtf(
                                      normCirc.at(i)->getX() * normCirc.at(i)->getX() + normCirc.at(i)->getY()
                                          * normCirc.at(i)->getY());
    if (radiusOnUnitCircle < upperConfidenceBound && radiusOnUnitCircle > lowerConfidenceBound)
    {
      pointsInConfidenceInterval++;
    }
  }

  double quality = (double)pointsInConfidenceInterval / (double)Npoints;

  output.push_back(quality);

  //find the occupied angle
  //how to do it: calculate the angle between all features and the x-axis and
  // look which areas are covered. We try to find a coherent area of angles and determine
  // the occupied angle by the difference between the lowest and the highest feature angle.


  std::vector<int> angleCounter(360, 0);
  for (unsigned int k = 0; k < Npoints; k++)
  {
    //calculate the distance of the current feature from the origin
    double distR = sqrtf(
                         normCirc.at(k)->getX() * normCirc.at(k)->getX() + normCirc.at(k)->getY()
                             * normCirc.at(k)->getY());

    //project the feature onto the unit circle (move it on a straight line in the direction of the unit circle)
    vision::FeaturePtr projectedOntoUnitCircle(
                                               new vision::Feature(normCirc.at(k)->getX() / distR,
                                                                   normCirc.at(k)->getY() / distR, 0.0));

    //calculate the angle between a line from the origin to the feature and the x-axis

    double angleToAxis = atan2(projectedOntoUnitCircle->getY(), projectedOntoUnitCircle->getX());

    //project the angle from the interval [-pi,pi] to [0,2pi]
    if (angleToAxis < 0)
      angleToAxis += 2 * M_PI;

    //convert from radians to degree
    angleToAxis = angleToAxis * 180.0 / M_PI;

    //convert the degree value to a integer (i.e 127.23 to 127) and add it to our counter
    if (int(floor(angleToAxis)) > 359)
      angleToAxis -= 360;

    //make sure we are in the angle interval, so that we do not get an out of memory exception
    if (int(floor(angleToAxis)) < 0 || int(floor(angleToAxis)) > 359)
    {
      //angle out of the interval should not happen, the only case where it does, is when it is undefined
      ROS_ERROR_STREAM_NAMED("FeatureBasedRevoluteJoint.circleFitByTaubin", "ERROR at " << k << " feature!");
      ROS_ERROR_STREAM_NAMED("FeatureBasedRevoluteJoint.circleFitByTaubin", "All feature values: ");
      for (int p = 0; p < Npoints; p++)
      {
        cout << p << ": x=" << f.at(p)->getX() << " y=" << f.at(p)->getY() << endl;
      }
      ROS_ERROR_STREAM_NAMED("FeatureBasedRevoluteJoint.circleFitByTaubin",
                             "x: " << projectedOntoUnitCircle->getX() << " y: " << projectedOntoUnitCircle->getY());
      ROS_ERROR_STREAM_NAMED("FeatureBasedRevoluteJoint.circleFitByTaubin", "Angle: " << angleToAxis);
      ROS_ERROR_STREAM_NAMED("FeatureBasedRevoluteJoint.circleFitByTaubin",
                             "NormCirc x: " << normCirc.at(k)->getX() << " y: " << normCirc.at(k)->getY());
      ROS_ERROR_STREAM_NAMED("FeatureBasedRevoluteJoint.circleFitByTaubin", "DistR: " << distR);
      ROS_ERROR_STREAM_NAMED("FeatureBasedRevoluteJoint.circleFitByTaubin",
                             "input: x=" << f.at(k)->getX() << " y=" << f.at(k)->getY());
      ROS_ERROR_STREAM_NAMED("FeatureBasedRevoluteJoint.circleFitByTaubin",
                             "ouput: 0: " << output.at(0) << " 1: " << output.at(1) << " 2: " << output.at(2));
      ROS_ERROR_STREAM_NAMED("FeatureBasedRevoluteJoint.circleFitByTaubin", int(floor(angleToAxis)));
    }
    else
    {
      angleCounter.at(int(floor(angleToAxis))) += 1;
    }
  }

  int k = 0;
  int lowBound = 0;
  std::vector<int> freeAngles;

  //find the free area where no angle values are located
  while (k < 360)
  {
    while (k < 360 && angleCounter.at(k) < 1)
    {
      k++;
    }
    //k should point to the first feature which is occupied
    freeAngles.push_back(k - lowBound);

    //lowBound is now the last occupied position
    lowBound = k;

    k++;
  }

  freeAngles.at(0) += freeAngles.at(freeAngles.size() - 1);
  freeAngles.at(freeAngles.size() - 1) = 0;

  int occupiedAngleMax = 360 - max(freeAngles);

  output.push_back(occupiedAngleMax);
  return output;
}

boost::numeric::ublas::matrix<double> FeatureBasedRevoluteJoint::magic_cosini(double A, double B, double C)
{
  //Original code in matlab from http://stackoverflow.com/questions/1023948/rotate-normal-vector-onto-axis-plane
  //translated to C++

  double bt = 0.0;
  if (sqrt(A * A + B * B) > 0.0)
  {
    bt = acos(B / sqrt(A * A + B * B));
  }
  else
  {
    //plane only defined in z-direction, which means all the features have equal x and z values
    //this case should not happen, but if it does, lets return the unit matrix, which does not affect the features
    boost::numeric::ublas::matrix<double> rot(3, 3);
    rot = eye(3);
    return rot;
  }
  boost::numeric::ublas::matrix<double> rotb(3, 3);

  if (A >= 0)
  {
    rotb(0, 0) = cos(-bt);
    rotb(0, 1) = -sin(-bt);
    rotb(0, 2) = 0;
    rotb(1, 0) = sin(-bt);
    rotb(1, 1) = cos(-bt);
    rotb(1, 2) = 0;
    rotb(2, 0) = 0;
    rotb(2, 1) = 0;
    rotb(2, 2) = 1;
  }
  else
  {
    rotb(0, 0) = cos(-bt);
    rotb(0, 1) = sin(-bt);
    rotb(0, 2) = 0;
    rotb(1, 0) = -sin(-bt);
    rotb(1, 1) = cos(-bt);
    rotb(1, 2) = 0;
    rotb(2, 0) = 0;
    rotb(2, 1) = 0;
    rotb(2, 2) = 1;
  }

  double B2 = A * rotb(0, 1) + B * rotb(1, 1) + C * rotb(2, 1);
  double C2 = A * rotb(0, 2) + B * rotb(1, 2) + C * rotb(2, 2);

  double a = 0;
  if (sqrt(C2 * C2 + B2 * B2) > 0.0)
  {
    a = acos(C2 / sqrt(B2 * B2 + C2 * C2));
  }
  else
  {
    //plane only defined in z-direction, which means all the features have equal x and z values
    //this case should not happen, but if it does, lets return the unit matrix, which does not affect the features
    boost::numeric::ublas::matrix<double> rot(3, 3);
    rot = eye(3);
    return rot;
  }
  boost::numeric::ublas::matrix<double> rota(3, 3);
  rota(0, 0) = 1;
  rota(0, 1) = 0;
  rota(0, 2) = 0;
  rota(1, 0) = 0;
  rota(1, 1) = cos(-a);
  rota(1, 2) = -sin(-a);
  rota(2, 0) = 0;
  rota(2, 1) = sin(-a);
  rota(2, 2) = cos(-a);

  // ROS_DEBUG_STREAM_NAMED("FeatureBasedRevoluteJoint.magic_cosini","Estimated alignment matrix: "<<std::endl<< prod(rotb, rota));

  return prod(rotb, rota);

}

std::vector<double> FeatureBasedRevoluteJoint::fitPlane(std::vector<vision::FeaturePtr> &featureVec)
{
  double A = 0;
  double B = 0;
  double C = 0;
  double D = 0;
  double qual = 0;

  int Nfeatures = featureVec.size();

  // Mean position of this Feature along the trajectory
  double x0 = 0.0;
  double y0 = 0.0;
  double z0 = 0.0;
  for (int i = 0; i < Nfeatures; i++)
  {
    x0 += featureVec.at(i)->getX();
    y0 += featureVec.at(i)->getY();
    z0 += featureVec.at(i)->getZ();
  }

  x0 /= (double)Nfeatures;
  y0 /= (double)Nfeatures;
  z0 /= (double)Nfeatures;
  ROS_DEBUG_STREAM_NAMED("FeatureBasedRevoluteJoint.fitPlane", "Number of frames: " << Nfeatures);

  // Standard deviation of the Feature with respect to its mean position
  boost::numeric::ublas::matrix<double> W = zeros(3, 3);
  for (int i = 0; i < Nfeatures; i++)
  {
    double xi = featureVec.at(i)->getX();
    double yi = featureVec.at(i)->getY();
    double zi = featureVec.at(i)->getZ();
    W(0, 0) += (xi - x0) * (xi - x0);
    W(0, 1) += (xi - x0) * (yi - y0);
    W(0, 2) += (xi - x0) * (zi - z0);

    W(1, 0) += (yi - y0) * (xi - x0);
    W(1, 1) += (yi - y0) * (yi - y0);
    W(1, 2) += (yi - y0) * (zi - z0);

    W(2, 0) += (zi - z0) * (xi - x0);
    W(2, 1) += (zi - z0) * (yi - y0);
    W(2, 2) += (zi - z0) * (zi - z0);
  }

  // Eigenvectors
  boost::numeric::ublas::matrix<double> eigen_values(3, 1);
  // Eigenvalues
  boost::numeric::ublas::matrix<double> eigen_vectors(3, 3);

  ROS_DEBUG_STREAM_NAMED("FeatureBasedRevoluteJoint.fitPlane",
                         "Compute Eigendecomposition of" << std::endl << "W=" << W);
  math::computeEig(W, eigen_values, eigen_vectors);
  ROS_DEBUG_STREAM_NAMED("FeatureBasedRevoluteJoint.fitPlane",
                         "Estimated Eigenvalues" << std::endl << "EValues=" << eigen_values);
  ROS_DEBUG_STREAM_NAMED("FeatureBasedRevoluteJoint.fitPlane",
                         "Estimated Eigenvectors" << std::endl << "EVectors=" << eigen_vectors);
  ROS_DEBUG_STREAM_NAMED(
                         "FeatureBasedRevoluteJoint.fitPlane",
                         "Relevant Eigenvalues: " << eigen_values(0, 2) << " " << eigen_values(0, 1) << " "
                             << eigen_values(0, 0));

  if (eigen_values(2, 0) < eigen_values(1, 0) && eigen_values(2, 0) < eigen_values(0, 0))
  {
    // Take the eigenvector to the eigenvalue s3 (third column)
    A = eigen_vectors(0, 2);
    B = eigen_vectors(1, 2);
    C = eigen_vectors(2, 2);
    D = -(eigen_vectors(0, 2) * x0 + eigen_vectors(1, 2) * y0 + eigen_vectors(2, 2) * z0);
  }
  else
  {
    if (eigen_values(1, 0) < eigen_values(2, 0) && eigen_values(1, 0) < eigen_values(0, 0))
    {
      // Take the eigenvector to the eigenvalue s2 (second column)
      A = eigen_vectors(0, 1);
      B = eigen_vectors(1, 1);
      C = eigen_vectors(2, 1);
      D = -(eigen_vectors(0, 1) * x0 + eigen_vectors(1, 1) * y0 + eigen_vectors(2, 1) * z0);

    }
    else
    {
      // Take the eigenvector to the eigenvalue s1 (first column)
      A = eigen_vectors(0, 0);
      B = eigen_vectors(1, 0);
      C = eigen_vectors(2, 0);
      D = -(eigen_vectors(0, 0) * x0 + eigen_vectors(1, 0) * y0 + eigen_vectors(2, 0) * z0);
    }
  }
  ROS_DEBUG_STREAM_NAMED("FeatureBasedRevoluteJoint.fitPlane",
                         "Estimated plane: A=" << A << " B=" << B << " C=" << C << " D=" << D);

  vector<double> output;
  output.push_back(A);
  output.push_back(B);
  output.push_back(C);
  output.push_back(D);
  output.push_back(qual); // TODO: qual is not set in the function (always =0). Do we need it?
  return output;

}
