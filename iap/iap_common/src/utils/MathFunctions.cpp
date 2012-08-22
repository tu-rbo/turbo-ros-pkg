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
#include "MathFunctions.h"

using namespace std;
using namespace vision;

double math::qerf(double x, double mean, double lambda)
{
  return (1.0 - boost::math::erf((x - mean) * lambda / sqrt(2.0))) / 2.0;
}
/*
 double math::erf(double x)
 {
 //erf(z) = 2/sqrt(pi) * Integral(0..x) exp( -t^2) dt
 //erf(0.01) = 0.0112834772 erf(3.7) = 0.9999998325
 //Abramowitz/Stegun: p299, |erf(z)-erf| <= 1.5*10^(-7)

 double y = 1.0 / ( 1.0 + 0.3275911 * x);
 return 1 - (((((
 + 1.061405429 * y
 - 1.453152027) * y
 + 1.421413741) * y
 - 0.284496736) * y
 + 0.254829592) * y)
 * exp (-x * x);
 }
 //*/
int math::numberSmallerThan(std::vector<double> &d, double threshold)
{
  int n = 0;
  for (unsigned int i = 0; i < d.size(); i++)
  {
    if (d.at(i) <= threshold)
      n++;
  }
  return n;

}
double math::stddev(std::vector<double> &vd)
{
  return double(sqrtf(math::var(vd)));

}
double math::correlation(std::vector<double> &d1, std::vector<double> &d2)
{
  double d1var = math::stddev(d1);
  double d2var = math::stddev(d2);

  double corr = math::cov(d1, d2) / (d1var * d2var);

  return corr;
}

double math::cov(std::vector<double> &d1, std::vector<double> &d2)
{
  assert(d1.size() == d2.size());
  double n = d1.size();
  double d1mean = math::mean(d1);
  double d2mean = math::mean(d2);

  double cov = 0.0;
  for (unsigned int i = 0; i < n; i++)
  {
    cov += (d1.at(i) - d1mean) * (d2.at(i) - d2mean);
  }
  return cov / n;
}

double math::highestDistanceFeature(vision::FeaturePtr &p, std::vector<vision::FeaturePtr> &f, int &bestIndex)
{
  double highestDistance = 0.0;
  bestIndex = math::iNaN;
  for (unsigned int i = 0; i < f.size(); i++)
  {
    double dist = math::abs(p, f.at(i));
    if (dist > highestDistance)
    {
      bestIndex = i;
      highestDistance = dist;
    }
  }
  return highestDistance;
}

double math::distanceFromLine(vision::FeaturePtr &p, vision::FeaturePtr &lineOne, vision::FeaturePtr &lineTwo)
{
#define DEBUGM 0
  if (DEBUGM)
    std::cout << "input: " << lineOne << " and " << lineTwo << std::endl;
  if (DEBUGM)
    std::cout << "input: " << p << std::endl;
  vision::FeaturePtr ca = math::sub(p, lineOne);
  vision::FeaturePtr cb = math::sub(p, lineTwo);
  vision::FeaturePtr ba = math::sub(lineTwo, lineOne);

  //std::cout << "ca: " << ca << " cb " << cb << " ba: " << ba << std::endl;
  vision::FeaturePtr cross = math::cross(ca, cb);
  //std::cout << "cross: " << cross << std::endl;

  if (DEBUGM)
    std::cout << "abs cross: " << math::abs(cross) << std::endl;

  if (DEBUGM)
    std::cout << "abs ba : " << math::abs(ba) << std::endl;
  double d = math::abs(cross) / math::abs(ba);

  //h = height of triangle ABP (with AB as base)
  //T = Area of triangle ABP
  //s = (AB + BP + PA)/2  This is called the semiperimeter

  double s = (math::abs(ba) + math::abs(ca) + math::abs(cb)) / 2.0;
  double T = sqrt(s * (s - math::abs(ba)) * (s - math::abs(ca)) * (s - math::abs(cb)));
  //Then, T = square root of {s*(s-AB)*(s-BP)*(s-PA)}

  //The desired distance h = 2*T/AB
  double h = 2.0 * T / math::abs(ba);
  if (fabs(d - h) > 0.2)
  {
    /*
     std::cout << "[math::distanceFromLine] Parallelogram check wrong.(d=" << d << " h="<< h << ")" << std::endl;
     vision::FeaturePtr a(1,2,3);
     vision::FeaturePtr b(4,5,6);
     vision::FeaturePtr c = math::cross(a, b);
     std::cout << "CROSS: " << c << std::endl;
     std::cout << "input: " << lineOne << " and " << lineTwo << std::endl;
     std::cout << "input: " << p << std::endl;

     std::cout << "ca: " << ca << " cb " << cb << " ba: " << ba << std::endl;

     std::cout << "cross: " << cross << std::endl;

     std::cout << "abs cross: " << math::abs(cross) << std::endl;

     std::cout << "abs ba : " << math::abs(ba) << std::endl;

     std::cout << "s : " << s << std::endl;

     std::cout << "T : " << T << std::endl;
     */

  }
  //std::cout << "distances: d=" << d << " and h=" << h << std::endl;


  return d;
}

std::vector<vision::FeaturePtr> math::abs(std::vector<vision::FeaturePtr> &featureVec)
{
  std::vector<vision::FeaturePtr> abs;
  for (unsigned int i = 0; i < featureVec.size(); i++)
  {
    vision::FeaturePtr f(new Feature(0.0, 0.0, 0.0));
    f->setX(fabs(featureVec.at(i)->getX()));
    f->setY(fabs(featureVec.at(i)->getY()));
    f->setZ(fabs(featureVec.at(i)->getZ()));
    abs.push_back(f);
  }
  return abs;
}

std::vector<vision::FeaturePtr> math::subtract(std::vector<vision::FeaturePtr> &fvl,
                                               std::vector<vision::FeaturePtr> &fvr)
{
  assert(fvl.size() == fvr.size());
  std::vector<vision::FeaturePtr> subtraction;
  for (unsigned int i = 0; i < fvl.size(); i++)
  {
    vision::FeaturePtr f(new Feature(0.0, 0.0, 0.0));
    f->setX(fvl.at(i)->getX() - fvr.at(i)->getX());
    f->setY(fvl.at(i)->getY() - fvr.at(i)->getY());
    f->setZ(fvl.at(i)->getZ() - fvr.at(i)->getZ());
    subtraction.push_back(f);
  }
  return subtraction;
}

double math::sum(std::vector<vision::FeaturePtr> &featureVec)
{

  std::vector<double> featureSums;
  for (unsigned int i = 0; i < featureVec.size(); i++)
  {
    featureSums.push_back(featureVec.at(i)->getX() + featureVec.at(i)->getY() + featureVec.at(i)->getZ());
  }

  double result = 0.0;
  for (unsigned int i = 0; i < featureSums.size(); i++)
  {
    result += featureSums.at(i);
  }
  return result;
}

double math::motionBetweenPointClouds(std::vector<vision::FeaturePtr> &fvl, std::vector<vision::FeaturePtr> &fvr)
{

  std::vector<vision::FeaturePtr> r = math::subtract(fvl, fvr);
  std::vector<vision::FeaturePtr> a = math::abs(r);
  return math::sum(a);
}

vision::FeaturePtr math::mean(vector<vision::FeaturePtr> &featureVec)
{
  //calculates the centroid of a point cloud:
  int numFeatures = featureVec.size();

  double xRes = 0.0, yRes = 0.0, zRes = 0.0;
  for (int counter = 0; counter < numFeatures; counter++)
  {
    xRes += featureVec[counter]->getX();
    yRes += featureVec[counter]->getY();
    zRes += featureVec[counter]->getZ();
  }
  vision::FeaturePtr meanVec(new Feature(xRes / (double)numFeatures, yRes / (double)numFeatures, zRes / (double)numFeatures));

  return meanVec;
}
vision::FeaturePtr math::var(vector<vision::FeaturePtr> &featureVec)
{
  int numFeatures = featureVec.size();
  vision::FeaturePtr meanF = math::mean(featureVec);
  double xRes = 0.0, yRes = 0.0, zRes = 0.0;
  for (int counter = 0; counter < numFeatures; counter++)
  {
    xRes += pow(featureVec[counter]->getX() - meanF->getX(), 2);
    yRes += pow(featureVec[counter]->getY() - meanF->getY(), 2);
    zRes += pow(featureVec[counter]->getZ() - meanF->getZ(), 2);
  }
  vision::FeaturePtr varVec(new Feature(xRes / numFeatures, yRes / numFeatures, zRes / numFeatures));
  return varVec;
}

double math::dvar(vector<vision::FeaturePtr> &featureVec)
{
  int numFeatures = featureVec.size();
  vision::FeaturePtr meanF = math::mean(featureVec);
  double xRes = 0.0, yRes = 0.0, zRes = 0.0;
  for (int counter = 0; counter < numFeatures; counter++)
  {
    xRes += pow(featureVec[counter]->getX() - meanF->getX(), 2);
    yRes += pow(featureVec[counter]->getY() - meanF->getY(), 2);
    zRes += pow(featureVec[counter]->getZ() - meanF->getZ(), 2);
  }
  return (xRes + yRes + zRes) / 3;
}

vision::FeaturePtr math::norm(vision::FeaturePtr &f)
{
  double absValue = sqrtf(pow(f->getX(), 2) + pow(f->getY(), 2) + pow(f->getZ(), 2));
  vision::FeaturePtr normF(new Feature(f->getX() / absValue, f->getY() / absValue, f->getZ() / absValue));
  return normF;
}

vision::FeaturePtr math::cross(vision::FeaturePtr &a, vision::FeaturePtr &b)
{
  vision::FeaturePtr cross(
                           new Feature(a->getY() * b->getZ() - a->getZ() * b->getY(),
                                       a->getZ() * b->getX() - a->getX() * b->getZ(),
                                       a->getX() * b->getY() - a->getY() * b->getX()));
  return cross;
}

double math::abs(vision::FeaturePtr &f)
{
  return sqrtf(f->getX() * f->getX() + f->getY() * f->getY() + f->getZ() * f->getZ());
}

double math::abs(const vision::FeaturePtr &f1, const vision::FeaturePtr &f2)
{
  return sqrtf(
               pow((double)f1->getX() - f2->getX(), 2) + pow((double)f1->getY() - f2->getY(), 2)
                   + pow((double)f1->getZ() - f2->getZ(), 2));
}

double math::l2norm(vision::FeaturePtr &f1, vision::FeaturePtr &f2)
{
  return math::abs(f1, f2);
}

double math::l_infinity_norm(vision::FeaturePtr &f1, vision::FeaturePtr &f2)
{

  vision::FeaturePtr x = math::sub(f1, f2);

  vector<double> t;
  t.push_back(std::abs(x->getX()));
  t.push_back(std::abs(x->getY()));
  t.push_back(std::abs(x->getZ()));

  return max(t);

}

double math::calculateFeatureMotion(vector<vision::FeaturePtr> &f)
{
  double motion = 0.0;
  for (unsigned int i = 1; i < f.size(); i++)
  {
    motion += math::abs(f.at(i), f.at(i - 1));
  }
  return motion;

}

double math::calculateOverallMotion(vector<vector<vision::FeaturePtr> > &bodyIn)
{
  std::vector<double> motion;
  for (unsigned int f = 0; f < bodyIn.at(0).size(); f++)
  {
    double motionCurrentFeature = 0.0;
    for (unsigned int t = 1; t < bodyIn.size(); t++)
    {
      motionCurrentFeature += math::abs(bodyIn.at(t).at(f), bodyIn.at(t - 1).at(f));
    }
    motion.push_back(motionCurrentFeature);
  }
  return math::max(motion);
}


double math::calculateOverallMotion(vector<vector<vision::FeaturePtr> > &bodyIn, int &index)
{
  std::vector<double> motion;
  for (unsigned int f = 0; f < bodyIn.at(0).size(); f++)
  {
    double motionCurrentFeature = 0.0;
    for (unsigned int t = 1; t < bodyIn.size(); t++)
    {
      motionCurrentFeature += math::abs(bodyIn.at(t).at(f), bodyIn.at(t - 1).at(f));
    }
    motion.push_back(motionCurrentFeature);
  }
  return math::max(motion, index);
}

vision::FeaturePtr math::max(vector<vision::FeaturePtr> &f)
{
  vision::FeaturePtr max(new Feature(f.at(0)->getX(), f.at(0)->getY(), f.at(0)->getZ()));
  for (unsigned int i = 0; i < f.size(); i++)
  {
    if (f.at(i)->getX() > max->getX())
      max->setX(f.at(i)->getX());
    if (f.at(i)->getY() > max->getY())
      max->setY(f.at(i)->getY());
    if (f.at(i)->getZ() > max->getZ())
      max->setZ(f.at(i)->getZ());
  }
  return max;
}
vision::FeaturePtr math::max(vector<vision::FeaturePtr> &f, int &index)
{
  vision::FeaturePtr max(new Feature(f.at(0)->getX(), f.at(0)->getY(), f.at(0)->getZ()));
  index = 0;
  for (unsigned int i = 0; i < f.size(); i++)
  {
    if (f.at(i)->getX() > max->getX())
    {
      max->setX(f.at(i)->getX());
      index = i;
    }
    if (f.at(i)->getY() > max->getY())
    {
      max->setY(f.at(i)->getY());
      index = i;
    }
    if (f.at(i)->getZ() > max->getZ())
    {
      max->setZ(f.at(i)->getZ());
      index = i;
    }
  }
  return max;
}

vision::FeaturePtr math::min(vector<vision::FeaturePtr> &f)
{
  vision::FeaturePtr min(new Feature(f.at(0)->getX(), f.at(0)->getY(), f.at(0)->getZ()));
  for (unsigned int i = 0; i < f.size(); i++)
  {
    if (f.at(i)->getX() < min->getX())
      min->setX(f.at(i)->getX());
    if (f.at(i)->getY() < min->getY())
      min->setY(f.at(i)->getY());
    if (f.at(i)->getZ() < min->getZ())
      min->setZ(f.at(i)->getZ());
  }
  return min;
}
double math::max(boost::numeric::ublas::matrix_column<boost::numeric::ublas::matrix<double> > &m)
{
  double result = numeric_limits<double>::min();
  for (unsigned int i = 0; i < m.size(); i++)
  {
    if (m(i) > result)
      result = m(i);
  }
  return result;

}

double math::min(boost::numeric::ublas::matrix_column<boost::numeric::ublas::matrix<double> > &m)
{
  double result = dMax;
  for (unsigned int i = 0; i < m.size(); i++)
  {
    if (m(i) < result)
      result = m(i);
  }
  return result;

}

double math::dot(vision::FeaturePtr &a, vision::FeaturePtr &b)
{
  return a->getX() * b->getX() + a->getY() * b->getY() + a->getZ() * b->getZ();

}

vision::FeaturePtr math::add(vision::FeaturePtr &a, vision::FeaturePtr &b)
{
  vision::FeaturePtr output(new Feature(a->getX() + b->getX(), a->getY() + b->getY(), a->getZ() + b->getZ()));
  return output;
}

vision::FeaturePtr math::sub(vision::FeaturePtr &a, vision::FeaturePtr &b)
{
  vision::FeaturePtr output(new Feature(a->getX() - b->getX(), a->getY() - b->getY(), a->getZ() - b->getZ()));
  return output;
}

vision::FeaturePtr math::prod(double scalar, vision::FeaturePtr &f)
{
  vision::FeaturePtr output(new Feature(scalar * f->getX(), scalar * f->getY(), scalar * f->getZ()));
  return output;
}

double math::det33(boost::numeric::ublas::matrix<double> &m)
{
  if (m.size1() > 3 || m.size1() < 3 || m.size2() > 3 || m.size2() < 3)
    throw "[det33] matrix is not of proper format.";

  return m(0, 0) * (m(1, 1) * m(2, 2) - m(1, 2) * m(2, 1)) - m(0, 1) * (m(1, 0) * m(2, 2) - m(2, 0) * m(1, 2))
      + m(0, 2) * (m(1, 0) * m(2, 1) - m(1, 1) * m(2, 0));

}

void math::transpose33(boost::numeric::ublas::matrix<double> &m)
{
  if (m.size1() > 3 || m.size1() < 3 || m.size2() > 3 || m.size2() < 3)
    throw "[transpose33] matrix is not of proper format.";

  boost::numeric::ublas::matrix<double> temp(m);
  temp(1, 0) = m(0, 1);
  temp(2, 0) = m(0, 2);
  temp(2, 1) = m(1, 2);

  temp(0, 1) = m(1, 0);
  temp(0, 2) = m(2, 0);
  temp(1, 2) = m(2, 1);

  m = temp;
}

vision::FeaturePtr math::mul33(vision::FeaturePtr f, boost::numeric::ublas::matrix<double> &m)
{
  if (m.size1() > 3 || m.size1() < 3 || m.size2() > 3 || m.size2() < 3)
    throw "[mul33] matrix is not of proper format.";

  vision::FeaturePtr output(new Feature(0.0, 0.0, 0.0));
  output->setX(f->getX() * m(0, 0) + f->getY() * m(1, 0) + f->getZ() * m(2, 0));
  output->setY(f->getX() * m(0, 1) + f->getY() * m(1, 1) + f->getZ() * m(2, 1));
  output->setZ(f->getX() * m(0, 2) + f->getY() * m(1, 2) + f->getZ() * m(2, 2));

  return output;
}

boost::numeric::ublas::matrix<double> math::inverse44(const boost::numeric::ublas::matrix<double> &m)
{
  if (m.size1() > 4 || m.size1() < 4 || m.size2() > 4 || m.size2() < 4)
    throw "[inverse44] matrix is not of proper format. Should be [4x4]";

  boost::numeric::ublas::matrix<double> temp(m);

  boost::numeric::ublas::matrix<double> R(3, 3);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      R(i, j) = temp(i, j);
  boost::numeric::ublas::matrix<double> T(3, 1);
  T(0, 0) = temp(0, 3);
  T(1, 0) = temp(1, 3);
  T(2, 0) = temp(2, 3);

  //the inverse is [R^T -R^T*T;0 0 0 1]
  transpose33(R);

  T = prod(R, T);

  temp(0, 3) = -T(0, 0);
  temp(1, 3) = -T(1, 0);
  temp(2, 3) = -T(2, 0);

  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      temp(i, j) = R(i, j);
  return temp;
}

double math::mean(vector<double> &vd)
{

  double mean = 0.0;
  for (unsigned int i = 0; i < vd.size(); i++)
  {
    mean += vd.at(i);
  }
  return mean / vd.size();
}

double math::sum(std::vector<double> &vd)
{
  double result = 0.0;
  for (unsigned int i = 0; i < vd.size(); i++)
  {
    result += vd.at(i);
  }
  return result;
}

long math::sum(std::vector<int> &vd)
{
  long result = 0;
  for (unsigned int i = 0; i < vd.size(); i++)
  {
    result += vd.at(i);
  }
  return result;
}

double math::max(std::vector<double> &vd)
{
  double result = numeric_limits<double>::min();
  for (unsigned int i = 0; i < vd.size(); i++)
  {
    if (vd.at(i) > result)
      result = vd.at(i);
  }
  return result;
}

int math::max(std::vector<int> &vd)
{
  int result = numeric_limits<int>::min();
  for (unsigned int i = 0; i < vd.size(); i++)
  {
    if (vd.at(i) > result)
      result = vd.at(i);
  }
  return result;
}

double math::max(std::vector<double> &vd, int &index)
{
  index = iNaN;
  double result = dMin;
  for (unsigned int i = 0; i < vd.size(); i++)
  {
    if (vd.at(i) > result)
    {
      result = vd.at(i);
      index = i;
    }
  }
  return result;
}

double math::max(const std::vector<double> &vd, int &index)
{
  index = iNaN;
  double result = dMin;
  for (unsigned int i = 0; i < vd.size(); i++)
  {
    if (vd.at(i) > result)
    {
      result = vd.at(i);
      index = i;
    }
  }
  return result;
}

double math::min(std::vector<double> &vd)
{
  double result = dMax;
  for (unsigned int i = 0; i < vd.size(); i++)
  {
    if (vd.at(i) < result)
    {
      result = vd.at(i);
    }
  }
  return result;
}

double math::min(std::vector<double> &vd, int &index)
{
  index = iNaN;
  double result = dMax;
  for (unsigned int i = 0; i < vd.size(); i++)
  {
    if (vd.at(i) < result)
    {
      result = vd.at(i);
      index = i;
    }
  }
  return result;
}

double math::var(vector<double> &vd)
{

  double variance = 0.0;
  double mean = math::mean(vd);
  for (unsigned int i = 0; i < vd.size(); i++)
  {
    variance += pow(vd.at(i) - mean, 2);
  }
  return variance / vd.size();
}

void math::norm(std::vector<double> &vd)
{
  double sum = 0.0;
  for (unsigned int i = 0; i < vd.size(); i++)
  {
    sum += vd.at(i);
  }
  for (unsigned int i = 0; i < vd.size(); i++)
  {
    vd.at(i) = vd.at(i) / sum;
  }
}

boost::numeric::ublas::matrix<double> math::eye(int s)
{
  assert(s > 0);
  boost::numeric::ublas::matrix<double> outcome(s, s);
  for (int i = 0; i < s; i++)
  {
    for (int j = 0; j < s; j++)
    {
      if (i == j)
        outcome(i, j) = 1.0;
      else
        outcome(i, j) = 0.0;
    }
  }
  return outcome;
}

boost::numeric::ublas::matrix<double> math::zeros(int s, int k)
{
  assert(s > 0 && k > 0);
  boost::numeric::ublas::matrix<double> outcome(s, k);
  for (int i = 0; i < s; i++)
  {
    for (int j = 0; j < k; j++)
    {
      outcome(i, j) = 0.0;
    }
  }
  return outcome;

}

boost::numeric::ublas::matrix<double> math::prodAll44(std::vector<boost::numeric::ublas::matrix<double> > &vm)
{
  if (vm.at(0).size1() > 4 || vm.at(0).size1() < 4 || vm.at(0).size2() > 4 || vm.at(0).size2() < 4)
    throw "[prodAll44] matrices are not of proper format. Should be [4x4]";

  boost::numeric::ublas::matrix<double> product(4, 4);
  product = eye(4);

  vector<boost::numeric::ublas::matrix<double> >::iterator vmIt;

  //for(vmIt=vm.begin();vmIt < vm.end();vmIt++)
  for (unsigned int i = 0; i < vm.size(); i++)
  {
    product = prod(vm.at(i), product);
  }
  return product;
}

double math::getRotationAngleFromMatrix(boost::numeric::ublas::matrix<double> &R)
{
  assert(R.size1() >= 3 && R.size2() >= 3);

  //calculating the quaternions
  double tr = R(0, 0) + R(1, 1) + R(2, 2);
  double qw = 0.0;
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  if (tr > 0)
  {
    double s = 0.5 / sqrtf(tr + 1.0);
    qw = 0.25 / s;
    qx = (R(2, 1) - R(1, 2)) * s;
    qy = (R(0, 2) - R(2, 0)) * s;
    qz = (R(1, 0) - R(0, 1)) * s;
  }
  else
  {
    if (R(0, 0) > R(1, 1) && R(0, 0) > R(2, 2))
    {
      double s = 2.0f * sqrtf(1.0f + R(0, 0) - R(1, 1) - R(2, 2));
      qw = (R(2, 1) - R(1, 2)) / s;
      qx = 0.25f * s;
      qy = (R(0, 1) + R(1, 0)) / s;
      qz = (R(0, 2) + R(2, 0)) / s;
    }
    else
    {
      if (R(1, 1) > R(2, 2))
      {
        double s = 2.0f * sqrtf(1.0f + R(1, 1) - R(0, 0) - R(2, 2));
        qw = (R(0, 2) - R(2, 0)) / s;
        qx = (R(0, 1) - R(1, 0)) / s;
        qy = 0.25f * s;
        qz = (R(1, 2) - R(2, 1)) / s;
      }
      else
      {
        double s = 2.0f * sqrtf(1.0f + R(2, 2) - R(0, 0) - R(1, 1));
        qw = (R(1, 0) - R(0, 1)) / s;
        qx = (R(0, 2) - R(2, 0)) / s;
        qy = (R(1, 2) - R(2, 1)) / s;
        qz = 0.25f * s;
      }

    }

  }
  //qx qy qz is the axis of rotation
  if (qw > 1)
    cout << "warning: quaternions not normalized.." << qw << endl;

  double angle = 2 * acos(qw);
  return (180.0 / 3.14159265) * angle;
}

void math::computeSVD(boost::numeric::ublas::matrix<double> &A, boost::numeric::ublas::matrix<double> &U,
                      boost::numeric::ublas::matrix<double> &S, boost::numeric::ublas::matrix<double> &V)
{
  //decompose a matrix A into its Singular Value Decomposition such that A= U*S*V'
  int n = A.size1();
  int m = A.size2();
  //cout << "matrix A [" << n << "," << m << "]" << endl;
  CvMat* a = cvCreateMat(n, m, CV_32FC1);
  for (int i = 0; i < n; i++)
  {
    for (int j = 0; j < m; j++)
    {
      cvmSet(a, i, j, A(i, j));
    }
  }
  CvMat* u = cvCreateMat(n, n, CV_32FC1);
  CvMat* ut = cvCreateMat(n, n, CV_32FC1);
  CvMat* d = cvCreateMat(n, m, CV_32FC1);
  CvMat* v = cvCreateMat(m, m, CV_32FC1);
  CvMat* vt = cvCreateMat(m, m, CV_32FC1);
  cvSVD(a, d, u, v, CV_SVD_U_T | CV_SVD_V_T); // A = U D V^T

  cvTranspose(u, ut);
  cvTranspose(v, vt);

  for (int i = 0; i < n; i++)
  {
    for (int j = 0; j < n; j++)
    {
      U(i, j) = cvmGet(ut, i, j);
    }
  }
  for (int i = 0; i < n; i++)
  {
    for (int j = 0; j < m; j++)
    {
      S(i, j) = cvmGet(d, i, j);
    }
  }

  for (int i = 0; i < m; i++)
  {
    for (int j = 0; j < m; j++)
    {
      V(i, j) = cvmGet(vt, i, j);
    }
  }

  cvReleaseMat(&a);
  cvReleaseMat(&u);
  cvReleaseMat(&ut);
  cvReleaseMat(&d);
  cvReleaseMat(&v);
  cvReleaseMat(&vt);
}

void math::computeEig(boost::numeric::ublas::matrix<double> &SourceMatrix, boost::numeric::ublas::matrix<double> &EigenValues,
                      boost::numeric::ublas::matrix<double> &EigenVectors)
{
  //compute the eigenvector of a symmetric matrix A

  int n = SourceMatrix.size1();
  int m = SourceMatrix.size2();

  if (n < m || m > n)
  {
    std::cout << "[ERROR] called computeEig with non-symmetric matrix!" << std::endl;
  }
  assert(n == m);

  CvMat* at = cvCreateMat(n, m, CV_32FC1);
  for (int i = 0; i < n; i++)
  {
    EigenValues(i,0) = 0.0;
    for (int j = 0; j < m; j++)
    {
      cvmSet(at, i, j, SourceMatrix(i, j)); // Set M(i,j)
      EigenVectors(i, j) = 0.0;
    }
  }

  cv::Mat source(at);
  cv::Mat eigen_values(n, 1, CV_32FC1);
  cv::Mat eigen_vectors(n, m, CV_32FC1);

  for (int i = 0; i < n; i++)
  {
    eigen_values.at<float> (i) = 0.0;
    for (int j = 0; j < m; j++)
    {
      eigen_vectors.at<float> (i, j) = 0.0;
    }
  }

  cv::eigen(source, eigen_values, eigen_vectors, 0, 0); // eigen_values = eigenvalues of A (descending order)
  // eigen_vectors = corresponding eigenvectors (rows)
  //write the values back, but in transposed order (so we can better read out the values)

  for (int i = 0; i < eigen_values.size().height; i++)
  {
    EigenValues(i,0) = (double)eigen_values.at<float> (i);
  }

  for (int i = 0; i < n; i++)
  {
    for (int j = 0; j < m; j++)
    {
      EigenVectors(j, i) = eigen_vectors.at<float> (i, j);
    }
  }

  //The eigenvalues are saved in EigenValues at the first row. The eigenvalue in EigenValues(0,0) corresponds
  //to eigenvector in the first column in EigenVectors(:,0).

  cvReleaseMat(&at);
}

double math::fitThreeDimensionalLine(std::vector<vision::FeaturePtr> &featureSet, vision::FeaturePtr &lowerLineEnd,
                                     vision::FeaturePtr &upperLineEnd)
{

  vision::FeaturePtr linePoint = math::mean(featureSet);

  //shift the feature set to the origin
  std::vector<vision::FeaturePtr> featureSetShiftedToOrigin;

  for (unsigned int i = 0; i < featureSet.size(); i++)
  {
    vision::FeaturePtr shiftedPoint(
                                    new Feature(featureSet.at(i)->getX() - linePoint->getX(),
                                                featureSet.at(i)->getY() - linePoint->getY(),
                                                featureSet.at(i)->getZ() - linePoint->getZ()));
    featureSetShiftedToOrigin.push_back(shiftedPoint);
  }

  //shifted data points
  boost::numeric::ublas::matrix<double> H(featureSet.size(), 3);

  for (unsigned int i = 0; i < featureSet.size(); i++)
  {
    H(i, 0) = featureSetShiftedToOrigin.at(i)->getX();
    H(i, 1) = featureSetShiftedToOrigin.at(i)->getY();
    H(i, 2) = featureSetShiftedToOrigin.at(i)->getZ();
  }

  //singular value decomposition
  //[U,S,V] = svd(H);
  boost::numeric::ublas::matrix<double> U(featureSet.size(), featureSet.size());
  boost::numeric::ublas::matrix<double> S(featureSet.size(), 3);
  boost::numeric::ublas::matrix<double> V(3, 3);

  math::computeSVD(H, U, S, V);

  vision::FeaturePtr lineDir(new Feature(V(0, 0), V(1, 0), V(2, 0)));

  boost::numeric::ublas::matrix_column<boost::numeric::ublas::matrix<double> > Sj(S, 1);
  double maxT = max(Sj) + 0.2;
  double minT = min(Sj) - 0.2;

  //line point and line dir contain now the end points
  lowerLineEnd->setX(linePoint->getX() + minT * lineDir->getX());
  lowerLineEnd->setY(linePoint->getY() + minT * lineDir->getY());
  lowerLineEnd->setZ(linePoint->getZ() + minT * lineDir->getZ());

  upperLineEnd->setX(linePoint->getX() + maxT * lineDir->getX());
  upperLineEnd->setY(linePoint->getY() + maxT * lineDir->getY());
  upperLineEnd->setZ(linePoint->getZ() + maxT * lineDir->getZ());

  double lengthOfLine = math::abs(lowerLineEnd, upperLineEnd);

  std::vector<double> pointDistanceToLine;

  double confidenceIntervalThreshold = 0.1 * lengthOfLine;//changed from 0.05 to 0.1

  for (unsigned int i = 0; i < featureSet.size(); i++)
  {
    pointDistanceToLine.push_back(math::distanceFromLine(featureSet.at(i), lowerLineEnd, upperLineEnd));
  }
  int pointsInInterval = math::numberSmallerThan(pointDistanceToLine, confidenceIntervalThreshold);

  return (double)pointsInInterval / (double)featureSet.size();
}

void math::printProgress(double k, double startK, double endK)
{
  cout << "\b\b\b\b\b" << flush;
  cout << setprecision(0) << setw(4);
  cout << fixed << 100 * ((k - startK) / (endK - startK)) + 1 << flush;
  cout << "%" << flush;
  cout << setprecision(3);
}

double math::normal(double x, double mu, double stddev)
{
  return (1.0 / (sqrtf(2.0 * M_PI) * stddev)) * exp((-(x - mu) * (x - mu)) / (2.0 * stddev * stddev));
}

double math::normalLambda(double x, double mu, double lambda, double stddev)
{
  return (1.0 / (sqrtf(2.0 * M_PI) * stddev)) * exp((-lambda * (x - mu) * (x - mu)) / (2.0 * stddev * stddev));
}

double math::gaussian( double mean, double std ) {
  const double norm = 1.0 / (RAND_MAX + 1.0);
  double u = 1.0 - rand() * norm;                  /* can't let u == 0 */
  double v = rand() * norm;
  double z = sqrt(-2.0 * log(u)) * cos(2.0 * M_PI * v);
  return mean + std * z;
}


double math::uniform(double max, double min) {
  static bool seeded = false;

  if(!seeded) {
    srand ( time(NULL) );
    seeded = true;
  }

  return (max - min) * ((double) rand()) / RAND_MAX + min;
}
