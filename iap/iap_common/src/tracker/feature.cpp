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
/** ***************************************************************************
 * Feature Model (implementation)
 ******************************************************************************
 * $Author: dubik $
 * $Date: 2009/06/15 12:23:49 $
 * $Revision: 1.5 $
 *****************************************************************************/
#include "feature.h"
#include <string>

#include "FeatureSet.h"

using namespace vision;

void Feature::init()
{
  static int _feature_id_counter = 0;
  _feature_id = _feature_id_counter;
  _feature_id_counter++;
  _x = -1.f;
  _y = -1.f;
  _z = -1.f;
  _lost = false;
  _error = -1.f;
  _quality = -1.f;
  _position_uncertainty = -1.f;
  _r = -1;
  _g = -1;
  _b = -1;
}

Feature::Feature(float x, float y, bool lost, float error, float quality, int groupId)
{
  init();
  _quality = quality;
  _group_id = groupId; // FIXME
  _x = x;
  _y = y;
  _lost = lost;
  _error = error;
}

Feature::Feature(float x, float y, Pixel &p, bool lost, float error, float quality, int groupId)
{
  init();
  _quality = quality;
  _group_id = groupId; // FIXME
  _x = x;
  _y = y;
  _lost = lost;
  _error = error;
  _r = (int)p.R;
  _g = (int)p.G;
  _b = (int)p.B;
}

Feature::Feature(float x, float y, float z, int r, int g, int b)
{
  init();
  _x = x;
  _y = y;
  _z = z;
  _r = r;
  _g = g;
  _b = b;
}

Feature::Feature(float x, float y, float z)
{
  init();
  _x = x;
  _y = y;
  _z = z;
}

Feature::Feature(float x, float y)
{
  init();
  _x = x;
  _y = y;
}

Feature::Feature(const Feature &f)
{
  _feature_id = f.getId(); // We copy the Id, so the copied Feature has the same one
  _group_id = f.getGroupId(); // FIXME
  _x = f.getX();
  _y = f.getY();
  _z = f.getZ();
  _r = f.getR();
  _g = f.getG();
  _b = f.getB();
  _lost = f.isLost();
  _error = f.getError();
  _quality = f.getQuality();
  _position_uncertainty = f.getPosUncertainty();
}

FeaturePtr Feature::clone() const
{
  return (FeaturePtr(doClone()));
}

FeaturePtr Feature::cloneAndUpdate(float x, float y) const
{
  FeaturePtr feature_ret = (FeaturePtr(doClone()));
  feature_ret->setX(x);
  feature_ret->setY(y);
  return feature_ret;
}

FeaturePtr Feature::cloneAndUpdate(float x, float y, float z) const
{
  FeaturePtr feature_ret = (FeaturePtr(doClone()));
  feature_ret->setX(x);
  feature_ret->setY(y);
  feature_ret->setZ(z);
  return feature_ret;
}
// ============================================================================

std::pair<float, float> Feature::getPos2D() const
{
  return std::pair<float, float>(_x, _y);
}

void Feature::setPos(float x, float y)
{
  _x = x;
  _y = y;
}

void Feature::setPos(float x, float y, float z)
{
  _x = x;
  _y = y;
  _z = z;
}

void Feature::setX(float x)
{
  _x = x;
}

void Feature::setY(float y)
{
  _y = y;
}

void Feature::setZ(float z)
{
  _z = z;
}

float Feature::getX() const
{
  return _x;
}

float Feature::getY() const
{
  return _y;
}

float Feature::getZ() const
{
  return _z;
}

void Feature::setR(int r)
{
  _r = r;
}

void Feature::setG(int g)
{
  _g = g;
}

void Feature::setB(int b)
{
  _b = b;
}

int Feature::getR() const
{
  return _r;
}

int Feature::getG() const
{
  return _g;
}

int Feature::getB() const
{
  return _b;
}

void Feature::setPosUncertainty(float posUncertainty)
{
  _position_uncertainty = posUncertainty;
}

float Feature::getPosUncertainty() const
{
  return _position_uncertainty;
}

void Feature::setLost(bool lost)
{
  _lost = lost;
}

bool Feature::isLost() const
{
  return _lost;
}

void Feature::setError(float error)
{
  _error = error;
}

float Feature::getError() const
{
  return _error;
}

int Feature::getId() const
{
  return _feature_id;
}

void Feature::setId(int id)
{
  _feature_id = id;
}

Feature& Feature::operator=(const Feature& f)
{
  if (this != &f)
  {
    _feature_id = f._feature_id;
    _x = f._x;
    _y = f._y;
    _z = f._z;
    _r = f._r;
    _g = f._g;
    _b = f._b;
    _lost = f._lost;
    _error = f._error;
    _position_uncertainty = f._position_uncertainty;
    _quality = f._quality;
  }
  return *this;
}

bool Feature::operator+=(Feature f)
{
  _x += f._x;
  _y += f._y;
  _z += f._z;
  return true;
}

float Feature::compareTo(std::pair<float, float> pt, const Image* img, const Image* mask)
{
  throw std::string("compareTo functionality is not defined for Feature");
  return 0.0;
}

bool Feature::isFeature(std::pair<float, float> pt, const Image* img, const Image* mask)
{
  throw std::string("isFeature functionality is not defined for Feature");
  return false;
}

void Feature::generateDescriptor(std::pair<float, float> pt, const Image *img, const Image *mask)
{
  throw std::string("generateDescriptor functionality is not defined for Feature");
}

void Feature::setIDOld(int id)
{
  clusterIdOld = id;
}
void Feature::setIDNew(int id)
{
  clusterIdNew = id;
}

int Feature::getGTLinkID() const
{
  return m_groundTruthLinkId;
}
void Feature::setGTLinkID(int id)
{
  m_groundTruthLinkId = id;
}

double Feature::norm() const
{
  return sqrt((this->_x) * (this->_x) + (this->_y) * (this->_y) + (this->_z) * (this->_z));
}

bool Feature::normalize()
{
  double norm = this->norm();
  if (norm == 0)
  {
    return false;
  }
  else
  {
    this->_x /= norm;
    this->_y /= norm;
    this->_z /= norm;
    return true;
  }
}

void Feature::setVolume(double v)
{
  _volume = v;
}
double Feature::getVolume() const
{
  return _volume;
}
ColorType Feature::getColor() const
{
  return _color;
}

void operator>>(std::string &s, vision::FeaturePtr output)
//std::istream &operator>>(std::istream &inputStream, vision::FeaturePtr output)
{
  std::istringstream iss(s);
  if (!iss)
  {
    std::cout << "ERROR[Feature::operatorFS(std::str, vision::FeaturePtr)]: Could not read ID" << std::endl;
    return;
  }

  std::string sub;
  iss >> sub;
  output->setId(atoi(sub.c_str()));

  if (!iss)
  {
    std::cout << "ERROR[Feature::operatorFS(std::str, vision::FeaturePtr)]: Could not read x" << std::endl;
    return;
  }
  iss >> sub;
  output->setX(atof(sub.c_str()));

  if (!iss)
  {
    std::cout << "ERROR[Feature::operatorFS(std::str, vision::FeaturePtr)]: Could not read x" << std::endl;
    return;
  }
  iss >> sub;
  output->setY(atof(sub.c_str()));

  if (!iss)
  {
    std::cout << "ERROR[Feature::operatorFS(std::str, vision::FeaturePtr)]: Could not read x" << std::endl;
    return;
  }
  iss >> sub;
  output->setZ(atof(sub.c_str()));
}

std::ostream &operator<<(std::ostream &outputStream, vision::Feature input)
{
  outputStream << input.getId() << " " << input.getX() << " " << input.getY() << " " << input.getZ();
  return outputStream;
}

std::ostream &operator<<(std::ostream &outputStream, vision::FeaturePtr input)
{
  outputStream << *input.get();
  return outputStream;
}
