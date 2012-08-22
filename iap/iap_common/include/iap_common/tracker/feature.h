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
#ifndef FEATURE_H
#define FEATURE_H

#include <utility>
#include "image.h"
#include "GlobalDefs.h"
#include <boost/shared_ptr.hpp>

namespace vision
{
//forward declaration
class Image;
class FeatureSet;
typedef boost::shared_ptr<FeatureSet> FeatureSetPtr;

class Feature;
typedef boost::shared_ptr<Feature> FeaturePtr;

/**
 *\class    Feature
 *\brief    Describes a region or a point of an image for tracking.
 *$Author: Katz&Hoefer&Martin $
 *$Revision: 2.0 $
 */
class Feature
{
public:
  /**
   * Feature constructor called by its subclasses, creates a feature and its
   * position.
   *
   * @param x - Floating point x coordinate of the center of the Feature
   * @param y - Floating point y coordinate of the center of the Feature
   * @param z - Floating point z coordinate of the center of the Feature
   * @param r - Red component of the color of the Feature
   * @param g - Green component of the color of the Feature
   * @param b - Blue component of the color of the Feature
   * @param isLost - If true the Feature will be considered to be no longer on the image
   * @param error - The measurement of how well the Feature have been tracked from last to current frame
   */
  Feature(float x, float y, bool isLost, float error, float quality, int groupId);
  Feature(float x, float y, Pixel &p, bool lost, float error, float quality, int groupId);
  Feature(float x, float y, float z, int r, int g, int b);
  Feature(float x, float y, float z);
  Feature(float x, float y);

  /**
   * Copy constructor
   * @param f - Feature to be copied
   */
  Feature(const Feature &f);

  /**
   * Default constructor
   */
  Feature()
  {
  }
  ;

  /**
   * Creates a new Feature object with the same values as this one and passes
   * its reference
   *
   * @return - A reference to the clone of this Feature object
   */
  FeaturePtr clone() const;

  /**
   * Creates a new Feature object with the same values as this one and passes
   * its reference, but changing the x and y coordinates of its center
   *
   * @param x - New x coordinate of the center of the feature
   * @param y - New y coordinate of the center of the feature
   * @return - A reference to the clone of this Feature object
   */
  FeaturePtr cloneAndUpdate(float x, float y) const;

  /**
   * Creates a new Feature object with the same values as this one and passes
   * its reference, but changing the x, y and z coordinates of its center
   *
   * @param x - New x coordinate of the center of the feature
   * @param y - New y coordinate of the center of the feature
   * @param z - New z coordinate of the center of the feature
   * @return - A reference to the clone of this Feature object
   */
  FeaturePtr cloneAndUpdate(float x, float y, float z) const;

  /**
   * Returns the position of the center of the Feature as an std::pair of floating point values
   *
   * @return - Pair of floating point values where first is x coordinate and second is y coordinate of the center of the feature
   */
  std::pair<float, float> getPos2D() const;

  /**
   * Sets the center position of the center of the feature
   * @param x - New x coordinate of the center of the feature
   * @param y - New y coordinate of the center of the feature
   */
  void setPos(float x, float y);

  /**
   * Sets the center position of the center of the feature
   * @param x - New x coordinate of the center of the feature
   * @param y - New y coordinate of the center of the feature
   * @param z - New z coordinate of the center of the feature
   */
  void setPos(float x, float y, float z);

  /**
   * Sets the x coordinate of the center of the feature
   * @param x - New x coordinate of the center of the feature
   */
  void setX(float x);

  /**
   * Sets the y coordinate of the center of the feature
   * @param y - New y coordinate of the center of the feature
   */
  void setY(float y);

  /**
   * Sets the z coordinate of the center of the feature
   * @param z - New z coordinate of the center of the feature
   */
  void setZ(float z);

  /**
   * Gets the x coordinate of the center of the feature
   * @return - x coordinate of the center of the feature
   */
  float getX() const;

  /**
   * Gets the y coordinate of the Feature
   * @return - the y position of the center of the feature
   */
  float getY() const;

  /**
   * Gets the z coordinate of the Feature
   * @return - the z position of the center of the feature
   */
  float getZ() const;

  /**
   * Sets the red component of the color of the Feature
   * @param r - Red component of the color of the Feature
   */
  void setR(int r);

  /**
   * Sets the green component of the color of the Feature
   * @param g - Green component of the color of the Feature
   */
  void setG(int g);

  /**
   * Sets the blue component of the color of the Feature
   * @param b - Blue component of the color of the Feature
   */
  void setB(int b);

  /**
   * Gets the red component of the color of the Feature
   * @return - Red component of the feature
   */
  int getR() const;

  /**
   * Gets the green component of the color of the Feature
   * @return - Green component of the feature
   */
  int getG() const;

  /**
   * Gets the blue component of the color of the Feature
   * @return - Blue component of the feature
   */
  int getB() const;

  /**
   * Sets the uncertainty value of the position of the center of the Feature
   * @param posUncertainty - New uncertainte value of the center of the Feature
   */
  void setPosUncertainty(float posUncertainty);

  /**
   * Gets the uncertainty value of the position of the center of the Feature
   * @return - Position uncertainty of the center of the Feature
   */
  float getPosUncertainty() const;

  /**
   * Marks this Feature as lost (not longer in the image) or not
   * @param lost - TRUE the feature is no longer on the image
   */
  void setLost(bool isLost);

  /**
   * Checks if the Feature is to be found in the image or not
   * @return - TRUE the Feature is no longer on the image
   */
  bool isLost() const;

  /**
   * Sets the error in the tracking from previous to current frame
   * @param error - Measure of how well the Feature has been tracked
   */
  void setError(float error);

  /**
   * Gets the error in the tracking from previous to current frame
   * @return - Measure of how well this feature was tracked
   */
  float getError() const;

  int getGroupId() const
  {
    return _group_id;
  }
  ;

  void setGroupId(int groupId)
  {
    _group_id = groupId;
  }
  ;

  float getQuality() const
  {
    return _quality;
  }
  ;

  void setQuality(float quality)
  {
    _quality = quality;
  }
  ;

  int getId() const;

  void setId(int id);

  /**
   * @brief
   *
   * @param color
   */
  void setColor(ColorType color);

  /**
   * @brief
   *
   * @param actorId
   */
  void setActorID(unsigned int actorId);
  /**
   * @brief
   *
   * @param v
   */
  void setVolume(double v);

  //getters
  /**
   * @brief
   *
   */
  double getVolume() const;
  /**
   * @brief
   *
   */
  ColorType getColor() const;
  /**
   * @brief
   *
   */
  unsigned int getActorID() const;

  /**
   * @brief
   *
   */
  int getIDOld() const;
  /**
   * @brief
   *
   */
  int getIDNew() const;
  /**
   * @brief
   *
   * @param id
   */
  void setIDOld(int id);
  /**
   * @brief
   *
   * @param id
   */
  void setIDNew(int id);

  /**
   * @brief
   *
   */
  int getGTLinkID() const;
  /**
   * @brief
   *
   * @param id
   */
  void setGTLinkID(int id);

  double norm() const;

  bool normalize();

  double dot(FeaturePtr f) {
    return this->_x * f->getX()
            + this->_y * f->getY()
            + this->_z * f->getZ();
  }

  /**
   * @param f - feature to compare to
   * @return - true/false depending on equality of two features
   */
  bool operator==(Feature &f)
  {
    if (f.getX() == _x && f.getY() == _y && f.getZ() == _z)
      return true;
    else
      return false;
  }

  Feature& operator=(const Feature& f);

  bool operator+=(Feature f);

  friend Feature operator/(Feature &f, float scale)
  {
    Feature scaledF(f);
    scaledF._x /= scale;
    scaledF._y /= scale;
    scaledF._z /= scale;
    return scaledF;
  }

  friend FeaturePtr operator/(FeaturePtr f, float scale)
  {
    FeaturePtr scaledF(new Feature (
        f->_x / scale,
        f->_y / scale,
        f->_z / scale
        ));
    return scaledF;
  }



  friend Feature operator-(Feature &f1, Feature &f2)
  {
    Feature f(f1);
    f._x -= f2._x;
    f._y -= f2._y;
    f._z -= f2._z;
    return f;
  }

  friend FeaturePtr operator+(FeaturePtr f1, FeaturePtr f2)
  {
    FeaturePtr f(new Feature(
        f1->getX() + f2->getX(),
        f1->getY() + f2->getY(),
        f1->getZ() + f2->getZ()));
    return f;
  }

  bool operator+=(FeaturePtr f1)
  {
    this->_x += f1->getX();
    this->_y += f1->getY();
    this->_z += f1->getZ();
    return true;
  }

  friend FeaturePtr operator-(FeaturePtr f1, FeaturePtr f2)
  {
    FeaturePtr f(new Feature(
        f1->getX() - f2->getX(),
        f1->getY() - f2->getY(),
        f1->getZ() - f2->getZ()));
    return f;
  }


  virtual float compareTo(std::pair<float, float> pt, const Image *img, const Image *mask);
  virtual bool isFeature(std::pair<float, float> pt, const Image *img, const Image *mask);
  virtual void generateDescriptor(std::pair<float, float> pt, const Image *img, const Image *mask);

  /**
   * Default destructor
   */
  virtual ~Feature()
  {

  }

private:
  void init();

protected:
  int _feature_id; //Unique identifier for every Feature
  int _group_id; //Deprecated (used by BlobDetector)
  float _x, _y, _z; //Position of the Feature
  int _r, _g, _b; //Color of the Feature in RGB scale
  bool _lost; //If _lost==TRUE the Feature is not longer on the image (_x, _y and _z are undefined)
  float _position_uncertainty; //Uncertainty of the position of the Feature (for tracking)
  float _error; //Error when the Feature has been tracked
  float _quality; //Level of goodness of this Feature

  int clusterIdOld; /**< TODO */
  int clusterIdNew; /**< TODO */
  unsigned int _actorId; // Physical actor identifier /**< TODO */
  int m_groundTruthLinkId; /**< TODO */
  ColorType _color; /**< TODO */
  double _volume; /**< TODO */

  virtual Feature* doClone() const
  {
    return (new Feature(*this));
  }

  // TODO optional feature descriptor object

};
}
;

/**
 * Initialize a feature from a string containing:
 *    id      x       y       z
 * separated
 */
void operator>>(std::string &s, vision::FeaturePtr input);


/**
 * ostream operator to use with Feature
 *
 * @param outputStream - std::ostream to append the content of the Feature
 * @param input - This Feature
 * @return A new std::ostream with the content of the Feature appended
 */
std::ostream &operator<<(std::ostream &outputStream, vision::Feature input);

/**
 * ostream operator to use with Feature
 *
 * @param outputStream - std::ostream to append the content of the Feature
 * @param input - This Feature
 * @return A new std::ostream with the content of the Feature appended
 */
std::ostream &operator<<(std::ostream &outputStream, vision::FeaturePtr input);

#endif
