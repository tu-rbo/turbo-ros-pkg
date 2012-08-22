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
#ifndef VERTEX_VN_H_
#define VERTEX_VN_H_

#include <vector>
#include <boost/shared_ptr.hpp>

namespace vision{
  class Feature;
  typedef boost::shared_ptr<Feature> FeaturePtr;
}

namespace VisualGraph
{

  /**
   \class    Vertex
   \brief    A vertex for the Graph class. It stores feature history
   and rigid body association (cluster)
   **/

  class Vertex
  {
  private:
    int ID;
    std::vector<vision::FeaturePtr> featureData;
    int clusterID;

  public:
    Vertex(void);

    Vertex(std::vector<vision::FeaturePtr> &featureData);
    ~Vertex(void);

    vision::FeaturePtr
    getFeatureData(int frame);

    std::vector<vision::FeaturePtr>
    getFeatureHistory();

    void
    changeFeatureHistory(std::vector<vision::FeaturePtr> &new_feature_data);

    void
    setClusterID(int id);

    int
    getClusterID();

    int
    getID();
  };
}
;

#endif
