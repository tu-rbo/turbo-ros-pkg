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
#ifndef GRAPH_VN_H_
#define GRAPH_VN_H_

#include <vector>
#include <string>
#include "Vertex.h"
#include "Edge.h"
#include "BKgraph.h"
#include "feature.h"
#include "Triangulate.h"
#include "Predictor.h"

namespace VisualGraph
{

/**
 \class    Graph
 \brief    A graph stores the visual net data structure.
 It supports all the basic operations of a graph + mincut/maxflow
 It is possible to reuse the previous graph just modifying the nodes, edges and weights
 **/

class Graph
{
private:
  Edge ***adjMatrix;
  Vertex **vList;
  int numVertices;
  std::vector<Predictor*> predictors;

public:
  Graph(void);

  Graph(Vertex **vlist, Edge ***adjmatrix, int numvertices);

  ~Graph(void);

  void clear(); //clear the object's memory

  //basic functionality
  int getNumEdges();

  int getNumVertices();

  bool addVertex(Vertex *v);

  bool removeVertex(int vertex_id);

  bool updateVertex(int vertex_id, std::vector<vision::FeaturePtr> &new_data);

  bool addEdge(Edge *e);

  bool isHighlyConnected();

  std::vector<vision::FeaturePtr> getFeatures(int frame);

  std::vector<int> getFeatureClusters();

  //graph algorithms
  void getSTMaxflow(int S, int T, bool *cut, double &maxflow);

  void getMinCut(bool *cut, double &mincut);

  void HCS(std::vector<int> &featuresID, std::map<int, int> &clusters);

  //utility functions
  void printConnectivity();

  bool areConnected(Vertex *v1, Vertex *v2);

  std::vector<std::vector<bool> > getConnectivityMatrix();

  //predictors
  void addPredictor(Predictor *p);

  void predict();

  std::vector<Predictor*> getPredictors();

  std::vector<std::vector<double> > getWeights();

private:
  int findVertex(Vertex *v);

  void recHCS(Graph *G, int *groupID);

  int getMaxVal(int *arr, int size);

  Graph* subGraph(bool *cut, bool val);
};

}
;

#endif
