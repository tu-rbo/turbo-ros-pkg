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
#include "Graph.h"
#include "BKgraph.cpp"
#include "BKmaxflow.cpp"
#include <iostream>

using namespace std;
using namespace vision;

namespace VisualGraph
{

Graph::Graph(void)
{
  vList = NULL;
  adjMatrix = NULL;
  numVertices = 0;
}

Graph::Graph(Vertex **vlist, Edge ***adjmatrix, int numvertices)
{
  numVertices = numvertices;
  vList = vlist;
  adjMatrix = adjmatrix;
}

Graph::~Graph(void)
{
  if (vList)
    delete (vList);
  vList = NULL;

  if (adjMatrix)
  {
    for (int i = 0; i < numVertices; i++)
      delete (adjMatrix[i]);
    delete (adjMatrix);
    adjMatrix = NULL;
  }

  //delete list of predictors
  for (vector<Predictor*>::iterator it = predictors.begin(); it != predictors.end(); it++)
  {
    delete (*it);
  }
}

void Graph::clear()
{
  if (vList)
  {
    for (int i = 0; i < numVertices; i++)
      delete (vList[i]);
    delete (vList);
    vList = NULL;
  }

  if (adjMatrix)
  {
    for (int i = 0; i < numVertices; i++)
      for (int j = 0; j < numVertices; j++)
        delete (adjMatrix[i][j]);
    for (int i = 0; i < numVertices; i++)
      delete (adjMatrix[i]);
    delete (adjMatrix);
    adjMatrix = NULL;
  }
}

int Graph::getNumVertices()
{
  return numVertices;
}

bool Graph::addEdge(Edge *e)
{
  if (numVertices < 2)
    return false;
  int i = findVertex(e->getVi());
  int j = findVertex(e->getVe());
  delete (adjMatrix[i][j]);
  adjMatrix[i][j] = e;
  return true;
}

int Graph::findVertex(Vertex *v)
{
  for (int i = 0; i < numVertices; i++)
    if (vList[i] == v)
      return i;
  throw("Vertex is not part of the graph!");
}

bool Graph::addVertex(Vertex *v)
{
  if (numVertices == 0)
  {
    vList = new Vertex*[1];
    vList[0] = v;
    numVertices++;
  }
  else
  {
    Vertex **newList = new Vertex*[numVertices + 1];
    for (int i = 0; i < numVertices; i++)
      newList[i] = vList[i];

    delete (vList);
    vList = newList;

    vList[numVertices] = v;
    numVertices++;
    //now we need to update the table of edges
    if (numVertices == 1)
      return true; //no need to create edges

    if (numVertices == 2)
    {
      adjMatrix = new Edge**[numVertices];
      for (int i = 0; i < numVertices; i++)
        adjMatrix[i] = new Edge*[numVertices];
      for (int i = 0; i < numVertices; i++)
      {
        for (int j = 0; j < numVertices; j++)
        {
          adjMatrix[i][j] = new Edge(vList[i], vList[j]);
          adjMatrix[j][i] = new Edge(vList[j], vList[i]);
        }
      }
    }
    else
    { //the case where numVertices>=3
      Edge ***newAdjMatrix = new Edge**[numVertices];
      for (int i = 0; i < numVertices; i++)
        newAdjMatrix[i] = new Edge*[numVertices];
      for (int i = 0; i < numVertices - 1; i++)
        for (int j = 0; j < numVertices - 1; j++)
          newAdjMatrix[i][j] = adjMatrix[i][j];
      for (int i = 0; i < numVertices; i++)
      {
        newAdjMatrix[i][numVertices - 1] = new Edge(vList[i], vList[numVertices - 1]);
        newAdjMatrix[numVertices - 1][i] = new Edge(vList[numVertices - 1], vList[i]);
      }
      delete (adjMatrix);
      adjMatrix = newAdjMatrix;
    }
  }
  return true;
}

bool Graph::removeVertex(int vertex_id)
{
  int index_to_remove = -1;
  for (int i = 0; i < numVertices; i++)
  {
    if (vList[i]->getID() == vertex_id)
    {
      index_to_remove = i;
      break;
    }
  }
  if (index_to_remove < 0) // There is no vertex with the given vertex_id in this graph
  {
    return false;
  }
  else
  {
    Vertex **newList = new Vertex*[numVertices - 1];
    for (int i = 0; i < numVertices - 1; i++)
    {
      if (i < index_to_remove)
      {
        newList[i] = vList[i];
      }
      else
      {
        newList[i] = vList[i + 1];
      }
    }
    delete (vList);
    vList = newList;

    Edge ***newAdjMatrix = new Edge**[numVertices - 1];
    for (int i = 0; i < numVertices - 1; i++)
    {
      newAdjMatrix[i] = new Edge*[numVertices - 1];
    }
    for (int i = 0; i < numVertices - 1; i++)
    {
      if (i < index_to_remove)
      {
        for (int j = 0; j < numVertices - 1; j++)
        {
          if (j < index_to_remove)
          {
            newAdjMatrix[i][j] = adjMatrix[i][j];
          }
          else
          {
            newAdjMatrix[i][j] = adjMatrix[i][j + 1];
          }
        }
      }
      else
      {
        for (int j = 0; j < numVertices - 1; j++)
        {
          if (j < index_to_remove)
          {
            newAdjMatrix[i][j] = adjMatrix[i + 1][j];
          }
          else
          {
            newAdjMatrix[i][j] = adjMatrix[i + 1][j + 1];
          }
        }

      }
    }
    numVertices--;
    delete (adjMatrix);
    adjMatrix = newAdjMatrix;
  }
  return true;
}

bool Graph::updateVertex(int vertex_id, std::vector<vision::FeaturePtr> &new_data)
{
  for (int i = 0; i < numVertices; i++)
  {
    if (vList[i]->getID() == vertex_id)
    {
      vList[i]->changeFeatureHistory(new_data);
      return true;
    }
  }
  return false;
}

void Graph::printConnectivity()
{
  for (int i = 0; i < numVertices; i++)
  {
    for (int j = 0; j < numVertices; j++)
    {
      cout << adjMatrix[i][j]->isConnected() << " ";
    }
    cout << endl;
  }
}

int Graph::getNumEdges()
{
  int numEdges = 0;
  for (int i = 0; i < numVertices; i++)
  {
    for (int j = 0; j < numVertices; j++)
    {
      numEdges += adjMatrix[i][j]->isConnected();
    }
  }
  return numEdges;
}

void Graph::getSTMaxflow(int S, int T, bool *cut, double &maxflow)
{
  int numEdges = getNumEdges();
  int srcEdges = 0;
  int sinkEdges = 0;
  for (int i = 0; i < numVertices; i++)
    srcEdges += adjMatrix[S][i]->isConnected();
  for (int i = 0; i < numVertices; i++)
    sinkEdges += adjMatrix[i][T]->isConnected();
  numEdges = numEdges - srcEdges - sinkEdges;

  typedef BKGraph::Graph<double, double, double> GraphType;
  GraphType g(numVertices, numEdges);

  //add regular nodes to the graph
  for (int i = 0; i < numVertices - 2; i++)
  {
    g.add_node();
  }

  //add src => node => sink edges with corresponding capacities
  for (int i = 0; i < numVertices; i++)
  {
    double srcToI = 0;
    double ItoSink = 0;
    if (i != S && i != T)
    {
      if (adjMatrix[S][i]->isConnected())
        srcToI = adjMatrix[S][i]->getCapacity();
      if (adjMatrix[i][T]->isConnected())
        ItoSink = adjMatrix[i][T]->getCapacity();

      int correctedI;
      if (i < S && i < T)
        correctedI = i;
      else if (i > S && i > T)
        correctedI = i - 2;
      else
        correctedI = i - 1;
      g.add_tweights(correctedI, srcToI, ItoSink);
    }
  }

  for (int i = 0; i < numVertices; i++)
  {
    if (i != S && i != T)
    {
      for (int j = i + 1; j < numVertices; j++)
      {
        if (j != S && j != T)
        {
          double IJcapacity = 0;
          double JIcapacity = 0;
          if (adjMatrix[i][j]->isConnected())
            IJcapacity = adjMatrix[i][j]->getCapacity();
          if (adjMatrix[j][i]->isConnected())
            JIcapacity = adjMatrix[j][i]->getCapacity();

          int correctedI, correctedJ;
          if (i < S && i < T)
            correctedI = i;
          else if (i > S && i > T)
            correctedI = i - 2;
          else
            correctedI = i - 1;

          if (j < S && j < T)
            correctedJ = j;
          else if (j > S && j > T)
            correctedJ = j - 2;
          else
            correctedJ = j - 1;

          g.add_edge(correctedI, correctedJ, IJcapacity, JIcapacity);
        }
      }
    }
  }

  maxflow = g.maxflow();

  for (int i = 0; i < numVertices; i++)
  {
    if (i != S && i != T)
    {
      int correctedI;
      if (i < S && i < T)
        correctedI = i;
      else if (i > S && i > T)
        correctedI = i - 2;
      else
        correctedI = i - 1;

      if (g.what_segment(correctedI) == GraphType::SOURCE)
        cut[i] = 0;
      else
        cut[i] = 1;
    }
  }
  cut[S] = 0;
  cut[T] = 1;
}

void Graph::getMinCut(bool *cut, double &mincut)
{
  int S = 0;
  double flow;
  bool *c = new bool[numVertices];
  getSTMaxflow(S, 1, cut, mincut);
  for (int T = 2; T < numVertices; T++)
  {
    getSTMaxflow(S, T, c, flow);
    if (flow < mincut)
    {
      mincut = flow;
      for (int i = 0; i < numVertices; i++)
        cut[i] = c[i];
    }
  }
  delete (c);
}

Graph*
Graph::subGraph(bool *cut, bool val)
{
  int numV = 0;
  for (int i = 0; i < numVertices; i++)
  {
    if (cut[i] == val)
      numV++;
  }

  Vertex **vlist = new Vertex*[numV];
  int j = 0;
  for (int i = 0; i < numVertices; i++)
  {
    if (cut[i] == val)
    {
      vlist[j] = vList[i];
      j++;
    }
  }

  Edge ***adjmatrix = new Edge**[numV];
  for (int i = 0; i < numV; i++)
    adjmatrix[i] = new Edge*[numV];

  int a = 0;
  int b = 0;
  for (int i = 0; i < numVertices; i++)
  {
    if (cut[i] == val)
    {
      b = 0;
      for (int j = 0; j < numVertices; j++)
      {
        if (cut[j] == val)
        {
          adjmatrix[a][b] = adjMatrix[i][j];
          b++;
        }
      }
      a++;
    }
  }
  return new Graph(vlist, adjmatrix, numV);
}

bool Graph::isHighlyConnected()
{
  if (numVertices <= 3)
    return true;

  bool *cut = new bool[numVertices];
  double mincut;
  getMinCut(cut, mincut);

  int crossEdges = 0;
  for (int i = 0; i < numVertices; i++)
  {
    for (int j = 0; j < numVertices; j++)
    {
      if (cut[i] != cut[j])
        crossEdges += adjMatrix[i][j]->isConnected();
    }
  }

  delete (cut);

  if (crossEdges / numVertices >= 0.5)
    return true;
  else
    return false;
}

void Graph::HCS(vector<int> &featuresID, map<int, int> &clusters)
{
  int *groupID = new int[numVertices];
  for (int i = 0; i < numVertices; i++)
    groupID[i] = 1;
  recHCS(this, groupID);

  std::vector<int> sortedGroupID;
  for (int i = 0; i < numVertices; i++)
  {
    bool found = false;
    for (unsigned int j = 0; j < sortedGroupID.size(); j++)
    {
      if (sortedGroupID[j] == groupID[i])
      {
        found = true;
      }
    }
    if (found == false)
    {
      sortedGroupID.push_back(groupID[i]);
    }
  }

  for (int i = 0; i < numVertices; i++)
  {
    for (unsigned int j = 0; j < sortedGroupID.size(); j++)
    {
      if (sortedGroupID[j] == groupID[i])
      {
        groupID[i] = j;
        break;
      }
    }
  }

  //contains the id for each feature
  featuresID.clear();
  for (int i = 0; i < numVertices; i++)
  {
    featuresID.push_back(groupID[i]+1); //transform from array to vector for output (starting at 1)
    vList[i]->setClusterID(groupID[i]); //assign cluster ID to each vertex in the graph
  }
  delete (groupID);

  //contains the number of features per id
  clusters.clear();
  for (vector<int>::iterator it = featuresID.begin(); it != featuresID.end(); it++)
  {
    if (clusters.find(*it) == clusters.end())
    {
      clusters[*it] = 1;
    }
    else
    {
      clusters[*it] = clusters.find(*it)->second + 1;
    }
  }

}

void Graph::recHCS(Graph *G, int *groupID)
{
  bool *cut = new bool[G->getNumVertices()];
  //ROS_INFO_STREAM_NAMED("Graph.recHCS","Number of vertices " << G->getNumVertices());
  std::cout << "[Graph::recHCS]: Number of vertices " << G->getNumVertices() << std::endl;
  double mincut;

  G->getMinCut(cut, mincut);
  int maxIndex = getMaxVal(groupID, numVertices);
  for (int i = 0; i < G->getNumVertices(); i++)
  {
    int realIndex = findVertex(G->vList[i]);
    if (cut[i] == true)
      groupID[realIndex] = maxIndex + 1;
    else
      groupID[realIndex] = maxIndex + 2;
  }

  Graph *G1 = G->subGraph(cut, false);
  Graph *G2 = G->subGraph(cut, true);

  if (G1->isHighlyConnected() == false)
    recHCS(G1, groupID);
  if (G2->isHighlyConnected() == false)
    recHCS(G2, groupID);

  delete (cut);
  delete (G1);
  delete (G2);
}

int Graph::getMaxVal(int *arr, int size)
{
  int val = arr[0];
  for (int i = 1; i < size; i++)
  {
    if (arr[i] > val)
      val = arr[i];
  }
  return val;
}

bool Graph::areConnected(Vertex *v1, Vertex *v2)
{
  int i1 = findVertex(v1);
  int i2 = findVertex(v2);
  return adjMatrix[i1][i2]->isConnected();
}

vector<vector<bool> > Graph::getConnectivityMatrix()
{
  vector<vector<bool> > M;
  for (int i = 0; i < numVertices; i++)
  {
    vector<bool> tmp;
    for (int j = 0; j < numVertices; j++)
    {
      tmp.push_back(adjMatrix[i][j]->isConnected());
    }
    M.push_back(tmp);
  }

  return M;
}

vector<FeaturePtr> Graph::getFeatures(int frame)
{
  vector<FeaturePtr> features;
  for (int i = 0; i < numVertices; i++)
  {
    features.push_back(vList[i]->getFeatureData(frame));
  }
  return features;
}

void Graph::addPredictor(Predictor *p)
{
  predictors.push_back(p);
}

void Graph::predict()
{
  for (vector<Predictor*>::iterator it = predictors.begin(); it != predictors.end(); it++)
  {
    (*it)->setEdgeList(adjMatrix);
    (*it)->setVertexList(vList);
    (*it)->setNumberVertices(numVertices);
    (*it)->compute();
  }
}

vector<int> Graph::getFeatureClusters()
{
  vector<int> featureClusters;
  for (int i = 0; i < numVertices; i++)
  {
    featureClusters.push_back(vList[i]->getClusterID());
  }
  return featureClusters;
}

std::vector<Predictor*> Graph::getPredictors()
{
  return predictors;
}

std::vector<std::vector<double> > Graph::getWeights()
{
  vector<vector<double> > ret_val;
  for (int i = 0; i < numVertices; i++)
  {
    vector<double> tmp;
    for (int j = 0; j < numVertices; j++)
    {
      tmp.push_back(adjMatrix[i][j]->getCapacity());
    }
    ret_val.push_back(tmp);
  }

  return ret_val;

}

}
;
