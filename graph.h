
// graph.h <Starter Code>
/// Dimitar Gjorgievski, UIN 650730211
/// CS 251
/// Project 6 - Open Street Map
//
// Basic graph class using adjacency matrix representation.  Currently
// limited to a graph with at most 100 vertices.
//
//
// Adam T Koehler, PhD
// University of Illinois Chicago
// CS 251, Spring 2023
//
// Project Original Variartion By:
// Joe Hummel, PhD
// University of Illinois at Chicago
//

#pragma once

#include <iostream>
#include <stdexcept>
#include <vector>
#include <set>
#include <map>

using namespace std;

template<typename VertexT, typename WeightT>
class graph {
 private:
  
  set<VertexT> AddedVertices;
  vector<VertexT> Vertices;
  int size;
  map<VertexT, map<VertexT, WeightT>> G;
  int edges;

 public:
  //
  // constructor:
  //
  graph() {
    size = 0;
    edges = 0;
  }

  //clear function 
  void clear(){
    G.clear();
    AddedVertices.clear();
    Vertices.clear();
    size = 0;
    edges = 0;
  }


  //
  // NumVertices
  //
  // Returns the # of vertices currently in the graph.
  //
  int NumVertices() const {
    return Vertices.size();
  }

  //
  // NumEdges
  //
  // Returns the # of edges currently in the graph.
  //
  int NumEdges() const {

    return edges;
  }

  //addVertex to adjacency list

  bool addVertex(VertexT v) {
    AddedVertices.insert(v);

    if(size!=AddedVertices.size()){
      size++;
      Vertices.push_back(v);
    }else{
      return false;
    }

    return true;
  }

  //
  // addEdge
  //
  // Adds the edge (from, to, weight) to the graph, and returns
  // true.  If the vertices do not exist or for some reason the
  // graph is full, false is returned.
  bool addEdge(VertexT from, VertexT to, WeightT weight) {
    if(AddedVertices.count(from)==0 || AddedVertices.count(to)==0){
      return false;
    }
    
    if(G[from].count(to)==0){
     edges++;
    }
    G[from][to] = weight;
    
    return true;
  }

  //
  // getWeight
  //
  // Returns the weight associated with a given edge.  If
  // the edge exists, the weight is returned via the reference
  // parameter and true is returned.  If the edge does not
  // exist, the weight parameter is unchanged and false is
  // returned.
  //
  bool getWeight(VertexT from, VertexT to, WeightT& weight) const {
    
    if(G.count(from)==0){
      return false;
    }else if(G.at(from).count(to)==0){
      return false;
    }

    weight = G.at(from).at(to);

    return true;
  }

  //
  // neighbors
  //
  // Returns a set containing the neighbors of v, i.e. all
  // vertices that can be reached from v along one edge.
  // Since a set is returned, the neighbors are returned in
  // sorted order
  //
  set<VertexT> neighbors(VertexT v) const {
    set<VertexT>  S;
    if(G.count(v)==1){
      for(auto vert : G.at(v)){
        S.insert(vert.first);
      }
    }

    return S;
  }

  //
  // getVertices
  //
  // Returns a vector containing all the vertices currently in
  // the graph.
  //
  vector<VertexT> getVertices() const {
    return this->Vertices;  // returns a copy:
  }

  //
  // dump
  //
  // Dumps the internal state of the graph for debugging purposes.
  //
  // Example:
  //    graph<string,int>  G(26);
  //    ...
  //    G.dump(cout);  // dump to console
  //
  void dump(ostream& output) const {
    output << "***************************************************" << endl;
    output << "********************* GRAPH ***********************" << endl;

    output << "**Num vertices: " << this->NumVertices() << endl;
    output << "**Num edges: " << this->NumEdges() << endl;

    output << endl;
    output << "**Vertices:" << endl;
    for (int i = 0; i < this->NumVertices(); ++i) {
      output << " " << i << ". " << this->Vertices[i] << endl;
    }

    output << endl;
    output << "**Edges:" << endl;
    for (int row = 0; row < this->NumVertices(); ++row) {
      output << " row " << row << ": ";

      for (int col = 0; col < this->NumVertices(); ++col) {
        if (this->AdjMatrix[row][col].EdgeExists == false) {
          output << "F ";
        } else {
          output << "(T,"
            << this->AdjMatrix[row][col].Weight
            << ") ";
        }
      }
      output << endl;
    }
    output << "**************************************************" << endl;
  }
};