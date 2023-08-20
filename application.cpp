// application.cpp <Starter Code>
/// Dimitar Gjorgievski, UIN 650730211
/// CS 251
/// Project 6 - Open Street Map
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
// 
// References:
// TinyXML: https://github.com/leethomason/tinyxml2
// OpenStreetMap: https://www.openstreetmap.org
// OpenStreetMap docs:
//   https://wiki.openstreetmap.org/wiki/Main_Page
//   https://wiki.openstreetmap.org/wiki/Map_Features
//   https://wiki.openstreetmap.org/wiki/Node
//   https://wiki.openstreetmap.org/wiki/Way
//   https://wiki.openstreetmap.org/wiki/Relation
//

#include <iostream>
#include <iomanip>  /*setprecision*/
#include <string>
#include <vector>
#include <map>
#include <queue>
#include <cstdlib>
#include <cstring>
#include <cassert>
#include <limits>
#include "tinyxml2.h"
#include "dist.h"
#include "graph.h"
#include "osm.h"


using namespace std;
using namespace tinyxml2;

//Functor for priority_queue
class prioritize 
{
public:
  bool operator()(const pair<long long,double>& p1, const pair<long long,double>& p2) const
  {
    return p1.second > p2.second; 
  }
};


const double INF = numeric_limits<double>::max();

void Dijkstra(graph<long long, double> G, Coordinates start, Coordinates destination, double& distance){
  priority_queue< pair<long long, double>, vector<pair<long long, double>>, prioritize> unvisited_q;
  map<long long, double> distances;
  set<long long> visited;
  map<long long, long long> pred;
  
  //Adding all vertices to priority queue & distance map with weigth of infinity
  for(auto v : G.getVertices()){
    unvisited_q.push(make_pair(v,INF));
    distances.emplace(v, INF); 
  }
  unvisited_q.push(make_pair(start.ID,0)); 
  distances.erase(start.ID);
  distances.emplace(start.ID, 0);
  pred.emplace(start.ID,0);

  //Finding shortest path to destination node from start node
  while(1){
    long long currV = unvisited_q.top().first;
    double currW = unvisited_q.top().second;
    
    unvisited_q.pop();
    
    if(currW==INF){
      break;  //Case when we reach end of loop
    }else if(visited.count(currV)==1){
      continue;
    }else{
      visited.insert(currV);
    } 
     
    //Visiting adjacent vertices of current vertex
    for(long long adj : G.neighbors(currV)){
      double weigth = 0;
      G.getWeight(currV, adj, weigth);
      double path = weigth + distances.at(currV);
      
      if(path<distances.at(adj)){ //Updating predecessors and distances if found to be less then previously calculated
        distances.erase(adj);
        distances.emplace(adj, path);
        pred.emplace(adj, currV);
        unvisited_q.push(make_pair(adj,path));  
      }
    }
  }

  //Arranging path
  vector<long long> path_arr;
  long long next_pred = destination.ID;
  path_arr.push_back(next_pred);
  while(1){
    next_pred = pred.at(next_pred);
    path_arr.push_back(next_pred);
    if(next_pred==start.ID){
      break;
    }
  }

  distance = distances.at(destination.ID);
  cout<<distance<<" miles"<<endl;
  cout<<"Path: ";
  cout<<path_arr.at(path_arr.size()-1);
  for(int i = path_arr.size()-2; i>=0; i--){
    cout<<"->"<<path_arr.at(i);
  }
  cout<<endl;
  

}

//Helper function that finds the nearest node of the corresponding buildings of person1, person2 and the destination
Coordinates NearestNode(Coordinates building, vector<FootwayInfo> Footways, map<long long, Coordinates> Nodes){
  double min = INF;
  Coordinates node;

  for(auto f : Footways){
    for(int i=0; i<f.Nodes.size(); i++){
      if(min>distBetween2Points(Nodes.at(f.Nodes[i]).Lat, Nodes.at(f.Nodes[i]).Lon, building.Lat, building.Lon)){
        min = distBetween2Points(Nodes.at(f.Nodes[i]).Lat, Nodes.at(f.Nodes[i]).Lon, building.Lat, building.Lon);
        node = Nodes.at(f.Nodes[i]);
      }
    }
  }
  return node;
}

//Helper function that return the building designated as the destination Person1 and Person2 should meet at
BuildingInfo FindDestination(Coordinates midpoint, vector<BuildingInfo> Buildings){
  double min = INF;
  BuildingInfo destination_building;

  for(auto b: Buildings){
    if(min>distBetween2Points(b.Coords.Lat, b.Coords.Lon, midpoint.Lat, midpoint.Lon)){
        min = distBetween2Points(b.Coords.Lat, b.Coords.Lon, midpoint.Lat, midpoint.Lon);
        destination_building = b;
    }
  }

  return destination_building;
}

//Helper function that finds Person1's and Person2's buildings
//Returns true if buildings are found. False if at least one building is not found
bool BuildingSearch(string person1Building, string person2Building, vector<BuildingInfo> Buildings, int &PB1, int &PB2){
  string p1 = "";
  string p2 = "";
  for(char& c : person1Building){ //Turning all characters of string to lowercase for easier search
    p1+=tolower(c);
  }
  
  for(char& c : person2Building){
    p2+=tolower(c);
  }

  int i=0; //index counter

  for(auto b : Buildings){
    string ab = "";
    string name = "";
    for(char& c : b.Abbrev){ //Turning all characters of string to lowercase for easier search
      ab+=tolower(c);
    }
    for(char& c : b.Fullname){ 
      name+=tolower(c);
    }

    //Checks if index for person 1's building is found
    if(PB1==-1){
      if(p1==ab){ //If abbreviation was found
          PB1 = i;
      }else if(p1==name){ //If exact name of building was found
          PB1 = i;
      }else{ //If partial name of building was found
        size_t found;
        found = name.find(p1);
        if(found!=string::npos){
          PB1 = i;
        }
      }
    }
    
    //Checks if index for person 2's building is found
    if(PB2==-1){
      if(p2==ab){ //If abbreviation was found
          PB2 = i;
      }else if(p2==name){ //If exact name of building was found
          PB2 = i;
      }else{ //If partial name of building was found
          size_t found;
          found = name.find(p2);
          if(found!=string::npos){
            PB2 = i;
          }
      }
    }
    
    //Returns true if both people's buildings are found
    if(PB1!=-1 && PB2!=-1){
      return true;
    }

    i++;
  }

  //Return false and display message if either person's building is not found
  if(PB1==-1 && PB2==-1){
    cout << "Person 1's building not found" << endl;
    cout << "Person 2's building not found" << endl;
  }else if(PB2==-1){
    cout << "Person 2's building not found" << endl;
  }else{
    cout << "Person 1's building not found" << endl;
  }
  return false;
}

//
// Implement your standard application here
//
void application(
  map<long long, Coordinates>& Nodes, vector<FootwayInfo>& Footways,
  vector<BuildingInfo>& Buildings, graph<long long, double>& G) {
  string person1Building, person2Building;

  cout << endl;
  cout << "Enter person 1's building (partial name or abbreviation), or #> ";
  getline(cin, person1Building);

  while (person1Building != "#") {
    cout << "Enter person 2's building (partial name or abbreviation)> ";
    getline(cin, person2Building);
    
    int PB1 = -1; //stores index found of building
    int PB2 = -1;

    //
    // TO DO: lookup buildings, find nearest start and dest nodes, find center
    if(BuildingSearch(person1Building, person2Building, Buildings, PB1, PB2)){
      cout<<endl;
      
      //Displaying Person 1 & 2 building if found
      cout<<"Person 1's point:"<<endl;
      cout<<" "<<Buildings[PB1].Fullname<<endl;
      cout<<" ("<<Buildings[PB1].Coords.Lat<<", "<<Buildings[PB1].Coords.Lon<<")"<<endl;
      cout<<"Person 2's point:"<<endl;
      cout<<" "<<Buildings[PB2].Fullname<<endl;
      cout<<" ("<<Buildings[PB2].Coords.Lat<<", "<<Buildings[PB2].Coords.Lon<<")"<<endl;
         
      //Finding closest destination midpoint and corresponding closest building
      Coordinates midpoint = centerBetween2Points(Buildings[PB1].Coords.Lat, Buildings[PB1].Coords.Lon, Buildings[PB2].Coords.Lat, Buildings[PB2].Coords.Lon);
      BuildingInfo destination_building = FindDestination(midpoint, Buildings);
      cout<<"Destination Building:"<<endl;
      cout<<" "<<destination_building.Fullname<<endl;
      cout<<" ("<<destination_building.Coords.Lat<<", "<<destination_building.Coords.Lon<<")"<<endl;   
      
      //Finding nearest nodes of corresponding buildings 
      Coordinates P1node = NearestNode(Buildings[PB1].Coords, Footways, Nodes);
      Coordinates P2node = NearestNode(Buildings[PB2].Coords, Footways, Nodes);
      Coordinates destination_node = NearestNode(destination_building.Coords, Footways, Nodes);
      cout<<endl;
      
      //Printing nearest nodes
      cout<<"Nearest P1 node:"<<endl;
      cout<<" "<<P1node.ID<<endl;
      cout<<" ("<<P1node.Lat<<", "<<P1node.Lon<<")"<<endl;
      cout<<"Nearest P2 node:"<<endl;
      cout<<" "<<P2node.ID<<endl;
      cout<<" ("<<P2node.Lat<<", "<<P2node.Lon<<")"<<endl;
      cout<<"Nearest destination node:"<<endl;
      cout<<" "<<destination_node.ID<<endl;
      cout<<" ("<<destination_node.Lat<<", "<<destination_node.Lon<<")"<<endl;
      
      cout<<endl;
      double distance;

      //Finding and printing Dijkstra shortest path algorithm
      cout<<"Person 1's distance to dest: ";
      Dijkstra(G, P1node, destination_node, distance);
      cout<<endl;
      cout<<"Person 2's distance to dest: ";
      Dijkstra(G, P2node, destination_node, distance);


    }

    cout << endl;
    cout << "Enter person 1's building (partial name or abbreviation), or #> ";
    getline(cin, person1Building);
  }    
}

int main() {

  // maps a Node ID to it's coordinates (lat, lon)
  map<long long, Coordinates> Nodes;
  // info about each footway, in no particular order
  vector<FootwayInfo> Footways;
  // info about each building, in no particular order
  vector<BuildingInfo> Buildings;
  XMLDocument xmldoc;

  cout << "** Navigating UIC open street map **" << endl;
  cout << endl;
  cout << std::setprecision(8);

  string def_filename = "map.osm";
  string filename;

  cout << "Enter map filename> ";
  getline(cin, filename);

  if (filename == "") {
    filename = def_filename;
  }

  //
  // Load XML-based map file
  //
  if (!LoadOpenStreetMap(filename, xmldoc)) {
    cout << "**Error: unable to load open street map." << endl;
    cout << endl;
    return 0;
  }

  //
  // Read the nodes, which are the various known positions on the map:
  //
  int nodeCount = ReadMapNodes(xmldoc, Nodes);

  //
  // Read the footways, which are the walking paths:
  //
  int footwayCount = ReadFootways(xmldoc, Footways);

  //
  // Read the university buildings:
  //
  int buildingCount = ReadUniversityBuildings(xmldoc, Nodes, Buildings);

  //
  // Stats
  //
  assert(nodeCount == (int)Nodes.size());
  assert(footwayCount == (int)Footways.size());
  assert(buildingCount == (int)Buildings.size());

  cout << endl;
  cout << "# of nodes: " << Nodes.size() << endl;
  cout << "# of footways: " << Footways.size() << endl;
  cout << "# of buildings: " << Buildings.size() << endl;
 
  graph<long long, double> G;

  //Adding vertices to graph 
  for(auto n : Nodes){
    G.addVertex(n.first);
  }

  //Adding edges to graph
  for(auto e : Footways){
    for(int i=0; i<e.Nodes.size()-1; i++){

     double weight = distBetween2Points(Nodes.at(e.Nodes[i]).Lat, Nodes.at(e.Nodes[i]).Lon, Nodes.at(e.Nodes[i+1]).Lat, Nodes.at(e.Nodes[i+1]).Lon);
     G.addEdge(e.Nodes[i], e.Nodes[i+1], weight);
     G.addEdge(e.Nodes[i+1], e.Nodes[i], weight);
    }
  }

  cout << "# of vertices: " << G.NumVertices() << endl;
  cout << "# of edges: " << G.NumEdges() << endl;
  cout << endl;

  // Execute Application
  application(Nodes, Footways, Buildings, G);

  //
  // done:
  //
  cout << "** Done **" << endl;
  return 0;
}
