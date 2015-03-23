/*
 * Directorio de trabajo en ../projects/shortestRouteTest/mapsDB
 * que es donde están los mapas
 */

#include <iostream>
#include <fstream>
#include <vector>
#include "opencv2/opencv.hpp"  //Image processing libraries
#include "simpleMDPPlanner.cpp"


using namespace std;
using namespace cv;

string dataFileName;
string mapFileName;

struct roomNode {
  //string roomName; (Future)
  int x;
  int y;
};

vector<roomNode> roomList;
Mat mapWithNodes;

  

float computeDistance(roomNode startRoom, roomNode endRoom, Mat mapPolicy) {
  const int allowedMovementsSize=9;
  int8_t allowedMovements[allowedMovementsSize][2] = { {0,1}, {1,1}, {1,0},{1,-1},{0,-1},{-1,-1}, {-1,0}, {-1,1}, {0,0}}; //In clock wise direction from Up
  Mat mapPolicyShow=mapPolicy*25; //Se debería colocar en la función escalar mapa que convierta valores a 0-255
  
  int xNow = startRoom.x;
  int yNow = startRoom.y;
  int xGoal = endRoom.x;
  int yGoal = endRoom.y;
  
  cout << "X Start: " << xNow << "  Y Start: " << yNow << endl;
  cout << "X Goal: " << xGoal << "  Y Goal: " << yGoal << endl;
  if ( mapPolicy.at<uint8_t>(yNow,xNow)==Free || mapPolicy.at<uint8_t>(yGoal,xGoal)==Free ) {
    cout << "Error: Starting or ending point into a wall" << endl;
    return 0;
  }
  
  int stepsToGoal=0;
  float distanceTraveled=0;
  
  while ( (xNow!=xGoal || yNow!=yGoal) && (stepsToGoal<1000) ) {
    xNow=xNow+allowedMovements[ mapPolicy.at<uint8_t>(yNow,xNow) ][0];
    yNow=yNow+allowedMovements[ mapPolicy.at<uint8_t>(yNow,xNow) ][1];
    stepsToGoal++;
    distanceTraveled+=sqrt(abs(allowedMovements[ mapPolicy.at<uint8_t>(yNow,xNow) ][0])+abs(allowedMovements[ mapPolicy.at<uint8_t>(yNow,xNow) ][1]));
    
    mapPolicyShow.at<int8_t >(yNow,xNow)=255; 
    imshow("Policy", mapPolicyShow);
    if (waitKey(1)>0) break;
  }

  cout << "Goal reached after " << stepsToGoal << " steps" << endl;
  return distanceTraveled;
}


void CallBackFunc(int event, int x, int y, int flags, void* userdata) {
  if  ( event == EVENT_LBUTTONDOWN ) {
    //cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
    cout << "Point selected at " << x << ", " << y << "." << endl;
    roomNode room;
    room.x=x;
    room.y=y;
    roomList.push_back(room);
    
    int size=5; int thickness = -1; int lineType = 8;
    circle( mapWithNodes, Point(x, y), size, Scalar( 0, 100, 255 ), thickness, lineType );
    imshow("Map", mapWithNodes);
  }
  else if  ( event == EVENT_RBUTTONDOWN ) {
    //cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
  }
  else if  ( event == EVENT_MBUTTONDOWN ) {
    //cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
  }
  else if ( event == EVENT_MOUSEMOVE ) {
    //cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;
  }
}


void loadAndShowMap() {
  Mat map;
  map = imread(mapFileName, CV_LOAD_IMAGE_GRAYSCALE );
  if ( map.empty() ) { 
    cout << "Error loading the image" << endl;
    return; 
  }

  cout << "Select room center with left click. Begin with room closest to front door." << endl;
  cout << "Press any key when all points be selected..." << endl;
  namedWindow("Map", 1);
  setMouseCallback("Map", CallBackFunc, NULL);
  
  //Prepare the map for using simpleMDPPlanner
  for (int i=0; i<map.rows; i++)
    for (int j=0; j<map.cols; j++)
      if (map.at<uint8_t>(i,j)>150) map.at<uint8_t>(i,j)=255;   //"Binarize values under 100 to 0 (Delete noise)
  map = 255-map; //Free space is black and walls a value > 0
  Mat kernel = Mat::ones(6,6,CV_8UC1);
  dilate(map, map, kernel, Point(-1,-1), 1); //Make ticker walls for consider the robot as a point

  cvtColor(map, mapWithNodes, CV_GRAY2RGB);  
  imshow("Map", mapWithNodes);
  waitKey(0);
  
  cout << "Computing distances between nodes..." << endl;
  
  Mat distancesTable = Mat::zeros(roomList.size(), roomList.size(), CV_32F);
  for (int i=1; i<roomList.size(); i++) {
    vector<goalNode> goalsList;
    Mat mapPolicy;
    goalNode tmp;
    //solveMDPPlanning takes map as rows(y), columns(x)
    tmp.x=roomList[i].x;
    tmp.y=roomList[i].y;
    tmp.reward=10255; //Aunque este valor sirve para ver los valores de la politica, no sirve para mapas grandes
    goalsList.push_back(tmp);
    simpleMDPPlanner(map, goalsList, mapPolicy);
      
    for (int j=0; j<i; j++) {
      float distanceTraveled = computeDistance(roomList[j], roomList[i], mapPolicy);
      distancesTable.at<float>(i,j)=distanceTraveled;
      distancesTable.at<float>(j,i)=distanceTraveled;
    }
  }
  cout << "Distances table" << endl;
  cout << distancesTable << endl;
 
  ofstream fileStream(dataFileName.c_str());
  fileStream << mapFileName << endl;
  fileStream << roomList.size() << endl;

  for (int i=0; i<roomList.size(); i++) {
      //cout << roomList[i].roomName << endl;
      fileStream << roomList[i].x << " " << roomList[i].y << endl;
  }
  for (int i=0; i<roomList.size(); i++) {
    for (int j=0; j<roomList.size(); j++) {
      fileStream << distancesTable.at<float>(i,j);
      if (j<roomList.size()-1) fileStream << " ";
    }
    fileStream << endl;
  }
  
  fileStream.close();
  
  fileStream.open(dataFileName.c_str(), ios::app);
  fileStream << endl;
  fileStream.close();
  
}


int main(int argc, char **argv) {
  
  if (argc!=2) {
    cout << "Usage: mapsmagager <image_map_file>" << endl;
    return 1;
  }
  mapFileName = argv[1];
  dataFileName = mapFileName.substr(0,mapFileName.find("."))+".txt";
 
  loadAndShowMap();
  
  cout << endl << "File: " << dataFileName << " created correctly." << endl;
  return 0;
}
