/*
 * Directorio de trabajo en ../projects/shortestRouteTest/mapsDB
 * que es donde están los mapas
 */

#include <iostream>
#include <fstream>
#include <vector>
#include "opencv2/opencv.hpp"  //Image processing libraries
#include <boost/concept_check.hpp>
#include "simpleMDPPlanner.cpp"

using namespace std;
using namespace cv;

const string MAP_EDITION_WINDOW="Map Edition Window";
const string MAP_POLICY_WINDOW="Map Policy and Route Window";
const string DATA_EXT=".txt";
const int ESC_KEY=27;
const int NOT_FOUND=-1;

const int ACTION_SELECT_ROOM=1;
const int ACTION_ADD_POINT=2;
const int ACTION_DELETE_POINT=3;
const int ACTION_CREATE_BORDER=4;
const int ACTION_CREATING_BORDER=5;
const int ACTION_DELETE_BORDER=6;

const int POINT_CLOSEST_THRESHOLD = 10;

struct pointType {
  int x;
  int y;
};

struct roomNode {
  //string roomName; (Future)
  pointType roomCenter;
  vector <pointType> border;
  double area;
};

vector<roomNode> roomList;
Mat originalImageMap;
Mat distancesTable;
int selectedAction=ACTION_SELECT_ROOM;
int selectedRoom;


void printInstructions() {
  cout << "Select with left click. Pres any of following keys to modify rooms and borders:" << endl;
  cout << "   a - Add room (center point)" << endl;
  cout << "   d - Delete room (center point)" << endl;
  cout << "   b - Create room border" << endl;
  cout << "   e - Delete room border" << endl;
  cout << "   c - Compute areas and distances between rooms" << endl;
  cout << "   s - Save data to file" << endl;
  cout << "   Esc - Exit" << endl;
}


float computeMinimumDistance(roomNode startRoom, roomNode endRoom, Mat mapPolicy) {
  const int allowedMovementsSize=9;
  int8_t allowedMovements[allowedMovementsSize][2] = { {0,1}, {1,1}, {1,0},{1,-1},{0,-1},{-1,-1}, {-1,0}, {-1,1}, {0,0}}; //In clock wise direction from Up
  Mat mapPolicyShow=mapPolicy*25; //Se debería colocar en la función escalar mapa que convierta valores a 0-255
  
  int xNow = startRoom.roomCenter.x;
  int yNow = startRoom.roomCenter.y;
  int xGoal = endRoom.roomCenter.x;
  int yGoal = endRoom.roomCenter.y;
  
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
    imshow(MAP_POLICY_WINDOW, mapPolicyShow);
    if (waitKey(1)>0) break;
  }
  
  cout << "Goal reached after " << stepsToGoal << " steps" << endl;
  return distanceTraveled;
}

int selectClosePoint(int x,int y, bool inBorder=false) {
  if (inBorder) {
    int i=0;
    while ( i<roomList[selectedRoom].border.size() ) {
      if ( abs( roomList[selectedRoom].border[i].x-x) <= POINT_CLOSEST_THRESHOLD && abs( roomList[selectedRoom].border[i].y-y) <= POINT_CLOSEST_THRESHOLD ) {
	cout << "Selected point (" << roomList[selectedRoom].border[i].x << ", " << roomList[selectedRoom].border[i].y << ")";
	cout << " at room " << i << endl;
	return i;
      }
      i++;
    }
    return NOT_FOUND;
  }
  else {
  int i=0;
  while ( i<roomList.size() ) {
    if ( abs( roomList[i].roomCenter.x-x) <= POINT_CLOSEST_THRESHOLD && abs( roomList[i].roomCenter.y-y) <= POINT_CLOSEST_THRESHOLD ) {
      //cout << "Selected point (" << roomList[i].roomCenter.x << ", " << roomList[i].roomCenter.y << ")";
      //cout << " at room " << i << endl;
      return i;
    }
    i++;
  }
  return NOT_FOUND;
  }
}

/*
//BORRAME
void deleteClosePoint(int x,int y) {
  int i=0;
  while ( i<roomList.size() ) {
    if ( abs( roomList[i].roomCenter.x-x) <= POINT_CLOSEST_THRESHOLD && abs( roomList[i].roomCenter.y-y) <= POINT_CLOSEST_THRESHOLD ) {
      cout << "Deleting point (" << roomList[i].roomCenter.x << ", " << roomList[i].roomCenter.y << ")" << endl;
      roomList.erase(roomList.begin()+i);
    }
    else {
      i++;
    }
  }
}
*/
void paintRooms(Mat imageMap) {
  Mat mapWithRooms = imageMap.clone();
  for (int i=0; i<roomList.size(); i++) {
    //paint room centers
    int size=5; int thickness = -1; int lineType = 8;
    int R, G, B;
    if (selectedRoom==i) { R=255; G=100; B=0; } else { R=100; G=100; B=100; }
    circle( mapWithRooms, Point(roomList[i].roomCenter.x, roomList[i].roomCenter.y), size, Scalar(B,G,R), thickness, lineType );
    
    //paint border vertex
    if (selectedRoom==i) { R=0; G=0; B=255; } else { R=50; G=50; B=50; }
    int borderCount = roomList[i].border.size();
    for (int j=0; j<borderCount; j++) {
      circle( mapWithRooms, Point(roomList[i].border[j].x, roomList[i].border[j].y), size, Scalar(B,G,R), thickness, lineType );
    }
    
    //paint border lines
    if (selectedRoom==i) { R=0; G=0; B=255; } else { R=50; G=50; B=50; }
    thickness = 1;
    for (int j=0; j<borderCount-1; j++) {
      line( mapWithRooms, Point(roomList[i].border[j].x, roomList[i].border[j].y), Point(roomList[i].border[j+1].x, roomList[i].border[j+1].y), Scalar(B,G,R), thickness );
    }
    //Close the border
    if (borderCount>0)
      line( mapWithRooms, Point(roomList[i].border[0].x, roomList[i].border[0].y), Point(roomList[i].border[borderCount-1].x, roomList[i].border[borderCount-1].y), Scalar(B,G,R), thickness );
  }
  imshow(MAP_EDITION_WINDOW, mapWithRooms);
}


void CallBackFunc(int event, int x, int y, int flags, void* userdata) {
  switch (event) {
    case EVENT_LBUTTONDOWN:
      //cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
      switch (selectedAction) {
	case ACTION_SELECT_ROOM: {
	  selectedRoom = selectClosePoint(x,y);
	  if (selectedRoom == NOT_FOUND) {
	    cout << "No room at this point" << endl;
	    selectedRoom=0;
	  }
	  else {
	    cout << "Room selected: " << selectedRoom << endl;
	  }
	  break;}
	case ACTION_ADD_POINT: {
	  int selectedPoint = selectClosePoint(x,y);
	  if (selectedPoint == NOT_FOUND) {
	    cout << "Room center created at (" << x << ", " << y << ")." << endl;
	    roomNode room;
	    room.roomCenter.x = x;
	    room.roomCenter.y = y;
	    roomList.push_back(room);
	    selectedRoom=selectedPoint;
	  }
	  else {
	    cout << "A room center already exists here";
	    selectedRoom=roomList.size();
	  }
	  break;}
	case ACTION_DELETE_POINT: {
	  selectedRoom = selectClosePoint(x,y);
	  if (selectedRoom != NOT_FOUND) {
	    cout << "Deleting point (" << roomList[selectedRoom].roomCenter.x << ", " << roomList[selectedRoom].roomCenter.y << ")" << endl;
	    roomList.erase(roomList.begin()+selectedRoom);
	    selectedRoom=roomList.size()-1;
	  }
	  else {
	    cout << "Not room center selected to delete. Try again." << endl;
	  }
	  break;}
	case ACTION_CREATE_BORDER: {
	  selectedRoom = selectClosePoint(x,y);
	  if (selectedRoom == NOT_FOUND) {
	    cout << "Not room center selected. Try again." << endl;
	  }
	  else {
	    cout << "Click on the border points now. It finish when you click at starting border point." << endl;
	    selectedAction = ACTION_CREATING_BORDER;
	  }
	  break; }
	case ACTION_CREATING_BORDER: {
	  int selectedPoint = selectClosePoint(x,y,true);
	  if (selectedPoint == NOT_FOUND) {
	    pointType vertexBorder;
	    vertexBorder.x = x;
	    vertexBorder.y = y;
	    roomList[selectedRoom].border.push_back(vertexBorder);
	    cout << "Vertex point (" << x << ", " << y << ")";
	    cout << " added to border of room " << selectedRoom << endl;
	  }
	  else {
	    cout << "Border created." << endl;
	    selectedAction = ACTION_CREATE_BORDER;
	  }
	  break;}
	case ACTION_DELETE_BORDER: {
	  selectedRoom = selectClosePoint(x,y);
	  if (selectedRoom == NOT_FOUND) {
	    cout << "Not room center selected. Try again." << endl;
	  }
	  else {
	    roomList[selectedRoom].border.clear();
	    cout << "Border deleted." << endl;
	    selectedAction = ACTION_CREATING_BORDER;
	  }
	  break; }
      }
      break;
    case EVENT_RBUTTONDOWN:
      //cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
      break;
    case EVENT_MBUTTONDOWN:
      //cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
      break;
    case EVENT_MOUSEMOVE:
      //cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;
      break;
  }
}


void loadMapData(Mat ImageMap, string fileName, string imageExt) {
  
  originalImageMap = imread(fileName+imageExt, CV_LOAD_IMAGE_GRAYSCALE );
  if ( originalImageMap.empty() ) { 
    cout << "Error loading the image" << endl;
    exit(-1); 
  }
  
  ifstream fileStream;
  fileName += DATA_EXT;
  fileStream.open (fileName.c_str(), fstream::in);  //std::fstream::in | std::fstream::out | std::fstream::app
  if (!fileStream.fail()) {
    cout << "Loading data from " << fileName << endl;
    string mapFileName;
    fileStream >> mapFileName;
    cout << "Map image file: " << mapFileName << endl;
    
    int roomListSize;
    fileStream >> roomListSize;
    cout << "Rooms: " << roomListSize << endl;
    
    for (int i=0; i<roomListSize; i++) {
      roomNode room;
      fileStream >> room.roomCenter.x >> room.roomCenter.y;
      fileStream >> room.area;
      int borderCount=0;
      fileStream >> borderCount;
      for (int j=0; j<borderCount; j++) {
	pointType vertexBorder;
	fileStream >> vertexBorder.x >> vertexBorder.y;
	room.border.push_back(vertexBorder);
      }
      roomList.push_back(room);
    }
    /*
     *  for (int i=0; i<roomListSize; i++) {
     *    for (int j=0; j<roomListSize; j++) {
     *      fileStream >> distancesTable[i][j];
     *      //cout << distancesTable[i][j] << " ";
  }
  //cout << endl;
  }
  */
    fileStream.close();
  } else {
    cout << "No data file. A new one will be created." << endl;
  }
}


void prepareMapforMinimalRoutes(Mat &dilatedMap) {
  //Prepare the map for using simpleMDPPlanner
  for (int i=0; i<dilatedMap.rows; i++) {
    for (int j=0; j<dilatedMap.cols; j++) {
      if (dilatedMap.at<uint8_t>(i,j)>150) dilatedMap.at<uint8_t>(i,j)=255;   //"Binarize values under 100 to 0 (Delete noise)
    }
  }
  dilatedMap = 255-dilatedMap; //Free space is black and walls a value > 0
  Mat kernel = Mat::ones(6,6,CV_8UC1);
  dilate(dilatedMap, dilatedMap, kernel, Point(-1,-1), 1); //Make ticker walls for consider the robot as a point
}


void computeDistances(Mat dilatedMap) {
  cout << "Computing distances between nodes..." << endl;
  
  distancesTable.release();
  distancesTable = Mat::zeros(roomList.size(), roomList.size(), CV_32F);
  for (int i=1; i<roomList.size(); i++) {
    vector<goalNode> goalsList;
    Mat mapPolicy;
    goalNode tmp;
    tmp.x=roomList[i].roomCenter.x;
    tmp.y=roomList[i].roomCenter.y;
    tmp.reward=10255; //Aunque este valor sirve para ver los valores de la politica, no sirve para mapas grandes
    goalsList.push_back(tmp);
    simpleMDPPlanner(dilatedMap, goalsList, mapPolicy);
    
    for (int j=0; j<i; j++) {
      float distanceTraveled = computeMinimumDistance(roomList[j], roomList[i], mapPolicy);
      distancesTable.at<float>(i,j)=distanceTraveled;
      distancesTable.at<float>(j,i)=distanceTraveled;
    }
  }
  cout << "Distances table" << endl;
  cout << distancesTable << endl;
}


void computeAreas() {
  for (int i=0; i<roomList.size(); i++) {
    vector<pointType> tmpBorder = roomList[i].border;
    tmpBorder.push_back(tmpBorder[0]);
    
    /*
    //Method 1
    int xySum=0;
    int yxSum=0;
    for (int j=0; j<tmpBorder.size()-1; j++) {
      xySum+=tmpBorder[j].x * tmpBorder[j+1].y;
      yxSum+=tmpBorder[j].y * tmpBorder[j+1].x;
    }
    roomList[i].area = abs((xySum-yxSum)/2.0);
    */
    //Method 2
    int area=0;
    int k=tmpBorder.size()-2;
    for (int j=0; j<tmpBorder.size()-1; j++) {
      area += (tmpBorder[k].x + tmpBorder[j].x) * (tmpBorder[k].y - tmpBorder[j].y);
      k = j;
    }
    roomList[i].area = abs(area/2.0);
  }
}

void saveData(string fileName, string imageExt) {
  string imageFileName = fileName+imageExt;
  fileName += DATA_EXT;
  
  ofstream fileStream;
  fileStream.open (fileName.c_str(), fstream::out);  //std::fstream::in | std::fstream::out | std::fstream::app
  if ( fileStream.fail() ) {
    cout << "Error opening file " << fileName << ". Data not saved." << endl;
    exit (-2);
  }
  
  fileStream << imageFileName << endl;
  fileStream << roomList.size() << endl;
  
  for (int i=0; i<roomList.size(); i++) {
    //cout << roomList[i].roomName << endl;
    //Save room centers
    fileStream << roomList[i].roomCenter.x << " " << roomList[i].roomCenter.y << endl;
    
    fileStream << roomList[i].area << endl;
    
    //Save vertex of borders
    fileStream << roomList[i].border.size() << endl;
    for (int j=0; j<roomList[i].border.size(); j++) {
      fileStream << roomList[i].border[j].x << " " << roomList[i].border[j].y << endl;
    }
  }

  //Save table of distances between rooms
  for (int i=0; i<distancesTable.rows; i++) {
    for (int j=0; j<distancesTable.cols; j++) {
      fileStream << distancesTable.at<float>(i,j);
      if (j<distancesTable.cols-1) fileStream << " ";
    }
    fileStream << endl;
  }
  
  fileStream.close();
  
  fileStream.open(fileName.c_str(), ios::app);
  fileStream << endl;
  fileStream.close();
  cout << endl << "File: " << fileName << " created correctly." << endl;
}


void showMap(Mat imageMap, string fileName, string imageExt) {
  
  Mat tmpDilatedMap;
  cvtColor(imageMap, tmpDilatedMap, CV_GRAY2RGB);  
  //imshow("Otra", tmpDilatedMap);
  int keyPressed=0;
  while (keyPressed!=ESC_KEY) {
    paintRooms(tmpDilatedMap);
    //imshow("Otra mas", tmpDilatedMap);
    keyPressed=waitKey(1);
    switch (keyPressed) {
      case 'a':
	cout << "Adding a room. Click on the point you want to be the room center" << endl;
	selectedAction = ACTION_ADD_POINT;
	break;
      case 'd':
	cout << "Delete room. Click on the center room point to delete" << endl;
	selectedAction = ACTION_DELETE_POINT;
	break;
      case 'b':
	cout << "Creating room border. Click the room center point you want to define border" << endl;
	selectedAction = ACTION_CREATE_BORDER;
	break;
      case 'e':
	cout << "Erase room border. Click the room center point you want to erase border" << endl;
	selectedAction = ACTION_DELETE_BORDER;
	break;
      case 'c':
	cout << "Computing areas and distances between rooms" << endl;
	computeDistances(imageMap);
	//computeAreas();
	break;
      case 's':
	cout << "Saving data to file" << endl;
	computeAreas();  //Puesto aqui para no calcular las distancias porque es muy lento
	saveData(fileName, imageExt);
	break;
      case ESC_KEY:
	cout << "Exiting" << endl;
	break;
      default:
	if (keyPressed>=0) {
	  cout << "Invalid key" << endl;
	  cout << "You pressed code: " << keyPressed << "  char: " << char(keyPressed) <<endl;
	}
    }
  }
}


int main(int argc, char **argv) {
  
  if (argc!=2) {
    cout << "Usage: mapsmagager <image_map_file>" << endl;
    return 1;
  }
  string fileName = argv[1];
  string imageExt = fileName.substr( fileName.find(".") );
  cout << "Extension: " << imageExt << endl;
  fileName = fileName.substr( 0, fileName.find(".") );
  
  printInstructions();
  loadMapData(originalImageMap, fileName, imageExt);
  
  namedWindow(MAP_EDITION_WINDOW, 1);
  setMouseCallback(MAP_EDITION_WINDOW, CallBackFunc, NULL);
  
  prepareMapforMinimalRoutes(originalImageMap);
  
  showMap(originalImageMap, fileName, imageExt);
  
  //computeDistances(originalImageMap);
  //saveData(fileName, imageExt);
  
  cout << "Press any key to quit...";
  waitKey(0);
  
  return 0;
}
