#include <iostream>
#include "opencv2/opencv.hpp"
#include <vector>

using namespace cv;
using namespace std;

struct goalNode {
  int x;
  int y;
  float reward;
};

const int Free = 0;

Mat scaleMap(Mat map, int scale){
  Mat mapScaled= Mat::zeros(scale*map.rows, scale*map.cols, map.type());
  
  //resize(map, mapScaled, Size(), scale, scale, CV_INTER_LINEAR);
  
  for (int i=0; i<scale*map.rows; i++) {
    for (int j=0; j<scale*map.cols;j++) {
      if (map.type()==CV_8UC1)
	mapScaled.at<int8_t >(i,j)=map.at<int8_t >(i/scale,j/scale);
      if (map.type()==CV_32F)
	mapScaled.at<float>(i,j)=map.at<float>(i/scale,j/scale);
    }
  }
  if (map.type()==CV_32F) mapScaled.convertTo(mapScaled, CV_8UC1);
  return mapScaled;
}


bool isGoal(int x, int y,vector<goalNode> goalsList) {
  bool found=false;
  vector<goalNode>::iterator it = goalsList.begin();
  while ( (it != goalsList.end()) && (!found) ) {
    if (it->x==x && it->y==y) found=true;
    it++;
  }
  return found;
}

float sumNeighborsStates(Mat map,Mat mapUtility,int xa,int ya) {
  //allowedMovements 0-North, 1-East, 2-South, 3-West
  //relative positions in allowedMovements 0-Leftfront, 1-front, 2-Rigthfront
  const int allowedExplorationSize=4;
  const int explorationProbabilitiesSize=3;
  //Las X y Y están en orden para esta matriz
  /*
  int8_t allowedMovements[allowedMovementsSize][movementsProbabilitiesSize][2] = { { {-1,1},{0,1},{1,1} }, 
                                                                                   { {1,1},{1,0},{1,-1} }, 
                                                                                   { {1,-1},{0,-1},{-1,-1} },
                                                                                   { {-1,-1},{-1,0},{-1,1} } };
  */
  int8_t allowedExploration[allowedExplorationSize][explorationProbabilitiesSize][2] = { { {-1,0},{0,1},{1,0} }, 
                                                                                   { {0,1},{1,0},{0,-1} }, 
                                                                                   { {1,0},{0,-1},{-1,0} },
                                                                                   { {0,-1},{-1,0},{0,1} } };
  //One probability for each allowed movement destiny posiibility (Three for this case)
  float explorationProbabilities[explorationProbabilitiesSize]={0.1,0.8,0.1 };
  float maxUtility=0.0;
  for (int mi=0; mi<allowedExplorationSize;mi++){
    float stateUtility=0.0;
    for (int pi=0; pi<explorationProbabilitiesSize;pi++) {
      int dx=allowedExploration[mi][pi][0];
      int dy=allowedExploration[mi][pi][1];
      if ( (xa+dx>=0 && xa+dx<map.cols) && (ya+dy>=0 && ya+dy<map.rows) && (map.at<uint8_t>(ya+dy,xa+dx)==Free) ){
        stateUtility=stateUtility+explorationProbabilities[pi]*mapUtility.at<float>(ya+dy,xa+dx);
      }
    }
    if (stateUtility>maxUtility) maxUtility=stateUtility;
  }
  return maxUtility;
}

int determineNextStep(int x, int y, Mat map, Mat mapUtility) {
  const int allowedMovementsSize=9;
  int8_t allowedMovements[allowedMovementsSize][2] = { {0,1}, {1,1}, {1,0},{1,-1},{0,-1},{-1,-1}, {-1,0}, {-1,1}, {0,0}}; //In clock wise direction from Up
  //int8_t allowedMovements[allowedMovementsSize][2] = { {-1, 1}, {0, 1}, {1, 1},
  //                                                     {-1, 0}, {0, 0}, {1, 0},
  //                                                     {-1,-1}, {0,-1}, {1,-1} };
  float maxUtility=mapUtility.at<float>(y,x);
  int posMaxUtility=4;
  for (int n=0; n<allowedMovementsSize; n++ ) {
    if ( (y+allowedMovements[n][1]>=0 && y+allowedMovements[n][1]<map.rows) &&
         (x+allowedMovements[n][0]>=0 && x+allowedMovements[n][0]<map.cols) &&
         (map.at<uint8_t>(y+allowedMovements[n][1],x+allowedMovements[n][0])==Free) && 
         (mapUtility.at<float>(y+allowedMovements[n][1],x+allowedMovements[n][0])>maxUtility) ) {
      maxUtility = mapUtility.at<float>(y+allowedMovements[n][1],x+allowedMovements[n][0]);
      posMaxUtility = n;
    }
  }
  return posMaxUtility;
}


void simpleMDPPlanner(Mat map, vector<goalNode> &goalsList, Mat &mapPolicy) {
  //La matriz se usará con Y(Height-Rows) real en la primera posicion de la matriz asi será mat(y,x)
  //La posicion Y incrementa al norte y decrementa al sur. Hay que corregit al visualizar
  

  
  int rewardAnyAction=-1; //Reward (Cost) of moving a cellSize
  
  Mat mapUtility = Mat::zeros(map.rows, map.cols, CV_32F);  //Mapa para dibujar la expansion de nodos de la busqueda  
  int gi=0;
  vector<goalNode>::iterator it = goalsList.begin();
  while (it != goalsList.end() ) {
    //goalsReal is nx2 matrix with 0 pos = x and 1 pos =y of n goals
    int xGoal=it->x;
    int yGoal=it->y;
    mapUtility.at<float>(yGoal,xGoal)=it->reward;
    
    //To view the locations position
    //locationNode tmpLocation;
    //tmpLocation.upperLeftX=-0.5;tmpLocation.upperLeftY=4;
    //tmpLocation.bottomRigthX=4;tmpLocation.bottomRigthY=0;
    //(location.bottomRigthX + location.upperLeftX)/2 )/cellSize+xCenter;
    //(location.upperLeftY + location.bottomRigthY)/2 )/cellSize+yCenter;
    //xGoal=(( 1.7 + (1.7) )/2) /cellSize+xCenter;
    //yGoal=(( 2.8 + (2.8) )/2) /cellSize+yCenter;
    //map=map-150;
    //map.at<int8_t>(yGoal,xGoal)=255;
    //map.at<int8_t>(yGoal+1,xGoal+1)=255;
    //map.at<int8_t>(yGoal-1,xGoal-1)=255;
    //map.at<int8_t>(yGoal+1,xGoal-1)=255;
    //map.at<int8_t>(yGoal-1,xGoal+1)=255;
    //imshow("Goal", map);
    //waitKey(0);
    //return;
    
    it->x=xGoal;
    it->y=yGoal;
    cout << "Goal " << gi << " ->  X Goal: " << xGoal << "  Y Goal: " << yGoal << endl;
    
    it++;
    gi++;
  }
  
  //Compute MDP utility 
  cout << "Starting planning process..." << endl;
  
  float thresholdDelta=0.05;
  float delta=thresholdDelta+1;
  
  int iterations=0;
  while (delta>thresholdDelta ) {
    delta=0;
    
    /*
    for (int yi=0; yi<map.rows; yi++) {
      for (int xi=0; xi<map.cols;xi++) {
        if ( (!isGoal(xi,yi,goalsList)) && (map.at<uint8_t>(yi,xi)==Free) ) {
	  float oldUtility = mapUtility.at<float>(yi,xi);
          mapUtility.at<float>(yi,xi) = rewardAnyAction + sumNeighborsStates(map,mapUtility,xi,yi);
	  if ( abs(oldUtility-mapUtility.at<float>(yi,xi)) > delta ) delta = abs(oldUtility-mapUtility.at<float>(yi,xi));
	}
      }
    }
    */
    
    if (iterations%8==0)
    for (int yi=0; yi<map.rows; yi++) {
      for (int xi=0; xi<map.cols;xi++) {
        if ( (!isGoal(xi,yi,goalsList)) && (map.at<uint8_t>(yi,xi)==Free) ) {
	  float oldUtility = mapUtility.at<float>(yi,xi);
          mapUtility.at<float>(yi,xi) = rewardAnyAction + sumNeighborsStates(map,mapUtility,xi,yi);
	  if ( abs(oldUtility-mapUtility.at<float>(yi,xi)) > delta ) delta = abs(oldUtility-mapUtility.at<float>(yi,xi));
	}
      }
    }
    
    if (iterations%8==1)
    for (int yi=0; yi<map.rows; yi++) {
      for (int xi=map.cols-1; xi>=0;xi--) {
        if ( (!isGoal(xi,yi,goalsList)) && (map.at<uint8_t>(yi,xi)==Free) ) {
	  float oldUtility = mapUtility.at<float>(yi,xi);
          mapUtility.at<float>(yi,xi) = rewardAnyAction + sumNeighborsStates(map,mapUtility,xi,yi);
	  if ( abs(oldUtility-mapUtility.at<float>(yi,xi)) > delta ) delta = abs(oldUtility-mapUtility.at<float>(yi,xi));
	}
      }
    }

    if (iterations%8==2)
    for (int yi=map.rows-1; yi>=0; yi--) {
      for (int xi=0; xi<map.cols;xi++) {
        if ( (!isGoal(xi,yi,goalsList)) && (map.at<uint8_t>(yi,xi)==Free) ) {
	  float oldUtility = mapUtility.at<float>(yi,xi);
          mapUtility.at<float>(yi,xi) = rewardAnyAction + sumNeighborsStates(map,mapUtility,xi,yi);
	  if ( abs(oldUtility-mapUtility.at<float>(yi,xi)) > delta ) delta = abs(oldUtility-mapUtility.at<float>(yi,xi));
	}
      }
    }
    
    if (iterations%8==3)
    for (int yi=map.rows-1; yi>=0; yi--) {
      for (int xi=map.cols-1; xi>=0;xi--) {
        if ( (!isGoal(xi,yi,goalsList)) && (map.at<uint8_t>(yi,xi)==Free) ) {
	  float oldUtility = mapUtility.at<float>(yi,xi);
          mapUtility.at<float>(yi,xi) = rewardAnyAction + sumNeighborsStates(map,mapUtility,xi,yi);
	  if ( abs(oldUtility-mapUtility.at<float>(yi,xi)) > delta ) delta = abs(oldUtility-mapUtility.at<float>(yi,xi));
	}
      }
    }
    
    if (iterations%8==4)
    for (int xi=0; xi<map.cols;xi++) {
      for (int yi=0; yi<map.rows; yi++) {
        if ( (!isGoal(xi,yi,goalsList)) && (map.at<uint8_t>(yi,xi)==Free) ) {
	  float oldUtility = mapUtility.at<float>(yi,xi);
          mapUtility.at<float>(yi,xi) = rewardAnyAction + sumNeighborsStates(map,mapUtility,xi,yi);
	  if ( abs(oldUtility-mapUtility.at<float>(yi,xi)) > delta ) delta = abs(oldUtility-mapUtility.at<float>(yi,xi));
	}
      }
    }
    if (iterations%8==5)
    for (int xi=0; xi<map.cols;xi++) {
      for (int yi=map.rows-1; yi>=0; yi--) {
        if ( (!isGoal(xi,yi,goalsList)) && (map.at<uint8_t>(yi,xi)==Free) ) {
	  float oldUtility = mapUtility.at<float>(yi,xi);
          mapUtility.at<float>(yi,xi) = rewardAnyAction + sumNeighborsStates(map,mapUtility,xi,yi);
	  if ( abs(oldUtility-mapUtility.at<float>(yi,xi)) > delta ) delta = abs(oldUtility-mapUtility.at<float>(yi,xi));
	}
      }
    }
    
    if (iterations%8==6)
    for (int xi=map.cols-1; xi>=0;xi--) {
      for (int yi=0; yi<map.rows; yi++) {
        if ( (!isGoal(xi,yi,goalsList)) && (map.at<uint8_t>(yi,xi)==Free) ) {
	  float oldUtility = mapUtility.at<float>(yi,xi);
          mapUtility.at<float>(yi,xi) = rewardAnyAction + sumNeighborsStates(map,mapUtility,xi,yi);
	  if ( abs(oldUtility-mapUtility.at<float>(yi,xi)) > delta ) delta = abs(oldUtility-mapUtility.at<float>(yi,xi));
	}
      }
    }
    
    if (iterations%8==7)
    for (int xi=map.cols-1; xi>=0;xi--) {
      for (int yi=map.rows-1; yi>=0; yi--) {
        if ( (!isGoal(xi,yi,goalsList)) && (map.at<uint8_t>(yi,xi)==Free) ) {
	  float oldUtility = mapUtility.at<float>(yi,xi);
          mapUtility.at<float>(yi,xi) = rewardAnyAction + sumNeighborsStates(map,mapUtility,xi,yi);
	  if ( abs(oldUtility-mapUtility.at<float>(yi,xi)) > delta ) delta = abs(oldUtility-mapUtility.at<float>(yi,xi));
	}
      }
    }
    
    
    iterations++;
    //cout << iterations << endl;
    //imshow("Pompa",  scaleMap(mapUtility,1));
    //waitKey(0);
  }
  
  //Determine policy for each state
  mapPolicy = Mat::zeros(map.rows, map.cols, CV_8UC1);
  for (int yi=0; yi<map.rows; yi++) {
      for (int xi=0; xi<map.cols;xi++) {
        if ( (map.at<uint8_t>(yi,xi)==Free) ) {
	  mapPolicy.at<uint8_t>(yi,xi) = determineNextStep(xi, yi, map, mapUtility);
	}
      }
    }
  
  cout << "Process finished" << endl;
  cout << "Optimal policy found in " << iterations << " iterations" << endl;
  
  //imshow("Map", scaleMap(map,2));
  //imshow("Utility", scaleMap(mapUtility,2));
  //waitKey(0);
  
  return;
}
