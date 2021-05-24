#include "TerrainMap.h"
#include <math.h>
#include <ompl/util/RandomNumbers.h>



//simple helper function.
int isPosInBox(float x, float y, Rectangle *rect){
  return (x > rect->x) && (x < (rect->x + rect->width)) &&
         (y > rect->y) && (y < (rect->y + rect->height));
}





SimpleTerrainMap::SimpleTerrainMap(){

}

SimpleTerrainMap::~SimpleTerrainMap(){

}


BekkerData soil_table[3] = {
    {29.76, 2083, .8,  0, 22.5*M_PI/180.0, "Medium Soil"},
    {102,   5301, .8, .2, 31.1*M_PI/180.0, "Sand"},
    {30.08, 499.7, .7, 0, 14.0*M_PI/180.0, "Clay"}
};

BekkerData SimpleTerrainMap::lookup_soil_table(int index){
    return soil_table[index];
}


BekkerData SimpleTerrainMap::getSoilDataAt(float x, float y){
  const int search_depth = 0;
  const float threshold = .5;
  //octomap::OcTreeNode* node = ocTree->search(val[0], val[1], 0, search_depth);
  //TODO. THis.
  return lookupSoilTable(0);
}


float get_altitude(float x, float y){
  return fmax(0,sinf(x*.6))-.16;//(4 / (2*sqrtf((x*x) + (y*y))+1));
}

void SimpleTerrainMap::generateObstacles(){
  //Just going to try a couple seed until I get a good obstacle field.
  //Not going to error check, i.e. if the starting point is inside an
  //obstacle or whatever. Ill just check and try a different seed.

  ompl::RNG rng;
  const int max_obstacles = 3;

  for(int i = 0; i < max_obstacles; i++){
    Rectangle *rect = new Rectangle();

    rect->width = rng.uniformReal(5, 10);
    rect->height = rng.uniformReal(40, 80);

    rect->x = -80 + (160*i/(max_obstacles-1)); //rng.uniformReal(-100, 100);
    rect->y = rng.uniformReal(-50, 50) - rect->height/2;


    obstacles.push_back(rect);
  }

}


void SimpleTerrainMap::isStateValid(float x, float y){
  for(unsigned i = 0; i < obstacles.size(); i++){
    if(isPosInBox(x, y, obstacles[i])){
      return false;
    }
  }

  return (x > Xmin) && (x < Xmax) &&
         (y > Ymin) && (y < Ymax);
}
