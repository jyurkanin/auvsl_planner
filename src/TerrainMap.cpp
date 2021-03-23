#include "TerrainMap.h"
#include <math.h>


BekkerData soil_table[3] = {
    {29.76, 2083, .8,  0, 22.5*M_PI/180.0, "Medium Soil"},
    {102,   5301, .8, .2, 31.1*M_PI/180.0, "Sand"},
    {30.08, 499.7, .7, 0, 14.0*M_PI/180.0, "Clay"}
};

BekkerData lookupSoilTable(int index){
    return soil_table[index];
}


BekkerData get_soil_data_at(float x, float y){
  const int search_depth = 0;
  const float threshold = .5;
  //octomap::OcTreeNode* node = ocTree->search(val[0], val[1], 0, search_depth);
  //TODO. THis.
  return lookupSoilTable(0);
}


float get_altitude(float x, float y){
  return 0;
}

