#include <vector>

typedef struct {
    float kc,kphi,n0,n1,phi;
    char *name;
} BekkerData;

typedef struct{
  float x, y; //bottom left
  float width, height;
} Rectangle;


BekkerData lookup_soil_table(int index);


class TerrainMap{
    virtual BekkerData getSoilDataAt(float x, float y) = 0;
    virtual float getAltitude(float x, float y) = 0;
    virtual int isStateValid(float x, float y) = 0;
};


class SimpleTerrainMap : TerrainMap{
  SimpleTerrainMap();
  ~SimpleTerrainMap();

  void generateObstacles();
  void generateUnknownObstacles();

  int detectObstacles(float x, float y);
  BekkerData getSoilDataAt(float x, float y);
  float getAltitude(float x, float y);
  int isStateValid(float x, float y); //Only 2D obstacle collision checking for now.
  
private:
  float Xmax;
  float Xmin;

  float Ymax;
  float Ymin;
  
  std::vector<Rectangle*> obstacles;
  std::vector<Rectangle*> unknown_obstacles;
};
