#pragma once

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
public:
    virtual BekkerData getSoilDataAt(float x, float y) const = 0;
    virtual float getAltitude(float x, float y) const = 0;
    virtual int isStateValid(float x, float y) const = 0;
    virtual std::vector<Rectangle*> getObstacles() const = 0;
};


class SimpleTerrainMap : public TerrainMap{
public:
  SimpleTerrainMap();
  ~SimpleTerrainMap();

  void generateObstacles();
  void generateUnknownObstacles();

  int detectObstacles(float x, float y);

  BekkerData getSoilDataAt(float x, float y) const override;
  float getAltitude(float x, float y) const override;
  int isStateValid(float x, float y) const override; //Only 2D obstacle collision checking for now.

  std::vector<Rectangle*> getObstacles() const override;
private:
  float Xmax;
  float Xmin;

  float Ymax;
  float Ymin;

  std::vector<Rectangle*> obstacles;
  std::vector<Rectangle*> unknown_obstacles;
};
