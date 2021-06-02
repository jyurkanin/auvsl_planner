#include "LocalPlanner.h"
#include "TerrainMap.h"

#include "Eigen/Core" //the eigen headers dont end in .h

#include <vector>



/*
 * Implementation for this algorithm is from
 * https://www.ri.cmu.edu/pub_files/pub3/stentz_anthony__tony__1994_2/stentz_anthony__tony__1994_2.pdf
 * And also partially from
 * https://www.cs.cmu.edu/~motionplanning/lecture/AppH-astar-dstar_howie.pdf
 * starting at slide 31. I'm using this resource instead of the original Stenz paper
 * because this slide show gives much more detailed pseudo code and is much easier to
 * understand.
 */

enum TAG {NEW, CLOSED, OPEN};
enum OCCUPANCY {FREE=0, OBSTACLE=10000};

#define COSTMAP_HEIGHT 100
#define COSTMAP_WIDTH 100

typedef struct{
    float min_cost;
    float curr_cost;
    StateData* b_ptr;
    TAG tag;
    OCCUPANCY occupancy;
} StateData;


class DStarPlanner : public LocalPlanner {
public:
    DStarPlanner(const TerrainMap *map);
    ~DStarPlanner();
    
    void initPlanner(Vector2f start, Vector2f goal);
    void runPlanner();
    void stepPlanner();
    
    void getStateData(Vector2f X);
    
    float getEdgeCost(Vector2f X, Vector2f Y);    //c(X)
    float getPathCost(Vector2f X, Vector2f G);    //h(X)
    float getMinPathCost(Vector2f X, Vector2f G); //Min Path Cost since state was added to open list. This is the "key function"
    
    void insertState(Vector2f X);
    
    void processState();
    
    StateData* readStateMap(Vector2f X); //will perform the necessary quantization to go from floating state to grid index
    Vector2f getRealPosition(unsigned x, unsigned y);
    
private:
    float x_range_;
    float x_offset_;

    float y_range_;
    float y_offset_;
    
    StateData state_map_[COSTMAP_WIDTH][COSTMAP_HEIGHT]; //states are 8 connected
    const TerrainMap *terrain_map_;
    std::vector<StateData*> open_list_; //This is going to be sorted by key function.
    
};
