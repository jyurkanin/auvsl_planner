#include "LocalPlanner.h"
#include "TerrainMap.h"

#include <Eigen/Dense> //the eigen headers dont end in .h
#include <rbdl/rbdl.h>

#include <vector>
#include <X11/keysymdef.h>
#include <X11/Xutil.h>
#include <X11/Xatom.h>
#include <X11/Xlib.h>


/*
 * Implementation for this algorithm is from
 * https://www.ri.cmu.edu/pub_files/pub3/stentz_anthony__tony__1994_2/stentz_anthony__tony__1994_2.pdf
 * And also partially from
 * https://www.cs.cmu.edu/~motionplanning/lecture/AppH-astar-dstar_howie.pdf
 * starting at slide 31. I'm using this resource instead of the original Stenz paper
 * because this slide show gives much more detailed pseudo code and is much easier to
 * understand.
 */

using namespace Eigen;

enum TAG {NEW, CLOSED, OPEN};
enum OCCUPANCY {FREE=0, OBSTACLE=10000};
enum STATE_TYPE {RAISE, LOWER, NORMAL};

#define COSTMAP_HEIGHT 100
#define COSTMAP_WIDTH 100
#define EPSILON 1e-4

//This is not efficient
struct StateData{
    float min_cost;
    float curr_cost;
    struct StateData *b_ptr;
    unsigned char x;
    unsigned char y;
    TAG tag;
    OCCUPANCY occupancy;
};

typedef struct StateData StateData;

class DStarPlanner {
public:
    DStarPlanner(SimpleTerrainMap *map);
    ~DStarPlanner();
    
    int initPlanner(Vector2f start, Vector2f goal);
    int runPlanner();
    int stepPlanner(StateData*& robot_state, Vector2f &robot_pos);
    int replan(StateData* robot_state);

    void setGlobalPath(const std::vector<RigidBodyDynamics::Math::Vector2d> &waypoints);
    
    void initWindow();
    void pressEnter();
    void drawStateType(StateData *state, STATE_TYPE s_type);
    void drawStateTag(StateData *state);
    void drawStateBPtr(StateData *state);
    void drawPath(StateData *start);
    void drawGoal(StateData *state);
    void drawFinishedGraph(StateData *state, std::vector<StateData*> &actual_path);
  
    float getEdgeCost(StateData* X, StateData* Y);    //c(X)
    float getPathCost(Vector2f X, Vector2f G);    //h(X)
    float getMinPathCost(Vector2f X, Vector2f G); //Min Path Cost since state was added to open list. This is the "key function"
    
    void insertState(StateData* X, float path_cost);
    void deleteState(StateData *state);
    
    float processState();
    

    void getMapIdx(Vector2f X, unsigned &x, unsigned &y);
    StateData* readStateMap(Vector2f X); //will perform the necessary quantization to go from floating state to grid index
    Vector2f getRealPosition(unsigned x, unsigned y);
    
private:
    float x_range_;
    float x_offset_;

    float y_range_;
    float y_offset_;
    
    StateData state_map_[COSTMAP_WIDTH][COSTMAP_HEIGHT]; //states are 8 connected
    SimpleTerrainMap *terrain_map_;
    std::vector<StateData*> open_list_; //This is going to be sorted by key function.

    unsigned curr_waypoint_;
    std::vector<Vector2f> waypoints_;

    Display *dpy;
    Window w;
    GC gc;
};
