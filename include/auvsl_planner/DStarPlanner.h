#include "LocalPlanner.h"

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

enum B_PTR {UP=0b00,DOWN=0b01,LEFT=0b00,RIGHT=0b10};
enum TAG {NEW, CLOSED, OPEN};
enum ;


typedef struct{
    float min_cost;
    float curr_cost;
    unsigned char b_ptr;
    TAG tag;
} StateData;


class DStarPlanner : public LocalPlanner {
public:
    DStarPlanner();
    ~DStarPlanner();
    
    void initPlanner();
    void update();
    
    void getStateData(Vector2f X);
    
    void getEdgeCost(Vector2f X, Vector2f Y);
    void getPathCost(Vector2f X, Vector2f G);
    void getMinPathCost(Vector2f X, Vector2f G); //Min Path Cost since state was added to open list. This is the "key function"
    
    void processState();
    void modifyCost();
    
private:
    StateData state_map_[100][100]; //states are 8 connected
    
    std::vector<Vector2f> open_list_; //This is going to be sorted by key function.
    //std::
    
};
