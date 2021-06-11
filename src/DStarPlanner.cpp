#include "DStarPlanner.h"

#include <algorithm>
#include <stdlib.h>
#include <unistd.h>

#define INIT_VALUE 0xDEADBEEF //A random value. Will be useful for debugging and detecting unitialized values
#define DEBUG_WINDOW 1


DStarPlanner::DStarPlanner(SimpleTerrainMap *map) : terrain_map_(map){
    dpy = 0;
    curr_waypoint_ = 0;
}

DStarPlanner::~DStarPlanner(){
    if(dpy){
        XDestroyWindow(dpy, w);
        XCloseDisplay(dpy);
    }
}

void DStarPlanner::setGlobalPath(const std::vector<RigidBodyDynamics::Math::Vector2d> &waypoints){
    for(unsigned i = 0; i < waypoints.size(); i++){
        waypoints_.push_back(Vector2f(waypoints[i][0], waypoints[i][1]));
    }
}

int DStarPlanner::initPlanner(Vector2f start, Vector2f goal){
    x_range_ = 2*fabs(start[0] - goal[0]);
    x_offset_ = std::min(start[0], goal[0]) - (x_range_*.5);

    y_range_ = 2*fabs(start[1] - goal[1]);
    y_offset_ = std::min(start[1], goal[1]) - (y_range_*.5);
    
    
    for(unsigned i = 0; i < COSTMAP_WIDTH; i++){
        for(unsigned j = 0; j < COSTMAP_HEIGHT; j++){
            state_map_[i][j].tag = NEW;

            Vector2f test_pos = getRealPosition(i, j);
            if(terrain_map_->isStateValid(test_pos[0], test_pos[1])){
                state_map_[i][j].occupancy = OBSTACLE; 
            }
            else{
                state_map_[i][j].occupancy = FREE;
            }

            state_map_[i][j].min_cost = -1;
            state_map_[i][j].curr_cost = INIT_VALUE;
            state_map_[i][j].b_ptr = 0;
            state_map_[i][j].x = i;
            state_map_[i][j].y = j;
        }
    }
    
    
    StateData *goal_state = readStateMap(goal);
    insertState(goal_state, 0);
    goal_state->b_ptr = 0;
    
    float k_min;
    StateData* state_xc = readStateMap(start);
    do{
        k_min = processState();
    } while((k_min == -1) || (state_xc->tag == CLOSED));

    return k_min;
}

void DStarPlanner::replan(StateData* robot_state){
    float k_min;
    
    do{
        k_min = processState();
    } while((k_min >= robot_state->curr_cost) || (k_min == -1));
}

int DStarPlanner::runPlanner(){
    Vector2f X_pos;
    StateData* X_state;
    
    X_pos = waypoints_[0];
    //idx is the current waypoint, idx+1 is the goal
    for(unsigned idx = 0; idx < (waypoints_.size()-1); idx++){
        X_state = readStateMap(X_pos);
        
        //This finds a solution. aka sets the backpointers from start to goal
        if(initPlanner(X_pos, waypoints_[idx+1]) == -1){
            return 0;
        }
        
        do{ //This scans for new obstacles, replans, and follows the backptr
            stepPlanner(X_state, X_pos);
        } while(readStateMap(waypoints_[idx+1]) != X_state);
        
        X_pos = getRealPosition(X_state->x, X_state->y);
    }
    
    return 1;
}

void DStarPlanner::stepPlanner(StateData *robot_state, Vector2f X_robot){
    //This is going to scan the environment and update the node occupancy/graph edge costs
    //The following is going to be slow as hell. But this is just a demo.
    //Just go over all the nodes in state_map_
    //Make sure all nodes that are inside of obstacles are marked obstacles
    //If the node wasn't marked obstacle and now is, and state was CLOSED
    //Then add node to the open_list_
    //This is like prepare_repair()
    
    if(terrain_map_->detectObstacles(X_robot[0], X_robot[1])){
        for(unsigned i = 0; i < COSTMAP_WIDTH; i++){
            for(unsigned j = 0; j < COSTMAP_HEIGHT; j++){
                Vector2f test_pos = getRealPosition(i, j);
                if((state_map_[i][j].occupancy != OBSTACLE) && !terrain_map_->isStateValid(test_pos[0], test_pos[1])){
                    state_map_[i][j].occupancy = OBSTACLE;
                    insertState(&state_map_[i][j], state_map_[i][j].curr_cost);
                }
            }
        }
    }
    
    replan(robot_state); //pass current robot position.
    robot_state = robot_state->b_ptr;
}

int DStarPlanner::processState(){
    if(open_list_.empty()){
        return -1;
    }
    
    StateData *X = open_list_[0];
    float k_old = X->min_cost;
    
    deleteState(X);

    // each state has 8 Neighbors
    const float dxdy[8][2] = {
        {-1, -1},
        {-1,  0},
        {-1,  1},
        {0, -1},
        {0,  1},
        {1, -1},
        {1,  0},
        {1,  1}
    };
    
    std::vector<StateData*> neighbors;
    for(int i = 0; i < 8; i++){
        unsigned nx = X->x + dxdy[i][0];
        unsigned ny = X->y + dxdy[i][1];
        if((nx >= 0 && nx < COSTMAP_WIDTH) &&
           (ny >= 0 && ny < COSTMAP_HEIGHT)){
            neighbors.push_back(&state_map_[nx][ny]);
        }
    }

    StateData *Y;

    //Raise
    if(k_old < X->curr_cost){
        drawState(X, RAISE);
        
        for(unsigned i = 0; i < neighbors.size(); i++){
            Y = neighbors[i];
            if(Y->tag != NEW &&
               Y->curr_cost <= k_old &&
               X->curr_cost > (Y->curr_cost + getEdgeCost(Y, X))){
                
                X->b_ptr = Y;
                X->curr_cost = Y->curr_cost + getEdgeCost(Y, X);
            }
        }
    }

    //Lower
    if(k_old == X->curr_cost){
        drawState(X, LOWER);
        
        for(unsigned i = 0; i < neighbors.size(); i++){
            if(Y->tag == NEW ||
              (Y->b_ptr == X && Y->curr_cost != (X->curr_cost + getEdgeCost(X, Y))) ||
              (Y->b_ptr != X && Y->curr_cost > (X->curr_cost + getEdgeCost(X, Y)))){
                Y->b_ptr = X;
                insertState(Y, X->curr_cost + getEdgeCost(X,Y));
            }
        }
    }
    else{ //Nothing?
        drawState(X, NORMAL);
        for(unsigned i = 0; i < neighbors.size(); i++){
            if(Y->tag == NEW || (Y->b_ptr == X && Y->curr_cost != (X->curr_cost + getEdgeCost(X, Y)))){
                Y->b_ptr = X;
                insertState(Y, X->curr_cost + getEdgeCost(X, Y));
            }
            else{
                if(Y->b_ptr != X && Y->curr_cost > (X->curr_cost + getEdgeCost(X, Y))){
                    insertState(X, X->curr_cost);
                }
                else if(Y->b_ptr != X &&
                        X->curr_cost > (Y->curr_cost + getEdgeCost(Y, X)) &&
                        Y->tag == CLOSED &&
                        Y->curr_cost > k_old){
                    insertState(Y, Y->curr_cost);
                }
            }
        }
    }
    
    if(open_list_.empty())
        return -1;
    else
        return open_list_[0]->min_cost;
}

void DStarPlanner::insertState(StateData* state, float path_cost){
    
    switch(state->tag){
    case NEW:
        state->min_cost = path_cost;
        break;
    case OPEN:
        //ensure no repeats in the open list.
        for(unsigned i = 0; i < open_list_.size(); i++){
            if(open_list_[i] == state){
                open_list_.erase(open_list_.begin() + i);
                break;
            }
        }
        state->min_cost = std::min(state->min_cost, path_cost);
        break;
    case CLOSED:
        state->min_cost = std::min(state->curr_cost, path_cost);
        break;        
    }

    state->curr_cost = path_cost;
    state->tag = OPEN;
    
    for(unsigned i = 0; i < open_list_.size(); i++){
        if(open_list_[i]->min_cost > state->min_cost){
            open_list_.insert(open_list_.begin() + i, state);
            return;
        }
    }

    open_list_.push_back(state);
}

void DStarPlanner::deleteState(StateData *state){
    for(unsigned i = 0; i < open_list_.size(); i++){
        if(open_list_[i] == state){
            open_list_.erase(open_list_.begin() + i);
            state->tag = CLOSED;
            return;
        }
    }
}


float DStarPlanner::getEdgeCost(StateData* X, StateData* Y){
    switch(Y->occupancy){
    case FREE:
        return 1;
    case OBSTACLE:
        return 100000;
    default:
        return 1;
    }
    //this is going to be a bit more involved
    //No, this is just going to check if moving between states intercepts an obstacle. Thats all. For now.
}

float DStarPlanner::getPathCost(Vector2f X, Vector2f G){
    StateData* state = readStateMap(X);
    return state->curr_cost;
}

float DStarPlanner::getMinPathCost(Vector2f X, Vector2f G){
    StateData* state = readStateMap(X);
    return state->min_cost;
}

void DStarPlanner::getMapIdx(Vector2f X, unsigned &x, unsigned &y){
    float x_scale = COSTMAP_WIDTH / x_range_;
    float y_scale = COSTMAP_HEIGHT / y_range_;
    
    int x_int = floorf((X[0] - x_offset_)*x_scale);
    int y_int = floorf((X[1] - y_offset_)*y_scale);
    
    x = std::min(std::max(x_int, 0), COSTMAP_WIDTH);
    y = std::min(std::max(y_int, 0), COSTMAP_HEIGHT);    
}

StateData* DStarPlanner::readStateMap(Vector2f X){
    unsigned x;
    unsigned y;
    
    getMapIdx(X, x, y);
    return &state_map_[x][y];
}

Vector2f DStarPlanner::getRealPosition(unsigned x, unsigned y){
    float x_scale = COSTMAP_WIDTH / x_range_;
    float y_scale = COSTMAP_HEIGHT / y_range_;
    Vector2f X;
    
    X[0] = ((float)x/x_scale) + x_offset_;
    X[1] = ((float)y/y_scale) + y_offset_;
    return X;
}






/*
 * Following functions are debug related. Creates a window and shows the search graph
 * as well as the state tags so the waves of RAISE/LOWER should be visible
 */


void DStarPlanner::initWindow(){
    dpy = XOpenDisplay(0);
    w = XCreateSimpleWindow(dpy, DefaultRootWindow(dpy), 0, 0, 400, 400, 0, 0, 0);
    
    XSelectInput(dpy, w, StructureNotifyMask | ExposureMask | KeyPressMask);
    XClearWindow(dpy, w);
    XMapWindow(dpy, w);
    gc = XCreateGC(dpy, w, 0, 0);
    
    XEvent e;
    do{
        XNextEvent(dpy, &e);
    } while(e.type != MapNotify);
    
    XSetBackground(dpy, gc, 0);
    XClearWindow(dpy, w);
}

void DStarPlanner::pressEnter(){
    XEvent e;
    char buf[2];
    KeySym ks = 0;
    
    while(1){
        if(XPending(dpy) > 0){
            XNextEvent(dpy, &e);
            if(e.type == KeyPress){
                XLookupString(&e.xkey, buf, 1, &ks, NULL);
                if(ks == 0xFF0D){
                    return;
                }
            }
        }

        usleep(1000);
    }
}

void DStarPlanner::drawState(StateData *state, STATE_TYPE s_type){
    unsigned color;
    float scalar; //from 1 to zero
    
    switch(s_type){
    case RAISE:
        color = 0xFF;
        break;
    case LOWER:
        color = 0xFF00;
        break;
    case NORMAL:
        color = 0xFF0000;
        break;
    }
    
    scalar = std::min(1.0f, std::max(0.0f, 1.0f / state->curr_cost));
    
    color = color & (unsigned) (color * scalar); //This is pretty clever. Hopefully it works
    XSetForeground(dpy, gc, color);
    
    unsigned x = state->x * 4;
    unsigned y = state->y * 4;
    
    XFillRectangle(dpy, w, gc, x, y, 4, 4);
    XFlush(dpy);
}



void DStarPlanner::drawPath(StateData *state){
    XSetForeground(dpy, gc, 0xFFFFFF);
    while(state->b_ptr){
        XDrawLine(dpy, w, gc, (state->x*4) + 2, (state->y*4) + 2,  (state->b_ptr->x*4) + 2, (state->b_ptr->y*4) + 2);
        state = state->b_ptr;
    }
    XFlush(dpy);
}

