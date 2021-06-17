#include "DStarPlanner.h"

#include <ros/ros.h>

#include <algorithm>
#include <stdlib.h>
#include <unistd.h>

#define INIT_VALUE 0xDEADBEEF //A random value. Will be useful for debugging and detecting unitialized values
#define DEBUG_WINDOW 1


//valgrind --leak-check=full --track-origins=yes


DStarPlanner::DStarPlanner(SimpleTerrainMap *map) : terrain_map_(map){
    dpy = 0;
    w = 0;
    gc = 0;

    
    curr_waypoint_ = 0;
    /*
    x_range_ = 0;
    x_offset_ = 0;
    y_range_ = 0;
    y_offset_ = 0;
    */
    
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
    x_offset_ = std::min(start[0], goal[0]) - (x_range_*.25);

    y_range_ = 2*fabs(start[1] - goal[1]);
    y_offset_ = std::min(start[1], goal[1]) - (y_range_*.25);
    
    
    for(unsigned i = 0; i < COSTMAP_WIDTH; i++){
        for(unsigned j = 0; j < COSTMAP_HEIGHT; j++){
            state_map_[i][j].tag = NEW;

            Vector2f test_pos = getRealPosition(i, j);
            if(terrain_map_->isStateValid(test_pos[0], test_pos[1])){
                state_map_[i][j].occupancy = FREE; 
            }
            else{
                state_map_[i][j].occupancy = OBSTACLE;
            }

            state_map_[i][j].min_cost = -1;
            state_map_[i][j].curr_cost = 1;
            state_map_[i][j].b_ptr = 0;
            state_map_[i][j].x = i;
            state_map_[i][j].y = j;

            drawStateTag(&state_map_[i][j]);
        }
    }

    
    StateData *goal_state = readStateMap(goal);
    insertState(goal_state, 0);
    goal_state->b_ptr = 0;
    drawGoal(goal_state);
    
    
    
    float k_min;
    StateData* state_xc = readStateMap(start);
    drawGoal(state_xc);
    XFlush(dpy);
    
    do{
        k_min = processState();
    } while((k_min != -1) && (state_xc->tag != CLOSED));

    return k_min;
}

int DStarPlanner::replan(StateData* robot_state){
    float k_min;
    
    do{
        k_min = processState();
    } while(!(k_min >= robot_state->curr_cost) && (k_min != -1));
    
    return k_min;
}

int DStarPlanner::runPlanner(){
    Vector2f X_pos;
    StateData* X_state;
    StateData* temp_start;
    StateData* temp_goal;
    
    std::vector<StateData*> actual_path;
    
    X_pos = waypoints_[0];
    //idx is the current waypoint, idx+1 is the goal
    for(unsigned idx = 0; idx < (waypoints_.size()-1); idx++){      
        if(initPlanner(X_pos, waypoints_[idx+1]) == -1){   //This finds a solution. aka sets the backpointers from start to goal
            ROS_INFO("Couldn't find an initial path");
            return 0;
        }
        
        temp_start = readStateMap(X_pos);
        temp_goal = readStateMap(waypoints_[idx+1]);
        
        //ROS_INFO("Start State %u %u", temp_start->x, temp_start->y);
        //ROS_INFO("Goal State %u %u", temp_goal->x, temp_goal->y);
        
        drawGoal(temp_start);
        drawGoal(temp_goal);
        
        X_state = temp_start;
        
        actual_path.clear();
        actual_path.push_back(X_state);
        
        do{ //This scans for new obstacles, replans, and follows the backptr
          if(stepPlanner(X_state, X_pos) == -1){ //could not find a path to goal.
            ROS_INFO("Couldn't find a new path");
            return 0;
          }

          actual_path.push_back(X_state);
          
          drawGoal(X_state);
          XFlush(dpy);
        } while(readStateMap(waypoints_[idx+1]) != X_state);
        
        pressEnter();
        
        drawFinishedGraph(temp_start, actual_path);
        pressEnter();
        
    }
    
    ROS_INFO("Successfully reached goal");
    
    return 1;
}

int DStarPlanner::stepPlanner(StateData*& robot_state, Vector2f &X_robot){
    //This is going to scan the environment and update the node occupancy/graph edge costs
    //The following is going to be slow as hell. But this is just a demo.
    //Just go over all the nodes in state_map_
    //Make sure all nodes that are inside of obstacles are marked obstacles
    //If the node wasn't marked obstacle and now is, and state was CLOSED
    //Then add node to the open_list_
    //This is like prepare_repair()
    
    //ROS_INFO("stepPlanner %f %f", X_robot[0], X_robot[1]);
    
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

    
    if(replan(robot_state) == -1) //pass current robot position.
      return -1;
    
    robot_state = robot_state->b_ptr;
    X_robot = getRealPosition(robot_state->x, robot_state->y);
    
    return 1;
}

float DStarPlanner::processState(){
    if(open_list_.empty()){
        return -1;
    }
    
    StateData *X = open_list_[0];
    float k_old = X->min_cost;
    
    deleteState(X);

    // each state has 8 Neighbors
    const float dxdy[8][2] = {
        {-1,-1},
        {-1, 0},
        {-1, 1},
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

    //ROS_INFO("Num neighbors %lu", neighbors.size());
    StateData *Y;
    
    ROS_INFO("State curr cost %f,   kmin %f", X->curr_cost, k_old);
    
    //Raise
    // k_old < X->curr_cost
    if((X->curr_cost - k_old) > EPSILON){
        drawStateType(X, RAISE);
        
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
    // k_old == X->curr_cost but for floats.
    if(fabs(k_old - X->curr_cost) < EPSILON){
        drawStateType(X, LOWER);
        
        for(unsigned i = 0; i < neighbors.size(); i++){
            Y = neighbors[i];
            if(Y->tag == NEW ||
              (Y->b_ptr == X && Y->curr_cost != (X->curr_cost + getEdgeCost(X, Y))) ||
              (Y->b_ptr != X && Y->curr_cost > (X->curr_cost + getEdgeCost(X, Y)))){
                Y->b_ptr = X;
                insertState(Y, X->curr_cost + getEdgeCost(X,Y));
            }
        }

    }
    else{ //Nothing?
        drawStateType(X, NORMAL);
        
        for(unsigned i = 0; i < neighbors.size(); i++){
            Y = neighbors[i];
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
    
    XFlush(dpy);
    
    if(open_list_.empty())
        return -1;
    else
        return open_list_[0]->min_cost;
}

void DStarPlanner::insertState(StateData* state, float path_cost){
  //ROS_INFO("insertState   open_lst.size()  %lu", open_list_.size());
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

    //if(open_list_.empty()){
    //  open_list_.push_back(state);
    //}
    
    //ROS_INFO("current state min cost %f", state->min_cost);
    
    for(unsigned i = 0; i < open_list_.size(); i++){
        if(open_list_[i]->min_cost > state->min_cost){
            open_list_.insert(open_list_.begin() + i, state);
            return;
        }
    }
    
    drawStateTag(state);
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
    int dx;
    int dy;
    switch(Y->occupancy){
    case FREE:
        dx = X->x - Y->x;
        dy = X->y - Y->y;
        return sqrtf(dx*dx + dy*dy);
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

    //ROS_INFO("getMapIdx    %f  %f      %d  %d",   X[0], X[1],   x_int, y_int);
    
    x = std::min(std::max(x_int, 0), COSTMAP_WIDTH);
    y = std::min(std::max(y_int, 0), COSTMAP_HEIGHT);
}

StateData* DStarPlanner::readStateMap(Vector2f X){
    unsigned x;
    unsigned y;
    
    getMapIdx(X, x, y);
    return &(state_map_[x][y]);
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
    w = XCreateSimpleWindow(dpy, DefaultRootWindow(dpy), 0, 0, 800, 800, 0, 0, 0);
    
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

void DStarPlanner::drawStateType(StateData *state, STATE_TYPE s_type){
    unsigned color;
    float scalar; //from 1 to zero
    
    switch(s_type){
    case RAISE:
        color = 0x0000FF;
        break;
    case LOWER:
        color = 0x00FF00;
        break;
    case NORMAL:
        color = 0xFF0000;
        break;
    }
    
    scalar = std::min(1.0f, std::max(0.0f, (float)(.1+ .01*state->curr_cost)));
    
    color = color & (unsigned) (color * scalar); //This is pretty clever. Hopefully it works
    XSetForeground(dpy, gc, color);
    
    unsigned x = state->x * 8;
    unsigned y = state->y * 8;
    
    XFillRectangle(dpy, w, gc, x, y, 8, 8);
    
    drawStateTag(state);
    drawStateBPtr(state);
}

void DStarPlanner::drawStateTag(StateData* state){
    float scalar = std::min(1.0f, std::max(0.0f, (float)(.1+ .01*state->curr_cost)));
    unsigned color;
    
    unsigned x = state->x * 8;
    unsigned y = state->y * 8;
    
    switch(state->tag){
    case NEW:
        color = 0x0000FF;//BLUE
        break;
    case CLOSED:
        color = 0x00FF00; //GREEN
        break;
    case OPEN:
        color = 0xFF0000; //RED
        break;
    }
    
    color = color & (unsigned) (color * scalar); //This is pretty clever. Hopefully it works
    XSetForeground(dpy, gc, 0x00);
    XFillRectangle(dpy, w, gc, x+2, y+2, 4, 4);
    
    XSetForeground(dpy, gc, color);
    XDrawLine(dpy, w, gc, x+2, y+4, x+6, y+4);
    XDrawLine(dpy, w, gc, x+4, y+2, x+4, y+6);
}

void DStarPlanner::drawStateBPtr(StateData *state){
  if(!state->b_ptr){
    return;
  }
       
  unsigned start_x = state->x*8 + 4;
  unsigned start_y = state->y*8 + 4;
  
  unsigned end_x = state->b_ptr->x*8 + 4;
  unsigned end_y = state->b_ptr->y*8 + 4;
  
  XSetForeground(dpy, gc, 0xFFFFFF);
  XDrawLine(dpy, w, gc, start_x, start_y, end_x, end_y);

  int dx = 1 + state->x - state->b_ptr->x;
  int dy = 1 + state->y - state->b_ptr->y;
  
  //Draws arrows.
  switch(dx){
  case 0:
    switch(dy){
    case 0:
      XDrawLine(dpy, w, gc, end_x, end_y, end_x-2, end_y);
      XDrawLine(dpy, w, gc, end_x, end_y, end_x, end_y-2);
      break;
    case 1:
      XDrawLine(dpy, w, gc, end_x, end_y, end_x-2, end_y-2);
      XDrawLine(dpy, w, gc, end_x, end_y, end_x-2, end_y+2);
      break;
    case 2:
      XDrawLine(dpy, w, gc, end_x, end_y, end_x-2, end_y);
      XDrawLine(dpy, w, gc, end_x, end_y, end_x, end_y+2);
      break;
    }
    break;
  case 1:
    switch(dy){
    case 0:
      XDrawLine(dpy, w, gc, end_x, end_y, end_x-2, end_y-2);
      XDrawLine(dpy, w, gc, end_x, end_y, end_x+2, end_y-2);
      break;
    case 1:
      ROS_INFO("This should never happen.");
      break;
    case 2:
      XDrawLine(dpy, w, gc, end_x, end_y, end_x-2, end_y+2);
      XDrawLine(dpy, w, gc, end_x, end_y, end_x+2, end_y+2);
      break;
    }
    break;
  case 2:
    switch(dy){
    case 0:
      XDrawLine(dpy, w, gc, end_x, end_y, end_x, end_y-2);
      XDrawLine(dpy, w, gc, end_x, end_y, end_x+2, end_y);
      break;
    case 1:
      XDrawLine(dpy, w, gc, end_x, end_y, end_x+2, end_y-2);
      XDrawLine(dpy, w, gc, end_x, end_y, end_x+2, end_y+2);
      break;
    case 2:
      XDrawLine(dpy, w, gc, end_x, end_y, end_x, end_y+2);
      XDrawLine(dpy, w, gc, end_x, end_y, end_x+2, end_y);
      break;
    }
    break;
  }
  
}


void DStarPlanner::drawGoal(StateData *state){
  XSetForeground(dpy, gc, 0xFF0000);
  XFillRectangle(dpy, w, gc, state->x*8, state->y*8, 8, 8);  
}

void DStarPlanner::drawPath(StateData *state){
    XSetForeground(dpy, gc, 0xFFFFFF);
    while(state->b_ptr){
        XDrawLine(dpy, w, gc, (state->x*8) + 4, (state->y*8) + 4,  (state->b_ptr->x*8) + 4, (state->b_ptr->y*8) + 4);
        state = state->b_ptr;
    }
    XFlush(dpy);
}


void DStarPlanner::drawFinishedGraph(StateData *start, std::vector<StateData*> &actual_path){
  terrain_map_->detectAllObstacles();

  Vector2f test_pos;
  
  XClearWindow(dpy, w);
  
  XSetForeground(dpy, gc, 0xFF0000);
  for(unsigned i = 0; i < COSTMAP_WIDTH; i++){
    for(unsigned j = 0; j < COSTMAP_HEIGHT; j++){
      test_pos = getRealPosition(i, j);
      
      if(!terrain_map_->isStateValid(test_pos[0], test_pos[1])){
        XFillRectangle(dpy, w, gc, i*8, j*8, 8, 8);  
      }
    }
  }
  
  XSetForeground(dpy, gc, 0xFF);
  StateData *state;
  for(unsigned i = 0 ; i < actual_path.size(); i++){
    state = actual_path[i];
    XFillRectangle(dpy, w, gc, state->x*8, state->y*8, 8, 8);
  }
  
  XFlush(dpy);
}
