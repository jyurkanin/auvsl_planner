#include "DStarPlanner.h"

#include <algorithm>


#define INIT_VALUE 0xDEADBEEF //A random value. Will be useful for debugging and detecting unitialized values

DStarPlanner::DStarPlanner(const TerrainMap *map) terrain_map_(map){
    
    
}

void DStarPlanner::initPlanner(Vector2f start, Vector2f goal){
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

            state_map_[i][j].min_cost = INIT_VALUE;
            state_map_[i][j].curr_cost = INIT_VALUE;
            state_map_[i][j].b_ptr = INIT_VALUE;
        }
    }
    
    
    updatePathCost(goal, goal, 0);
    
    
    
    
}

void DStarPlanner::runPlanner(){
    unsigned robot_x;
    unsigned robot_y; //position in the state map.
    Vector2f X;
    
    //idx is the current waypoint, idx+1 is the goal
    for(unsigned idx = 0; idx < (waypoints_.size()-1); idx++){
        initPlanner(waypoints_[idx], waypoints_[idx+1]);
        
        do{
            X = getRealPosition(robot_x, robot_y);
            
            if(terrain_map_->detectObstacles(X[0], X[1])){
                //The following is going to be slow as hell. But this is just a demo.
                for(unsigned i = 0; i < COSTMAP_WIDTH; i++){
                    for(unsigned j = 0; j < COSTMAP_HEIGHT; j++){
                        Vector2f test_pos = getRealPosition(i, j);
                        if((state_map_[i][j].occupancy != OBSTACLE) && !terrain_map_->isStateValid(test_pos[0], test_pos[1])){
                            state_map_[i][j].occupancy = OBSTACLE;
                            insertState(test_pos);
                            
                        }
                    }
                    
                }
                //Just go over all the nodes in state_map_
                //Make sure all nodes that are inside of obstacles are marked obstacles
                //If the node wasn't marked obstacle and now is, and state was CLOSED
                //Then add node to the open_list_
            }
            
            stepPlanner();
        } while(waypoints_[idx+1] != state_map_[robot_x][robot_y]);
    }
    
}

void DStarPlanner::stepPlanner(){
    unsigned x, y;
    
    
    
    
}

void DStarPlanner::insertState(Vector2f X, float path_cost){
    StateData* state = readStateMap(X);
    
    switch(state->tag){
    case NEW:
        state->min_cost = path_cost;
        break;
    case OPEN:
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


float DStarPlanner::getEdgeCost(Vector2f X, Vector2f Y){
    StateData* state = readStateMap(X);
    //this is going to be a bit more involved
    //No, this is just going to check if moving between states intercepts an obstacle. Thats all.
}

float DStarPlanner::getPathCost(Vector2f X, Vector2f G){
    StateData* state = readStateMap(X);
    return state.curr_cost;
}

float DStarPlanner::getMinPathCost(Vector2f X, Vector2f G){
    StateData* state = readStateMap(X);
    return state.min_cost;
}


StateData* DStarPlanner::readStateMap(Vector2f X){
    float x_scale = COSTMAP_WIDTH / x_range_;
    float y_scale = COSTMAP_HEIGHT / y_range_;
    
    int x = floorf((X[0] - x_offset_)*x_scale_);
    int y = floorf((X[1] - y_offset_)*y_scale_);
    
    x = std::min(std::max(x, 0), COSTMAP_WIDTH);
    y = std::min(std::max(y, 0), COSTMAP_HEIGHT);
    
    return &state_map_[x][y];
}

Vector2f DStarPlanner::getRealPosition(unsigned x, unsigned y){
    Vector2f X;
    X[0] = (x/x_scale_) + x_offset_;
    X[1] = (y/y_scale_) + y_offset_;
    return X;
}
