#include "DStarPlanner.h"

#include <ros/ros.h>

#include <algorithm>
#include <stdlib.h>
#include <unistd.h>

#include <assert.h>

#define INIT_VALUE 0xDEADBEEF //A random value. Will be useful for debugging and detecting unitialized values
#define DEBUG_WINDOW 1


//valgrind --track-origins=yes --log-file=/home/justin/temp/log.txt 


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
  /*
  float temp[688] = {50.005318, 30.000135, 51.515915, 29.943779, 54.085880, 29.894531, 56.531544, 29.899601, 59.224026, 29.902296, 61.008316, 29.804209, 62.200592, 29.486027, 62.307465, 29.443037, 63.248497, 28.871017, 63.624275, 27.366173, 61.545410, 25.565388, 57.465889, 23.754435, 56.754200, 22.171093, 56.890259, 21.921593, 56.854515, 21.657330, 56.379047, 21.444605, 56.300194, 21.478121, 56.133354, 21.555782, 55.626213, 21.586813, 54.800575, 21.682808, 54.241837, 21.756905, 53.475906, 22.158577, 52.437035, 22.071800, 51.485943, 22.059385, 50.520130, 22.260021, 50.309113, 22.291372, 49.769203, 22.560165, 49.250061, 22.866581, 48.722706, 23.281155, 48.505810, 23.542593, 48.192532, 24.136061, 47.642200, 25.153612, 47.092308, 26.006124, 46.980782, 26.159800, 46.823582, 26.369879, 45.813736, 26.555925, 45.679214, 26.537151, 45.427177, 26.504354, 44.696571, 26.414984, 44.223743, 26.390165, 43.917133, 26.410431, 43.316303, 26.511402, 42.309441, 26.365337, 40.728809, 25.667301, 40.188606, 24.787556, 39.794430, 24.004606, 40.552666, 23.268311, 41.166821, 23.196060, 41.257477, 23.183035, 41.307011, 23.174814, 41.471615, 23.142094, 42.443325, 22.892309, 43.596680, 22.935583, 44.152191, 23.012228, 44.544582, 22.921164, 45.087036, 22.795900, 45.384758, 22.727400, 45.994457, 22.617363, 47.855354, 22.077627, 48.526577, 21.088009, 49.077335, 20.279444, 49.586052, 19.421925, 50.200634, 18.019773, 50.518566, 17.177940, 50.895561, 16.064957, 50.980110, 15.828439, 51.207783, 14.748932, 51.430004, 13.715414, 51.488155, 13.326453, 51.435253, 12.494951, 51.034145, 11.620695, 50.031612, 10.187464, 48.305763, 7.782977, 46.965164, 5.755279, 46.372620, 3.606594, 46.443077, 2.699655, 46.677917, 1.970012, 46.989231, 0.917756, 46.988873, 0.022051, 46.889629, -0.757176, 46.715797, -1.026788, 46.206253, -1.433213, 45.800541, -1.625837, 45.439697, -1.584986, 44.903992, -1.486623, 44.059414, -1.327714, 42.658661, -0.872657, 41.689762, -0.853961, 41.386776, -1.852529, 40.874405, -2.716685, 40.001087, -3.501657, 38.885254, -4.100448, 38.121681, -4.484042, 37.753700, -4.730412, 36.661587, -5.082652, 35.952957, -5.243845, 35.089916, -5.558176, 34.544735, -5.708814, 33.980930, -5.634753, 32.936695, -5.297773, 31.071653, -4.860189, 30.356012, -4.633878, 29.901739, -4.442781, 29.604244, -4.268929, 28.714470, -3.604245, 27.982462, -3.105493, 27.941246, -2.290416, 27.835815, -1.581250, 27.537563, -0.814780, 26.755953, 0.577656, 25.738714, 2.951191, 25.952721, 3.765440, 26.648022, 6.091685, 26.869221, 8.485463, 26.127014, 10.562794, 25.492037, 11.841997, 25.455402, 13.099862, 25.614836, 13.544606, 25.088516, 13.939178, 25.034792, 14.595338, 25.315052, 16.137911, 25.027176, 17.404785, 24.481291, 17.667437, 23.359871, 17.357632, 22.679625, 17.159174, 21.470253, 17.121620, 20.861996, 17.315767, 20.862459, 17.331713, 20.999336, 17.446766, 20.875443, 17.397400, 20.753881, 17.359392, 20.727692, 17.440445, 20.692869, 17.545074, 20.718288, 17.903177, 20.555977, 19.439661, 20.401766, 20.113949, 20.081814, 21.016481, 19.457186, 22.130669, 18.791393, 23.241983, 18.614967, 24.010012, 18.548254, 27.221928, 18.511679, 28.868702, 18.534739, 30.865166, 18.581831, 32.876930, 18.496382, 33.968628, 18.453426, 34.753834, 18.499821, 36.220196, 18.556654, 38.127388, 18.594698, 39.591919, 18.651743, 41.813251, 18.685902, 44.202835, 18.390312, 45.914700, 18.301027, 46.224819, 18.115215, 46.689114, 17.862297, 47.151592, 16.564556, 49.299355, 15.792487, 51.047203, 15.777085, 52.038460, 15.774219, 52.793072, 15.746538, 53.859161, 15.754825, 54.074398, 15.793963, 55.802696, 15.786139, 56.801189, 15.789214, 57.542919, 15.763505, 58.193222, 15.724699, 60.026028, 15.753854, 60.824699, 15.834355, 61.618027, 15.947989, 62.358070, 15.962209, 63.369560, 15.949546, 63.522190, 15.916842, 63.959129, 15.906703, 64.285492, 15.919900, 65.340996, 15.912084, 66.240730, 15.891031, 67.025703, 15.880109, 67.817627, 15.893418, 68.264603, 15.906763, 69.153549, 15.887577, 70.014511, 15.879313, 70.603897, 15.861731, 71.361519, 15.859714, 72.064301, 15.855601, 72.765244, 15.850578, 73.852127, 15.841427, 74.857719, 15.836405, 75.862076, 15.844195, 76.959755, 15.851917, 77.566025, 15.839817, 78.949135, 15.839629, 79.446983, 15.865393, 80.680420, 15.870180, 82.161133, 15.871732, 82.713074, 15.875482, 84.216225, 15.856354, 86.089951, 15.889951, 87.067520, 15.906724, 89.141045, 15.711377, 91.821442, 15.329746, 92.536499, 15.320786, 92.888779, 15.115924, 93.652084, 14.513083, 93.689575, 13.827322, 93.485794, 12.589967, 93.990410, 11.423451, 94.314919, 9.966251, 93.935753, 7.822869, 92.748177, 6.145060, 92.315491, 5.267500, 91.997063, 4.985054, 91.980637, 4.058858, 91.993057, 3.558775, 92.047783, 2.386326, 92.300415, 1.459943, 92.546005, 0.883638, 93.052788, 0.367319, 92.268661, 0.000955, 90.887596, -0.020187, 90.175430, -0.073740, 89.407608, -0.097802, 88.160828, -0.115944, 87.633820, -0.137284, 87.549713, -0.165067, 86.839630, -0.192861, 86.008766, -0.232074, 85.233345, -0.386648, 84.556068, -0.785205, 83.799644, -1.296872, 83.191849, -1.662263, 82.905106, -2.246819, 82.623360, -2.781936, 82.447945, -3.987811, 82.175087, -5.251384, 81.575012, -5.725069, 81.325470, -6.148537, 81.114449, -7.149138, 80.709648, -7.858162, 80.740540, -8.600241, 80.972069, -9.477668, 81.121773, -10.615854, 81.037338, -11.391839, 80.911148, -12.505326, 80.766647, -13.451824, 80.763458, -14.797818, 80.852135, -15.204774, 80.887253, -15.883556, 80.998299, -16.323965, 81.064400, -17.108759, 81.272758, -17.969612, 81.525017, -18.525061, 81.682564, -19.625618, 81.992950, -21.213083, 82.170265, -21.358091, 82.186493, -22.162613, 82.377502, -22.389578, 82.456024, -22.590010, 82.529350, -22.793003, 82.604706, -23.495787, 82.898224, -24.683704, 83.583694, -25.754288, 84.342934, -26.458233, 84.927849, -26.746412, 85.036415, -27.157909, 85.077873, -27.861685, 85.138779, -28.618719, 85.205421, -29.013130, 85.232826, -29.098896, 85.250267, -29.980732, 85.482101, -30.932953, 85.618958, -31.780878, 85.367104, -31.781506, 85.365097, -32.213867, 84.858673, -32.825237, 84.344894, -33.017105, 84.229813, -33.671993, 83.915565, -35.329567, 83.346054, -36.321487, 83.028740, -36.546669, 82.911385, -36.671425, 82.403450, -36.447712, 81.620605, -36.425972, 81.235870, -36.434551, 80.764366, -36.434479, 80.718033, -36.415161, 78.600655, -36.406090, 77.092323, -36.279221, 75.735886, -36.029648, 71.661980, -35.633549, 69.462822, -34.402504, 68.058815, -32.966305, 66.177124, -31.478743, 64.187889, -30.434723, 60.911419, -30.322123, 58.990688, -31.383533, 56.830513, -31.497231, 52.318333, -31.506422, 48.839890, -31.619532, 46.141697, -32.054573, 43.860229, -32.947918, 41.584583, -34.746826, 40.453762, -36.727726, 38.355270, -37.189907, 37.095764, -37.032104, 36.215538, -36.779625, 35.261448, -36.736038, 33.735844, -36.642525, 31.054108, -36.722584, 29.057604, -36.818623, 27.659191, -37.093029, 25.859911, -37.701752, 24.383934, -37.806133, 24.118402, -38.284176, 23.567268, -38.818150, 23.647202, -39.445080, 23.725851, -40.359055, 23.499554, -42.104908, 23.430710, -43.206036, 23.587664, -43.269333, 23.839581, -43.272121, 24.751894, -43.214882, 25.873823, -43.260914, 26.171583, -43.464340, 27.179945, -43.851212, 28.070444, -44.124191, 28.135799, -44.513866, 28.273603, -45.218403, 28.688103, -45.775070, 29.048580, -45.905132, 29.112816, -46.724205, 29.531950, -47.082638, 29.898575, -47.292660, 30.160807, -47.759094, 30.647224, -48.030384, 30.867615, -48.186996, 30.991249, -48.702892, 31.310638, -49.304760, 31.359320, -50.094673, 31.434666, -51.046574, 31.600115, -51.931789, 31.586945, -52.900837, 31.215466, -53.290428, 30.665762, -53.413155, 30.502661, -53.417442, 30.499048};
  
  //float temp[6] = {26.869221, 8.485463, 26.127014, 10.562794, 25.492037, 11.841997};
  //float temp[4] = {26.127014, 10.562794, 25.492037, 11.841997};
  for(unsigned i = 0; i < 688; i+=2){
    waypoints_.push_back(Vector2f(temp[i], temp[i+1]));
  }
  ROS_INFO("num waypoints %lu", waypoints_.size());
}
  */
  
  Vector2f prev_wp(waypoints[0][0], waypoints[0][0]);
  const float dist = 5;

  waypoints_.push_back(prev_wp); 
  
  for(unsigned i = 1; i < waypoints.size(); i++){
    float dx = prev_wp[0] - waypoints[i][0];
    float dy = prev_wp[0] - waypoints[i][0];
    
    if(sqrtf((dx*dx) + (dy*dy)) > dist){
      waypoints_.push_back(Vector2f(waypoints[i][0], waypoints[i][1])); 
    }
  }

  ROS_INFO("num waypoints %lu", waypoints_.size());
}


int DStarPlanner::initPlanner(Vector2f start, Vector2f goal){
    open_list_.clear();
    
    x_range_ = 2*fabs(start[0] - goal[0]);
    x_offset_ = std::min(start[0], goal[0]) - (x_range_*.25);

    y_range_ = 2*fabs(start[1] - goal[1]);
    y_offset_ = std::min(start[1], goal[1]) - (y_range_*.25);
    
    XClearWindow(dpy, w);
    
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
        k_min = processState(0);
    } while((k_min != -1) && (state_xc->tag != CLOSED));

    return k_min;
}

int DStarPlanner::replan(StateData* robot_state){
    float k_min;

    ROS_INFO(" ");
    do{
        k_min = processState(1);
        ROS_INFO("REEEEplanning kmin=%f", k_min);
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
        ROS_INFO("Start State %f %f", X_pos[0], X_pos[1]);
        ROS_INFO("Goal State %f %f", waypoints_[idx+1][0], waypoints_[idx+1][1]);

        if(!terrain_map_->isRealStateValid(X_pos[0], X_pos[1])){
          ROS_INFO("Starting sTATE IS invalid.");
        }
        if(!terrain_map_->isRealStateValid(waypoints_[idx+1][0], waypoints_[idx+1][1])){
          ROS_INFO("Goal sTATE IS invalid.");
        }
        
        if(initPlanner(X_pos, waypoints_[idx+1]) == -1){   //This finds a solution. aka sets the backpointers from start to goal
            ROS_INFO("Couldn't find an initial path");
            pressEnter();
            return 0;
        }
        
        temp_start = readStateMap(X_pos);
        temp_goal = readStateMap(waypoints_[idx+1]);
        
        drawGoal(temp_start);
        drawGoal(temp_goal);
        
        X_state = temp_start;
        
        actual_path.clear();
        actual_path.push_back(X_state);
        
        do{ //This scans for new obstacles, replans, and follows the backptr
          stepPlanner(X_state, X_pos);
          if(!X_state){ //could not find a path to goal.
            ROS_INFO("Couldn't find a new path");
            return 0;
          }

          actual_path.push_back(X_state);
          
          drawRobotPos(X_state);
          XFlush(dpy);
          pressEnter();
        } while(readStateMap(waypoints_[idx+1]) != X_state);

        ROS_INFO("Reached goal");
        pressEnter();
        drawFinishedGraph(temp_start, actual_path);
        pressEnter();
        
        
    }
    
    
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
        ROS_INFO("New Obstacles Detected");
        for(unsigned i = 0; i < COSTMAP_WIDTH; i++){
            for(unsigned j = 0; j < COSTMAP_HEIGHT; j++){
                Vector2f test_pos = getRealPosition(i, j);
                if((state_map_[i][j].occupancy != OBSTACLE) && !terrain_map_->isStateValid(test_pos[0], test_pos[1])){
                    ROS_INFO("obstacle at idx %d %d", i, j);
                    state_map_[i][j].occupancy = OBSTACLE;
                    insertState(&state_map_[i][j], state_map_[i][j].curr_cost);
                }
            }
        }
    }
    
    
    int k_min = replan(robot_state); //pass current robot position.
    
    robot_state = robot_state->b_ptr;
    X_robot = getRealPosition(robot_state->x, robot_state->y);
    
    return k_min;
}

void DStarPlanner::getNeighbors(std::vector<StateData*> &neighbors, StateData* X, int replan){
  // each state has 8 Neighbors, top left, top, top right, left, right, etc...
  const char dxdy[8][2] = {
                           {-1,-1},
                           {-1, 0},
                           {-1, 1},
                           {0, -1},
                           {0,  1},
                           {1, -1},
                           {1,  0},
                           {1,  1}
  };
  
  if(X->occupancy == OBSTACLE){
    drawObstacle(&state_map_[X->x][X->y]);
  }

  
  Vector2f pos;
  for(int i = 0; i < 8; i++){
    char nx = X->x + dxdy[i][0];
    char ny = X->y + dxdy[i][1];
    pos = getRealPosition(nx, ny);

    //Make sure state is within the grid. Or else a segfault will occur
    if((nx >= 0) && (nx < COSTMAP_WIDTH) && (ny >= 0) && (ny < COSTMAP_HEIGHT)){

      //This if is important. If during replanning, a state that was discovered to be an obstacle
      //is not expanded, then the b_ptrs will not be updated correctly.
      //But during the initial planning phase it's not necessary to expand states that have
      //obstacles.
      neighbors.push_back(&state_map_[nx][ny]);
      if(state_map_[nx][ny].occupancy == OBSTACLE){
        drawObstacle(&state_map_[nx][ny]);
      }
      /*
      if(replan){
        neighbors.push_back(&state_map_[nx][ny]);
        if(state_map_[nx][ny].occupancy == OBSTACLE){
          drawObstacle(&state_map_[nx][ny]);
        }
      }
      else{
        if(terrain_map_->isStateValid(pos[0], pos[1])){
          //ROS_INFO("nx ny    %u %u", nx, ny);
          neighbors.push_back(&state_map_[nx][ny]);
        }
        else{
          drawObstacle(&state_map_[nx][ny]);
        } 
      }
      */
    }
  }
}

float DStarPlanner::processState(int replan){
    if(open_list_.empty()){
        return -1;
    }
    
    StateData *X = open_list_[0];
    float k_old = X->min_cost;
    
    deleteState(X);
    
    //ROS_INFO("Current x y   %u %u", X->x, X->y);
    std::vector<StateData*> neighbors;
    getNeighbors(neighbors, X, replan);

    //ROS_INFO("Num neighbors %lu", neighbors.size());
    StateData *Y;
    
    //ROS_INFO("State curr cost %f,   kmin %f", X->curr_cost, k_old);
    
    //Raise
    // k_old < X->curr_cost
    if(k_old < X->curr_cost){
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

    //Lower bluh
    // k_old == X->curr_cost but for floats.
    if(fabs(k_old - X->curr_cost) < EPSILON){
        drawStateType(X, LOWER);
        
        for(unsigned i = 0; i < neighbors.size(); i++){
            Y = neighbors[i];
            //ROS_INFO("Neighbor %u %u", Y->x, Y->y);
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

Vector2f DStarPlanner::getRealPosition(int x, int y){
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

void DStarPlanner::drawObstacle(StateData *state){
    unsigned x = state->x * 8;
    unsigned y = state->y * 8;

    XSetForeground(dpy, gc, 0xFF0000); //Red
    XFillRectangle(dpy, w, gc, x, y, 8, 8);
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

void DStarPlanner::drawRobotPos(StateData* state){
  XSetForeground(dpy, gc, 0xFF);
  XFillRectangle(dpy, w, gc, state->x*8, state->y*8, 8, 8);  
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
    pressEnter();
    
    state = actual_path[i];
    XFillRectangle(dpy, w, gc, state->x*8, state->y*8, 8, 8);
    XFlush(dpy);
  }
  
  
}
