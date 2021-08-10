#include "DStarPlanner.h"


#include <algorithm>
#include <mutex>
#include <stdlib.h>
#include <unistd.h>

#include <assert.h>


#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>



#define INIT_VALUE 0xDEADBEEF //A random value. Will be useful for debugging and detecting unitialized values
#define DEBUG_WINDOW 1


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(auvsl::DStarPlanner, nav_core::BaseLocalPlanner)


//valgrind --track-origins=yes --log-file=/home/justin/temp/log.txt 
using namespace auvsl;

DStarPlanner::DStarPlanner(){
    dpy = 0;
    w = 0;
    gc = 0;

    
    curr_waypoint_ = 0;
    initialized_ = 0;

    private_nh_ = new ros::NodeHandle("~/local_planner");
    private_nh_->getParam("/move_base/local_costmap/resolution", map_res_); //map_res_ = .05;
    state_map_ = 0;
    
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
    if(initialized_){
        delete private_nh_;
        planner_thread_->join();
        delete planner_thread_;
    }
}


void DStarPlanner::initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros){
    ros::Rate loop_rate(10);

    //This is still useful just for the getRobotPose function
    costmap_ros_ = costmap_ros; //Ignore this param.
    
    char *pcl_topic;
    
    private_nh_->getParam("/move_base/DStarPlanner/goal_tol", goal_tol_);
    private_nh_->getParam("/move_base/local_costmap/pcl_topic", pcl_topic);

    //SECTION is going to be for building the global map.
    global_terrain_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    global_obstacle_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    local_terrain_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    local_obstacle_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);    
    
    has_init_map_ = 0;
    ros::Subscriber init_sub = private_nh_->subscribe<sensor_msgs::PointCloud2>("/rtabmap/octomap_occupied_space", 100, getGlobalCloudCallback, this);
    do{
        ros::spinOnce();
        loop_rate.sleep();
    } while(!has_init_map_);
    //END global map section
    
    
    //This subscriber is going to be running to provide edge cost updates for the D* algorithm
    pcl_sub_ = private_nh_->subscribe<sensor_msgs::PointCloud2>(pcl_topic, 100, updateEdgeCostsCallback, this);
    
    
    planner_failed_ = 0;
    planner_thread_ = new boost::thread(&DStarPlanner::runPlanner, this);
    
    
    initialized_ = 1;
}

bool DStarPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel){
    std::vector<Vector2f> waypoints;
    {
        std::lock_guard<std::mutex> lock(mu_);
        for(unsigned i = 0; i < local_waypoints_.size(); i++){
            waypoints.push_back(local_waypoints_[i]);
        }
    }
    
    /*
     * TODO:
     * Use the waypoints to compute a velocity control. Idk how, not my job boss.
     * Contact Woojin when youre ready for this function.
     */
    
    return true;
}

bool DStarPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan){
    Vector2f prev_wp(plan[0].pose.position.x, plan[0].pose.position.y);
    const float dist = 1;
    
    global_waypoints_.push_back(prev_wp); 
    
    for(unsigned i = 1; i < plan.size(); i++){
        float dx = prev_wp[0] - plan[i].pose.position.x;
        float dy = prev_wp[1] - plan[i].pose.position.y;
    
        if(sqrtf((dx*dx) + (dy*dy)) > dist){
            global_waypoints_.push_back(Vector2f(plan[i].pose.position.x, plan[i].pose.position.y));
            prev_wp[0] = plan[i].pose.position.x;
            prev_wp[1] = plan[i].pose.position.y;
        }
    }
  
    global_waypoints_.push_back(Vector2f(plan[plan.size()-1].pose.position.x, plan[plan.size()-1].pose.position.y));
  
    ROS_INFO("num waypoints %lu", global_waypoints_.size());
    return true;
}

bool DStarPlanner::isGoalReached(){
    Vector2f goal = global_waypoints_[global_waypoints_.size()-1];
    Vector2f current_pose = getCurrentPose();
    
    float dx = goal[0] - current_pose[0];
    float dy = goal[1] - current_pose[1];
    return sqrtf(dx*dx + dy*dy) < goal_tol_;
}

int DStarPlanner::isStateValid(float x, float y){
    return;
}


//TODO: process sampled point cloud
//1. temp_cloud = sample_cloud + local_terrain_cloud_
//2. Run region growing segmentation on temp_cloud
//3. local_terrain_cloud_ = temp_ground_cloud
//4. temp_obstacle_cloud gets used to update the occupancy grid with nearest neighbors approach
//This all needs to happen faster than the lidar can publish.
//Could become problematic if point clouds get massive

//Change of plans. I'm just going to add the pre-segmented point cloud directly into the costmap.
//THis is going to be the fastest way of doing things.
void DStarPlanner::updateEdgeCostsCallback(const sensor_msgs::PointCloud2ConstPtr& msg){
    pcl::PointCloud<pcl::PointXYZ> sample_cloud;
    pcl::fromROSMsg(*msg, sample_cloud);
    
    for(unsigned i = 0; i < sample_cloud.points.size(); i++){
        sample_cloud.points[i].z = 0;
        
    }
    
    
    
    {
        std::lock_guard<std::mutex> lock(update_mu_);
    }
}

//Done: process aggregate point cloud
//This function has one job. Segment the full aggregated point cloud into ground and terrain
//Having these will help segment the for the local grid.
//Computes the global_terrain_cloud_ and global_obstacle_cloud_
void DStarPlanner::getGlobalCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg){
    pcl::PointCloud<pcl::PointXYZ> full_cloud;
    pcl::fromROSMsg(*msg, full_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    *cloudPtr = full_cloud;
    
    float radius;
    float normal_radius;
    float smoothness_threshold;
    float curvature_threshold;
    int num_neighbors;
    
    nh.getParam("/TerrainMap/filter_radius", radius);
    nh.getParam("/TerrainMap/normal_radius", normal_radius);
    nh.getParam("/TerrainMap/curvature_threshold", curvature_threshold);
    nh.getParam("/TerrainMap/smoothness_threshold", smoothness_threshold);
    nh.getParam("/TerrainMap/num_neighbors", num_neighbors);
    nh.getParam("/TerrainMap/num_neighbors_avg", num_neighbors_avg);
    nh.getParam("/TerrainMap/occupancy_threshold", occupancy_threshold_);    
    
    ROS_INFO("Starting normal estimation");
    pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (cloudPtr);
    normal_estimator.setRadiusSearch (normal_radius);
    normal_estimator.compute (*normals);

    ROS_INFO("Done estimating normals, onto region growing");
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize (50);
    reg.setMaxClusterSize (1000000000);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (num_neighbors);
    reg.setInputCloud (cloudPtr);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (smoothness_threshold);
    reg.setCurvatureThreshold (curvature_threshold);
    
    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);
    ROS_INFO("Num clusters %lu", clusters.size());
    
    
    unsigned biggest_cluster = 0;
    unsigned most_points = clusters[0].indices.size();
    unsigned temp_num_points;
    ROS_INFO("Size of cluster %u is %lu", 0, clusters[0].indices.size());
    for(unsigned i = 1; i < clusters.size(); i++){
        temp_num_points = clusters[i].indices.size();
        ROS_INFO("Size of cluster %u is %u", i, temp_num_points);
        if(temp_num_points > most_points){
            most_points = temp_num_points;
            biggest_cluster = i;
        }
    }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointIndices::Ptr ground_indices(new pcl::PointIndices());
    *ground_indices = clusters[biggest_cluster];
    pcl::ExtractIndices<pcl::PointXYZ> extract_ground;
    extract_ground.setInputCloud(cloudPtr);
    extract_ground.setIndices(ground_indices);
    extract_ground.setNegative(false);
    extract_ground.filter(*ground_cloudPtr);
    extract_ground.setNegative(true);
    extract_ground.filter(*global_obstacle_cloud_);
    
    *cloudPtr = *ground_cloudPtr;
    
    pcl::search::KdTree<pcl::PointXYZ>::Ptr mls_tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
    
    mls.setInputCloud(cloudPtr);
    mls.setPolynomialOrder(2);
    mls.setSearchMethod(mls_tree);
    mls.setSearchRadius(radius);
    mls.setComputeNormals(false);
    
    ROS_INFO("about to smooth the point cloud");
    mls.process(pcl_cloud);
    ROS_INFO("point cloud smoothed");
    
    *global_terrain_cloud_ = pcl_cloud;
    
    has_init_map_ = 1;
}

//May need to check how getRobotPose works. It could block while it waits to hear from odom.
//Which would really slow things down.
Vector2f DStarPlanner::getCurrentPose(){
    Vector2f current_pose;
    geometry_msgs::PoseStamped pose;
    costmap_ros_->getRobotPose(pose);
    current_pose[0] = pose.pose.position.x;
    current_pose[1] = pose.pose.position.y;
    return current_pose;
}

//TODO: init the occupancy grid duh.
//1. Use a window filter to set the local terrain and obstacle point clouds
//2. Build an initial occupancy grid the same way you did for the global grid.
void DStarPlanner::initOccupancyGrid(Vector2f start, Vector2f goal){
    pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> out_cloud;
    pcl::PassThrough<pcl::PointXYZ> pass;
    
    pass.setInputCloud(global_terrain_cloud_);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(x_offset_,x_offset_ + x_range_);
    pass.filter(out_cloud);
    *in_cloud = out_cloud;

    pass.setInputCloud(in_cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(y_offset_,y_offset_ + y_range_);
    pass.filter(out_cloud);
    *local_terrain_cloud_ = out_cloud;
    
    
    
    pass.setInputCloud(global_obstacle_cloud_);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(x_offset_,x_offset_ + x_range_);
    pass.filter(out_cloud);
    *in_cloud = out_cloud;
    
    pass.setInputCloud(in_cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(y_offset_,y_offset_ + y_range_);
    pass.filter(out_cloud);
    *local_obstacle_cloud_ = out_cloud;

    for(unsigned i = 0; i < local_obstacle_cloud_->points.size(); i++){
        local_obstacle_cloud_->points[i].z = 0; //flatten obstacle point cloud
    }
    
    
    pcl::search::KdTree<pcl::PointXYZ>::Ptr obs_tree(new pcl::search::KdTree<pcl::PointXYZ>);
    obs_tree->setInputCloud(local_obstacle_cloud_);
    obs_tree->setSortedResults(true);

    unsigned max_neighbors = 16; //num nearest neighbors
    std::vector<int> pointIdxKNNSearch(max_neighbors);
    std::vector<float> pointKNNSquaredDistance(max_neighbors);
    
    pcl::PointXYZ searchPoint;
    int sum;
    int num_neighbors;
    unsigned offset;
    
    ROS_INFO("Begin initializing occupancy grid");
    for(unsigned y = 0; y < height_; y++){
        offset = y*width_;
        for(unsigned x = 0; x < width_; x++){
            sum = 0;
            
            searchPoint.x = (x*map_res_) + x_origin_;
            searchPoint.y = (y*map_res_) + y_origin_;
            searchPoint.z = 0;
            
            num_neighbors = obs_tree->nearestKSearch(searchPoint, max_neighbors, pointIdxKNNSearch, pointKNNSquaredDistance);
            for(unsigned i = 0; i < num_neighbors; i++){
                if(sqrtf(pointKNNSquaredDistance[i]) < (map_res_)){
                    sum++;
                }
            }
            state_map_[offset+x] = (float) sum; //(float)std::min(sum, 4);
        }
    }
    
}

int DStarPlanner::initPlanner(Vector2f start, Vector2f goal){
    open_list_.clear();

    //Following code sets up occupancy grid properties
    //Occupancy grid width width and height are twice the distance from start to goal.
    //x and y_offset are used to make sure start and goal are centered in the graph.
    //x and y_offset are the origin of the occ_grid
    x_range_ = 2*fabs(start[0] - goal[0]);
    x_offset_ = std::min(start[0], goal[0]) - (x_range_*.25);
    
    y_range_ = 2*fabs(start[1] - goal[1]);
    y_offset_ = std::min(start[1], goal[1]) - (y_range_*.25);
    
    width_ = ceilf(x_range_ / map_res_);
    height_ = ceilf(y_range_ / map_res_);
    
    if(state_map_){
        delete[] state_map_;
    }
    state_map_ = new StateData[width_*height_];
    
    
    XClearWindow(dpy, w);
    
    unsigned offset;
    for(unsigned i = 0; i < height_; i++){
        offset = width_*i;
        for(unsigned j = 0; j < width_; j++){
            state_map_[offset+j].tag = NEW;

            //TODO: need to init occupancy grid here.
            /*
            Vector2f test_pos = getRealPosition(i, j);
            if(isStateValid(test_pos[0], test_pos[1])){
                state_map_[offset+j].occupancy = FREE; 
            }
            else{
                state_map_[offset+j].occupancy = OBSTACLE;
            }
            */

            state_map_[offset+j].min_cost = -1;
            state_map_[offset+j].curr_cost = 1;
            state_map_[offset+j].b_ptr = 0;
            state_map_[offset+j].x = i;
            state_map_[offset+j].y = j;

            drawStateTag(&state_map_[offset+j]);
        }
    }
    
    //This is going to get a local obstacle and terrain point clouds.
    //And then build a local occ grid
    initOccupancyGrid(start, goal);
    
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

    do{
        k_min = processState(1);
    } while(!(k_min >= robot_state->curr_cost) && (k_min != -1));
    
    return k_min;
}


//Main Loop thread
void DStarPlanner::runPlanner(){
    Vector2f X_pos;
    StateData* X_state;
    StateData* temp_start;
    StateData* temp_goal;
    
    std::vector<StateData*> actual_path;
    
    ros::AsyncSpinner spinner(1);
    
    X_pos = global_waypoints_[0];
    //idx is the current waypoint, idx+1 is the goal
    for(unsigned idx = 0; idx < (global_waypoints_.size()-1); idx++){
        ROS_INFO("Start State %f %f", X_pos[0], X_pos[1]);
        ROS_INFO("Goal State %f %f", global_waypoints_[idx+1][0], global_waypoints_[idx+1][1]);
        
        if(isStateValid(X_pos[0], X_pos[1])){
            ROS_INFO("Starting sTATE IS invalid.");
        }
        
        if(initPlanner(X_pos, global_waypoints_[idx+1]) == -1){   //This finds a solution. aka sets the backpointers from start to goal
            ROS_INFO("Couldn't find an initial path");
            //pressEnter();
            planner_failed_ = 1;
            return;
        }
        
        temp_start = readStateMap(X_pos);
        temp_goal = readStateMap(global_waypoints_[idx+1]);
        
        drawGoal(temp_start);
        drawGoal(temp_goal);
        
        X_state = temp_start;
        
        actual_path.clear();
        actual_path.push_back(X_state);
        
        //Start asynchronous callback thread. Will listen for new lidar point clouds and it will
        //provide updated edge_costs for D* as the backpointers are followed
        spinner.start();
        
        do{ //This scans for new obstacles, replans, and follows the backptr
          stepPlanner(X_state, X_pos);
          usleep(10000);
          if(!X_state){ //could not find a path to goal.
              planner_failed_ = 1;
              return;
          }

          actual_path.push_back(X_state);
          
          drawRobotPos(X_state);
          XFlush(dpy);
          //pressEnter();
        } while(readStateMap(global_waypoints_[idx+1]) != X_state);
        spinner.stop();
        
        ROS_INFO("Reached waypoint");
        drawFinishedGraph(temp_start, actual_path);
        
        
    }
    
    
    return;
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

    //TODO: check for updates in the costmap here and add the updated edge weights to be explored
    
    std::vector<StateData> updated_nodes; //this threads local copy
    {
        //Critical section for recieving which nodes in the graph are updated
        std::lock_guard<std::mutex> lock(update_mu_);
        for(unsigned i = 0; i < updated_nodes_.size(); i++){
            updated_nodes.push_back(updated_nodes_[i]);
        }
        updated_nodes_.clear();
    }

    ROS_INFO("Num New Obstacles To be Processed: %lu", updated_nodes.size());
    std::vector<StateData*> neighbors;
    
    for(unsigned i = 0; i < updated_nodes.size(); i++){
        char mx = updated_nodes.x;
        char my = updated_nodes.y;
        
        if(updated_nodes[i].occupancy != state_map_[mx][my].occupancy){
            getNeighbors(neighbors, &state_map_[mx][my], 1);
            
            state_map_[mx][my].occupancy = updated_nodes[i].occupancy;
            if(state_map_[mx][my].tag == CLOSED){
                insertState(&state_map_[mx][my], state_map_[mx][my].curr_cost);
            }
        
            for(unsigned k = 0; k < neighbors.size(); k++){
                if(neighbors[k]->tag == CLOSED){
                    insertState(neighbors[k], neighbors[k]->curr_cost);
                } 
            }
        }
    }
    
    
    /*
    std::vector<StateData*> neighbors;
    
    if(terrain_map_->detectObstacles(X_robot[0], X_robot[1])){
        
        for(unsigned i = 0; i < COSTMAP_WIDTH; i++){
            for(unsigned j = 0; j < COSTMAP_HEIGHT; j++){
                Vector2f test_pos = getRealPosition(i, j);
                if((state_map_[i][j].occupancy != OBSTACLE) && !isStateValid(test_pos[0], test_pos[1])){
                    getNeighbors(neighbors, &state_map_[i][j], 1);
                    
                    state_map_[i][j].occupancy = OBSTACLE;
                    if(state_map_[i][j].tag == CLOSED){
                      insertState(&state_map_[i][j], state_map_[i][j].curr_cost);
                    }
                    
                    for(unsigned k = 0; k < neighbors.size(); k++){
                      if(neighbors[k]->tag == CLOSED){
                        insertState(neighbors[k], neighbors[k]->curr_cost);
                      } 
                    }
                }
            }
        }
    }
    */
    
    int k_min = replan(robot_state); //pass current robot position.

    followBackpointer(robot_state);
    X_robot = getRealPosition(robot_state->x, robot_state->y);
    
    return k_min;
}

void DStarPlanner::followBackpointer(StateData*& robot_state){
    unsigned x_actual; //map physical location to local map index
    unsigned y_actual;
    
    unsigned x_desired = robot_state->b_ptr->x;
    unsigned y_desired = robot_state->b_ptr->y;
    
    
    //Critical Section. Sets local waypoints for control system to use as lookahead.
    //Need to guard local_waypoints which is being read in the computeVelocityCommands function/move_base thread.
    {
        std::lock_guard<std::mutex> lock(mu_);
        StateData* temp_state = robot_state;
        
        local_waypoints_.clear();
        for(unsigned i = 0; i < lookahead_len_; i++){
            Vector2f temp_vec = getRealPosition(temp_state->x, temp_state->y);
            local_waypoints_.push_back(temp_vec);
            temp_state = temp_state->b_ptr;
            if(temp_state == NULL){
                break;
            }
        }
    }
    
    do{
        Vector2f current_pose = getCurrentPose();
        getMapIdx(current_pose, x_actual, y_actual);
    } while(x_actual != x_desired || y_actual != y_desired);
    
    robot_state = robot_state->b_ptr;
}

void DStarPlanner::getNeighbors(std::vector<StateData*> &neighbors, StateData* X, int replan){
  neighbors.clear();
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

  /*
  if(X->occupancy == OBSTACLE){
    drawObstacle(&state_map_[X->x][X->y]);
  }
  */
  
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
      //obstacles. I think.
      neighbors.push_back(&state_map_[nx][ny]);
      /*
      if(state_map_[nx][ny].occupancy == OBSTACLE){
        drawObstacle(&state_map_[nx][ny]);
      }
      */
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
    //terrain_map_->detectAllObstacles();

  Vector2f test_pos;
  
  XClearWindow(dpy, w);
  
  XSetForeground(dpy, gc, 0xFF0000);
  for(unsigned i = 0; i < COSTMAP_WIDTH; i++){
    for(unsigned j = 0; j < COSTMAP_HEIGHT; j++){
      test_pos = getRealPosition(i, j);
      
      if(!isStateValid(test_pos[0], test_pos[1])){
        XFillRectangle(dpy, w, gc, i*8, j*8, 8, 8);  
      }
    }
  }
  
  XSetForeground(dpy, gc, 0xFF);
  StateData *state;
  for(unsigned i = 0 ; i < actual_path.size(); i++){
    //pressEnter();
    
    state = actual_path[i];
    XFillRectangle(dpy, w, gc, state->x*8, state->y*8, 8, 8);
    XFlush(dpy);
  }
  
  
}


