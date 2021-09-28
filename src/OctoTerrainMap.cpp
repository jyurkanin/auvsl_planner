#include "OctoTerrainMap.h"

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

#include <iostream>

unsigned OctoTerrainMap::has_octomap_ground = 0;
pcl::PointCloud<pcl::PointXYZ> OctoTerrainMap::pcl_cloud;
pcl::KdTreeFLANN<pcl::PointXYZ> OctoTerrainMap::kdtree;



void OctoTerrainMap::get_cloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg){
  ROS_INFO("Point CLoud frame id %s", msg->header.frame_id.c_str());
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *temp_cloud);
  
  pcl::PassThrough<pcl::PointXYZ> pass;
  
  pass.setInputCloud(temp_cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(-10,.5);
  pass.filter(pcl_cloud);
  
  /*
  std::ofstream log_file;
  log_file.open("/home/justin/raw_pcl.csv", std::ofstream::out);
  log_file << "x,y,alt\n";
  for(unsigned i = 0; i < pcl_cloud.size(); i++){
      log_file << pcl_cloud.points[i].x << ',' << pcl_cloud.points[i].y << ',' << pcl_cloud.points[i].z << '\n';
  }
  log_file.close();
  */
  
  has_octomap_ground = 1;
}



OctoTerrainMap::OctoTerrainMap(costmap_2d::Costmap2D *occ_grid){
    private_nh_ = new ros::NodeHandle("~/octo_terrain_map");
    ros::Rate loop_rate(10);
    ros::Subscriber client = private_nh_->subscribe<sensor_msgs::PointCloud2>("/rtabmap/cloud_ground", 100, get_cloud_callback);
    
    ROS_INFO("Listening for octomap_ground");
    
    while(!has_octomap_ground){
      ros::spinOnce();
      loop_rate.sleep();
    }
    client.shutdown();
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    *cloudPtr = pcl_cloud;
    ROS_INFO("Got octomap_ground pointcloud");
    
    float radius;
    float normal_radius;
    float smoothness_threshold;
    float curvature_threshold;
    int num_neighbors;
    
    private_nh_->getParam("/TerrainMap/filter_radius", radius);
    private_nh_->getParam("/TerrainMap/normal_radius", normal_radius);
    private_nh_->getParam("/TerrainMap/curvature_threshold", curvature_threshold);
    private_nh_->getParam("/TerrainMap/smoothness_threshold", smoothness_threshold);
    private_nh_->getParam("/TerrainMap/num_neighbors", num_neighbors);
    private_nh_->getParam("/TerrainMap/num_neighbors_avg", num_neighbors_avg);
    private_nh_->getParam("/TerrainMap/occupancy_threshold", occupancy_threshold_);    
    
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
    extract_ground.filter(*obstacle_cloudPtr);
    
    
    cloud_pub1_ = private_nh_->advertise<sensor_msgs::PointCloud2>("ground_cloud", 100);
    cloud_pub2_ = private_nh_->advertise<sensor_msgs::PointCloud2>("raw_cloud", 100);    

    /*
    std::ofstream log_file;
    log_file.open("/home/justin/ground_pcl.csv", std::ofstream::out);
    log_file << "x,y,alt\n";
    for(unsigned i = 0; i < ground_cloudPtr->size(); i++){
        log_file << ground_cloudPtr->points[i].x << ',' << ground_cloudPtr->points[i].y << ',' << ground_cloudPtr->points[i].z << '\n';
    }
    log_file.close();
    */
    
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
    
    *cloudPtr = pcl_cloud;
    
    kdtree.setInputCloud(cloudPtr);
    kdtree.setSortedResults(true);
    
    ROS_INFO("Created KDtree");
        
    occ_grid_ = occ_grid;
    
    //cols_ = occ_grid_->getSizeInCellsX();
    //rows_ = occ_grid_->getSizeInCellsY();
    //map_res_ = occ_grid_->getResolution();

    //x_origin_ = occ_grid_->getOriginX();
    //y_origin_ = occ_grid_->getOriginY();
    
    //Right?
    //x_origin_ = -15;
    //y_origin_ = -15;
    //map_res_ = .1;
    //cols_ = 300;
    //rows_ = 300;
    
    computePclOriginSize();
    
    ROS_INFO("cols %u  rows %u   res %f   x_origin %f   y_origin %f", cols_, rows_, map_res_, x_origin_, y_origin_);
    
    float *temp_elev_map = new float[rows_*cols_];
    elev_map_ = new float[rows_*cols_];
    computeElevationGrid(temp_elev_map);
    ROS_INFO("Done precomputing elevation grid");
    
    
    ROS_INFO("Goind to compute our own occupancy grid");
    //gaussian blur of occ_grid to smooth it out and reduce noise from terrain incorrectly labeled as obstacle.
    float *temp_occ_grid = new float[rows_*cols_];
    computeOccupancyGrid(obstacle_cloudPtr, temp_occ_grid);
    occ_grid_blur_ = new float[rows_*cols_];
    ROS_INFO("We are now going to blur the grid");
    
    
    const int kernel_size = 20;
    
    for(int i = 0; i < rows_; i++){
      for(int j = 0; j < cols_; j++){
          occ_grid_blur_[(i*cols_) + j] = temp_occ_grid[(i*cols_) + j];//msg->data[(i*cols_) + j];
          elev_map_[(i*cols_) + j] = temp_elev_map[(i*cols_) + j];
      }
    }
    
    float sigma_sq = 90;
    for(int i = 0; i < rows_; i++){
      for(int j = 0; j < cols_; j++){
        
        float dist_sq;
        float weight;
        float total_weight = 0;
        float occ_sum = 0;
        float elev_sum = 0;
        
        for(int k = std::max((int)0,(int)(i-kernel_size)); k <= std::min((int)(rows_-1),(int)(i+kernel_size)); k++){
            for(int m = std::max((int)0,int(j-kernel_size)); m <= std::min(int(cols_-1),int(j+kernel_size)); m++){
                float dx = m - j;
                float dy = k - i;
                
                dist_sq = (dx*dx) + (dy*dy);
                weight = expf(-1.0*dist_sq/sigma_sq);
                occ_sum += weight*temp_occ_grid[(k*cols_) + m];//msg->data[((i+k)*cols_) + j+m];
                elev_sum += weight*temp_elev_map[(k*cols_) + m];
                total_weight += weight;
                //ROS_INFO("Dist %f   weight %f", dist_sq, weight);          
            }
        }
        //ROS_INFO("Sum %f    total_weight %f", sum, total_weight);
        
        occ_grid_blur_[(i*cols_) + j] = occ_sum / total_weight;
        elev_map_[(i*cols_) + j] = elev_sum / total_weight;
      }
    }
    
    ROS_INFO("THE GRID IS A BLUR");


    /*
    ros::Publisher smooth_pub = private_nh_->advertise<sensor_msgs::PointCloud2>("/smooth_ground", 100);
    sensor_msgs::PointCloud2 smooth_pcl_msg;
    unsigned seq = 0;
    while(ros::ok()){
      pcl::toROSMsg(pcl_cloud, smooth_pcl_msg);
      smooth_pcl_msg.header.stamp = ros::Time::now();
      smooth_pcl_msg.header.seq = seq;
      smooth_pcl_msg.header.frame_id = "map";
      
      smooth_pub.publish(smooth_pcl_msg);
      
      
      sensor_msgs::PointCloud2 cloud_msg;
      cloud_msg.header.frame_id = "map";
      cloud_msg.header.seq = seq;
      cloud_msg.header.stamp = ros::Time::now();
      
      pcl::toROSMsg(*ground_cloudPtr, cloud_msg);
      cloud_pub1_.publish(cloud_msg);
      
      pcl::toROSMsg(pcl_cloud, cloud_msg);
      cloud_pub2_.publish(cloud_msg);
      

      
      ROS_INFO("Publishing smooth pcl");
      ros::spinOnce();
      loop_rate.sleep();
      seq++;
    }
    */

    
    

    float *inflated_occ_grid = new float[rows_*cols_];
    ROS_INFO("Precomputing robot footprint");
    //Not currently working for some reason. I don't get it.
    //computeInflationGrid(occ_grid_blur_, inflated_occ_grid);
    
    
    /*
    std::ofstream log_file;
    log_file.open("/home/justin/occ_grid.csv", std::ofstream::out);
    log_file << "x,y,inflated,occupancy\n";
    float x;
    float y;
    for(int i = 0; i < rows_; i++){
        for(int j = 0; j < cols_; j++){
            x = (j*map_res_) + x_origin_;
            y = (i*map_res_) + y_origin_;
            //if(occ_grid_blur_[(i*cols_) + j] > 0){
            log_file << x << ',' << y << ',' << inflated_occ_grid[(i*cols_) + j] << ',' << occ_grid_blur_[(i*cols_) + j] << '\n';
            //}
        }
    }
    log_file.close();
    */
    delete temp_elev_map;
    delete inflated_occ_grid;
}


OctoTerrainMap::~OctoTerrainMap(){
  delete private_nh_;
  delete elev_map_;
  //delete octomap_;
}


void OctoTerrainMap::computePclOriginSize(){
  float x_min = pcl_cloud.points[0].x;
  float y_min = pcl_cloud.points[0].y;
  float x_max = pcl_cloud.points[0].x;
  float y_max = pcl_cloud.points[0].y;
  
  
  for(unsigned i = 1; i < pcl_cloud.points.size(); i++){
    if(pcl_cloud.points[i].x < x_min){
      x_min = pcl_cloud.points[i].x;
    }
    if(pcl_cloud.points[i].y < y_min){
      y_min = pcl_cloud.points[i].y;
    }
    if(pcl_cloud.points[i].x > x_max){
      x_max = pcl_cloud.points[i].x;
    }
    if(pcl_cloud.points[i].y > y_max){
      y_max = pcl_cloud.points[i].y;
    }
  }

  map_res_ = .05f;
  
  cols_ = (unsigned) ceilf((x_max - x_min) / map_res_);
  rows_ = (unsigned) ceilf((y_max - y_min) / map_res_);
  
  x_origin_ = x_min;
  y_origin_ = y_min;
  
  x_max_ = x_origin_ + (cols_*map_res_);
  y_max_ = y_origin_ + (rows_*map_res_);
}

//takes uninflated costmap
void OctoTerrainMap::computeInflationGrid(float *costmap, float *inflated_costmap){    
    float robot_radius;
    private_nh_->getParam("/move_base/global_costmap/robot_radius", robot_radius);
    
    int radius_cells = ceilf((float)robot_radius/map_res_);
    unsigned side_len = (radius_cells*2) + 1;
    
    ROS_INFO("Side len %u, radius cells %d", side_len, radius_cells);
    int *robot_cells = new int[side_len*side_len];

    for(int i = -radius_cells; i <= radius_cells; i++){
        for(int j = -radius_cells; j <= radius_cells; j++){
            float x = (j*map_res_) + map_res_*.5;
            float y = (i*map_res_) + map_res_*.5;
            int i_off = i + radius_cells;
            int j_off = j + radius_cells; 
            
            if(sqrtf((x*x) + (y*y)) <= robot_radius){
                robot_cells[(i_off*side_len)+j_off] = 1;
            }
            else{
                robot_cells[(i_off*side_len)+j_off] = 0;
            }
        }
    }
    

    //std::ofstream log_file;
    //log_file.open("/home/justin/temp_grid.csv", std::ofstream::out);
    //log_file << "x,y,cost,k,m,robot\n";
    
    ROS_INFO("Computed footprint");
    for(int i = 0; i < rows_; i++){
        for(int j = 0; j < cols_; j++){
            inflated_costmap[(i*cols_) + j] = 0;
        }
    }
    for(int i = 0; i < rows_; i++){
        for(int j = 0; j < cols_; j++){
            if(costmap[(i*cols_) + j] > occupancy_threshold_){
                for(int k = std::max((int)0,int(i-radius_cells)); k <= std::min((int)(rows_-1),(int)(i+radius_cells)); k++){
                    for(int m = std::max((int)0,int(j-radius_cells)); m <= std::min(int(cols_-1),int(j+radius_cells)); m++){
                        int k_off = k - i + radius_cells;
                        int m_off = m - j + radius_cells;
                        
                        //ROS_INFO("Robot Cell <%d,%d> = %d", k_off, m_off, robot_cells[(k_off*side_len)+m_off]);
                        
                        if(robot_cells[(k_off*side_len)+m_off]){
                            inflated_costmap[(k*cols_) + m] = 1;
                        }
                    }
                }
            }
        }
    }
    
    //log_file.close();
    
    delete robot_cells;
}

void OctoTerrainMap::computeElevationGrid(float *temp_elev_map){
    unsigned offset;
    float elev_guess = 0;
    ROS_INFO("Begin precomputing elevation grid");
    for(unsigned y = 0; y < rows_; y++){
        offset = y*cols_;
        for(unsigned x = 0; x < cols_; x++){
            temp_elev_map[offset + x] = averageNeighbors(x_origin_+(x*map_res_), y_origin_+(y*map_res_), elev_guess);
            elev_guess = temp_elev_map[offset + x];
        }
    }
}

//I really don't like how the costmap_2d is working.
//RTabMap doesn't really give me a choice over how the global map is generated
void OctoTerrainMap::computeOccupancyGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud, float* temp_occ_grid){
    //Step 1. Project it down. to z = 0
    //step 2. Build search tree and search nearest neighbors to build grid.
    
    /*
    std::ofstream log_file;
    log_file.open("/home/justin/raw_occ.csv", std::ofstream::out);
    log_file << "x,y,z\n";
    for(unsigned i = 0; i < obstacle_cloud->size(); i++){
        log_file << obstacle_cloud->points[i].x << ',' << obstacle_cloud->points[i].y << ',' << obstacle_cloud->points[i].z << '\n';
        obstacle_cloud->points[i].z = 0;
    }
    log_file.close();
    */

    
    for(unsigned i = 0; i < obstacle_cloud->points.size(); i++){
        obstacle_cloud->points[i].z = 0;
    }
    
    pcl::search::KdTree<pcl::PointXYZ>::Ptr obs_tree(new pcl::search::KdTree<pcl::PointXYZ>);
    obs_tree->setInputCloud(obstacle_cloud);
    obs_tree->setSortedResults(true);
    
    unsigned max_neighbors = 16; //num nearest neighbors
    std::vector<int> pointIdxKNNSearch(max_neighbors);
    std::vector<float> pointKNNSquaredDistance(max_neighbors);
    
    pcl::PointXYZ searchPoint;
    int sum;
    int num_neighbors;
    unsigned offset;
    ROS_INFO("Begin precomputing occupancy grid");
    for(unsigned y = 0; y < rows_; y++){
        offset = y*cols_;
        for(unsigned x = 0; x < cols_; x++){
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
            temp_occ_grid[offset+x] = (float) sum; //(float)std::min(sum, 4);
        }
    }
}

float OctoTerrainMap::getMapRes(){
    return map_res_;
}


void OctoTerrainMap::getBounds(float &max_x, float &min_x, float &max_y, float &min_y) const{
  max_x = (cols_*map_res_) + x_origin_;
  min_x = x_origin_;

  max_y = (rows_*map_res_) + y_origin_;
  min_y = y_origin_;
}




//overriden methods

BekkerData OctoTerrainMap::getSoilDataAt(float x, float y) const{
  return test_bekker_data_;
  //return lookup_soil_table(0);
}

float OctoTerrainMap::getAltitude(float x, float y, float z_guess) const{
  float col_intrp = ((x - x_origin_) / map_res_);
  float row_intrp = ((y - y_origin_) / map_res_);
  
  int oob = 0; //out of bounds
  if(col_intrp <= 0 || col_intrp > (cols_-1)){
      col_intrp = std::max(std::min(col_intrp, (float)cols_-1), 0.0f);
      oob = 1;
  }
  if(row_intrp <= 0 || row_intrp > (rows_-1)){
      row_intrp = std::max(std::min(row_intrp, (float)rows_-1), 0.0f);
      oob = 1;
  }
  if(oob){
      return elev_map_[(unsigned(row_intrp)*cols_) + unsigned(col_intrp)];
  }
  
  unsigned col_l = floorf(col_intrp);
  unsigned row_l = floorf(row_intrp);
  unsigned col_u = col_l+1;
  unsigned row_u = row_l+1;
  
  float neighbors[4][3];
  neighbors[0][0] = col_l;
  neighbors[0][1] = row_l;
  neighbors[0][2] = elev_map_[(row_l*cols_) + col_l];
  
  neighbors[1][0] = col_l;
  neighbors[1][1] = row_u;
  neighbors[1][2] = elev_map_[(row_u*cols_) + col_l];
  
  neighbors[2][0] = col_u;
  neighbors[2][1] = row_l;
  neighbors[2][2] = elev_map_[(row_l*cols_) + col_u];
  
  neighbors[3][0] = col_u;
  neighbors[3][1] = row_u;
  neighbors[3][2] = elev_map_[(row_u*cols_) + col_u];
  
  
  
  float total_weight = 0;
  float weight;
  float sum = 0;
  float dx = neighbors[0][0] - neighbors[1][0];
  float dy = neighbors[0][1] - neighbors[1][1];
  float dist;
  //float max_dist = sqrtf(dx*dx + dy*dy);
  for(unsigned i = 0; i < 4; i++){
    dx = neighbors[i][0] - col_intrp;
    dy = neighbors[i][1] - row_intrp;
    dist = dx*dx + dy*dy;
    
    weight = expf(-.5*dist/map_res_); //hopefully this weighting function works.
    sum += weight*neighbors[i][2];
    total_weight += weight;
  }
  return sum / total_weight;
}

float OctoTerrainMap::averageNeighbors(float x, float y, float z_guess) const{
  pcl::PointXYZ searchPoint;
  searchPoint.x = x;
  searchPoint.y = y;
  searchPoint.z = z_guess;

  
  int K = num_neighbors_avg;

  std::vector<int> pointIdxKNNSearch(K);
  std::vector<float> pointKNNSquaredDistance(K);

  float weight;
  float total_weight = 0;
  float sum = 0;
  float dist = 0;
  float dx;
  float dy;
  float dz;
  int num_nearest = kdtree.nearestKSearch(searchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance);
  float best_dist = 10;
  if(num_nearest > 0){
    for(unsigned i = 0; i < num_nearest; i++){
      dx = pcl_cloud[pointIdxKNNSearch[i]].x - searchPoint.x;
      dy = pcl_cloud[pointIdxKNNSearch[i]].y - searchPoint.y;
      dz = pcl_cloud[pointIdxKNNSearch[i]].z - searchPoint.z;
      
      dist = sqrtf((dx*dx) + (dy*dy) + (dz*dz)) + 1e-5f; //Prevents divide by zero.

      if(dist < best_dist)
        best_dist = dist;
      
      weight = 1 / dist; //sqrtf(pointKNNSquaredDistance[i]);
      total_weight += weight;
      sum += pcl_cloud[pointIdxKNNSearch[i]].z*weight;
    }
  }
  else{
      ROS_INFO("UHOHUHOHUHOHUHOHUHOHUHOHUHOHUHOHUHOHUHOHUHOHUHOHUHOHUHOHUHOHUHOHUHOHUHOH");
  }
  
  if(best_dist < 1.0f){
    return sum/total_weight;
    
  }
  else{
    return -1;
  }
}

int OctoTerrainMap::isStateValid(float x, float y) const{
    //look up in the oc_grid_ to see if thing is occupied
    unsigned mx;
    unsigned my;
    
    //might not need this conversion.
    /*
    if(!occ_grid_->worldToMap(x, y, mx, my)){
      ROS_INFO("worldToMap OOB");
      return 0; //Out of Bounds
    }
    */
    if(x < x_origin_ || x > x_max_ || y < y_origin_ || y > y_max_){
      ROS_INFO("Out of bounds of elevation map");
      return 0;
    }
    
    
    if(occ_grid_blur_[(my*cols_) + mx] > occupancy_threshold_){
      //ROS_INFO("Lethal Obstacle Detected");
      //return 0; //state is occupied if occupancy > 50%. At least I think thats how it all works.
    }
    
    //ROS_INFO("Returning true from OctoTerrainMap::isStateValid");
    return 1;//(x > Xmin) && (x < Xmax) && (y > Ymin) && (y < Ymax);
}

std::vector<Rectangle*> OctoTerrainMap::getObstacles() const{
    std::vector<Rectangle*> obstacles; //Lol, idk what I'm gonna do here exactly. I might remove this method from the base class
    return obstacles;
}
