#include "OctoTerrainMap.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/fast_bilateral.h>


unsigned OctoTerrainMap::has_octomap_ground = 0;
pcl::PointCloud<pcl::PointXYZ> OctoTerrainMap::pcl_cloud;
pcl::KdTreeFLANN<pcl::PointXYZ> OctoTerrainMap::kdtree;



void OctoTerrainMap::get_cloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg){
  ROS_INFO("Point CLoud frame id %s", msg->header.frame_id.c_str());
  pcl::fromROSMsg(*msg, pcl_cloud);
  has_octomap_ground = 1;
}



OctoTerrainMap::OctoTerrainMap(costmap_2d::Costmap2D *occ_grid){
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);
    ros::Subscriber client = nh.subscribe<sensor_msgs::PointCloud2>("/rtabmap/octomap_ground", 100, get_cloud_callback);
    
    ROS_INFO("Listening for octomap_ground");
    
    while(!has_octomap_ground){
      ros::spinOnce();
      loop_rate.sleep();
    }
    client.shutdown();

    ROS_INFO("Got octomap_ground pointcloud");
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    *cloudPtr = pcl_cloud;
    
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> pcl_smooth;

    float radius;
    nh.getParam("/TerrainMap/filter_radius", radius);
    
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
    
    mls.setInputCloud(cloudPtr);
    mls.setPolynomialOrder(2);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(radius);
    mls.setComputeNormals(false);
    
    ROS_INFO("about to smooth the point cloud");
    mls.process(pcl_smooth);
    ROS_INFO("point cloud smoothed");
    
    *cloudPtr = pcl_smooth;
    
    kdtree.setInputCloud(cloudPtr);
    kdtree.setSortedResults(true);
    
    ROS_INFO("Created KDtree");
    
    /*
    ros::Publisher smooth_pub = nh.advertise<sensor_msgs::PointCloud2>("/smooth_ground", 100);
    sensor_msgs::PointCloud2 smooth_pcl_msg;
    unsigned seq = 0;
    while(ros::ok()){
      pcl::toROSMsg(pcl_smooth, smooth_pcl_msg);
      smooth_pcl_msg.header.stamp = ros::Time::now();
      smooth_pcl_msg.header.seq = seq;
      
      smooth_pub.publish(smooth_pcl_msg);
      
      ros::spinOnce();
      loop_rate.sleep();
      seq++;
    }
    */    

    occ_grid_ = occ_grid;
    cols_ = occ_grid_->getSizeInCellsX();
    rows_ = occ_grid_->getSizeInCellsY();
    map_res_ = occ_grid->getResolution();

    x_origin_ = occ_grid_->getOriginX();
    y_origin_ = occ_grid_->getOriginY();

    float *temp_elev_map = new float[rows_*cols_];
    elev_map_ = new float[rows_*cols_];
    
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
    ROS_INFO("Done precomputing elevation grid");
    
    
    ROS_INFO("We are now going to... BLUR THE GRID");
    //gaussian blur of occ_grid to smooth it out and reduce noise from terrain incorrectly labeled as obstacle.
    const unsigned char *occupancy_grid = occ_grid->getCharMap();//new unsigned char[rows_*cols_];
    occ_grid_blur_ = new float[rows_*cols_];
    const int kernel_size = 10;
    
    for(int i = 0; i < rows_; i++){
      for(int j = 0; j < cols_; j++){
        occ_grid_blur_[(i*cols_) + j] = occupancy_grid[(i*cols_) + j];//msg->data[(i*cols_) + j];
        elev_map_[(i*cols_) + j] = temp_elev_map[(i*cols_) + j];
      }
    }
    
    float sigma_sq = 16;
    for(int i = kernel_size; i < rows_-kernel_size; i++){
      for(int j = kernel_size; j < cols_-kernel_size; j++){
        
        float dist_sq;
        float weight;
        float total_weight = 0;
        float occ_sum = 0;
        float elev_sum = 0;
        for(int k = -kernel_size; k <= kernel_size; k++){
          for(int m = -kernel_size; m <= kernel_size; m++){
            dist_sq = (k*k) + (m*m);
            weight = expf(-.5*dist_sq/(sigma_sq));
            occ_sum += weight*occupancy_grid[((i+k)*cols_) + j+m];//msg->data[((i+k)*cols_) + j+m];
            elev_sum += weight*temp_elev_map[((i+k)*cols_) + j+m];
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
    
    delete temp_elev_map;
}


OctoTerrainMap::~OctoTerrainMap(){
  delete elev_map_;
  //delete octomap_;
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
    return lookup_soil_table(0);
}

float OctoTerrainMap::getAltitude(float x, float y, float z_guess) const{
  float col_intrp = ((x - x_origin_) / map_res_);
  float row_intrp = ((y - y_origin_) / map_res_);
  
  
  col_intrp = std::max(std::min(col_intrp, (float)cols_), 0.0f);
  row_intrp = std::max(std::min(row_intrp, (float)rows_), 0.0f);
  
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
  float dx = neighbors[0][0] - neighbors[3][0];
  float dy = neighbors[0][1] - neighbors[3][1];
  float max_dist = sqrtf(dx*dx + dy*dy);
  for(unsigned i = 0; i < 4; i++){
    dx = neighbors[i][0] - col_intrp;
    dy = neighbors[i][1] - row_intrp;
    
    weight = (max_dist - sqrtf(dx*dx + dy*dy)); //hopefully this weighting function works.
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
  
  int K = 5;

  std::vector<int> pointIdxKNNSearch(K);
  std::vector<float> pointKNNSquaredDistance(K);

  float weight;
  float total_weight = 0;
  float sum = 0;
  float dist = 0;
  float dx;
  float dy;
  if(kdtree.nearestKSearch(searchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0){
    for(unsigned i = 0; i < K; i++){
      dx = pcl_cloud[pointIdxKNNSearch[i]].x - searchPoint.x;
      dy = pcl_cloud[pointIdxKNNSearch[i]].y - searchPoint.y;
      
      dist = sqrtf((dx*dx) + (dy*dy));
      
      weight = 1/dist; //sqrtf(pointKNNSquaredDistance[i]);
      total_weight += weight;
      sum += pcl_cloud[pointIdxKNNSearch[i]].z*weight;
    }
  }
  
  return sum/total_weight;
}

int OctoTerrainMap::isStateValid(float x, float y) const{
    //look up in the oc_grid_ to see if thing is occupied
    unsigned mx;
    unsigned my;

    //might not need this conversion.
    if(!occ_grid_->worldToMap(x, y, mx, my)){
      //ROS_INFO("worldToMap OOB");
      return 0; //Out of Bounds
    }
    
    if(occ_grid_blur_[(my*cols_) + mx] > 20){
      //ROS_INFO("Lethal OBstacle Detected");
      return 0; //state is occupied if occupancy > 50%. At least I think thats how it all works.
    }

    //ROS_INFO("Returning true from OctoTerrainMap::isStateValid");
    return 1;//(x > Xmin) && (x < Xmax) && (y > Ymin) && (y < Ymax);
}

std::vector<Rectangle*> OctoTerrainMap::getObstacles() const{
    std::vector<Rectangle*> obstacles; //Lol, idk what I'm gonna do here exactly. I might remove this method from the base class
    return obstacles;
}
