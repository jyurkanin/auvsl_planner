#include <iostream>
#include <thread>
#include <vector>
#include <math.h>
#include <stdlib.h>

#include "GlobalParams.h"
#include "JackalDynamicSolver.h"
#include "TerrainMap.h"

#include <std_srvs/Empty.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

#include <ompl/util/RandomNumbers.h>
#include <ompl/base/goals/GoalSpace.h>
#include <ompl/control/SimpleDirectedControlSampler.h>

#include <rbdl/rbdl.h>
#include <tf/tf.h>

#include <tf2_ros/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <string>
#include <fenv.h>







unsigned got_init_pose = 0;
geometry_msgs::Pose initial_pose;

unsigned got_grid_map = 0;
geometry_msgs::Pose origin;
float map_res;
unsigned height;
unsigned width;

SimpleTerrainMap simple_terrain_map;

typedef struct{
  double vl;
  double vr;
  double ts;
} ODOM_LINE;

typedef struct{
  double ts;
  float x;
  float y;
  float vx; //will be approximated by derivative of x
  float vy;
  float yaw;
} GT_LINE;

typedef struct{
  double ts;
  double wx;
  double wy;
  double wz;
} IMU_LINE;


std::vector<ODOM_LINE> odom_vec;
std::vector<GT_LINE> gt_vec;
std::vector<IMU_LINE> imu_vec;

int readOdomFile(std::ifstream &odom_file, ODOM_LINE &odom_line){
  char comma;
  int cmd_mode;    //ignore
  double dist_left;  //ignore
  double dist_right; //ignore
  
  odom_file >> cmd_mode >> comma;
  odom_file >> odom_line.vl >> comma; //velocity left
  odom_file >> dist_left >> comma;
  odom_file >> odom_line.vr >> comma; //velocity right
  odom_file >> dist_right >> comma;
  odom_file >> odom_line.ts; //time (s)
  return odom_file.peek() != EOF;
}

int readGTFile(std::ifstream &gt_file, GT_LINE &gt_line){
  char comma;
  gt_file >> gt_line.ts >> comma;
  gt_file >> gt_line.x >> comma;
  gt_file >> gt_line.y >> comma;
  gt_file >> gt_line.yaw;
  return gt_file.peek() != EOF;
}

int readIMUFile(std::ifstream &imu_file, IMU_LINE &imu_line){
  char comma;
  double ignore;
  imu_file >> ignore >> comma; //ax
  imu_file >> ignore >> comma; //ay
  imu_file >> ignore >> comma; //az
  
  imu_file >> imu_line.wx >> comma;
  imu_file >> imu_line.wy >> comma;
  imu_file >> imu_line.wz >> comma;

  imu_file >> ignore >> comma; //qx
  imu_file >> ignore >> comma; //qy
  imu_file >> ignore >> comma; //qz
  imu_file >> ignore >> comma; //qw

  imu_file >> imu_line.ts;
  
  return imu_file.peek() != EOF;
}

void load_files(const char *odom_fn, const char *imu_fn, const char *gt_fn){
  std::ifstream odom_file(odom_fn);
  std::ifstream gt_file(gt_fn);
  std::ifstream imu_file(imu_fn);
  
  ODOM_LINE odom_line;
  GT_LINE gt_line;
  IMU_LINE imu_line;

  odom_vec.clear();
  gt_vec.clear();
  imu_vec.clear();
  
  //readGTFile(gt_file, gt_line);
  
  while(readOdomFile(odom_file, odom_line)){
    odom_vec.push_back(odom_line);
  }
  
  while(readGTFile(gt_file, gt_line)){
    gt_vec.push_back(gt_line);
  }
  
  while(readIMUFile(imu_file, imu_line)){
    imu_vec.push_back(imu_line);
  }
  
  
  float *vx_list = new float[gt_vec.size()];
  float *vy_list = new float[gt_vec.size()];
  
  float dt;
  for(int i = 0; i < gt_vec.size()-1; i++){
    dt = gt_vec[i+1].ts - gt_vec[i].ts;
    vx_list[i] = (gt_vec[i+1].x - gt_vec[i].x)/dt;
    vy_list[i] = (gt_vec[i+1].y - gt_vec[i].y)/dt;
  }
  
  vx_list[gt_vec.size()-1] = vx_list[gt_vec.size()-2];
  vy_list[gt_vec.size()-1] = vy_list[gt_vec.size()-2];
  
  
  //moving average
  float temp_x;
  float temp_y;
  int cnt;
  for(int i = 0; i < gt_vec.size()-1; i++){
    temp_x = 0;
    temp_y = 0;
    cnt = 0;
    for(int j = std::max(0, i-2); j <= std::min((unsigned)gt_vec.size()-1, (unsigned) i+2); j++){ 
      temp_x += vx_list[j];
      temp_y += vy_list[j];
      cnt++;
    }
    gt_vec[i].vx = temp_x / (float)cnt;
    gt_vec[i].vy = temp_y / (float)cnt;
  }
  
  
  ROS_INFO("gt vec size %lu", gt_vec.size());
  ROS_INFO("odom vec size %lu", odom_vec.size());
  ROS_INFO("imu vec size %lu", odom_vec.size());
  

  odom_file.close();
  gt_file.close();
  imu_file.close();
  
  delete[] vx_list;
  delete[] vy_list;
}



//simulate 6 second intervals just like the paper.
void simulatePeriod(double start_time, float *X_start, float *X_end){
  //find corresponding index in odom_vec. By finding timestamp with
  //minimum difference.
  unsigned start_idx = 0;
  double time_min = 100;
  double diff;
  for(unsigned i = 0; i < odom_vec.size(); i++){
    diff = fabs(odom_vec[i].ts - start_time);
    if(diff < time_min){
      time_min = diff;
      start_idx = i;
    }
  }
  
  
  double dur;
  float Xn[21];
  float Xn1[21];
  for(int i = 0; i < 11; i++){
    Xn[i] = X_start[i]; //assign position and orientation
  }
  for(int i = 11; i < 21; i++){
    Xn[i] = 0; //zero velocities
  }
  
  JackalDynamicSolver solver;
  solver.stabilize_sinkage(Xn, Xn);

  for(int i = 11; i < 21; i++){
    Xn[i] = X_start[i]; //assign velocities
  }
  
  
  for(unsigned idx = start_idx; (odom_vec[idx].ts - start_time) < 6; idx++){
    Xn[17] = std::max(1e-4d, odom_vec[idx].vr);
    Xn[18] = std::max(1e-4d, odom_vec[idx].vr);
    Xn[19] = std::max(1e-4d, odom_vec[idx].vl);
    Xn[20] = std::max(1e-4d, odom_vec[idx].vl);
    
    dur = odom_vec[idx+1].ts - odom_vec[idx].ts;
    solver.solve(Xn, Xn1, dur);

    
    for(int i = 0; i < 21; i++){
      Xn[i] = Xn1[i];
    }
    
    
  }
  
  for(int i = 0; i < 21; i++){
    X_end[i] = Xn[i];
  }
}


void getDisplacement(unsigned start_i, unsigned end_i, float &lin_displacement, float &ang_displacement){
  lin_displacement = 0;
  ang_displacement = 0;
  
  float dx;
  float dy;
  float dyaw;
  
  for(unsigned i = start_i; (i < end_i) && (i < (gt_vec.size()-1)); i++){
    dx = gt_vec[i].x - gt_vec[i+1].x;
    dy = gt_vec[i].y - gt_vec[i+1].y;
    dyaw = gt_vec[i].yaw - gt_vec[i+1].yaw;
    
    lin_displacement += sqrtf((dx*dx) + (dy*dy));
    ang_displacement += fabs(dyaw);
  }
  
  
}


//odometry filename, ground truth filename
void simulateDataset(){
  float Xn[21];
  float Xn1[21];
  for(int i = 0; i < 21; i++){
    Xn[i] = 0;
  }
  
  tf::Quaternion initial_quat;
  tf::Quaternion temp_quat;
  
  float x_err, y_err, yaw_err, lin_displacement, ang_displacement;
  
  double dt;
  double trans_sum = 0;
  double ang_sum = 0;
  double ang_err;
  
  int num_trials_lin = 0;
  int num_trials_ang = 0;
  
  //so it doesnt skip the first one.
  double time = -10;//gt_vec[0].ts;
  //sampled 30x a second. 200 samples is a bit more than 6s to be safe.
  for(unsigned i = 0; i < gt_vec.size()-200; i++){
    if((gt_vec[i].ts - time) > 6){
      time = gt_vec[i].ts;
      
      Xn[0] = gt_vec[i].x;
      Xn[1] = gt_vec[i].y;
      Xn[2] = 0; //.16;
      
      initial_quat.setRPY(0,0,gt_vec[i].yaw);
      initial_quat = initial_quat.normalize();
      
      Xn[3] = initial_quat.getX();
      Xn[4] = initial_quat.getY();
      Xn[5] = initial_quat.getZ();
      Xn[10] = initial_quat.getW();
      
      //get rotational velocity from IMU.
      double diff;
      unsigned imu_idx = 0;
      double time_min = 100;
      for(unsigned i = 0; i < imu_vec.size(); i++){
        diff = fabs(imu_vec[i].ts - time);
        if(diff < time_min){
          time_min = diff;
          imu_idx = i;
        }
      }
      
      Xn[14] = imu_vec[i].wx;
      Xn[15] = imu_vec[i].wy;
      Xn[16] = imu_vec[i].wz;
      
      Xn[11] = gt_vec[i].vx;
      Xn[12] = gt_vec[i].vy;
      Xn[13] = 0;
      
      simulatePeriod(time, Xn, Xn1);
      
      unsigned j;
      for(j = i; (gt_vec[j].ts - time) < 6.0f && j < gt_vec.size(); j++){}
      
      getDisplacement(i, j, lin_displacement, ang_displacement);
      
      x_err = Xn1[0] - gt_vec[j].x;
      y_err = Xn1[1] - gt_vec[j].y;
      
      temp_quat = tf::Quaternion(Xn1[3],Xn1[4],Xn1[5],Xn1[10]);
      tf::Matrix3x3 m(temp_quat);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      
      yaw_err = fabs(yaw - gt_vec[j].yaw);
      yaw_err = yaw_err > M_PI ?  yaw_err - (2*M_PI) : yaw_err;
      
      ROS_INFO("Error lin %f    yaw %f", (x_err*x_err + y_err*y_err), yaw_err);
      ROS_INFO("Displacement lin %f    yaw %f", lin_displacement, ang_displacement);
      
      num_trials_lin++;
      num_trials_ang++;

      float rel_lin_err = sqrtf(x_err*x_err + y_err*y_err) / lin_displacement;
      float rel_ang_err = yaw_err / ang_displacement;
      
      ROS_INFO("Relative lin err %f       ang err %f", rel_lin_err, rel_ang_err);
      
      trans_sum += rel_lin_err*rel_lin_err;
      ang_sum += rel_ang_err*rel_ang_err;
    }
  }
  
  ROS_INFO(" ");
  ROS_INFO(" ");
  ROS_INFO(" ");
  ROS_INFO("MSE Translation Error %f   MSE Heading Error %f", sqrt(trans_sum/num_trials_lin), sqrtf(ang_sum/num_trials_ang));
  
  
  return;
}



float search_bekker(){
  BekkerData best_bekker_params = lookup_soil_table(4);
  simple_terrain_map.test_bekker_data_ = best_bekker_params;
  
  float best_err = 1000000;
  
  double dur;
  float Xn[21];
  float Xn1[21];
  float X_start[21];
  
  
  for(int i = 0; i < 21; i++){
    Xn[i] = 0;
  }
  
  tf::Quaternion initial_quat;
  
  Xn[0] = gt_vec[0].x;
  Xn[1] = gt_vec[0].y;
  Xn[2] = 0;//.16;
  
  initial_quat.setRPY(0,0,gt_vec[0].yaw);
  initial_quat = initial_quat.normalize();
  
  Xn[3] = initial_quat.getX();
  Xn[4] = initial_quat.getY();
  Xn[5] = initial_quat.getZ();
  Xn[10] = initial_quat.getW();

  
  JackalDynamicSolver solver;
  solver.stabilize_sinkage(Xn, X_start);
  
  //for(float phi = .2; phi < .6; phi+=.01){
  for(float n0 = .7421; n0 < .7423; n0 += .0001f){
    
    simple_terrain_map.test_bekker_data_.n0 = n0;
    
    for(int i = 0; i < 21; i++){
      Xn[i] = X_start[i];
    }
  
    for(unsigned idx = 0; idx < 1000; idx++){
      Xn[17] = odom_vec[idx].vr;
      Xn[18] = odom_vec[idx].vr;
      Xn[19] = odom_vec[idx].vl;
      Xn[20] = odom_vec[idx].vl;
    
      dur = odom_vec[idx+1].ts - odom_vec[idx].ts;
      solver.solve(Xn, Xn1, dur);
    
      for(int i = 0; i < 21; i++){
        Xn[i] = Xn1[i];
      }
    }

    float dx = gt_vec[1504].x - Xn[0];
    float dy = gt_vec[1504].y - Xn[1];
    
    ROS_INFO("n0 %f     Err %f", n0, sqrtf(dx*dx + dy*dy));
    if(sqrtf(dx*dx + dy*dy) < best_err){
      ROS_INFO("kc %f, kphi %f, n0 %f, n1 %f, phi %f", best_bekker_params.kc, best_bekker_params.kphi, best_bekker_params.n0, best_bekker_params.n1, best_bekker_params.phi);
      best_err = sqrtf(dx*dx + dy*dy);
      ROS_INFO("Best Err %f", sqrtf(dx*dx + dy*dy));
      best_bekker_params = simple_terrain_map.test_bekker_data_;
    }
  }
  
  ROS_INFO("Best params: kc %f, kphi %f, n0 %f, n1 %f, phi %f", simple_terrain_map.test_bekker_data_.kc, simple_terrain_map.test_bekker_data_.kphi, simple_terrain_map.test_bekker_data_.n0, simple_terrain_map.test_bekker_data_.n1, simple_terrain_map.test_bekker_data_.phi);

  return 0;
}




int main(int argc, char **argv){
  feenableexcept(FE_INVALID | FE_OVERFLOW);
  
  ros::init(argc, argv, "auvsl_global_planner");
  ros::NodeHandle nh;
  
  ROS_INFO("Starting up test_terrain_node\n");
  
  int plot_res;
  nh.getParam("/TerrainMap/plot_res", plot_res); 
  
  GlobalParams::load_params(&nh);
  ompl::RNG::setSeed(GlobalParams::get_seed());
  srand(GlobalParams::get_seed());

  ros::Rate loop_rate(10);
  
  load_files(
"/home/justin/Downloads/LD3/extracted_data/odometry/0001_odom_data.txt",
"/home/justin/Downloads/LD3/extracted_data/imu/0001_imu_data.txt",
"/home/justin/Downloads/LD3/localization_ground_truth/0001_LD_grass_GT.txt"
);
  
  JackalDynamicSolver::set_terrain_map((TerrainMap*) &simple_terrain_map);
  JackalDynamicSolver::init_model(2);
  
  simple_terrain_map.test_bekker_data_ = lookup_soil_table(0);
  
  //for(float n0 = .7; n0 < .8; n0 += .01f){
  //  for(float phi = .38; phi < .45; phi+=.01){
  //  simple_terrain_map.test_bekker_data_.phi = phi;
  simulateDataset();
  //  ROS_INFO("phi:   %f", phi);
  //}
  
  //search_bekker();
  JackalDynamicSolver::del_model();
  
  return 0;
}
