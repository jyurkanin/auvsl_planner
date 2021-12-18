#include <iostream>
#include <thread>
#include <vector>
#include <math.h>
#include <stdlib.h>

#include "GlobalParams.h"
#include "LinearModel.h"
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


LinearModel *g_linear_model;

unsigned got_init_pose = 0;
geometry_msgs::Pose initial_pose;

unsigned got_grid_map = 0;
geometry_msgs::Pose origin;
float map_res;
unsigned height;
unsigned width;


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
  
  g_linear_model->init_state(X_start);  

  float vl, vr;
  for(unsigned idx = start_idx; (odom_vec[idx].ts - start_time) < 6; idx++){
    vr = std::max(1e-4d, odom_vec[idx].vr);
    vl = std::max(1e-4d, odom_vec[idx].vl);
    
    g_linear_model->step(vl, vr);
    g_linear_model->log_xout();
  }
  
  for(int i = 0; i < 3; i++){
    X_end[i] = g_linear_model->vehicle_state[i];
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
    dyaw = fabs(dyaw);
    
    dyaw = dyaw > M_PI ?  dyaw - (2*M_PI) : dyaw;
    
    lin_displacement += sqrtf((dx*dx) + (dy*dy));
    ang_displacement += fabs(dyaw);
  }
  
  
}


//odometry filename, ground truth filename
void simulateDataset(){
  float Xn[3];
  float Xn1[3];
  for(int i = 0; i < 3; i++){
    Xn[i] = 0;
  }
  
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
      Xn[2] = gt_vec[i].yaw;
            
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
            
      simulatePeriod(time, Xn, Xn1);
      
      unsigned j;
      for(j = i; (gt_vec[j].ts - time) < 6.0f && j < gt_vec.size(); j++){}
      
      getDisplacement(i, j, lin_displacement, ang_displacement);
      
      x_err = Xn1[0] - gt_vec[j].x;
      y_err = Xn1[1] - gt_vec[j].y;
            
      yaw_err = fabs(Xn1[2] - gt_vec[j].yaw);
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



int main(int argc, char **argv){
  feenableexcept(FE_INVALID | FE_OVERFLOW);
  ros::init(argc, argv, "auvsl_global_planner");
  ros::NodeHandle nh;  
  GlobalParams::load_params(&nh);


  g_linear_model = new LinearModel();
  
  g_linear_model->init_state(); //set start pos to 0,0,.16 and orientation to 0,0,0,1
  
  load_files(
"/home/justin/Downloads/LD3/extracted_data/odometry/0001_odom_data.txt",
"/home/justin/Downloads/LD3/extracted_data/imu/0001_imu_data.txt",
"/home/justin/Downloads/LD3/localization_ground_truth/0001_LD_grass_GT.txt"
);
  
  simulateDataset();
  
  delete g_linear_model;
  
  return 0;
}
