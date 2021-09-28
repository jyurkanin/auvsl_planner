#include "JackalDynamicSolver.h"
#include "auvsl_planner_node.h"
#include <unistd.h>
#include <math.h>
#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <stdlib.h>
#include <fstream>


using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;



void break_on_me(){}
    


PIDController::PIDController(float P, float I, float D, float step){
  P_gain = P;
  I_gain = I;
  D_gain = D;

  timestep = step;
  
  reset();
}

PIDController::~PIDController(){}

void PIDController::reset(){
  err_sum = 0;
  prev_err = 0;
}

float PIDController::step(float err){
  float d_err = (err - prev_err)/timestep;
  float tau = (P_gain*err) - (D_gain*d_err);
  prev_err = err;

  return tau;
}




SpatialVector get_body_vel(SpatialVector world_vel, float *X){
    Quaternion quat(X[3], X[4], X[5], X[10]);
    Vector3d r(X[0], X[1], X[2]);
    SpatialTransform X_base(quat.toMatrix(), r);
    return X_base.apply(world_vel); //transform from world to body coordinates.    
}


SpatialVector get_world_force(SpatialVector f_ext_body, float *X){
    Quaternion quat(X[3], X[4], X[5], X[10]);
    Vector3d r(X[0], X[1], X[2]);
    SpatialTransform X_base(quat.toMatrix(), r);
    return X_base.applyTranspose(f_ext_body);  //transform from body to world coordinates.
}




//static stuff


std::ofstream JackalDynamicSolver::log_file;
std::ofstream JackalDynamicSolver::temp_log;


Vector3d JackalDynamicSolver::base_size;
int JackalDynamicSolver::debug_level;
Model* JackalDynamicSolver::model = 0;
float JackalDynamicSolver::stepsize;
float JackalDynamicSolver::tire_radius;

const TerrainMap* JackalDynamicSolver::terrain_map_;

Eigen::Matrix<float,JackalDynamicSolver::num_hidden_nodes,JackalDynamicSolver::num_in_features> JackalDynamicSolver::weight0;
Eigen::Matrix<float,JackalDynamicSolver::num_hidden_nodes,1> JackalDynamicSolver::bias0;
Eigen::Matrix<float,JackalDynamicSolver::num_hidden_nodes,JackalDynamicSolver::num_hidden_nodes> JackalDynamicSolver::weight2;
Eigen::Matrix<float,JackalDynamicSolver::num_hidden_nodes,1> JackalDynamicSolver::bias2;
//Eigen::Matrix<float,JackalDynamicSolver::num_hidden_nodes,JackalDynamicSolver::num_hidden_nodes> weight4;
//Eigen::Matrix<float,JackalDynamicSolver::num_hidden_nodes,1> bias4;
Eigen::Matrix<float,JackalDynamicSolver::num_out_features,JackalDynamicSolver::num_hidden_nodes> JackalDynamicSolver::weight4;
Eigen::Matrix<float,JackalDynamicSolver::num_out_features,1> JackalDynamicSolver::bias4;

Eigen::Matrix<float,JackalDynamicSolver::num_out_features,1> JackalDynamicSolver::out_mean;
Eigen::Matrix<float,JackalDynamicSolver::num_out_features,1> JackalDynamicSolver::out_std;

Eigen::Matrix<float,JackalDynamicSolver::num_in_features,1> JackalDynamicSolver::in_mean;
Eigen::Matrix<float,JackalDynamicSolver::num_in_features,1> JackalDynamicSolver::in_std;


//actual class members

JackalDynamicSolver::JackalDynamicSolver(){
  stepsize = GlobalParams::get_timestep();
  tau = VectorNd::Zero(model->qdot_size);

  
  internal_controller[0] = PIDController(GlobalParams::get_p_gain(),0,0, stepsize);
  internal_controller[1] = PIDController(GlobalParams::get_p_gain(),0,0, stepsize);
  
  
  
  
  for(int i = 0; i < model->mBodies.size(); i++){
    f_ext.push_back(SpatialVector::Zero());
  }
  
  timestep = 0;
}

void JackalDynamicSolver::init_model(int debug){
  debug_level = debug;//;GlobalParams::get_debug_level();
  if(debug_level == 2 && !log_file.is_open()){
      ROS_INFO("DEBUG LEVEL 2");
      log_file.open("/home/justin/xout_file.csv", std::ofstream::out);
      log_file << "qw,qx,qy,qz,x,y,z,wx,wy,wz,vx,vy,vz,q1,q2,q3,q4,qd1,qd2,qd3,qd4\n";
      
      temp_log.open("/home/justin/feature_file.csv");
      temp_log << "zr1,zr2,zr3,zr4\n";
  }
  
  //model = NULL;
  Vector3d initial_pos(0,0,0);
  
  unsigned int base_id;
  unsigned int tire_id[4];
  
  Body tire[4];
  Body floating_base;
  
  Joint tire_joints[4];
  Joint floating_joint;

  if(model){
    return;
  }
  
  model = new Model();
  
  
  model->gravity = Vector3d(0., 0., -9.8); //gravity off. lmao
  
  float base_mass = 13; //kg
  float tire_mass = 1;
  float tire_thickness = .05;
  tire_radius = .1;
  
  base_size = Vector3d(.428, .323, .180);
  Vector3d wheel_offset(base_size[0]*.4, (base_size[1] + tire_thickness)/2, .06);
  Vector3d tire_trans[4];
  
  tire_trans[0] = Vector3d(wheel_offset[0], -wheel_offset[1], -wheel_offset[2]);  
  tire_trans[1] = Vector3d(-wheel_offset[0], -wheel_offset[1], -wheel_offset[2]);
  tire_trans[2] = Vector3d(wheel_offset[0], wheel_offset[1], -wheel_offset[2]);
  tire_trans[3] = Vector3d(-wheel_offset[0], wheel_offset[1], -wheel_offset[2]);
  
  floating_joint = Joint(JointTypeFloatingBase);
  floating_base = Body(base_mass, Vector3d(0,0,0), get_box_inertia(base_mass, base_size));
  base_id = model->AddBody(0, Xtrans(initial_pos), floating_joint, floating_base, "Floating Base");
  
  const char *names[] = {"Front Right", "Back Right","Front Left","Back Left"};
  for(int i = 0; i < 4; i++){
    tire[i] = Body(tire_mass, Vector3d(0,0,0), get_tire_inertia(tire_mass, tire_radius, tire_thickness));
    tire_joints[i] = Joint(JointTypeRevolute, Vector3d(0.,1,0.));
    model->AddBody(base_id, Xtrans(tire_trans[i]), tire_joints[i], tire[i], names[i]);
  }
  
  
  //float ts = .001;
  //solver = new Solver(model, ts, debug);
  load_nn_gc_model();
  
}

void JackalDynamicSolver::set_terrain_map(const TerrainMap *terrain_map){
    terrain_map_ = terrain_map;
}


JackalDynamicSolver::~JackalDynamicSolver(){
}

void JackalDynamicSolver::del_model(){
  //This is probably not a really smart thing to be doing.
  if(model){
    delete model;
    model = 0;
  }
  else{
    return;
  }
  if(debug_level == 2){
    log_file.close();
    temp_log.close();
  }
}


Eigen::Matrix<float,JackalDynamicSolver::JackalDynamicSolver::num_in_features,1> JackalDynamicSolver::scale_input(Eigen::Matrix<float,JackalDynamicSolver::num_in_features,1> features){
    //Do a transform to standard normal.
    Eigen::Matrix<float,JackalDynamicSolver::num_in_features,1> scaled_features;
    for(int i = 0; i < JackalDynamicSolver::num_in_features; i++){
        scaled_features[i] = (features[i] - in_mean[i])/in_std[i];
    }
    return scaled_features;
}

Eigen::Matrix<float,JackalDynamicSolver::JackalDynamicSolver::num_out_features,1> JackalDynamicSolver::scale_output(Eigen::Matrix<float,JackalDynamicSolver::num_out_features,1> labels){
    //Do an inverse transform from standard normal.
    Eigen::Matrix<float,JackalDynamicSolver::num_out_features,1> scaled_labels;
    for(int i = 0; i < JackalDynamicSolver::num_out_features; i++){
        scaled_labels[i] = (labels[i]*out_std[i]) + out_mean[i];
    }
    return scaled_labels;
}


/*
 * This function checks for contact with the soil, and gets the maximum tire sinkage.
 * It will have to discretize the tire into a bunch of points and check all of them
 * For intersection with the soil. It's going to have to return the highest sinkage
 * Found. It's not going to consider multiple contacts with the soil. Thats not
 * What the bekker soil model is good at.
 * cpt_X[i] is the spatial transform from the contact point to the base frame.
 */
void JackalDynamicSolver::get_tire_sinkages_and_cpts(float *X, float *tire_sinkages, SpatialTransform *cpt_X){
    Quaternion quat(X[3], X[4], X[5], X[10]);
    Vector3d r(X[0], X[1], X[2]);
    
    Matrix3d vehicle_rot = quat.toMatrix();
    Matrix3d test_rot;
    
    Vector3d center_of_tire;
    Vector3d radius_vec(0,0,tire_radius);
    Vector3d cpt; //contact_point
    
    double theta_limit = -M_PI*.25;
    int max_checks = 5;
    float test_sinkage;
    
    for(int i = 0; i < 4; i++){
      //3 is the front right tire.
      center_of_tire = r + (vehicle_rot.transpose()*model->GetJointFrame(3+i).r);

      tire_sinkages[i] = -1;
      cpt_X[i].r = Vector3d(0,0,0);
      cpt_X[i].E = Matrix3dIdentity;

      //if(i==0)
      
      for(int j = 0; j < max_checks; j++){
        test_rot = roty(theta_limit - (2*theta_limit*j/((float)(max_checks - 1))));
        
        Matrix3d temp_rot = (vehicle_rot*test_rot).transpose();     //Rot vector from cpt frame to world.
        cpt = center_of_tire - (temp_rot*radius_vec); //Translate vector from cpt frame to world
        //ROS_INFO("Tire Contact Point <%f %f %f>     Sinkage %f", cpt[0], cpt[1], cpt[2],    test_sinkage);
        test_sinkage = terrain_map_->getAltitude(cpt[0], cpt[1], cpt[2]) - cpt[2];
        //if(i==0)
        
        
        if(test_sinkage > tire_sinkages[i]){
          tire_sinkages[i] = test_sinkage;
          cpt_X[i].E = temp_rot.transpose();
          cpt_X[i].r = cpt;
        }
      }
      
    }
    
    //ROS_INFO("best tire cpt  r <%f %f %f>  E*z <%f %f %f>   sinkage %f",  cpt_X[0].r[0], cpt_X[0].r[1], cpt_X[0].r[2],    z_rot[0], z_rot[1], z_rot[2],    tire_sinkages[0]);
}

void JackalDynamicSolver::get_tire_vels(float *X, Vector3d *tire_vels, SpatialTransform *cpt_X){
  Quaternion quat(X[3], X[4], X[5], X[10]);
  Vector3d r(X[0], X[1], X[2]);
  
  SpatialTransform X_base1(Quaternion(0,0,0,1).toMatrix(), r);
  SpatialTransform X_base2(quat.toMatrix(), r);
  
  SpatialVector body_vel_lin(0,0,0,X[11],X[12],X[13]);
  SpatialVector body_vel_ang(X[14],X[15],X[16],0,0,0);
  
  SpatialVector sp_vel_world = X_base2.inverse().apply(body_vel_ang) + X_base1.inverse().apply(body_vel_lin);
  
  for(int i = 0; i < 4; i++){
    SpatialVector sp_vel_cp = cpt_X[i].apply(sp_vel_world); 
    Vector3d lin_vel(sp_vel_cp[3], sp_vel_cp[4], sp_vel_cp[5]);
    //Vector3d ang_vel(sp_vel_cp[0], sp_vel_cp[1], sp_vel_cp[2]);
    
    tire_vels[i] = lin_vel;// + VectorCrossMatrix(ang_vel)*cpt_X[i].r;
  }

  //ROS_INFO("Tire 0 Velocity Vector <%f, %f, %f>", tire_vels[0][0],tire_vels[0][1],tire_vels[0][2]);
}

void JackalDynamicSolver::get_tire_f_ext(float *X){
    SpatialVector tire_wrench;
    
    Quaternion quat(X[3], X[4], X[5], X[10]);
    Vector3d r(X[0], X[1], X[2]);
    
    Eigen::Matrix<float,JackalDynamicSolver::num_hidden_nodes,1> layer0_out;
    Eigen::Matrix<float,JackalDynamicSolver::num_hidden_nodes,1> layer2_out;
    Eigen::Matrix<float,JackalDynamicSolver::num_out_features,1> layer4_out;
    Eigen::Matrix<float,JackalDynamicSolver::num_out_features,1> labels;
    Eigen::Matrix<float,JackalDynamicSolver::num_in_features,1> scaled_features;
    Eigen::Matrix<float,JackalDynamicSolver::num_in_features,1> features;
    
    SpatialTransform cpt_X[4];
    float sinkages[4];
    get_tire_sinkages_and_cpts(X, sinkages, cpt_X);
    
    sinkages_[0] = sinkages[0];
    sinkages_[1] = sinkages[1];
    sinkages_[2] = sinkages[2];
    sinkages_[3] = sinkages[3];
    
    Vector3d tire_vels[4];
    get_tire_vels(X, tire_vels, cpt_X);
    
    SpatialTransform X_tire;
    BekkerData ts_data;
    
    //ROS_INFO("Sinkage %f %f %f %f\n", sinkages[0], sinkages[1], sinkages[2], sinkages[3]);
    
    for(int i = 0; i < 4; i++){    
      //cpt_X[i].r = r + quat.toMatrix().transpose() * model->GetJointFrame(3+i).r;
      //cpt_X[i].E = quat.toMatrix();
      
        features[0] = sinkages[i]; //.0026;        
        if(sinkages[i] <= 0){ //Tire is not in contact with the ground.
            f_ext[3+i] = cpt_X[i].applyTranspose(SpatialVector(0,0,0,0,0,0));
            continue;
        }
        
        if(X[17+i] == 0){
            if(tire_vels[i][0] == 0){
                features(1) = 0; //otherwise would be 1 - 0/0 = 1. Which would be wrong.
            }
            else{
                features(1) = 1 - (tire_vels[i][0]/NUM_HACK);
            }
        }
        else{
            if(tire_vels[i][0] == 0){
                features(1) = 1 - (NUM_HACK/(tire_radius*X[17+i]));
            }
            else{
                features(1) = 1 - (tire_vels[i][0]/(tire_radius*X[17+i]));
            }
        }
	
        if(tire_vels[i][0] == 0){ //prevent dividing by zero
            features(2) = atanf(tire_vels[i][1]/(NUM_HACK+tire_vels[i][0]));
        }
        else{
            features(2) = atanf(tire_vels[i][1]/tire_vels[i][0]);
        }
        
        ts_data = terrain_map_->getSoilDataAt(cpt_X[i].r[0], cpt_X[i].r[1]);
        
        features(3) = ts_data.kc;
        features(4) = ts_data.kphi;
        features(5) = ts_data.n0;
        features(6) = ts_data.n1;
        features(7) = ts_data.phi;
        
        scaled_features = scale_input(features);
        
        layer0_out = (weight0*scaled_features) + bias0;
        layer0_out = layer0_out.unaryExpr(&tanhf);
        layer2_out = (weight2*layer0_out) + bias2;
        layer2_out = layer2_out.unaryExpr(&tanhf);
	//        layer4_out = (weight4*layer2_out) + bias4;
	//        layer4_out = layer4_out.unaryExpr(&tanhf);
        layer4_out = (weight4*layer2_out) + bias4;
        
        labels = scale_output(layer4_out);

        //printf("features %f %f %f", features[0], features[1], features[2]);
        //printf("   labels %f %f %f\n", labels[0], labels[1], labels[2]);
	
        //tire_wrench = SpatialVector(0,labels[3],0,labels[0],labels[1],labels[2]);
        //tire_wrench = SpatialVector(0,0,0,0,0,41.65);
        float Ty = -.9*((tire_radius*X[17+i]) - tire_vels[i][0]);
        tire_wrench = SpatialVector(0,Ty,0,labels[0],labels[1],labels[2]);
        
        //Sign corrections.
        if(X[17+i] > 0){
            tire_wrench[3] = tire_wrench[3]*1;
            //tire_wrench[1] = tire_wrench[1]*1;
        }
        else{
            tire_wrench[3] = tire_wrench[3]*-1;
            //tire_wrench[1] = tire_wrench[1]*-1;
        }
        
        if(tire_vels[i][1] > 0){
            tire_wrench[4] = -fabs(tire_wrench[4]);
        }
        else{
            tire_wrench[4] = fabs(tire_wrench[4]);
        }
        
        if(tire_vels[i][2] > 0){ //prevent bouncing. If tire is moving upwards, no Fz. 
            tire_wrench[5] *= .1;//fmin(tire_wrench[5], 10); //add code to artificially increase other forces when high sinkage is detected
        }
        if(tire_wrench[5] < 0){ //Fz should never point down. But the neural net might not know that.
            tire_wrench[5] = 0;
        }
	
        f_ext[3+i] = cpt_X[i].applyTranspose(tire_wrench);
    }

    
    
    //int forgetme;
    //std::cin >> forgetme;
    //printf("\n");
    //wait_for_x();
}

//This doesn't actually do anything because the PID controllers are actually PD controllers
//and don't have anything to reset.
void JackalDynamicSolver::reset(){
  internal_controller[0].reset();
  internal_controller[1].reset();
}

float JackalDynamicSolver::get_timestep(){
  return timestep*stepsize;
}

void JackalDynamicSolver::log_xout(float *Xout){
  //Log a single row from Xout;
  /*
//qw,qx,qy,qz,x,y,z,wx,wy,wz,vx,vy,vz,q1,q2,q3,q4,qd1,qd2,qd3,qd4
    x_init = [1 0 0 0  0 0 zoff   0 0 0  0 0 0]';
              |______| |_______| |_____| |____|
               |        |        |       |->Linear Velocity in F_1
               |        |        |          Coordinates
               |        |        |
               |        |        |->Angular Velocity in F_1 coordinates
               |        |        
               |        |->Position relative to F_0
               |
               |->Orientation Quaternion

q_init = [0 .4 .8 1.2];
qd_init = [0 0 0 0];
*/
  
  float temp[21];
  temp[0] = Xout[10];
  temp[1] = Xout[3];
  temp[2] = Xout[4];
  temp[3] = Xout[5];
  temp[4] = Xout[0];
  temp[5] = Xout[1];
  temp[6] = Xout[2];
  
  Quaternion quat(Xout[3], Xout[4], Xout[5], Xout[10]);
  Vector3d r(Xout[0], Xout[1], Xout[2]);
  
  SpatialTransform X_base1(Quaternion(0,0,0,1).toMatrix(), r);
  SpatialTransform X_base2(quat.toMatrix(), r);
  
  SpatialVector body_vel_lin(0,0,0,Xout[11],Xout[12],Xout[13]);
  SpatialVector body_vel_ang(Xout[14],Xout[15],Xout[16],0,0,0);
  
  SpatialVector base_vel = X_base2.inverse().apply(body_vel_ang) + X_base1.inverse().apply(body_vel_lin);
  
  temp[7] = base_vel[0]; //Wx
  temp[8] = base_vel[1]; //Wy
  temp[9] = base_vel[2]; //Wz
  temp[10] = base_vel[3]; //Vx
  temp[11] = base_vel[4]; //Vy
  temp[12] = base_vel[5]; //Vz
    

  /*
  temp[17] = Xout[6]; //q1
  temp[18] = Xout[7];
  temp[19] = Xout[8];
  temp[20] = Xout[9];
  
  temp[13] = Xout[17]; //qd1
  temp[14] = Xout[18];
  temp[15] = Xout[19];
  temp[16] = Xout[20];
  */
  
  temp[13] = Xout[6]; //q1
  temp[14] = Xout[7];
  temp[15] = Xout[8];
  temp[16] = Xout[9];

  temp[17] = Xout[17]; //qd1
  temp[18] = Xout[18];
  temp[19] = Xout[19];
  temp[20] = Xout[20];
  
  for(int i = 0; i < 20; i++){
      log_file << temp[i] << ',';
  }
  log_file << temp[20] << "\n";
}


void JackalDynamicSolver::log_features(float *Xout, float vl, float vr){
  static float old_Xout[21] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

  //if(timestep%10 == 0){
    /*
    for(int i = 0; i < 21; i++){
      log_file << Xout[i] << ',';
    }

    log_file << vl << ',';
    log_file << vr << ',';
    */
    
    temp_log << fmax(0,sinkages_[0]) << ',';
    temp_log << fmax(0,sinkages_[1]) << ',';
    temp_log << fmax(0,sinkages_[2]) << ',';
    temp_log << fmax(0,sinkages_[3]) << '\n';
    
    /*
    for(int i = 0; i < 20; i++){
      log_file << Xout[i] - old_Xout[i] << ',';
    }
    log_file << Xout[20] - old_Xout[20] << '\n';

    for(int i = 0; i < 21; i++){
      Xout[i] = old_Xout[i];
    }
    */
    //}
}


void JackalDynamicSolver::simulateRealTrajectory(const char * odom_fn, const char *joint_state_fn, float *X_final){
  //================== Step 1. get starting position and orientation ============================
  std::ifstream odom_file(odom_fn);
  const int num_odom_cols = 90;
  std::string line;
  for(unsigned i = 0; i < 1; i++){
    std::getline(odom_file, line);
  }
  
  std::string word; //skip ahead by 4 words
  for(int i = 0; i < 5; i++){
    std::getline(odom_file, word, ',');
    ROS_INFO("Word yo: %s", word.c_str());
  }
  
  float Xn[21];
  for(int i = 0; i < 21; i++){
    Xn[i] = 0;
  }
  
  char comma;
  
  odom_file >> Xn[0] >> comma;
  odom_file >> Xn[1] >> comma;
  odom_file >> Xn[2] >> comma;
  
  odom_file >> Xn[3] >> comma;
  odom_file >> Xn[4] >> comma;
  odom_file >> Xn[5] >> comma;
  odom_file >> Xn[10] >> comma;
    
  //==================== Step 2. Allow vehicle to come to rest/reach equillibrium sinkage before moving =================
  
  ROS_INFO("Starting z position: %f %f %f", Xn[0], Xn[1], Xn[2]);
  ROS_INFO("Starting orientation %f %f %f %f", Xn[3], Xn[4], Xn[5], Xn[10]);
  float Xn1[21];
  //hack to allow vehicle to settle and reach a stable sinkage before actually starting.
  for(int i = 0; i < 10; i++){
    solve(Xn, Xn1, 0,0, .1f);
    
    for(int j = 11; j < 21; j++){
      Xn[j] = Xn1[j];
    }
    Xn[2] = Xn1[2];
    
    Xn[3] = Xn1[3];
    Xn[4] = Xn1[4];
    Xn[5] = Xn1[5];
    Xn[10] = Xn1[10];
    
  }
  ROS_INFO("Settled z position: %f %f %f", Xn[0], Xn[1], Xn[2]);
  ROS_INFO("Starting orientation %f %f %f %f", Xn[3], Xn[4], Xn[5], Xn[10]);
  
  odom_file.close();

  /*
  for(int i = 0; i < 21; i++){
    Xn[i] = 0;
  }
  */
  
  //Kinematics:
  //Xn[2] = 2.1; //get_yaw(Xn[10], Xn[3], Xn[4], Xn[5]);
  //ROS_INFO("Calculated initial yaw: %f", Xn[2]);
  
  //================== Step 3. Do the simulation bro  ============================
  std::ifstream joint_file(joint_state_fn);
  std::getline(joint_file, line);

  float time;
  float front_left_vel;
  float front_right_vel;
  float back_left_vel;
  float back_right_vel;
  
  while(joint_file.peek() != EOF){
    joint_file >> time >> comma;
    joint_file >> front_left_vel >> comma;
    joint_file >> front_right_vel >> comma;
    joint_file >> back_left_vel >> comma;
    joint_file >> back_right_vel;// >> comma; //ends the line without a comma. its not necessary.
    
    //ROS_INFO("joint file, vel: %f %f %f %f", front_left_vel, front_right_vel, back_left_vel, back_right_vel);
    
    Xn[17] = front_right_vel;
    Xn[18] = back_right_vel;
    Xn[19] = front_left_vel;
    Xn[20] = back_left_vel;
    

    solve(Xn, Xn1, .02f); //(front_left_vel+back_left_vel)*.5, (front_right_vel+back_right_vel)*.5, .1f);
    for(int i = 0; i < 21; i++){
      Xn[i] = Xn1[i];
    }
    
    if(!terrain_map_->isStateValid(Xn[0], Xn[1])){
      ROS_INFO("State is equal to LIBERAL BULLSHIT");
      break;
    }
  }
  
  joint_file.close();
  
  for(unsigned i = 0; i < 21; i++){
    X_final[i] = Xn1[i]; 
  }
  
  return;
}


void JackalDynamicSolver::solve(float *x_init, float *x_end, float vl, float vr, float sim_time){
  int sim_steps = floorf(sim_time/stepsize);
  float Xout[21];
  float Xout_next[21];
  
  reset();
  
  // 0 1 2   3  4  5    6  7  8  9    10   11 12 13   14 15 16   17  18  19  20
  // x,y,z,  qx,qy,qz,  q1,q2,q3,q4,  qw,  vx,vy,vz,  ax,ay,az,  qd1,qd2,qd3,qd4
  for(int i = 0; i < 21; i++){
    Xout[i] = x_init[i];
  }

  unsigned max_steps = sim_steps + timestep;
  for(; timestep < max_steps; timestep++){
    step(Xout, Xout_next, vl, vr);
    
    if(debug_level == 2){
      log_xout(Xout);
      log_features(Xout, vl, vr);
    }
    
    for(int i = 0; i < 21; i++){
      Xout[i] = Xout_next[i];
    } 
  }
  
  for(int i = 0; i < 21; i++){
    x_end[i] = Xout[i];
  }
}

void JackalDynamicSolver::step(float *X_now, float *X_next, float vl, float vr){
  float right_err = vr - (X_now[17] + X_now[18])/2;
  float left_err = vl - (X_now[19] + X_now[20])/2;
  
  //X_now[17] = X_now[18] = vr;
  //X_now[19] = X_now[20] = vl;
  
  tau[6] = tau[7] = std::min(std::max(internal_controller[0].step(right_err), -20.0f), 20.0f);
  tau[8] = tau[9] = std::min(std::max(internal_controller[1].step(left_err), -20.0f), 20.0f);
  
  
  //euler_method(X_now, X_next);
  runge_kutta_method(X_now, X_next); //runge kutta is fucked rn.
}




//solve with constant tire rotational speed
void JackalDynamicSolver::solve(float *x_init, float *x_end, float sim_time){
  int sim_steps = floorf(sim_time/stepsize);
  float Xout[21];
  float Xout_next[21];
  
  reset();
  
  for(int i = 0; i < 21; i++){
    Xout[i] = x_init[i];
  }
  
  unsigned max_steps = sim_steps + timestep;
  for(; timestep < max_steps; timestep++){
    runge_kutta_method(Xout, Xout_next);
    //euler_method(Xout, Xout_next);
    
    if(debug_level == 2){
      log_xout(Xout);
      //log_features(Xout, 0, 0);
    }
    
    for(int i = 0; i < 17; i++){ //dont assign tire velocities, hold them constant
      Xout[i] = Xout_next[i];
    } 
  }
  
  for(int i = 0; i < 21; i++){
    x_end[i] = Xout[i];
  }
}


void JackalDynamicSolver::apply_force(SpatialVector wrench, int body){
  f_ext[body] = wrench;
}

//kinematic
//     0, 1, 2,     3,4,5,  6,7,8,9,  10,11, 12,13, 14,15,16, 17,  18,  19,  20
//X = [x, y, theta, -,-,-,  -,-,-,-,  -, vx, vy,-,  -, -, Wz, qd1, qd2, qd3, qd4  ]
void JackalDynamicSolver::ode_kinematic(float *X, float *Xd){
  for(int i = 0; i < model->q_size; i++){
    Xd[i] = 0;
  }
  for(int i = 0; i < model->qdot_size; i++){
    Xd[i+model->q_size] = 0;
  }

  
  float vr = tire_radius*(X[17]+X[18])*.5;
  float vl = tire_radius*(X[19]+X[20])*.5;
  float vf = (vl+vr)*.5;
  
  Xd[0] = vf*cosf(X[2]);
  Xd[1] = vf*sinf(X[2]);
  Xd[2] = (vr-vl)/.646;
  
  
}

void JackalDynamicSolver::ode(float *X, float *Xd){
  VectorNd Q = VectorNd::Zero(model->q_size);
  VectorNd QDot = VectorNd::Zero(model->qdot_size);
  VectorNd QDDot = VectorNd::Zero(model->qdot_size);

  SpatialTransform temp;
  SpatialVector tire_wrench;
  
  for(int i = 0; i < model->q_size; i++){
    Q[i] = X[i];
  }
  for(int i = 0; i < model->qdot_size; i++){
    QDot[i] = X[i+model->q_size];
  }
  
  get_tire_f_ext(X);
  
  ForwardDynamics(*model, Q, QDot, tau, QDDot, &f_ext);

  //2D simplification. Ignore Fz. Assume constant sinkage.
  //QDDot[2] = 0; // Corresponds to Z-acceleration
  //QDDot[3] = 0; // roll
  //QDDot[4] = 0; // pitch

  //for(int i = 6; i < 10; i++){
  //  printf("%f ", QDDot[i]);
  //}
  //printf("\n");
  
  Quaternion quat(Q[3],Q[4],Q[5],Q[10]);
  Vector3d omega(QDot[3], QDot[4], QDot[5]);
  Eigen::Vector4d qnd = quat.omegaToQDot(omega); //get quaternion derivative.

  //qnd = get_qnd(quat, omega);
  
  //check main.cpp for the order of X and Xd
  for(int i = 0; i < model->qdot_size; i++){
    Xd[i] = QDot[i];
  }
  
  Xd[3] = qnd[0]; //quaternion derivative. Not even going to use a loop for this dum shit
  Xd[4] = qnd[1];
  Xd[5] = qnd[2];
  Xd[10] = qnd[3]; //qw is at the end for literally no reason. thanks rbdl.
  
  for(int i = 0; i < model->qdot_size; i++){
    Xd[i+model->q_size] = QDDot[i];
  }
}

void JackalDynamicSolver::euler_method(float *X, float *Xt1){
  int len = model->q_size + model->qdot_size;
  float ts = stepsize;
  float Xd[len];
  
  ode(X, Xd);
  
  for(int i = 0; i < len; i++){
    Xt1[i] = X[i] + Xd[i]*ts; 
  }
  
  float temp[4] = {Xt1[3], Xt1[4], Xt1[5], Xt1[10]};
  normalize_quat(temp); //this is needed because the quaternion is updated under a small angle assumption using what is essentially a hack for integratin quaternions and it doesnt preserve the quaternion unit norm property
  Xt1[3] = temp[0];
  Xt1[4] = temp[1];
  Xt1[5] = temp[2];
  Xt1[10]= temp[3];
}

void JackalDynamicSolver::runge_kutta_method(float *X, float *Xt1){
  int len = model->q_size + model->qdot_size;
  float ts = stepsize;
  
  //make sure quaternion is normalized after every step or error will be terrible.
  auto normalize_helper = [](float *X_temp){
                            float l2_norm = sqrtf((X_temp[3]*X_temp[3]) + (X_temp[4]*X_temp[4]) + (X_temp[5]*X_temp[5]) + (X_temp[10]*X_temp[10]));
                            X_temp[3] /= l2_norm;
                            X_temp[4] /= l2_norm;
                            X_temp[5] /= l2_norm;
                            X_temp[10] /= l2_norm;
                          };
  
  float temp[len];
  float k1[len];
  ode(X, k1); //Calculate derivative.
  for(int i = 0; i < len; i++) temp[i] = X[i]+.5*ts*k1[i];
  normalize_helper(temp);
  
  float k2[len];
  ode(temp, k2);
  for(int i = 0; i < len; i++) temp[i] = X[i]+.5*ts*k2[i];
  normalize_helper(temp);
  
  float k3[len];
  ode(temp, k3);
  for(int i = 0; i < len; i++) temp[i] = X[i]+ts*k3[i];
  normalize_helper(temp);
  
  float k4[len];
  ode(temp, k4);
  for(int i = 0; i < len; i++){
    Xt1[i] = X[i] + (ts/6)*(k1[i] + 2*k2[i] + 2*k3[i] + k4[i]);
  }
  normalize_helper(Xt1);
}

void JackalDynamicSolver::normalize_quat(float *q){
  float norm = sqrtf((q[0]*q[0]) + (q[1]*q[1]) + (q[2]*q[2]) + (q[3]*q[3]));
  for(int i = 0; i < 4; i++){
    q[i] /= norm;
  }
}



// Auto Generated by pretrain.py
void JackalDynamicSolver::load_nn_gc_model(){
weight0 << -3.0757e-01,  4.5158e-01, -8.6971e-01, -9.4187e-03, -1.4703e-02,
         2.4411e-02,  4.1082e-02, -8.3041e-02,  2.6301e-01, -6.7464e-01,
         6.4491e-03,  1.3034e-01,  2.4910e-01, -6.4585e-01,  2.2633e-01,
        -6.4997e-02,  1.2865e-01,  8.0372e-01,  9.4947e-01,  3.5665e-03,
         8.6763e-03, -3.6471e-02,  3.2657e-04,  4.5787e-02, -2.4430e-01,
        -4.2582e-01, -1.2725e-02, -1.0841e-01, -2.1472e-01,  4.9610e-01,
         2.3274e-01,  2.9815e-02, -1.2027e+00,  3.0974e-01,  1.8968e-02,
        -2.6923e-01, -5.2283e-01,  1.8559e+00, -4.8907e-02,  7.2780e-02,
        -1.1636e-01,  6.3620e-02,  5.1191e+00, -9.1471e-03, -2.0971e-02,
         8.0658e-02,  2.3246e-02, -7.5882e-03, -1.3940e-01,  1.4406e-01,
        -1.2218e-02, -5.2722e-01, -9.7825e-01,  9.0007e-01, -8.9577e-03,
         1.8498e-02, -2.5601e-01, -1.5812e+00,  1.7300e+00, -8.8535e-03,
        -1.3572e-02, -6.4284e-02,  4.0104e-03, -7.9490e-02,  1.6481e-02,
        -3.1891e-01, -7.8703e-01,  3.2874e-02,  5.8721e-02, -2.7756e-01,
         4.0584e-03, -9.0753e-02,  1.0853e-03,  1.1744e-01,  2.1227e-02,
        -9.1870e-02, -1.7518e-01,  5.7496e-01, -1.9094e-03,  1.4005e-02,
        -3.1393e-01, -9.3394e-02,  1.8764e-02,  1.6336e-02,  2.8101e-02,
        -3.0056e-01,  1.0791e-03, -1.9911e-02, -1.4277e-01, -4.1306e-02,
        -6.3734e-03,  5.5870e-03,  1.0692e-02,  1.1983e-02, -7.4318e-04,
        -1.1866e-01, -1.6848e-01,  2.7657e-01, -2.4399e-01, -1.4805e-02,
        -2.4987e-02,  1.7792e-01,  3.3442e-03, -6.9257e-02,  8.4286e-02,
        -3.5089e-01, -6.2389e-01,  6.6779e-04,  1.0996e-03, -1.1581e-01,
        -1.0911e-02,  2.0002e-01, -3.5730e-01,  3.7346e-02, -3.3559e-01,
        -1.9614e-02, -3.6473e-02,  2.7896e-01,  2.3536e-03,  9.7601e-02,
        -1.7027e+00,  6.0427e-02,  3.6799e-02, -2.4301e-02, -3.3986e-02,
         2.4305e-01,  1.3463e-02,  8.1703e-02,  8.6335e-02,  8.1211e-01,
        -5.7705e-02, -3.2795e-02, -5.2858e-02,  1.8470e-01,  4.5316e-02,
         6.8549e-02, -3.0211e-01,  4.3771e-04,  2.9658e-02,  3.8798e-03,
         8.4882e-03, -1.3085e-02,  1.8031e-03,  4.9406e-02,  2.3156e-01,
         2.0355e-01, -3.6413e-01,  2.1225e-02,  4.1485e-02, -2.1657e-01,
        -1.2900e-03, -2.6266e-02, -2.7016e-01, -3.2048e-01,  9.0620e-01,
         5.8255e-03,  1.2003e-02, -6.5937e-02, -4.4583e-03, -1.5083e-01;
bias0 <<  0.2365,  1.7241, -0.5457, -1.7475, -1.5611,  0.1750, -2.3015,  0.5107,
         0.1806, -0.2466, -1.8114,  0.4330, -0.4945,  1.0216,  0.5284, -2.6947,
         0.6773,  0.1596, -0.7357,  0.5587;
weight2 <<  6.3841e-01,  1.0913e-01,  8.8629e-01,  5.6885e-02,  3.2797e-01,
        -9.6149e-02, -9.5561e-02, -2.2513e-01, -1.3071e-01,  1.1713e-01,
         5.3334e-01,  2.7876e-01, -5.0397e-01,  1.4510e-01,  1.4175e-01,
        -1.0400e-01, -5.8599e-01,  4.3520e-01, -2.3931e-01, -4.0993e-02,
         5.3444e-01,  1.1707e+00,  8.4216e-02, -1.5766e+00,  1.2735e+00,
        -4.4264e-01,  1.1395e+00, -1.7643e-01,  1.1465e-01,  5.1482e-01,
        -1.3857e-01,  8.1581e-01, -1.5213e-01, -5.8023e-01,  4.5505e-01,
         1.0688e+00, -5.3727e-01, -2.4968e-01,  9.6358e-02,  9.6716e-02,
         6.9882e-01,  1.4896e-01,  3.0196e-01, -8.7884e-02, -2.5827e-01,
        -5.0188e-01,  1.5196e-01, -2.2371e-01, -2.6923e-01,  8.7836e-01,
        -9.3726e-01,  1.1642e+00, -4.3215e-01,  3.9516e-02,  7.0848e-02,
         7.2571e-01, -5.3598e-01,  6.2107e-01,  1.3951e+00, -1.7713e-01,
         4.9164e-04,  2.6033e-01,  9.1808e-02, -3.1758e-01,  2.2699e-01,
         6.0231e-02,  2.5400e-01, -1.8620e-02,  1.4248e-01,  2.2707e-01,
        -1.7587e+00, -7.3367e-01,  2.9391e-01, -9.0699e-02, -1.2770e-01,
         2.3392e-01, -3.5882e-01, -7.5737e-01,  4.9512e-02,  3.6311e-02,
        -4.4186e-01, -2.5864e-01,  1.1929e-01,  2.3552e-01, -6.8349e-01,
        -4.7598e-01, -1.2326e-01,  1.7547e-01, -1.2610e+00, -3.2279e-01,
         1.1724e+00,  3.8397e-01, -1.0983e+00,  1.8815e-02,  1.1902e+00,
        -9.7368e-01,  6.7346e-01, -4.5400e-01,  8.3197e-01, -7.5918e-01,
         3.1066e-01,  4.2173e-01,  1.4428e+00, -3.0529e-01, -1.3797e-01,
         2.3798e-01,  2.0153e-01, -4.7514e-01, -9.0637e-01,  1.0092e+00,
        -5.4790e-01,  1.7044e+00, -7.0156e-01, -4.0925e-01, -1.2339e+00,
         9.7696e-01, -7.5380e-01,  1.1807e+00, -1.0154e-01, -3.4981e-01,
        -5.5482e-01, -7.9721e-01, -7.0261e-01,  8.8843e-01, -4.5356e-01,
         9.0197e-01, -5.7364e-01, -2.3764e-02, -3.3345e-01, -5.3837e-01,
         5.1479e-01, -7.7475e-01, -9.4408e-02, -5.6572e-02, -5.0043e-01,
        -8.3901e-01,  5.9377e-01,  9.5993e-01, -3.5356e-01, -2.7602e-01,
         1.0550e+00,  5.8723e-01, -2.2779e-01, -7.9331e-01,  3.0435e-01,
        -8.0489e-01,  4.6054e-01, -5.9280e-01, -9.5758e-01,  1.0040e+00,
         2.1531e-01,  7.3787e-01, -1.5650e+00, -5.5854e-01,  8.9461e-01,
         1.0102e+00, -6.0759e-01, -7.1512e-02,  1.3246e+00, -2.5983e-01,
        -2.5970e-01, -3.0016e-01, -1.0718e+00,  8.1976e-01, -5.4447e-01,
        -3.7831e-01, -5.4951e-01,  1.0760e-01, -8.2371e-03, -4.0568e-01,
        -1.8806e-01, -3.9994e-01, -4.4272e-01, -7.9957e-01, -3.7531e-01,
        -4.9762e-01,  2.0456e-01, -6.8291e-01,  4.7991e-01, -1.1082e-01,
        -1.1519e-01,  5.7396e-01,  1.7426e+00, -4.7058e-01, -5.7818e-02,
         7.8449e-01,  2.5659e-01, -7.8117e-01, -9.5348e-01,  1.1451e+00,
        -1.2526e+00,  3.1607e+00, -1.9496e+00, -4.0261e-01, -1.4683e+00,
         1.0433e+00, -7.2029e-01,  1.5367e+00, -9.3137e-01, -9.0326e-01,
        -2.6219e-01,  1.1565e+00,  7.9025e-01, -1.3614e+00,  1.2234e+00,
         6.3717e-01,  1.0774e+00, -2.0316e-01, -6.4238e-02,  9.6123e-01,
         2.9007e-02,  1.7216e+00,  3.8757e-01,  6.8946e-01,  2.9859e-01,
         1.1079e+00, -4.2236e-01,  3.8743e-01,  3.4634e-01,  8.7941e-02,
         3.0811e-01, -1.0556e+00, -4.8165e-01,  1.0292e+00, -6.5805e-01,
        -6.8760e-01, -7.5331e-01,  2.5880e-01, -1.3755e-01, -1.2938e+00,
         8.3125e-01,  1.2847e+00, -1.1660e+00,  9.4490e-02, -5.1085e-01,
        -8.1267e-01,  8.1055e-01, -2.3783e-01,  5.0223e-01, -2.2424e-02,
         2.2988e-01, -1.2762e+00, -5.0109e-01,  1.2978e+00, -1.5649e+00,
        -6.4074e-01, -1.0377e+00,  2.6521e-01,  4.3253e-01,  4.7807e-01,
         1.2646e-01, -6.3078e-01, -1.0196e+00, -9.0059e-01, -8.8785e-02,
        -1.1611e+00,  5.1011e-01,  1.0618e+00,  1.2482e-01, -1.0296e-01,
         3.3332e-01,  6.2145e-01, -4.4998e-03, -8.7578e-01,  1.3705e-01,
        -5.9887e-02,  9.5999e-01, -1.4830e-01,  3.1013e-01,  2.1291e+00,
        -2.0053e-01,  2.2676e-01, -1.2066e-01, -2.4791e-01, -1.3112e-01,
         1.0269e+00, -5.6121e-01,  1.5415e+00,  8.0892e-01,  5.0821e-01,
        -6.3694e-01, -3.0000e-01, -1.0056e+00, -3.6133e-02, -3.4362e-01,
         9.2529e-02,  5.3726e-02,  3.1067e-01,  1.9102e-02, -3.4483e-01,
         1.4069e+00,  6.4720e-01,  4.1809e-01, -4.9726e-02, -1.8258e-01,
         1.0818e-01,  7.5863e-01, -8.6315e-01, -9.3980e-01, -2.8580e-01,
         1.8502e-02, -6.3593e-01, -1.4531e-01,  7.3905e-01, -1.8928e-01,
        -5.8085e-02, -5.4844e-01, -1.1270e-02, -2.0540e-01, -1.4852e+00,
        -5.0052e-01,  1.6684e-01, -9.5706e-01,  1.7594e-01,  1.5994e-02,
        -7.1521e-01,  5.4138e-01, -1.5469e+00,  1.7135e-01, -4.6655e-03,
        -4.5301e-01, -3.5370e-01, -1.2060e-02,  3.8223e-01, -1.2700e-01,
        -2.9121e-02, -2.9646e-01, -4.0691e-03,  5.8960e-01, -1.0624e+00,
         2.9538e-01, -8.5763e-01, -3.1151e-01, -2.0163e-01,  8.5392e-01,
        -8.4575e-01,  5.6019e-01, -9.9658e-01, -1.1399e+00, -1.5484e-01,
        -2.4357e-01, -1.5341e+00,  5.5112e-02,  1.8636e+00, -1.6528e+00,
         2.3577e-01, -1.5884e+00,  1.6281e-01, -1.5038e-01, -1.8446e-01,
         8.5134e-01,  7.1016e-01, -3.3512e-01,  8.7143e-01, -6.4924e-01,
        -1.0678e+00,  6.7112e-01,  5.3996e-02, -1.3424e-02, -9.2855e-02,
         2.4030e-01, -1.5934e+00, -8.4078e-01,  1.8547e+00, -1.7199e+00,
        -3.4308e-01, -1.5988e+00, -2.0446e-02, -2.4151e-01, -4.8027e-01,
         6.2702e-01, -2.1684e-01, -1.2652e+00, -3.8348e-01, -2.6925e-01,
        -1.0955e+00,  5.7520e-01, -3.2857e-01,  1.3948e-01, -6.0062e-02,
        -8.0515e-01, -7.1633e-01, -9.6545e-01,  6.1169e-01, -5.4998e-01,
         1.1438e-01, -7.1973e-01,  3.4134e-01,  2.3326e-01, -4.0983e-01,
         7.7852e-01, -1.3596e+00, -6.9100e-01, -2.7822e-01, -4.7841e-01,
        -8.5512e-01,  1.1862e+00,  9.7414e-01, -1.2394e+00,  1.3948e-01;
bias2 <<  0.6844,  0.4367,  1.2892,  0.3388, -2.0143,  2.0785, -1.2678,  1.8999,
        -0.6035,  1.0766,  0.2994, -0.6204, -1.0561,  1.0236, -0.3963, -0.6886,
        -0.6433, -0.9638, -0.8384, -2.3682;
weight4 <<  1.8663,  0.2278,  0.8404,  1.1222,  2.2349,  3.0787,  1.9501,  1.3459,
        -1.2015, -1.9207, -0.7448,  0.0819, -1.4413, -1.8322,  2.6974, -1.5895,
         1.6025,  0.2821,  0.9728, -2.3731, -0.1802,  2.1299,  1.1524, -0.2279,
        -0.3123, -0.2980,  1.1065, -0.4806, -0.3937,  0.0579, -2.7544, -0.8982,
        -1.3661,  0.2456, -0.0526,  0.1802,  0.2202,  2.6727, -1.9103,  0.3105,
         0.1030,  0.0885, -0.0791,  1.5541,  0.0292, -0.0296, -0.0827,  0.0146,
        -0.0482, -0.0250,  0.1108, -0.0401,  0.0103, -0.1111,  0.0167,  1.1322,
         0.0578,  0.2452,  0.3156,  0.0421;
 bias4 << -0.2535, -0.1371, -1.4202;
 out_mean << -24.245327789699125, -4.972238784710126, 858.4484288696027;
 out_std << 106.60507509883605, 342.38221052518674, 505.27382432744236;
 in_mean << 0.04755175580815579, -0.1031091248791658, -0.00822193571007142, 59.96615465535339, 1989.5623071836276, 0.8023074986664168, 0.0998446902747603, 0.34080175360478987;
 in_std << 0.028762000257431114, 0.5305530689625024, 0.9490676466883241, 23.21776169763347, 869.4323970393215, 0.29001461209388496, 0.05770084150436896, 0.10050279338464481;
}

