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
std::ofstream JackalDynamicSolver::feature_log;
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
      log_file << "qw,qx,qy,qz,x,y,z,wx,wy,wz,vx,vy,vz,q1,q2,q3,q4,qd1,qd2,qd3,qdd4\n";
  }
  

  feature_log.open("/home/justin/feature_file.csv", std::ofstream::out);
  feature_log << "x,y,z,qx,qy,qz,q1,q2,q3,q4,qw,vx,vy,vz,ax,ay,az,qd1,qd2,qd3,qd4, vl,vr,zr1,zr2,zr3,zr4,   dx,dy,dz,dqx,dqy,dqz,dq1,dq2,dq3,dq4,dqw,dvx,dvy,dvz,dax,day,daz,dqd1,dqd2,dqd3,dqd4\n";
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
  }

  feature_log.close();
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


void JackalDynamicSolver::log_features(float *Xout, float *Xd){
  if(timestep%10 == 0){
    for(int i = 0; i < 21; i++){
      feature_log << Xout[i] << ',';
    }

    feature_log << vel_left_ << ',';
    feature_log << vel_right_ << ',';

    feature_log << fmax(0,sinkages_[0]) << ',';
    feature_log << fmax(0,sinkages_[1]) << ',';
    feature_log << fmax(0,sinkages_[2]) << ',';
    feature_log << fmax(0,sinkages_[3]) << ',';
    
    for(int i = 0; i < 20; i++){
      feature_log << Xd[i] << ',';
    }
    feature_log << Xd[20] << '\n';
  }
}


void JackalDynamicSolver::simulateRealTrajectory(const char * odom_fn, const char *joint_state_fn){
  //================== Step 1. get starting position and orientation ============================
  std::ifstream odom_file(odom_fn);
  const int num_odom_cols = 90;
  std::string line;
  for(unsigned i = 0; i < 1865; i++){
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
  
  ROS_INFO("Starting z position: %f", Xn[2]);
  ROS_INFO("Starting orientation %f %f %f %f", Xn[3], Xn[4], Xn[5], Xn[10]);
  float Xn1[21];
  //hack to allow vehicle to settle and reach a stable sinkage before actually starting.
  for(int i = 0; i < 10; i++){
    solve(Xn, Xn1, 0,0, .1f);
    
    for(int j = 11; j < 21; j++){
      Xn[j] = Xn1[j];
    }
    Xn[2] = Xn1[2];
    
  }
  ROS_INFO("Settled z position: %f", Xn[2]);
  ROS_INFO("Starting orientation %f %f %f %f", Xn[3], Xn[4], Xn[5], Xn[10]);
  
  odom_file.close();
  
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
    joint_file >> back_right_vel >> comma;

    
    Xn[17] = front_right_vel;
    Xn[18] = back_right_vel;
    Xn[19] = front_left_vel;
    Xn[20] = back_left_vel;
        

    solve(Xn, Xn1, .1f); //(front_left_vel+back_left_vel)*.5, (front_right_vel+back_right_vel)*.5, .1f);
    for(int i = 0; i < 21; i++){
      Xn[i] = Xn1[i];
    }
    
    if(!terrain_map_->isStateValid(Xn[0], Xn[1])){
      break;
    }
  }
  
  joint_file.close();
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

  vel_left_ = vl;
  vel_right_ = vr;
  
  tau[6] = tau[7] = std::min(std::max(internal_controller[0].step(right_err), -20.0f), 20.0f);
  tau[8] = tau[9] = std::min(std::max(internal_controller[1].step(left_err), -20.0f), 20.0f);
  
  euler_method(X_now, X_next);
  //runge_kutta_method(X_now, X_next); //runge kutta is fucked rn.
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
    //runge_kutta_method(Xout, Xout_next);
    euler_method(Xout, Xout_next);
    
    if(debug_level == 2){
      log_xout(Xout);
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
  log_features(X, Xd);
  
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




//#include "solver.h"
//Auto Generated by pretrain.py

void Solver::load_nn_gc_model(){
weight0 << -7.0340e-02, -1.3777e+00,  9.2441e-03, -1.7163e-03,  2.0788e-02,
        -6.5836e-02, -5.4273e-02, -2.2705e-02,  4.6693e-02, -2.0944e-01,
        -7.4550e-01, -1.9709e-02, -3.8534e-02,  1.8317e-01, -3.0310e-02,
         1.8648e-01,  2.7617e-01,  2.9756e-01,  2.7833e-01,  2.4891e-02,
         5.6775e-02, -5.1937e-01, -1.0698e-01, -4.4940e-02, -1.4342e-01,
        -7.1330e-01,  6.5361e-01,  3.8525e-02,  7.3327e-02, -3.5951e-01,
         3.1534e-02, -1.1789e-01, -1.8221e+00, -3.9410e-02,  2.3617e-02,
         1.5971e-02,  3.2743e-02, -9.4661e-02, -2.6995e-03,  1.1001e-01,
        -4.3236e-01,  7.7764e-02, -1.0650e-01, -8.3629e-03, -1.1048e-02,
        -3.1443e-02,  1.4808e-02,  1.1603e-01, -2.9052e-01,  6.6207e-01,
        -2.4430e-01, -6.1835e-02, -1.0357e-01,  3.3118e-01,  5.2785e-02,
         1.4259e-01,  7.5585e-02,  6.1426e-01, -1.6899e-02, -6.5739e-02,
        -1.3602e-01,  1.8971e-01, -2.0040e-01,  2.7124e-02,  2.4390e-01,
         7.2640e-01,  2.6235e-01, -9.3236e-02, -1.6883e-01,  7.4714e-01,
         4.7909e-02, -3.8195e-02, -1.5118e-01, -9.0689e-01,  1.0439e+00,
        -2.0902e-02, -4.2463e-02,  1.8473e-01, -1.3392e-02, -1.0732e-01,
        -1.8233e-01,  2.0588e-01, -1.0471e-01, -2.4849e-02, -6.4332e-02,
         2.2000e-01, -4.6985e-02, -3.1092e-01,  2.1942e-02, -3.8950e-02,
         8.8361e-01,  2.7245e-02,  5.6515e-02, -1.4182e-01,  2.2345e-03,
        -1.9046e-01, -5.1976e-02, -6.9437e-01, -3.3204e-02, -4.7367e-02,
        -8.9564e-02,  1.5075e-01, -4.4362e-01,  2.1843e-02,  3.6864e-01,
        -2.5539e-01,  1.0064e+00,  1.0120e-02,  2.9172e-02, -6.9503e-02,
         4.2828e-03,  6.1831e-02, -9.6779e-01,  2.3174e-01,  1.3941e-02,
        -3.2607e-01, -6.1971e-01,  1.5685e+00, -1.5550e-02,  7.7931e-03,
         3.8460e-01, -1.9626e-01,  7.4334e-01,  2.9892e-02,  5.7317e-02,
        -1.5233e-01, -2.1422e-03,  1.5339e-01,  2.8305e-01, -3.2595e-01,
        -1.1001e-01,  2.2723e-01,  4.2590e-01, -1.1131e+00,  7.2330e-03,
         4.9166e-02, -9.2643e-02,  3.9087e-01,  2.2222e-02,  5.1695e-02,
         9.6915e-02,  2.1895e-01, -4.3362e-03,  5.2372e-02, -7.5266e-02,
        -7.9956e-02,  4.5829e-01, -2.2301e-02, -4.8310e-02,  3.2761e-01,
         5.1552e-04,  2.3895e-01,  2.4211e-01,  7.4231e-01, -1.8944e+00,
        -7.3728e-03, -2.7242e-03,  8.6473e-03,  3.3436e-02,  1.7271e-01,
        -8.7093e-02, -7.3527e-01, -2.0603e+00, -8.8246e-03, -7.4441e-04,
         8.0686e-02,  9.1644e-03, -1.1765e-01, -2.9140e-01, -3.7120e-01,
         1.1184e-01, -9.5647e-02, -1.5332e-01,  5.8286e-01,  3.4750e-01,
        -3.1053e-02,  1.1354e+00, -6.7248e-02, -2.0055e-02, -5.0945e-02,
        -8.8254e-02, -8.1927e-01,  1.6615e-02, -2.5247e-02,  3.3555e-01,
         1.8358e-01, -8.5899e-02,  1.9126e-02,  3.4695e-02, -9.9606e-02,
        -7.3102e-03,  1.8530e-01, -3.6791e-01, -3.6774e-02, -5.0938e-02,
         5.3178e-02,  9.8901e-02, -4.5716e-01,  1.4086e-02, -9.5289e-02,
         5.6299e-02,  5.7048e-01, -2.3632e-02,  4.7377e-02,  9.4888e-02,
        -1.7535e-01, -1.3386e-01,  3.8088e-02,  4.6375e-01, -2.1648e-01,
        -4.7512e-01,  9.9960e-02,  1.8881e-01, -4.7040e-01, -1.3445e-02,
         6.0112e-02,  8.2626e-02,  1.5673e-01,  2.9168e-02, -5.5691e-01,
        -1.0639e+00,  4.8495e-01, -1.6344e-02,  2.3664e-02, -2.8039e-01,
        -2.0418e-01,  7.2995e-01, -4.1323e-03, -1.1207e-02,  6.5584e-02,
         1.0729e-02,  2.9808e-01, -8.0489e-02,  7.8434e-02, -6.1693e+00,
        -7.7285e-03,  1.6149e-03,  3.5902e-02, -3.8272e-03,  1.5479e-02,
         6.4393e-01,  3.2122e-01,  8.0331e-02,  3.6701e-02,  6.2860e-02,
        -2.8004e-01, -1.1950e-02,  1.8044e-02, -7.0171e-02, -1.1324e+00,
        -1.2108e+00,  7.9270e-03,  1.5751e-02, -1.1707e-01,  2.4348e-02,
        -8.0677e-02, -2.4695e-01,  4.3320e-01,  9.8883e-03, -9.7938e-02,
        -1.8934e-01,  7.0550e-01, -1.4812e-01, -4.8580e-02,  1.2918e-01,
        -1.1589e+00, -1.8172e+00,  1.8160e-02,  4.6420e-02, -1.3362e-01,
         1.2045e-02,  8.6222e-02, -9.6760e-02, -1.6702e+00,  2.1706e+00,
         2.6485e-02,  5.2936e-02, -1.5183e-01,  3.3976e-04, -1.9722e-03;
bias0 <<  2.4444,  0.3685, -0.7155,  0.8065, -3.2940,  0.4666, -0.5354, -2.0618,
         0.0760,  0.7083,  2.2906,  1.7222, -1.2814, -0.9732, -1.3169, -0.0287,
         0.5439, -0.1801,  0.6052,  0.4868, -1.8635,  0.6370,  1.6670, -0.7754,
        -0.6588,  1.9269,  0.7599, -2.4936, -0.8000, -0.0328,  0.3675,  0.6599,
         0.6075, -0.7946, -0.2237;
weight2 << 0.1645, 0.2208, 0.2368,  ..., 1.3988, 0.1229, 0.0562;
bias2 << -0.3107,  0.1317, -0.9490,  1.1658, -1.4644,  1.4196,  1.0762, -0.7573,
        -0.2004,  0.3568,  1.0685, -0.2663,  0.3834,  1.3697, -0.5341,  0.6816,
        -0.1637, -0.2753, -0.3535,  0.1152, -0.1880, -0.3566, -0.1795, -0.2791,
        -0.0446,  1.4193, -0.5676,  0.0403, -1.3635, -0.8971, -0.5339, -0.7329,
         0.8001, -1.4294,  0.3646;
weight4 << -1.5888e-01, -8.5999e-02,  6.4393e-01, -1.0959e+00, -4.3857e-01,
        -3.6698e-03,  1.3612e-01, -7.6085e-01,  7.3661e-01,  4.9741e-01,
        -1.0724e+00, -3.7233e-01,  8.8011e-01,  1.0901e+00, -6.1351e-01,
         1.7229e-01,  4.3010e-01, -4.3096e-01,  8.8046e-01, -6.6597e-01,
        -1.1715e-01, -3.6362e-01, -5.8057e-01,  6.6985e-01, -1.1758e+00,
         6.1249e-01,  8.4842e-01,  8.7052e-01,  2.7611e-01,  6.0196e-04,
        -1.8411e-01, -1.4727e-01, -1.7658e-01, -3.4017e-01,  2.4207e-01,
         2.7771e-02, -9.0728e-01,  1.2098e-01, -1.2561e-02, -2.5137e-01,
        -1.0995e+00, -5.9352e-03, -4.4799e-02,  3.8899e-02, -1.3031e-02,
         1.2957e-01,  1.4258e+00,  5.1375e-02,  3.6055e-02, -1.2223e-02,
        -8.9858e-01, -3.4949e-02, -1.1596e-02, -7.0568e-02, -4.3861e-02,
         6.7121e-01, -6.5101e-02,  1.4147e-01,  3.0597e-02, -4.2248e-02,
        -1.6033e-01,  5.3689e-02, -6.2959e-02, -1.0653e+00, -4.4656e-02,
        -1.1018e-01, -6.9813e-01,  1.4351e-02, -1.9473e-02,  1.4403e+00,
         4.7841e-03,  1.4823e-01,  6.4216e-01,  2.9696e-02,  5.8048e-03,
         2.8374e-03,  7.3030e-01, -6.3179e-03,  6.2396e-02, -4.9180e-02,
         4.7028e-02, -8.5298e-02,  1.3268e-02,  1.2936e-02,  2.5581e-03,
         6.5898e-02,  5.0503e-02,  1.8335e-04, -7.7136e-02, -8.1141e-03,
         6.0088e-02, -6.7164e-03,  6.2757e-01, -1.5225e-02, -1.3339e-02,
         4.4225e-02,  4.4246e-01,  9.6403e-03, -2.3488e-02, -2.2245e-03,
        -6.9258e-03, -3.4872e-02,  1.2314e-01, -1.2847e-02,  6.1940e-02,
         9.5351e-01, -1.8617e-01, -4.0019e-02,  1.3935e+00, -2.7912e-01,
        -8.0205e-02,  1.6921e-01,  7.5401e-01, -4.8469e-01, -5.8824e-01,
        -1.0366e+00, -9.4052e-02, -1.0235e+00,  6.5909e-01,  5.9372e-01,
        -2.2173e-01, -5.1704e-01,  4.2185e-01,  1.5376e-01,  7.5608e-01,
        -5.6568e-02,  4.4606e-01, -4.9589e-01, -6.8244e-01,  1.4772e+00,
        -6.2847e-01,  1.8209e-02, -8.6742e-01,  2.5132e-01, -4.1317e-01,
         7.3504e-01,  4.4873e-02,  7.8186e-01, -4.2391e-01,  5.4203e-02;
bias4 << -0.1873, -0.1453, -1.0477,  0.1538;
out_mean << -24.245327789699125, -4.972238784710127, 858.4484288696027, -1.6173056382356812;
out_std << 106.60507509883607, 342.38221052518674, 505.27382432744236, 6.871454392023513;
in_mean << 0.04755175580815579, -0.10310912487916579, -0.00822193571007142, 59.96615465535339, 1989.5623071836276, 0.8023074986664168, 0.0998446902747603, 0.34080175360478987;
in_std << 0.028762000257431117, 0.5305530689625024, 0.9490676466883241, 23.21776169763347, 869.4323970393215, 0.29001461209388496, 0.057700841504368966, 0.10050279338464484;
}
