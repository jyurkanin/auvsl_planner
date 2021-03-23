#include <iostream>
#include <thread>


#include "dynamic_model.h"
#include "auvsl_planner_node.h"



  

octomap::OcTree *ocTree;



  
/*
 * State of dynamic vehicle is actually:
 * [x,y,z,qx,qy,qz,q1,q2,q3,q4,qw,vx,vy,vz,wx,wy,wz,qd1,qd2,qd3,qd4]
 *
 * Reduced state for planner is going to be:
 * [x, y, theta, vx, vy, wz]
 *
 * Vx,Vy,Wz are important, but x and y are going to be most important
 * for determining distance between states 
 * 
 */


DynamicVehicleStateSampler::DynamicVehicleStateSampler(const ob::SpaceInformation *si) : ValidStateSampler(si){
  name_ = "Vehicle State Sampler";
}
  
  
bool DynamicVehicleStateSampler::sample(ob::State *state){
  double* val = static_cast<ob::RealVectorStateSpace::StateType*>(state)->values;
  
  //this is all place holder until I can see how it works and iterate on it.
  val[0] = rng_.uniformReal(-10,10);
  val[1] = rng_.uniformReal(-10,10);
  val[2] = rng_.uniformReal(0,2*M_PI);
  val[3] = rng_.uniformReal(-1,1);
  val[4] = rng_.uniformReal(-1,1);
  val[5] = rng_.uniformReal(-1,1);
  
  while(!isStateValid(state)){ //this is not smart.
    val[0] = rng_.uniformReal(-10,10); 
    val[1] = rng_.uniformReal(-10,10);
  }

  ROS_INFO("State %f %f %f %f %f %f\n", val[0], val[1], val[2], val[3], val[4], val[5]);
  assert(si_->isValid(state));
  return true;
}



//This is supposed to generate a sample close to "near"
bool DynamicVehicleStateSampler::sampleNear(ob::State* state, const ob::State* near, const double distance){
  //  double* state_values = static_cast<ob::RealVectorStateSpace::StateType*>(state)->values;
  //  double* near_values = static_cast<ob::RealVectorStateSpace::StateType*>(near)->values;
  
  const ob::RealVectorStateSpace::StateType& state_vector = *state->as<ob::RealVectorStateSpace::StateType>();
  const ob::RealVectorStateSpace::StateType& near_vector = *near->as<ob::RealVectorStateSpace::StateType>();
  
  int got_valid_state = 1;
  int counter = 0;
  do{
    float r = distance;
    float theta = rng_.uniformReal(0, 2*M_PI);
    
    //TODO: this is important, replace this with dynamics function
    state_vector.values[0] = near_vector.values[0] + (r*cosf(theta));
    state_vector.values[1] = near_vector.values[1] + (r*sinf(theta));
    state_vector.values[2] = theta;
    
    counter++;
    if(counter == 10){
      got_valid_state = 0;
      break;
    }
    
  } while(!isStateValid(near));

  
  return got_valid_state;
}


 



bool isStateValid(const ob::State *state){
  //not sure what this next line does.
  const ob::RealVectorStateSpace::StateType& state_vector = *state->as<ob::RealVectorStateSpace::StateType>();
  
  ROS_INFO("State %f %f %f %f %f %f\n", state_vector[0], state_vector[1], state_vector[2], state_vector[3], state_vector[4], state_vector[5]);
  
  const int search_depth = 0; //tunable parameters
  const float threshold = .5; //occupancy threshold
  
  int hasCollision = 0;
  
  octomap::point3d min(state_vector.values[0]-.1, state_vector.values[1]-.1, 0);
  octomap::point3d max(state_vector.values[0]+.1, state_vector.values[1]+.1, 0); //This defines the opposite vertices of a bounding box.
  
  octomap::OcTree::leaf_bbx_iterator end = ocTree->end_leafs_bbx();
  for(octomap::OcTree::leaf_bbx_iterator it = ocTree->begin_leafs_bbx(min,max);  it!= end; ++it){
    if(it->getOccupancy() > threshold){
      hasCollision = 1;
      break;
    }
  }
  
  //octomap::OcTreeNode* node = ocTree->search(val[0], val[1], 0, search_depth);
  return hasCollision;
}
  
// Returns an instance of Dynamic Vehicle State Sampler
// Why ompl makes me use an allocator, I have no idea.
ob::ValidStateSamplerPtr allocDynamicVehicleStateSampler(const ob::SpaceInformation *si){
  return std::make_shared<DynamicVehicleStateSampler>(si);
}
  



octomap::OcTree* newSimpleOctreeMap(){
  octomap::OcTree *ocTree = new octomap::OcTree(.1);

  //Make a square grid of unoccupied cells.
  for(int x = -10; x <= 10; x++){
    for(int y = -10; y <= 10; y++){
      octomap::point3d fake_point((float)x, (float)y, 0);
      octomap::OcTreeNode* node = ocTree->updateNode(fake_point, false);
      
    }
  }

  //make a line obstacle as a series of points every .1m
  for(int i = -50; i < 50; i++){
    octomap::point3d fake_point(i/10.0f, i/40.0f, 0);
    ocTree->updateNode(fake_point, true);
  }

  return ocTree;  
}


TireSoilData get_soil_data_at(float x, float y){
  const int search_depth = 0;
  const float threshold = .5;
  //octomap::OcTreeNode* node = ocTree->search(val[0], val[1], 0, search_depth);
  //TODO. THis.
  
  
  return lookupSoilTable(0);
}


float get_altitude(float x, float y){
  return 0;
}




void plan(){
  // construct the state space we are planning in
  std::shared_ptr<ob::RealVectorStateSpace> space(std::make_shared<ob::RealVectorStateSpace>(6));
  
  // set the bounds
  ob::RealVectorBounds bounds(5);
  bounds.setLow(0, -10);
  bounds.setHigh(0, 10);
  
  bounds.setLow(1, -10);
  bounds.setHigh(1, 10);
  
  bounds.setLow(2, 0);
  bounds.setHigh(2, 2*M_PI);
  
  bounds.setLow(3, -100);
  bounds.setHigh(3, 100);
  
  bounds.setLow(4, -100);
  bounds.setHigh(4, 100);

  bounds.setLow(5, -100);
  bounds.setHigh(5, 100);
  space->setBounds(bounds);
  
  og::SimpleSetup ss(space);
  
  ss.setStateValidityChecker(isStateValid);
  
  ob::ScopedState<> start(space);
  start[0] = 0;
  start[1] = 0;
  start[2] = 0;
  start[3] = 0;
  start[4] = 0;
  start[5] = 0;
  
  ob::ScopedState<> goal(space);
  goal[0] = -5;
  goal[1] = -8;
  goal[2] = 0;
  goal[3] = 0;
  goal[4] = 0;
  goal[5] = 0;
  
  ss.setStartAndGoalStates(start, goal);
  

  
  ob::SpaceInformationPtr si = ss.getSpaceInformation();
  si->setValidStateSamplerAllocator(allocDynamicVehicleStateSampler);
  
  
  std::shared_ptr<og::RRT> planner(std::make_shared<og::RRT>(si));
  planner->setGoalBias(.05); //.05 was recommended.
  planner->setRange(.1); //should match octomap detail level
  planner->setIntermediateStates(true);
  
  ss.setPlanner(planner);
  ss.setup();
  
  std::vector<double> cs(2);
  cs[0] = cs[1] = 0.1;
  ss.getStateSpace()->getDefaultProjection()->setCellSizes(cs);
  
  
  //ob::PlannerTerminationCondition condition;
  ob::PlannerStatus solved = ss.solve(ob::timedPlannerTerminationCondition(100.0));
  if(solved){
    std::cout << "Found solution:" << std::endl;
    ss.getSolutionPath().print(std::cout);
  }
  else{
    std::cout << "No solution found" << std::endl;
  }
}






int main(int argc, char **argv){
  ros::init(argc, argv, "auvsl_planner");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<std_msgs::String>("Global_Planner", 1); //buffer size is one. Only one global plan needed
  
  ros::Rate loop_rate(10);
  
  ocTree = newSimpleOctreeMap();
  
  init_model(0);
  
  //while(ros::ok()){

  plan();
    //pub.publish(msg);
    
  ros::spinOnce();
  loop_rate.sleep();
    //}
  
  return 0;
}





