#include <fstream>
#include <eigen3/Eigen/Dense>


class LinearModel{
public:
  LinearModel();
  ~LinearModel();
  
  void init_state();
  void init_state(float *start_state);
  void step(float vl, float vr);
  void log_xout();
  
  std::ofstream log_file;
  float vehicle_state[3];
  static constexpr float timestep = .05; //The rate that nn_model operates at.
};

