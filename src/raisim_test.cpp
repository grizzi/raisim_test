#include "raisim/World.hpp"
#include "omp.h"

std::string binaryDir;

class MultiThreadedSim{
public: 
  MultiThreadedSim(int env_count){
    for(size_t i=0; i<env_count; i++) {
      createEnv();
    }
  }
  
  void run(int it){
    for(size_t i=0; i<it; i++){
# pragma omp parallel for
      for(size_t j=0; j<envs_.size(); j++)
        stepEnv(j, 100);
    }
  }

private:
  void stepEnv(int i, int n){
    for(size_t t=0; t<n; t++)
      envs_[i]->integrate();
  }

  void createEnv(){
    envs_.push_back(std::make_unique<raisim::World>());
    std::unique_ptr<raisim::World>& world = envs_.back();

    world->setTimeStep(0.003);
    world->setERP(world->getTimeStep(), world->getTimeStep());
    
    // ground
    auto ground = world->addGround();
    
    // robot hand
    auto hand = world->addArticulatedSystem(binaryDir + "/../rsc/panda_hand/panda_hand.urdf");
    hand->setGeneralizedCoordinate({0.04, 0.04});
    hand->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
    Eigen::VectorXd kp(2), kd(2), q_des(2), v_des(2);
    q_des << 0.04, 0.04;
    v_des << 0.0, 0.0;
    kp << 100.0, 100.0;
    kd << 1.0, 1.0;
    hand->setPdGains(kp, kd);
    hand->setPdTarget(q_des, v_des);

    // rings
    int N = 3;
    double gap = 0.01;
    std::string torusFile = binaryDir + "/../rsc/torus/torus.obj";
    raisim::Mat<3, 3> inertia; inertia.setIdentity();
    const raisim::Vec<3> com = {0, 0, 0};
    for(int row = 0; row < N; row++) {
      for(int col = 0; col < N; col++) {
        auto torus = world->addMesh(torusFile, 1.0, inertia, com);
        torus->setPosition(-gap*(N/2) + gap*row, -gap*(N/2) + gap*col, 2.0 + 0.1*(row*N+col));
      }
    }
  }

private:
  std::vector<std::unique_ptr<raisim::World>> envs_;
};

int main(int argc, char **argv) {
  /// get binary path
  binaryDir = raisim::Path::setFromArgv(argv[0]).getDirectory();
  raisim::World::setActivationKey(binaryDir + "/../rsc/activation.raisim");
  
  omp_set_num_threads(4);
  std::cout << "Creating multithreaded simulation. " << std::endl;
  MultiThreadedSim sim(16);
  sim.run(1000);
  return 0; 
}
