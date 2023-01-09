#ifndef FSM_STATE_RLJOINTPD_H
#define FSM_STATE_RLJOINTPD_H

#include "FSM_State.h"
#include <cstring>
#include <experimental/filesystem>
#include "cpuMLP.hpp"
#include "../common/include/Controllers/StateEstimatorContainer.h"
#include "../common/include/Controllers/LegController.h"
#include "contactPlanning.hpp"

#define OBSDIM 147  // TODO

/**
 *
 */
template <typename T>
class FSM_State_RLJointPD : public FSM_State<T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FSM_State_RLJointPD(ControlFSMData<T>* _controlFSMData);

  // Behavior to be carried out when entering a state
  void onEnter();

  // Run the normal behavior for the state
  void run();

  // Checks for any transition triggers
  FSM_StateName checkTransition();

  // Manages state specific transitions
  TransitionData<T> transition();

  // Behavior to be carried out when exiting a state
  void onExit();

  virtual const Eigen::Matrix<float, OBSDIM, 1>& getObservation();

 private:
  // Keep track of the control iterations
  int iter = 0;

 protected:

  Eigen::Matrix<float, 3, 1> _bodyOri;
  Eigen::Matrix<float, 3, 1> _bodyAngularVel;
  Eigen::Matrix<float, 12, 1> _jointQ, _jointQd;
  Eigen::Matrix<float, 12, 1> previousJointQ_, previousJointQd_;
  Eigen::Matrix<float, OBSDIM, 1> _obs;
  int _obsDim, historyLength_, nJoints_, actionDim_;
  Eigen::VectorXf _obsMean, _obsVar;
  Eigen::VectorXf jointPosHist_, jointVelHist_, historyTempMem_;
  Eigen::VectorXf torqueInput_;
  Eigen::VectorXf previousAction_;
  Eigen::Vector4f isContact_, contactPhase_;
  Eigen::Vector3f command_;

  std::string _loadPath;

  planning::contactPlanning contactPlanning_;

  std::chrono::steady_clock::time_point begin_;
  std::chrono::steady_clock::time_point end_;

  rai::FuncApprox::MLP_fullyconnected<float, OBSDIM, 12, rai::FuncApprox::ActivationType::leakyrelu> policy;
  double control_dt_ = 0.026;

  int iterationCounter = 0;
  int iterationsBetweenMPC = 13;

  bool emergency_stop = false;

  LegControllerCommand<float> preCommands[4];
};

#endif  // FSM_STATE_RLJOINTPD_H
