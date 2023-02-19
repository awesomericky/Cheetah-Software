#ifndef FSM_STATE_SAFERLJOINTPD_H
#define FSM_STATE_SAFERLJOINTPD_H

#include "FSM_State.h"
#include <cstring>
#include <experimental/filesystem>
#include "cpuNN.hpp"
#include "../common/include/Controllers/StateEstimatorContainer.h"
#include "../common/include/Controllers/LegController.h"

#define OBSDIM 138
#define ESTDIM 24
#define ACTDIM 12
#define COMDIM 3
#define ENCOUTDIM 128
#define ENCNUMLAYER 1

/**
 *
 */
template <typename T>
class FSM_State_SAFERLJointPD : public FSM_State<T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FSM_State_SAFERLJointPD(ControlFSMData<T>* _controlFSMData);

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

  virtual void updateHistory();
  virtual void updateObservation();
  virtual void updatePreviousActions();
  virtual const Eigen::Matrix<float, OBSDIM, 1>& getObservation();
  virtual void resetControlClock();
  virtual void updateControlClock();

 private:
  // Keep track of the control iterations
  int iter = 0;
  DVec<T> _ini_jpos;

 protected:

  float _bodyHeight;
  Eigen::Matrix<float, 3, 1> _bodyOri;
  Eigen::Matrix<float, 12, 1> _jointQ;
  Eigen::Matrix<float, 3, 1> _bodyVel;
  Eigen::Matrix<float, 3, 1> _bodyAngularVel;
  Eigen::Matrix<float, 12, 1> _jointQd;
  Eigen::Matrix<float, OBSDIM, 1> _obs;
  Eigen::Matrix<float, OBSDIM, 1> smoothObs_;
  Eigen::Matrix<float, OBSDIM + ESTDIM, 1> actorObs_;
  Eigen::Matrix<float, ESTDIM, 1> privilege_est_;
  int _obsDim, historyLength_, nJoints_, actionDim_;
  Eigen::VectorXf _obsMean, _obsVar;
  Eigen::VectorXf jointPosErrorHist_, jointVelHist_, historyTempMem_;
  Eigen::VectorXf q_init;
  Eigen::VectorXf pTarget12_;
  Eigen::VectorXf previousAction_, prepreviousAction_;
  Eigen::VectorXf footPos_;
  Eigen::Vector3f command_;

  std::string _loadPath;

  int iterationCounter = 0;

  std::chrono::steady_clock::time_point begin_;
  std::chrono::steady_clock::time_point end_;

  rai::FuncApprox::GRU<float, OBSDIM, ENCOUTDIM> privilegeEstimator_;
  rai::FuncApprox::Linear<float, ENCOUTDIM, ESTDIM, rai::FuncApprox::ActivationType::linear> privilegeProjector_;
  rai::FuncApprox::Linear<float, OBSDIM + ESTDIM, ACTDIM, rai::FuncApprox::ActivationType::leaky_relu> actor_;

//  rai::FuncApprox::MLP_fullyconnected<float, OBSDIM + UNOBSDIM, 12, rai::FuncApprox::ActivationType::leakyrelu> policy;
//  rai::FuncApprox::MLP_fullyconnected<float, OBSDIM, UNOBSDIM, rai::FuncApprox::ActivationType::leakyrelu> estimator;
  double control_dt_ = 0.01;
  double historyUpdatePeriod = 0.02; // s
  int historyUpdateSteps = (int) (historyUpdatePeriod / control_dt_);

  bool emergency_stop = false;

  LegControllerCommand<float> preCommands[4];

  // controller specific inputs
  bool standingMode_ = true;
  double gait_freq_ = 2.;
  double controlClock;
  Eigen::VectorXf jointPositionLimits_;
  const int nJoints_ = 12.;
  const double lowPassFilterParam = 0.7;
};

#endif  // FSM_STATE_SAFERLJOINTPD_H
