/*============================= Joint PD ==============================*/
/**
 * FSM State that allows PD control of the joints.
 */

#include "FSM_State_SAFERLJointPD.h"
#include <Configuration.h>
#include <iostream>
#include <thread>
#include <unistd.h>
#include <iomanip>

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_SAFERLJointPD<T>::FSM_State_SAFERLJointPD(ControlFSMData<T>* _controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::SAFERL_JOINT_PD, "SAFE_RL_JOINT_PD"),
      _ini_jpos(cheetah::num_act_joint),
      privilegeEstimator_(ENCNUMLAYER),
      privilegeProjector_({}),
      actor_({256, 160, 128}),
  {

  std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
  std::cout << "Setup Joint Position Control learned by Safe Reinforcement Learning" << std::endl;
  std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;

  std::string model_itertaion = "#####1";  // TODO
  std::string teacher_model_itertaion = "#####2";  // TODO;
  std::string privilege_estimator_path = std::string(get_current_dir_name()) + "/../actor_model/privilege_estimator__" + model_itertaion + ".txt";
  std::string privilege_projector_path = std::string(get_current_dir_name()) + "/../actor_model/privilege_projector_" + model_itertaion + ".txt";
  std::string actor_path = std::string(get_current_dir_name()) + "/../actor_model/base_net_" + model_itertaion + ".txt";
  std::string obsMean_path = std::string(get_current_dir_name()) + "/../actor_model/mean" + teacher_model_itertaion + ".csv";
  std::string obsVariance_path = std::string(get_current_dir_name()) + "/../actor_model/var" + teacher_model_itertaion + ".csv";

  privilegeEstimator_.readParamFromTxt(privilege_estimator_path.string());
  privilegeProjector_.readParamFromTxt(privilege_projector_path.string());
  actor_.readParamFromTxt(actor_path.string());

  _obsDim = OBSDIM;
  historyLength_ = 3;
  nJoints_ = 12;
  actionDim_ = nJoints_;
  actorObs_.setZero(OBSDIM + ESTDIM);
  estOut_.setZero(ESTDIM);

  _obs.setZero(_obsDim);
  smoothObs_.setZero(_obsDim);
  _obsMean.setZero(_obsDim);
  _obsVar.setZero(_obsDim);
  previousAction_.setZero(actionDim_);
  prepreviousAction_.setZero(actionDim_);
  footPos_.setZero(4 * 3);
  command_.setZero();
  jointPositionLimits_.setZero(2 * nJoints_);
  jointPositionLimits_
      << -1.5, -5., -2.705259778, -1.5, -5., -2.705259778, -1.5, -5., -2.705259778, -1.5, -5., -2.705259778,
      1.5, 5., 2.705259778, 1.5, 5., 2.705259778, 1.5, 5., 2.705259778, 1.5, 5., 2.705259778;

  q_init.setZero(12);
  pTarget12_.setZero(12);
  q_init << 0, -0.9, 1.8, 0, -0.9, 1.8, 0, -0.9, 1.8, 0, -0.9, 1.8;

  std::string in_line;
  std::ifstream obsMean_file, obsVariance_file;
  obsMean_file.open(obsMean_path);
  obsVariance_file.open(obsVariance_path);

  if(obsMean_file.is_open()) {
    for(int i = 0; i < _obsMean.size(); i++){
      std::getline(obsMean_file, in_line);
      _obsMean(i) = std::stod(in_line);
    }
  }
  if(obsVariance_file.is_open()) {
    for(int i = 0; i < _obsVar.size(); i++){
      std::getline(obsVariance_file, in_line);
      _obsVar(i) = std::stod(in_line);
    }
  }
  obsMean_file.close();
  obsVariance_file.close();

  jointPosErrorHist_.setZero(nJoints_ * historyLength_ * historyUpdateSteps);
  jointVelHist_.setZero(nJoints_ * historyLength_ * historyUpdateSteps);

  begin_ = std::chrono::steady_clock::now();
}

template <typename T>
void FSM_State_ConcurrentRLJointPD<T>::onEnter() {
  std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
  std::cout << "Start Joint Position Control learned by Reinforcement Learning" << std::endl;
  std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
  // Default is to not transition
  this->nextStateName = this->stateName;

  // Reset the transition data
  this->transitionData.zero();

  _obs.setZero();
  smoothObs_.setZero();
  jointPosErrorHist_.setZero(); jointVelHist_.setZero();
  historyTempMem_.setZero();
  for(int i = 0; i < historyLength_ * historyUpdateSteps; i++) {
    jointVelHist_.segment(nJoints_ * i, nJoints_) <<
      this->_data->_legController->datas[0].qd, this->_data->_legController->datas[1].qd, this->_data->_legController->datas[2].qd, this->_data->_legController->datas[3].qd;
  }
  pTarget12_ << this->_data->_legController->datas[0].q, this->_data->_legController->datas[1].q, this->_data->_legController->datas[2].q, this->_data->_legController->datas[3].q;
  previousAction_ << pTarget12_; prepreviousAction_ << pTarget12_;
  resetControlClock();

  for(int i=0; i<4; i++) {
    preCommands[i].zero();
  }

  emergency_stop = false;

}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_ConcurrentRLJointPD<T>::run() {

  if(std::acos(this->_data->_stateEstimator->getResult().rBody.transpose().row(2)(2)) > 3.1415*45./180.) {
    std::cout << "Orientation is in bad condition!!!" << std::endl;
    std::cout << "Concurrent RL Joint PD Control mode is denied!!!" << std::endl;
    emergency_stop = true;
    return;
  }

  if (emergency_stop) return;  // don't start the code if it violated orientation condition

  end_ = std::chrono::steady_clock::now();

  if (std::chrono::duration_cast<std::chrono::microseconds>(end_ - begin_).count() > int(control_dt_ * 1000000)) {
    begin_ = std::chrono::steady_clock::now();
  } else {
    for (int leg(0); leg < 4; ++leg) {
      this->_data->_legController->commands[leg] = preCommands[leg];
    }
    updateControlClock();
    return;
  }

  if (this->_data->controlParameters->use_rc) {
    command_(0) = this->_data->_desiredStateCommand->rcCommand->v_des[0];
    command_(1) = this->_data->_desiredStateCommand->rcCommand->v_des[1];
    command_(2) = this->_data->_desiredStateCommand->rcCommand->omega_des[2];
  }
  else {
    command_(0) = this->_data->_desiredStateCommand->gamepadCommand->leftStickAnalog(1);
    command_(1) = -this->_data->_desiredStateCommand->gamepadCommand->leftStickAnalog(0);
    command_(2) = this->_data->_desiredStateCommand->gamepadCommand->rightStickAnalog(0);
  }

  if (command_.norm() < 0.3 && std::abs(sin(2 * M_PI * gait_freq_ * controlClock)) < 0.1) {
    command_.setZero();
    standingMode_ = true;
    resetControlClock();
  } else {
    standingMode_ = false;
  }

  double kneeGearRatio = 9.33 / 6.;
  bool isMinicheetah = true;  /// TODO: true for the real robot, false for the simulation test

  if (isMinicheetah) {
    this->kpMat = Vec3<T>(17, 17, 17 / (kneeGearRatio * kneeGearRatio)).asDiagonal();
    this->kdMat = Vec3<T>(0.4, 0.4, 0.4 / (kneeGearRatio * kneeGearRatio)).asDiagonal();
  }
  else {
    this->kpMat = Vec3<T>(17, 17, 17).asDiagonal();
    this->kdMat = Vec3<T>(0.4, 0.4, 0.4).asDiagonal();
  }

  updateObservation();
  updateHistory();
  updatePreviousActions();

  _obs = getObservation();

  if (controlClock != 0. && standingMode_)
    smoothObs_ = smoothObs_ * (1. - lowPassFilterParam) + _obs * lowPassFilterParam;
  else
    smoothObs_ = _obs;

  for (int i = 0; i < _obs.size(); i++) {
    smoothObs_(i) = (smoothObs_(i) - _obsMean(i)) / std::sqrt(_obsVar(i) + 1e-8);
  }

  // forward the obs to the encoder
  Eigen::Matrix<float, OBSDIM, 1> privilege_est_input = smoothObs_;
  Eigen::Matrix<float, ENCOUTDIM, 1> latent = privilegeEstimator_.forward(privilege_est_input);
  privilege_est_ = privilegeProjector_.forward(latent);

  // concat obs and e_out and forward to the actor
  actorObs_ << smoothObs_, privilege_est_;
  pTarget12_ = actor_.forward(actorObs_);

  // action scaling
  pTarget12_ *= 0.4;
  pTarget12_ += q_init;

  // action clipping
  pTarget12_ = pTarget12_.cwiseMax(jointPositionLimits_.segment(0, nJoints_)).cwiseMin(jointPositionLimits_.segment(nJoints_, nJoints_));

  this->_data->_legController->_legsEnabled = true;

  for (int leg(0); leg < 4; ++leg) {
    for (int jidx(0); jidx < 3; ++jidx) {
      this->_data->_legController->commands[leg].qDes[jidx] = pTarget12_(leg * 3 + jidx);
      this->_data->_legController->commands[leg].qdDes[jidx] = 0.;
    }
    this->_data->_legController->commands[leg].kpJoint = this->kpMat;
    this->_data->_legController->commands[leg].kdJoint = this->kdMat;

    preCommands[leg] = this->_data->_legController->commands[leg];
  }
}

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template <typename T>
FSM_StateName FSM_State_ConcurrentRLJointPD<T>::checkTransition() {
  this->nextStateName = this->stateName;

  iter++;

  // Switch FSM control mode
  switch ((int)this->_data->controlParameters->control_mode) {
    case K_CONCURRENT_RL_JOINT_PD:
      break;

    case K_JOINT_PD:
      // Normal operation for state based transitions
      this->nextStateName = FSM_StateName::JOINT_PD;
      break;

    case K_IMPEDANCE_CONTROL:
      // Requested change to impedance control
      this->nextStateName = FSM_StateName::IMPEDANCE_CONTROL;

      // Transition time is 1 second
      this->transitionDuration = 1.0;
      break;

    case K_STAND_UP:
      // Requested change to impedance control
      this->nextStateName = FSM_StateName::STAND_UP;

      // Transition time is immediate
      this->transitionDuration = 0.0;
      break;

    case K_PASSIVE:
      // Requested change to BALANCE_STAND
      this->nextStateName = FSM_StateName::PASSIVE;

      // Transition time is immediate
      this->transitionDuration = 0.0;

      break;

    case K_RECOVERY_STAND:
      // Requested change to BALANCE_STAND
      this->nextStateName = FSM_StateName::RECOVERY_STAND;

      // Transition time is immediate
      this->transitionDuration = 0.0;

      break;

    default:
      std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                << K_CONCURRENT_RL_JOINT_PD << " to "
                << this->_data->controlParameters->control_mode << std::endl;
  }

  // Get the next state
  return this->nextStateName;
}

/**
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */
template <typename T>
TransitionData<T> FSM_State_ConcurrentRLJointPD<T>::transition() {
  // Switch FSM control mode
  switch (this->nextStateName) {
    case FSM_StateName::IMPEDANCE_CONTROL:

      iter++;
      if (iter >= this->transitionDuration * 1000) {
        this->transitionData.done = true;
      } else {
        this->transitionData.done = false;
      }
      break;

    case FSM_StateName::STAND_UP:
      this->transitionData.done = true;
      break;

    case FSM_StateName::PASSIVE:
      this->turnOffAllSafetyChecks();
      this->transitionData.done = true;
      break;

    case FSM_StateName::RECOVERY_STAND:
      this->transitionData.done = true;
      break;

    default:
      std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                << K_CONCURRENT_RL_JOINT_PD << " to "
                << this->_data->controlParameters->control_mode << std::endl;
  }
  // Finish transition
  this->transitionData.done = true;

  // Return the transition data to the FSM
  return this->transitionData;
}

/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void FSM_State_ConcurrentRLJointPD<T>::onExit() {
  // Nothing to clean up when exiting
}

// template class FSM_State_JointPD<double>;
template class FSM_State_ConcurrentRLJointPD<float>;


template <typename T>
void FSM_State_ConcurrentRLJointPD<T>::updateHistory() {
  historyTempMem_ = jointVelHist_;
  jointVelHist_.head(nJoints_ * historyLength_ * historyUpdateSteps - nJoints_) = historyTempMem_.tail(nJoints_ * historyLength_ * historyUpdateSteps - nJoints_);
  jointVelHist_.tail(nJoints_) = _jointQd;

  historyTempMem_ = jointPosErrorHist_;
  jointPosErrorHist_.head(nJoints_ * historyLength_ * historyUpdateSteps - nJoints_) = historyTempMem_.tail(nJoints_ * historyLength_ * historyUpdateSteps - nJoints_);
  jointPosErrorHist_.tail(nJoints_) = pTarget12_ - _jointQ;
}

template <typename T>
void FSM_State_ConcurrentRLJointPD<T>::updatePreviousActions() {
  prepreviousAction_ = previousAction_;
  previousAction_ = pTarget12_;
}

template <typename T>
void FSM_State_ConcurrentRLJointPD<T>::updateObservation() {
  // contact estimation
  // by using threshold
  double contactThreshold = -0.4;
  Vec4<T> isContact; isContact.setZero();
  isContact << ((pTarget12_(2) - _jointQ(2)) < contactThreshold),
      ((pTarget12_(5) - _jointQ(5)) < contactThreshold),
      ((pTarget12_(8) - _jointQ(8)) < contactThreshold),
      ((pTarget12_(1) - _jointQ(11)) < contactThreshold);

  this->_data->_stateEstimator->setContactPhase(isContact);

  this->_data->_stateEstimator->run();
  _bodyHeight = this->_data->_stateEstimator->getResult().position(2);  // body height 1
  _bodyOri << this->_data->_stateEstimator->getResult().rBody.transpose().row(2).transpose(); // body orientation 3 cf) rBody: R_bw
  _jointQ << this->_data->_legController->datas[0].q, this->_data->_legController->datas[1].q, this->_data->_legController->datas[2].q, this->_data->_legController->datas[3].q;  // joint angles 3 3 3 3 = 12
  _bodyVel << this->_data->_stateEstimator->getResult().vBody;  // velocity 3
  _bodyAngularVel << this->_data->_stateEstimator->getResult().omegaBody;  // angular velocity 3
  _jointQd << this->_data->_legController->datas[0].qd, this->_data->_legController->datas[1].qd, this->_data->_legController->datas[2].qd, this->_data->_legController->datas[3].qd;  // joint velocity 3 3 3 3 = 12
  footPos_ << this->_data->_quadruped->getHipLocation(0) + this->_data->_legController->datas[0].p, this->_data->_quadruped->getHipLocation(1) + this->_data->_legController->datas[1].p,
      this->_data->_quadruped->getHipLocation(2) + this->_data->_legController->datas[2].p, this->_data->_quadruped->getHipLocation(3) + this->_data->_legController->datas[3].p;  // foot position  in the body frame 3 3 3 3 = 12

}

template <typename T>
const Eigen::Matrix<float, OBSDIM, 1> & FSM_State_ConcurrentRLJointPD<T>::getObservation() {
  _obs <<
      command_,
      _bodyOri,
      _bodyAngularVel,
      _jointQ,
      _jointQd,
      (1.0 - static_cast<double>(standingMode_)) * cos(2 * M_PI * gait_freq_ * controlClock),
      (1.0 - static_cast<double>(standingMode_)) * sin(2 * M_PI * gait_freq_ * controlClock),
      (1.0 - static_cast<double>(standingMode_)) * cos(2 * M_PI * gait_freq_ * controlClock + M_PI),
      (1.0 - static_cast<double>(standingMode_)) * sin(2 * M_PI * gait_freq_ * controlClock + M_PI),
      (1.0 - static_cast<double>(standingMode_)) * cos(2 * M_PI * gait_freq_ * controlClock + M_PI),
      (1.0 - static_cast<double>(standingMode_)) * sin(2 * M_PI * gait_freq_ * controlClock + M_PI),
      (1.0 - static_cast<double>(standingMode_)) * cos(2 * M_PI * gait_freq_ * controlClock),
      (1.0 - static_cast<double>(standingMode_)) * sin(2 * M_PI * gait_freq_ * controlClock),
      static_cast<double>(standingMode_),
      jointPosErrorHist_.segment(nJoints_ * (historyLength_ - historyLength_) * historyUpdateSteps, nJoints_),
      jointPosErrorHist_.segment(nJoints_ * (historyLength_ - (historyLength_-1)) * historyUpdateSteps, nJoints_),
      jointPosErrorHist_.segment(nJoints_ * (historyLength_ - (historyLength_-2)) * historyUpdateSteps, nJoints_),
      jointVelHist_.segment(nJoints_ * (historyLength_ - historyLength_) * historyUpdateSteps, nJoints_),
      jointVelHist_.segment(nJoints_ * (historyLength_ - (historyLength_-1)) * historyUpdateSteps, nJoints_),
      jointVelHist_.segment(nJoints_ * (historyLength_ - (historyLength_-2)) * historyUpdateSteps, nJoints_),
      previousAction_,
      prepreviousAction_;

  return _obs;
}

template <typename T>
void FSM_State_ConcurrentRLJointPD<T>::resetControlClock() { controlClock = 0.; }

template <typename T>
void FSM_State_ConcurrentRLJointPD<T>::updateControlClock() {
  if (!standingMode_)
    controlClock += control_dt_;
}