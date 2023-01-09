/*============================= Joint PD ==============================*/
/**
 * FSM State that allows PD control of the joints.
 */

#include "FSM_State_RLJointPD.h"
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
FSM_State_RLJointPD<T>::FSM_State_RLJointPD(ControlFSMData<T>* _controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::RL_JOINT_PD, "RL_JOINT_PD"),
        contactPlanning_(10, 0.026), policy({512, 256, 64}){

  std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
  std::cout << "Setup Joint Position Control learned by Reinforcement Learning" << std::endl;
  std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;

  std::string modelNumber = "100";
  _loadPath = std::string(get_current_dir_name()) + "/../actor_model/actor_" + modelNumber + ".txt";
  policy.updateParamFromTxt(_loadPath);

  contactPlanning_.setStanceAndSwingTime(0.13, 0.13);

  _obsDim = OBSDIM;
  historyLength_ = 12;
  nJoints_ = 12;
  actionDim_ = nJoints_;

  _obs.setZero(_obsDim);
  _obsMean.setZero(_obsDim);
  _obsVar.setZero(_obsDim);
  previousAction_.setZero(actionDim_);
  command_.setZero();

  std::string in_line;
  std::ifstream obsMean_file, obsVariance_file;
  obsMean_file.open(std::string(get_current_dir_name()) + "/../actor_model/mean" + modelNumber + ".csv");
  obsVariance_file.open(std::string(get_current_dir_name()) + "/../actor_model/var" + modelNumber + ".csv");

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

  jointPosHist_.setZero(nJoints_ * historyLength_);
  jointVelHist_.setZero(nJoints_ * historyLength_);

  begin_ = std::chrono::steady_clock::now();
}

template <typename T>
void FSM_State_RLJointPD<T>::onEnter() {
  std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
  std::cout << "Start Joint Position Control learned by Reinforcement Learning" << std::endl;
  std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
  // Default is to not transition
  this->nextStateName = this->stateName;

  // Reset the transition data
  this->transitionData.zero();

  contactPlanning_.reset();

  historyTempMem_.setZero();
  previousAction_.setZero();
  _jointQ << this->_data->_legController->datas[0].q, this->_data->_legController->datas[1].q, this->_data->_legController->datas[2].q, this->_data->_legController->datas[3].q;
  _jointQd << this->_data->_legController->datas[0].qd, this->_data->_legController->datas[1].qd, this->_data->_legController->datas[2].qd, this->_data->_legController->datas[3].qd;
  for(int i = 0; i < historyLength_; i++) {
    jointPosHist_.segment(nJoints_ * i, nJoints_) << _jointQ;
    jointVelHist_.segment(nJoints_ * i, nJoints_) << _jointQd;
  }

  for(int i=0; i<4; i++) {
    this->_data->_legController->commands->zero();
    preCommands[i].zero();
  }

  emergency_stop = false;
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_RLJointPD<T>::run() {

  if(std::acos(this->_data->_stateEstimator->getResult().rBody.transpose().row(2)(2)) > 3.1415*30./180.) {
    std::cout << "Orientation is in bad condition!!!" << std::endl;
    std::cout << "RL Joint PD Control mode is denied!!!" << std::endl;
    emergency_stop = true;
    return;
  }

  if (emergency_stop) return;  // don't start the code if it violated orientation condition

  end_ = std::chrono::steady_clock::now();

  if (std::chrono::duration_cast<std::chrono::microseconds>(end_ - begin_).count() > int(control_dt_ * 1000000)) {
    begin_ = std::chrono::steady_clock::now();
  }
  else {
    for (int leg(0); leg < 4; ++leg) {
      this->_data->_legController->commands[leg] = preCommands[leg];
    }
    return;
  }

  if (this->_data->controlParameters->use_rc) {
    command_(0) = this->_data->_desiredStateCommand->rcCommand->v_des[0];
    command_(1) = this->_data->_desiredStateCommand->rcCommand->v_des[1];
    command_(2) = this->_data->_desiredStateCommand->rcCommand->omega_des[2];
    if (command_.norm() < 0.4) command_.setZero();
  }
  else {
    command_(0) = this->_data->_desiredStateCommand->gamepadCommand->leftStickAnalog(1);
    command_(1) = -this->_data->_desiredStateCommand->gamepadCommand->leftStickAnalog(0);
    command_(2) = this->_data->_desiredStateCommand->gamepadCommand->rightStickAnalog(0);
    if (command_.norm() < 0.4) command_.setZero();
  }

  previousJointQ_ = _jointQ;
  previousJointQd_ = _jointQd;
  _jointQ << this->_data->_legController->datas[0].q, this->_data->_legController->datas[1].q, this->_data->_legController->datas[2].q, this->_data->_legController->datas[3].q;
  _jointQd << this->_data->_legController->datas[0].qd, this->_data->_legController->datas[1].qd, this->_data->_legController->datas[2].qd, this->_data->_legController->datas[3].qd;

  historyTempMem_ = jointVelHist_;
  jointVelHist_.head((historyLength_-1) * nJoints_) = historyTempMem_.tail((historyLength_-1) * nJoints_);
  jointVelHist_.tail(nJoints_) = previousJointQd_;
  historyTempMem_ = jointPosHist_;
  jointPosHist_.head((historyLength_-1) * nJoints_) = historyTempMem_.tail((historyLength_-1) * nJoints_);
  jointPosHist_.tail(nJoints_) = previousJointQ_;

  contactPlanning_.updateContactSequence();
  isContact_ = contactPlanning_.getIsContact();
  contactPhase_ = contactPlanning_.getContactPhase();
  this->_data->_stateEstimator->setContactPhase(isContact_);
  this->_data->_stateEstimator->run();
  _bodyOri << this->_data->_stateEstimator->getResult().rBody.transpose().row(2).transpose(); // body orientation 3 cf) rBody: R_bw
  _bodyAngularVel << this->_data->_stateEstimator->getResult().omegaBody;  // angular velocity 3

  _obs = getObservation();

  for (int i = 0; i < _obs.size(); i++) {
    _obs(i) = (_obs(i) - _obsMean(i)) / std::sqrt(_obsVar(i) + 1e-8);
    if (_obs(i) > 10) _obs(i) = 10.0;
    if (_obs(i) < -10) _obs(i) = -10.0;
  }

  // action scaling
  torqueInput_ = policy.forward(_obs);
  previousAction_ = torqueInput_;

  this->_data->_legController->_legsEnabled = true;

  for (int leg(0); leg < 4; ++leg) {
    for (int jidx(0); jidx < 3; ++jidx) {
      this->_data->_legController->commands[leg].tauFeedForward[jidx] = torqueInput_(leg * 3 + jidx);
    }

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
FSM_StateName FSM_State_RLJointPD<T>::checkTransition() {
  this->nextStateName = this->stateName;

  iter++;

  // Switch FSM control mode
  switch ((int)this->_data->controlParameters->control_mode) {
    case K_RL_JOINT_PD:
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
                << K_RL_JOINT_PD << " to "
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
TransitionData<T> FSM_State_RLJointPD<T>::transition() {
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
                << K_RL_JOINT_PD << " to "
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
void FSM_State_RLJointPD<T>::onExit() {
  // Nothing to clean up when exiting
}

// template class FSM_State_JointPD<double>;
template class FSM_State_RLJointPD<float>;

template <typename T>
const Eigen::Matrix<float, OBSDIM, 1> & FSM_State_RLJointPD<T>::getObservation() {
  _obs <<
      _bodyOri, _bodyAngularVel,
      _jointQ, _jointQd, previousAction_,
      jointPosHist_.segment((historyLength_ - 12) * nJoints_, nJoints_),
      jointPosHist_.segment((historyLength_ - 9) * nJoints_, nJoints_),
      jointPosHist_.segment((historyLength_ - 6) * nJoints_, nJoints_),
      jointPosHist_.segment((historyLength_ - 3) * nJoints_, nJoints_),
      jointVelHist_.segment((historyLength_ - 12) * nJoints_, nJoints_),
      jointVelHist_.segment((historyLength_ - 9) * nJoints_, nJoints_),
      jointVelHist_.segment((historyLength_ - 6) * nJoints_, nJoints_),
      jointVelHist_.segment((historyLength_ - 3) * nJoints_, nJoints_),
      isContact_, std::sin(contactPhase_(0)), std::cos(contactPhase_(0)),
      command_;

  return _obs;
}