/*! @file MiniCheetah.h
 *  @brief Utility function to build a Mini Cheetah Quadruped object
 *
 * This file is based on MiniCheetahFullRotorModel_mex.m and builds a model
 * of the Mini Cheetah robot.  The inertia parameters of all bodies are
 * determined from CAD.
 *
 */

#ifndef PROJECT_MINICHEETAH_H
#define PROJECT_MINICHEETAH_H

#include "FloatingBaseModel.h"
#include "Quadruped.h"

/*!
 * Generate a Quadruped model of Mini Cheetah
 */
template <typename T>
Quadruped<T> buildMiniCheetah() {
  Quadruped<T> cheetah;
  cheetah._robotType = RobotType::MINI_CHEETAH;

  cheetah._bodyMass = 4.832;
  cheetah._bodyLength = 0.20275 * 2;
  cheetah._bodyWidth = 0.049 * 2;
  cheetah._bodyHeight = 0.05 * 2;
  cheetah._abadGearRatio = 6;
  cheetah._hipGearRatio = 6;
  cheetah._kneeGearRatio = 9.33;
  cheetah._abadLinkLength = 0.068;  /// TODO: check!
  cheetah._hipLinkLength = 0.2085;
  //cheetah._kneeLinkLength = 0.175;
  //cheetah._maxLegLength = 0.384;
  cheetah._kneeLinkY_offset = 0.004;
  //cheetah._kneeLinkLength = 0.20;
  cheetah._kneeLinkLength = 0.195;  // 0.195
  cheetah._maxLegLength = 0.409;


  cheetah._motorTauMax = 3.f;
  cheetah._batteryV = 24;
  cheetah._motorKT = .05;  // this is flux linkage * pole pairs
  cheetah._motorR = 0.173;
  cheetah._jointDamping = .01;
  cheetah._jointDryFriction = .2;
  //cheetah._jointDamping = .0;
  //cheetah._jointDryFriction = .0;


  // rotor inertia if the rotor is oriented so it spins around the z-axis  /// TODO: check!
  Mat3<T> rotorRotationalInertiaZ;
  rotorRotationalInertiaZ << 33, 0, 0, 0, 33, 0, 0, 0, 141;
  rotorRotationalInertiaZ = 1e-6 * rotorRotationalInertiaZ;
//  rotorRotationalInertiaZ.setZero();  // test without rotor inertia

  Mat3<T> RY = coordinateRotation<T>(CoordinateAxis::Y, M_PI / 2);
  Mat3<T> RX = coordinateRotation<T>(CoordinateAxis::X, M_PI / 2);
  Mat3<T> rotorRotationalInertiaX =
      RY * rotorRotationalInertiaZ * RY.transpose();  // 63 33 33
  Mat3<T> rotorRotationalInertiaY =
      RX * rotorRotationalInertiaZ * RX.transpose();  // 33 63 33

  // spatial inertias
  Mat3<T> abadRotationalInertia;
  abadRotationalInertia << 381, 58, 0.45, 58, 560, 0.95, 0.45, 0.95, 444;
  abadRotationalInertia = abadRotationalInertia * 1e-6;
  Vec3<T> abadCOM(0, 0.0, 0);  // LEFT
  SpatialInertia<T> abadInertia(0.54, abadCOM, abadRotationalInertia);

  Mat3<T> hipRotationalInertia;
  hipRotationalInertia << 1983, 245, 13, 245, 2103, 1.5, 13, 1.5, 408;
  hipRotationalInertia = hipRotationalInertia * 1e-6;
  Vec3<T> hipCOM(0, 0.017, -0.02);  /// TODO: check!  // y axis value has an opposite sign?
  SpatialInertia<T> hipInertia(0.634, hipCOM, hipRotationalInertia);

  Mat3<T> kneeRotationalInertia, kneeRotationalInertiaRotated;
  kneeRotationalInertiaRotated << 15, 0, 0, 0, 612, 0, 0, 0, 605;
  kneeRotationalInertiaRotated = kneeRotationalInertiaRotated * 1e-6;
  kneeRotationalInertia = RY * kneeRotationalInertiaRotated * RY.transpose();  // 245, ..., 248, ..., 6
  Vec3<T> kneeCOM(0, 0, -0.061);
  SpatialInertia<T> kneeInertia(0.158, kneeCOM, kneeRotationalInertia);

  Vec3<T> rotorCOM(0, 0, 0);
  SpatialInertia<T> rotorInertiaX(0.055, rotorCOM, rotorRotationalInertiaX);
  SpatialInertia<T> rotorInertiaY(0.055, rotorCOM, rotorRotationalInertiaY);

  Mat3<T> bodyRotationalInertia;
  bodyRotationalInertia << 16477, 0, 0, 0, 62480, 0, 0, 0, 70325;  /// TODO: check!
  bodyRotationalInertia = bodyRotationalInertia * 1e-6;
  Vec3<T> bodyCOM(0, 0, 0);
  SpatialInertia<T> bodyInertia(cheetah._bodyMass, bodyCOM,
                                bodyRotationalInertia);

  cheetah._abadInertia = abadInertia;
  cheetah._hipInertia = hipInertia;
  cheetah._kneeInertia = kneeInertia;
  cheetah._abadRotorInertia = rotorInertiaX;
  cheetah._hipRotorInertia = rotorInertiaY;
  cheetah._kneeRotorInertia = rotorInertiaY;
  cheetah._bodyInertia = bodyInertia;

  // locations
  cheetah._abadRotorLocation = Vec3<T>(0.14775, 0.049, 0);  /// TODO: check!
  cheetah._abadLocation =
      Vec3<T>(cheetah._bodyLength, cheetah._bodyWidth, 0) * 0.5;
  cheetah._hipLocation = Vec3<T>(0, cheetah._abadLinkLength, 0);  /// TODO: check!
  cheetah._hipRotorLocation = Vec3<T>(0, 0.019, 0);  /// TODO: check!
  cheetah._kneeLocation = Vec3<T>(0, 0, -cheetah._hipLinkLength);  /// TODO: check!
  cheetah._kneeRotorLocation = Vec3<T>(0, 0, 0);

  return cheetah;
}

#endif  // PROJECT_MINICHEETAH_H