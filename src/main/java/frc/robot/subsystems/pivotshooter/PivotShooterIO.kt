// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.pivotshooter

import org.littletonrobotics.junction.AutoLog

interface PivotShooterIO {
  @AutoLog
  class PivotShooterIOInputs {
    var pivotShooterMotorVoltage: Double = 0.0
    var pivotShooterMotorVelocity: Double = 0.0
    var pivotShooterMotorPosition: Double = 0.0
    var pivotShooterMotorDegrees: Double = 0.0
    var pivotShooterMotorStatorCurrent: Double = 0.0
    var pivotShooterMotorSupplyCurrent: Double = 0.0
    var pivotShooterMotorTemperature: Double = 0.0
    var pivotShooterMotorReferenceSlope: Double = 0.0
  }

  fun updateInputs(inputs: PivotShooterIOInputs) {}

  fun setPosition(position: Double) {}

  fun setVoltage(voltage: Double) {}

  fun off() {}

  fun zero() {}
}
