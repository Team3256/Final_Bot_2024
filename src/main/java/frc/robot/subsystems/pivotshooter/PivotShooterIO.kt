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
  public open class PivotShooterIOInputs {
    @JvmField var pivotShooterMotorVoltage: Double = 0.0
    @JvmField var pivotShooterMotorVelocity: Double = 0.0
    @JvmField var pivotShooterMotorPosition: Double = 0.0
    @JvmField var pivotShooterMotorDegrees: Double = 0.0
    @JvmField var pivotShooterMotorStatorCurrent: Double = 0.0
    @JvmField var pivotShooterMotorSupplyCurrent: Double = 0.0
    @JvmField var pivotShooterMotorTemperature: Double = 0.0
    @JvmField var pivotShooterMotorReferenceSlope: Double = 0.0
  }

  fun updateInputs(inputs: PivotShooterIOInputs) {}

  fun setPosition(position: Double) {}

  fun setVoltage(voltage: Double) {}

  fun off() {}

  fun zero() {}
}
