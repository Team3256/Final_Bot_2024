// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter

import org.littletonrobotics.junction.AutoLog

interface ShooterIO {
  @AutoLog
  public open class ShooterIOInputs {
    @JvmField var shooterMotorVoltage: Double = 0.0
    @JvmField var shooterMotorVelocity: Double = 0.0
    @JvmField var shooterMotorStatorCurrent: Double = 0.0
    @JvmField var shooterMotorSupplyCurrent: Double = 0.0
    @JvmField var shooterMotorTemperature: Double = 0.0
    @JvmField var shooterMotorReferenceSlope: Double = 0.0

    @JvmField var shooterMotorFollowerVoltage: Double = 0.0
    @JvmField var shooterMotorFollowerVelocity: Double = 0.0
    @JvmField var shooterMotorFollowerStatorCurrent: Double = 0.0
    @JvmField var shooterMotorFollowerSupplyCurrent: Double = 0.0
    @JvmField var shooterMotorFollowerTemperature: Double = 0.0
    @JvmField var shooterMotorFollowerReferenceSlope: Double = 0.0
  }

  fun updateInputs(inputs: ShooterIOInputs) {}

  fun setShooterVoltage(voltage: Double) {}

  fun setShooterVelocity(velocity: Double) {}

  fun setShooterFollowerVoltage(voltage: Double) {}

  fun setShooterFollowerVelocity(velocity: Double) {}

  fun off() {}
}
