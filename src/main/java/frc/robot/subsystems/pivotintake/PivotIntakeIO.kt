// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.pivotintake

import org.littletonrobotics.junction.AutoLog

interface PivotIntakeIO {
  @AutoLog
  public open class PivotIntakeIOInputs {
    @JvmField var pivotIntakeMotorVoltage: Double = 0.0
    @JvmField var pivotIntakeMotorPosition: Double = 0.0
    @JvmField var pivotIntakeMotorVelocity: Double = 0.0
    @JvmField var pivotIntakeMotorDegrees: Double = 0.0
    @JvmField var pivotIntakeMotorStatorCurrent: Double = 0.0
    @JvmField var pivotIntakeMotorSupplyCurrent: Double = 0.0
    @JvmField var pivotIntakeMotorTemperature: Double = 0.0
    @JvmField var pivotIntakeMotorReferenceSlope: Double = 0.0
  }

  fun updateInputs(inputs: PivotIntakeIOInputs) {}

  fun setPosition(position: Double) {}

  fun setVoltage(voltage: Double) {}

  fun off() {}

  fun zero() {}
}
