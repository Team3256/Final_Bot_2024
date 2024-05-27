// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.ampbar

import org.littletonrobotics.junction.AutoLog

interface AmpBarIO {
  @AutoLog
  open class AmpBarIOInputs {
    @JvmField var ampBarMotorVoltage: Double = 0.0
    @JvmField var ampBarMotorPosition: Double = 0.0
    @JvmField var ampBarMotorVelocity: Double = 0.0
    @JvmField var ampBarMotorStatorCurrent: Double = 0.0
    @JvmField var ampBarMotorSupplyCurrent: Double = 0.0
    @JvmField var ampBarMotorTemperature: Double = 0.0
  }

  fun updateInputs(inputs: AmpBarIOInputs) {}

  fun setVoltage(voltage: Double) {}

  public val isCurrentSpiking: Boolean;

  fun off() {}
}
