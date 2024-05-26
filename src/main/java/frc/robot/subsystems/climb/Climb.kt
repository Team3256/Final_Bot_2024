// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.climb

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.StartEndCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger

class Climb(private val climbIO: ClimbIO) : SubsystemBase() {
  private val climbIOAutoLogged: ClimbIOInputsAutoLogged = ClimbIOInputsAutoLogged()

  override fun periodic() {
    climbIO.updateInputs(climbIOAutoLogged)
    Logger.processInputs(this.javaClass.name, climbIOAutoLogged)
  }

  fun setPosition(position: Double): Command {
    return StartEndCommand({ climbIO.setPosition(position * ClimbConstants.gearRatio) }, {}, this)
  }

  fun setVoltage(voltage: Double): Command {
    return StartEndCommand({ climbIO.setVoltage(voltage) }, { climbIO.setVoltage(0.0) }, this)
  }

  fun off(): Command {
    return StartEndCommand({ climbIO.off() }, {}, this)
  }

  fun zero(): Command {
    return StartEndCommand({ climbIO.zero() }, {}, this)
  }

  fun extendClimber(): Command {
    return setPosition(ClimbConstants.kClimbUpPosition)
  }

  fun retractClimber(): Command {
    return setPosition(ClimbConstants.kClimbDownPosition)
  }
}
