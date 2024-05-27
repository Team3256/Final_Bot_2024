// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.ampbar

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.StartEndCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger

class AmpBar(private val ampBarIO: AmpBarIO) : SubsystemBase() {
  private val ampBarIOAutoLogged: AmpBarIOInputsAutoLogged = AmpBarIOInputsAutoLogged()

  override fun periodic() {
    ampBarIO.updateInputs(ampBarIOAutoLogged)
    Logger.processInputs(this.javaClass.name, ampBarIOAutoLogged)
  }

  fun setVoltage(voltage: Double): Command {
    return StartEndCommand({ ampBarIO.setVoltage(voltage) }, { ampBarIO.off() }, this)
  }

  fun setAmpPosition(): Command {
    return object : Command() {
      override fun initialize() {
        ampBarIO.setVoltage(AmpBarConstants.kAmpBarAmpVoltage)
      }

      override fun end(interrupted: Boolean) {
        ampBarIO.off()
      }

      override fun isFinished(): Boolean {
        return ampBarIO.isCurrentSpiking
      }
    }
  }

  fun setStowPosition(): Command {
    return object : Command() {
      override fun initialize() {
        ampBarIO.setVoltage(AmpBarConstants.kAmpBarStowVoltage)
      }

      override fun end(interrupted: Boolean) {
        ampBarIO.off()
      }

      override fun isFinished(): Boolean {
        return ampBarIO.isCurrentSpiking
      }
    }
  }

  fun off(): Command {
    return StartEndCommand({ ampBarIO.off() }, {}, this)
  }
}
