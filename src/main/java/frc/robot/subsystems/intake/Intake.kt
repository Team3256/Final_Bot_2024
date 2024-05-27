// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.StartEndCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.helpers.TimedBoolean
import org.littletonrobotics.junction.Logger

class Intake(private val intakeIO: IntakeIO) : SubsystemBase() {
  private val intakeIOAutoLogged: IntakeIOInputsAutoLogged = IntakeIOInputsAutoLogged()

  override fun periodic() {
    intakeIO.updateInputs(intakeIOAutoLogged)
    Logger.processInputs(this.javaClass.name, intakeIOAutoLogged)
  }

  fun setVoltage(voltage: Double, passthroughVoltage: Double): Command {
    return StartEndCommand(
        {
          intakeIO.setIntakeVoltage(voltage)
          intakeIO.setPassthroughVoltage(passthroughVoltage)
        },
        { intakeIO.off() },
        this)
  }

  fun setVelocity(velocity: Double, passthroughVelocity: Double): Command {
    return StartEndCommand(
        {
          intakeIO.setIntakeVelocity(velocity)
          intakeIO.setPassthroughVelocity(passthroughVelocity)
        },
        { intakeIO.off() },
        this)
  }

  fun setIntakeVoltage(voltage: Double): Command {
    return StartEndCommand({ intakeIO.setIntakeVoltage(voltage) }, { intakeIO.off() }, this)
  }

  fun setIntakeVelocity(velocity: Double): Command {
    return StartEndCommand({ intakeIO.setIntakeVelocity(velocity) }, { intakeIO.off() }, this)
  }

  fun setPassthroughVoltage(voltage: Double): Command {
    return StartEndCommand({ intakeIO.setPassthroughVoltage(voltage) }, { intakeIO.off() }, this)
  }

  fun setPassthroughVelocity(velocity: Double): Command {
    return StartEndCommand({ intakeIO.setPassthroughVelocity(velocity) }, { intakeIO.off() }, this)
  }

  fun off(): Command {
    return StartEndCommand({ intakeIO.off() }, {}, this)
  }

  fun intakeIn(): Command {
    return object : Command() {
      var beamBreak: TimedBoolean =
          TimedBoolean({ intakeIO.isBeamBroken }, IntakeConstants.kBeamBreakDelayTime)

      override fun initialize() {
        intakeIO.setIntakeVoltage(IntakeConstants.kIntakeIntakeVoltage)
        intakeIO.setPassthroughVoltage(IntakeConstants.kPassthroughIntakeVoltage)
      }

      override fun execute() {
        beamBreak.update()
      }

      override fun isFinished(): Boolean {
        return beamBreak.hasBeenTrueForThreshold()
      }
    }
  }

  val isBeamBroken: Boolean
    get() = intakeIO.isBeamBroken
}
