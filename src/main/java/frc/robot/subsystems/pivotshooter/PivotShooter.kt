// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.pivotshooter

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap
import edu.wpi.first.wpilibj2.command.*
import frc.robot.subsystems.vision.Vision
import org.littletonrobotics.junction.Logger

class PivotShooter(private val pivotShooterIO: PivotShooterIO) : SubsystemBase() {
  private val pivotShooterIOAutoLogged: PivotShooterIOInputs =
      PivotShooterIOInputs()

  private val aprilTagMap: InterpolatingDoubleTreeMap =
      object : InterpolatingDoubleTreeMap() {
        init {
          put(0.0, 0.0)
          put(1.0, 1.0)
        }
      }

  override fun periodic() {
    pivotShooterIO.updateInputs(pivotShooterIOAutoLogged)
    Logger.processInputs(this.javaClass.name, pivotShooterIOAutoLogged)
  }

  fun setPosition(position: Double): Command {
    return StartEndCommand(
        { pivotShooterIO.setPosition(position * PivotShooterConstants.kPivotMotorGearing) },
        {},
        this)
  }

  fun setVoltage(voltage: Double): Command {
    return StartEndCommand(
        { pivotShooterIO.setVoltage(voltage) }, { pivotShooterIO.setVoltage(0.0) }, this)
  }

  fun off(): Command {
    return StartEndCommand({ pivotShooterIO.off() }, {}, this)
  }

  fun slamZero(): Command {
    return object : Command() {
      override fun initialize() {
        pivotShooterIO.setVoltage(PivotShooterConstants.kPivotSlamShooterVoltage)
      }

      override fun end(interrupted: Boolean) {
        pivotShooterIO.off()
        if (!interrupted) {
          pivotShooterIO.zero()
        }
      }

      override fun isFinished(): Boolean {
        return (pivotShooterIOAutoLogged.pivotShooterMotorStatorCurrent >
            PivotShooterConstants.kPivotSlamStallCurrent)
      }
    }
  }

  fun slamAndPID(): Command {
    return SequentialCommandGroup(this.setPosition(0.0), this.slamZero())
  }

  fun zero(): Command {
    return StartEndCommand({ pivotShooterIO.zero() }, {}, this)
  }

  fun bruh(vision: Vision): Command {
    return RunCommand(
        {
          pivotShooterIO.setPosition(
              aprilTagMap[
                  vision.lastCenterLimelightY - vision.lastLastCenterLimelightY +
                      vision.centerLimelightY] * PivotShooterConstants.kPivotMotorGearing)
        },
        this)
  }
}
