// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.StartEndCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger

class Shooter(private val shooterIO: ShooterIO) : SubsystemBase() {
  private val shooterIOAutoLogged: ShooterIOInputsAutoLogged = ShooterIOInputsAutoLogged()

  override fun periodic() {
    shooterIO.updateInputs(shooterIOAutoLogged)
    Logger.processInputs(this.javaClass.name, shooterIOAutoLogged)
  }

  fun setVoltage(voltage: Double, followerVoltage: Double): Command {
    return StartEndCommand(
        {
          shooterIO.setShooterVoltage(voltage)
          shooterIO.setShooterFollowerVoltage(followerVoltage)
        },
        { shooterIO.off() },
        this)
  }

  fun setVelocity(velocity: Double, followerVelocity: Double): Command {
    return StartEndCommand(
        {
          shooterIO.setShooterVelocity(velocity)
          shooterIO.setShooterFollowerVelocity(followerVelocity)
        },
        { shooterIO.off() },
        this)
  }

  fun off(): Command {
    return StartEndCommand({ shooterIO.off() }, {}, this)
  }
}
