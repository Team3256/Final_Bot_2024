// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.PIDCommand
import frc.robot.limelight.LimelightHelpers
import frc.robot.subsystems.swerve.SwerveConstants
import frc.robot.subsystems.swerve.SwerveDrive
import java.util.function.DoubleConsumer
import java.util.function.DoubleSupplier

class RotateToSpeaker(swerve: SwerveDrive) :
    PIDCommand(
        PIDController(
            SwerveConstants.crosshairAngleKP,
            SwerveConstants.crosshairAngleKI,
            SwerveConstants.crosshairAngleKD),
        DoubleSupplier { LimelightHelpers.getTX("limelight") },
        0.0,
        DoubleConsumer { output: Double -> swerve.setAngularVelocity(output) },
        swerve) {
  init {
    controller.setTolerance(
        SwerveConstants.crosshairTurnTolerance, SwerveConstants.crosshairTurnToleranceVel)
  }

  override fun isFinished(): Boolean {
    return controller.atSetpoint()
  }
}
