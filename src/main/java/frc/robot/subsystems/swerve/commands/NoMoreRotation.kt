// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve.commands

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.util.Units
import frc.robot.Constants.stickDeadband
import frc.robot.helpers.DebugCommandBase
import frc.robot.limelight.LimelightHelpers
import frc.robot.subsystems.swerve.SwerveConstants
import frc.robot.subsystems.swerve.SwerveDrive
import java.util.function.DoubleSupplier
import kotlin.math.abs
import org.littletonrobotics.junction.Logger

class NoMoreRotation(
    private val swerveSubsystem: SwerveDrive,
    translationAxis: DoubleSupplier,
    strafeAxis: DoubleSupplier,
    fieldRelative: Boolean,
    openLoop: Boolean
) : DebugCommandBase() {
  private lateinit var translation: Translation2d
  private val fieldRelative: Boolean
  private val openLoop: Boolean

  private val translationAxis: DoubleSupplier
  private val strafeAxis: DoubleSupplier
  private val rotateToSpeakerController: PIDController

  /** Driver control */
  init {
    addRequirements(swerveSubsystem)

    this.translationAxis = translationAxis
    this.strafeAxis = strafeAxis
    this.fieldRelative = fieldRelative
    this.openLoop = openLoop
    this.rotateToSpeakerController =
        PIDController(
            SwerveConstants.crosshairAngleKP,
            SwerveConstants.crosshairAngleKI,
            SwerveConstants.crosshairAngleKD)
    rotateToSpeakerController.setTolerance(0.0, SwerveConstants.crosshairTurnToleranceVel)
    rotateToSpeakerController.enableContinuousInput(-180.0, 180.0)
  }

  override fun execute() {
    println("nmr running")
    // Obtains axis values for x and y for translation command
    var yAxis = -translationAxis.asDouble
    var xAxis = -strafeAxis.asDouble

    // Safety area, insures that joystick movement will not be tracked within a
    // certain area,
    // prevents unintentional drifting
    yAxis = if ((abs(yAxis) < stickDeadband)) 0.0 else yAxis
    xAxis = if ((abs(xAxis) < stickDeadband)) 0.0 else xAxis

    translation = Translation2d(yAxis, xAxis).times(SwerveConstants.maxTranslationalVelocity * 0.5)

    // Converts from coordinates to angle, sets joystick forward input as 0,
    // converts angle to
    // degrees

    // PID controller takes current robot position (getYaw) and compares to the
    // azimuth angle to
    // calculate error
    var rotationPIDOutput =
        rotateToSpeakerController.calculate(LimelightHelpers.getTX("limelight") / 2, 0.0)
    println("LimelightTX" + LimelightHelpers.getTX("limelight"))
    println("NMRRotationOutput $rotationPIDOutput")
    //     Units.radiansToDegrees(rotationPIDOutput));
    Logger.recordOutput("NMRrotationOutput", rotationPIDOutput)
    Logger.recordOutput("NMRrotationOutputDeg", Units.radiansToDegrees(rotationPIDOutput))
    Logger.recordOutput("NMRrotationSetpoint", 0.0)

    translation = Translation2d(yAxis, xAxis).times(SwerveConstants.maxTranslationalVelocity)
    rotationPIDOutput =
        MathUtil.clamp(
            rotationPIDOutput,
            -SwerveConstants.maxAngularVelocity,
            SwerveConstants.maxAngularVelocity)

    swerveSubsystem.drive(translation, rotationPIDOutput, fieldRelative, openLoop)
  }

  override fun isFinished(): Boolean {
    // return manualRotating.getAsBoolean() || azimuthController.atSetpoint();
    // return rotateToSpeakerController.atSetpoint();
    return false
  }
}
