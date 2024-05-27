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
import frc.robot.helpers.DebugCommandBase
import frc.robot.limelight.LimelightHelpers
import frc.robot.subsystems.swerve.SwerveConstants
import frc.robot.subsystems.swerve.SwerveDrive
import java.util.function.DoubleSupplier
import kotlin.math.abs

class HoldForNote(
    private val swerveSubsystem: SwerveDrive,
    translationAxis: DoubleSupplier,
    openLoop: Boolean
) : DebugCommandBase() {
  private lateinit var translation: Translation2d
  private val openLoop: Boolean

  private val translationAxis: DoubleSupplier

  private var translationPIDOutput = 0.0
  private var strafePIDOutput = 0.0

  private val translationPidController: PIDController
  private val strafePidController: PIDController

  /** Driver control */
  init {
    addRequirements(swerveSubsystem)
    this.translationAxis = translationAxis

    this.openLoop = openLoop
    this.translationPidController =
        PIDController(
            SwerveConstants.translationNoteKP,
            SwerveConstants.translationNoteKI,
            SwerveConstants.translationNoteKD)
    translationPidController.setTolerance(
        SwerveConstants.translationNoteTolerance, SwerveConstants.translationNoteToleranceVel)
    translationPidController.enableContinuousInput(-180.0, 180.0)

    this.strafePidController =
        PIDController(
            SwerveConstants.strafeNoteKP,
            SwerveConstants.strafeNoteKI,
            SwerveConstants.strafeNoteKD)
    strafePidController.setTolerance(
        SwerveConstants.strafeNoteTolerance, SwerveConstants.strafeNoteToleranceVel)
    strafePidController.enableContinuousInput(-180.0, 180.0)
  }

  override fun execute() {
    println("SAM NO MORE TRANSLATION FOR YOU")

    // Obtains axis values for x and y for translation command

    // Safety area, insures that joystick movement will not be tracked within a
    // certain area,
    // prevents unintentional drifting

    // Converts from coordinates to angle, sets joystick forward input as 0,
    // converts angle to
    // degrees

    // PID controller takes current robot position (getYaw) and compares to the
    // azimuth angle to
    // calculate error
    val compensatedMaxVelocity =
        SwerveConstants.maxTranslationalVelocity * abs(translationAxis.asDouble)

    translationPIDOutput =
        translationPidController.calculate(LimelightHelpers.getTY("limelight-note"), -9.0)
    translationPIDOutput =
        MathUtil.clamp(translationPIDOutput, -compensatedMaxVelocity, compensatedMaxVelocity)
    strafePIDOutput =
        translationPidController.calculate(LimelightHelpers.getTX("limelight-note") * -1, 0.0)
    strafePIDOutput =
        MathUtil.clamp(strafePIDOutput, -compensatedMaxVelocity, compensatedMaxVelocity)
    translation = Translation2d(translationPIDOutput, strafePIDOutput)

    swerveSubsystem.drive(translation!!, 0.0, false, openLoop)
  }

  override fun isFinished(): Boolean {
    // return manualRotating.getAsBoolean() || azimuthController.atSetpoint();
    // return rotateToSpeakerController.atSetpoint();
    return false
  }
}
