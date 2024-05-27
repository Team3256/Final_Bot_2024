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
import frc.robot.subsystems.swerve.SwerveConstants
import frc.robot.subsystems.swerve.SwerveDrive
import org.littletonrobotics.junction.Logger
import java.util.function.BooleanSupplier
import java.util.function.DoubleSupplier
import kotlin.math.abs

class Azimuth(
    private val swerveSubsystem: SwerveDrive,
    translationAxis: DoubleSupplier,
    strafeAxis: DoubleSupplier,
    setpointAngle: DoubleSupplier,
    manualRotating: BooleanSupplier,
    fieldRelative: Boolean,
    openLoop: Boolean
) : DebugCommandBase() {
    private lateinit var translation: Translation2d
    private val fieldRelative: Boolean
    private val openLoop: Boolean

    private val translationAxis: DoubleSupplier
    private val strafeAxis: DoubleSupplier
    private val setpointAngle: DoubleSupplier
    private val manualRotating: BooleanSupplier
    private val azimuthController: PIDController

    /** Driver control  */
    init {
        addRequirements(swerveSubsystem)

        this.translationAxis = translationAxis
        this.strafeAxis = strafeAxis
        this.setpointAngle = setpointAngle
        this.manualRotating = manualRotating
        this.fieldRelative = fieldRelative
        this.openLoop = openLoop
        this.azimuthController =
            PIDController(SwerveConstants.aziDrivekP, SwerveConstants.aziDrivekI, SwerveConstants.aziDrivekD)
        azimuthController.enableContinuousInput(-180.0, 180.0)
        azimuthController.setTolerance(5.0, 5.0)
    }

    override fun execute() {
        // Obtains axis values for x and y for translation command
        var yAxis = -translationAxis.asDouble
        var xAxis = -strafeAxis.asDouble

        // Safety area, insures that joystick movement will not be tracked within a
        // certain area,
        // prevents unintentional drifting
        yAxis = if ((abs(yAxis) < stickDeadband)) 0.0 else yAxis
        xAxis = if ((abs(xAxis) < stickDeadband)) 0.0 else xAxis

        translation = Translation2d(yAxis, xAxis).times(SwerveConstants.maxTranslationalVelocity)

        // Converts from coordinates to angle, sets joystick forward input as 0,
        // converts angle to
        // degrees
        val azimuthAngle = setpointAngle.asDouble

        // PID controller takes current robot position (getYaw) and compares to the
        // azimuth angle to
        // calculate error
        var rotationPIDOutput =
            azimuthController.calculate(swerveSubsystem.heading!!.degrees, azimuthAngle)

        Logger.recordOutput("rotationOutput", rotationPIDOutput)
        Logger.recordOutput("rotationOutputDeg", Units.radiansToDegrees(rotationPIDOutput))
        Logger.recordOutput("rotationSetpoint", azimuthAngle)

        translation = Translation2d(yAxis, xAxis).times(SwerveConstants.maxTranslationalVelocity)
        rotationPIDOutput =
            MathUtil.clamp(rotationPIDOutput, -SwerveConstants.maxAngularVelocity, SwerveConstants.maxAngularVelocity)

        swerveSubsystem.drive(translation, rotationPIDOutput, fieldRelative, openLoop)
    }

    override fun isFinished(): Boolean {
        // return manualRotating.getAsBoolean() || azimuthController.atSetpoint();
        return azimuthController.atSetpoint()
    }
}
