// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.
package frc.robot.subsystems.swerve

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import frc.robot.helpers.Conversions
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger

class SwerveModule(private val io: ModuleIO, val moduleNumber: Int) {
    private val inputs: ModuleIOInputsAutoLogged = ModuleIOInputsAutoLogged()

    fun updateInputs() {
        io.updateInputs(inputs)
    }

    fun periodic() {
        Logger.processInputs("Swerve/Module$moduleNumber", inputs)
    }

    fun setDesiredState(desiredState: SwerveModuleState, isOpenLoop: Boolean) {
        var desiredState = desiredState
        desiredState = SwerveModuleState.optimize(desiredState, state.angle)
        io.setAnglePosition(desiredState.angle.rotations)
        setSpeed(desiredState, isOpenLoop)
    }

    private fun setSpeed(desiredState: SwerveModuleState, isOpenLoop: Boolean) {
        io.setDriveVelocity(desiredState.speedMetersPerSecond, isOpenLoop)
    }

    @get:AutoLogOutput
    val state: SwerveModuleState
        get() = SwerveModuleState(
            Conversions.RPSToMPS(inputs.driveMotorVelocity, SwerveConstants.wheelCircumference),
            Rotation2d.fromRotations(inputs.angleMotorPosition)
        )

    @get:AutoLogOutput
    val position: SwerveModulePosition
        get() = SwerveModulePosition(
            Conversions.rotationsToMeters(
                inputs.driveMotorPosition, SwerveConstants.wheelCircumference
            ),
            Rotation2d.fromRotations(inputs.angleMotorPosition)
        )

    fun resetToAbsolute() {
        io.resetToAbsolute()
    }

    val cANcoder: Rotation2d
        get() = io.cANcoder

    fun off() {}
}
