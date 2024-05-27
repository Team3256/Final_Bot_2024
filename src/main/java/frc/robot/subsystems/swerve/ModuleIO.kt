// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.
package frc.robot.subsystems.swerve

import edu.wpi.first.math.geometry.Rotation2d
import org.littletonrobotics.junction.AutoLog

interface ModuleIO {
    @AutoLog
    open class ModuleIOInputs {
        @JvmField var driveMotorVoltage: Double = 0.0
        @JvmField var driveMotorVelocity: Double = 0.0
        @JvmField var driveMotorPosition: Double = 0.0
        @JvmField var driveMotorStatorCurrent: Double = 0.0
        @JvmField var driveMotorSupplyCurrent: Double = 0.0
        @JvmField var driveMotorTemperature: Double = 0.0
        @JvmField var driveMotorReferenceSlope: Double = 0.0

        @JvmField var absolutePosition: Rotation2d = Rotation2d()

        @JvmField var angleMotorVoltage: Double = 0.0
        @JvmField var angleMotorPosition: Double = 0.0
        @JvmField var angleMotorStatorCurrent: Double = 0.0
        @JvmField var angleMotorSupplyCurrent: Double = 0.0
        @JvmField var angleMotorTemperature: Double = 0.0
        @JvmField var angleMotorReferenceSlope: Double = 0.0
    }

    fun updateInputs(inputs: ModuleIOInputs) {}

    fun setDriveVelocity(velocity: Double, isOpenLoop: Boolean) {}

    fun setAnglePosition(position: Double) {}

    fun resetToAbsolute() {}

    val cANcoder: Rotation2d
        get() = Rotation2d()
}
