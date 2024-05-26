// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.
package frc.robot.subsystems.intake

import org.littletonrobotics.junction.AutoLog

interface IntakeIO {
    @AutoLog
    class IntakeIOInputs {
        @JvmField
        var intakeMotorVoltage: Double = 0.0
        @JvmField
        var intakeMotorVelocity: Double = 0.0
        @JvmField
        var intakeMotorStatorCurrent: Double = 0.0
        @JvmField
        var intakeMotorSupplyCurrent: Double = 0.0
        @JvmField
        var intakeMotorTemperature: Double = 0.0
        @JvmField
        var intakeMotorReferenceSlope: Double = 0.0

        @JvmField
        var passthroughMotorVoltage: Double = 0.0
        @JvmField
        var passthroughMotorVelocity: Double = 0.0
        @JvmField
        var passthroughMotorStatorCurrent: Double = 0.0
        @JvmField
        var passthroughMotorSupplyCurrent: Double = 0.0
        @JvmField
        var passthroughMotorTemperature: Double = 0.0
        @JvmField
        var passthroughMotorReferenceSlope: Double = 0.0

        @JvmField
        var isBeamBroken: Boolean = false
    }

    fun updateInputs(inputs: IntakeIOInputs?) {}

    fun setIntakeVoltage(voltage: Double) {}

    fun setIntakeVelocity(velocity: Double) {}

    fun setPassthroughVoltage(voltage: Double) {}

    fun setPassthroughVelocity(velocity: Double) {}

    fun off() {}

    val isBeamBroken: Boolean
        get() = false
}
