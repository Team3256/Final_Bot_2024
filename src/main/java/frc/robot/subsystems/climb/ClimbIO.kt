// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.
package frc.robot.subsystems.climb

import org.littletonrobotics.junction.AutoLog

interface ClimbIO {
    @AutoLog
    class ClimbIOInputs {
        var climbMotorVoltage: Double = 0.0
        var climbMotorPosition: Double = 0.0
        var climbMotorVelocity: Double = 0.0
        var climbMotorStatorCurrent: Double = 0.0
        var climbMotorSupplyCurrent: Double = 0.0
        var climbMotorTemperature: Double = 0.0
        var climbMotorReferenceSlope: Double = 0.0
    }

    fun updateInputs(inputs: ClimbIOInputs?) {}

    fun setPosition(position: Double) {}

    fun setVoltage(voltage: Double) {}

    fun off() {}

    fun zero() {}
}
