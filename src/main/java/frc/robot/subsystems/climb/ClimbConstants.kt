// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.
package frc.robot.subsystems.climb

import com.ctre.phoenix6.configs.*
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue

object ClimbConstants {
    const val kLeftClimbMotorID: Int = 18
    const val gearRatio: Double = 20.0 // needs to be tuned

    const val kClimbUpPosition: Double = (150 / 20).toDouble()

    const val kClimbDownPosition: Double = 0.0
    const val wheelRadius: Double = 1.0

    var kCurrentThreshold: Double = 4.5

    // // What about the tension from the spring?
    // public static double gyroRollStableThreshold = 1; // 1 degree of error is
    // tolerated
    var updateFrequency: Double = 0.0
    var kUseMotionMagic: Boolean = false

    val motorConfig: TalonFXConfiguration = TalonFXConfiguration()
        .withSlot0(Slot0Configs().withKS(0.0).withKV(0.0).withKP(1.0).withKI(0.0).withKD(0.0))
        .withMotorOutput(
            MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive)
        )
        .withMotionMagic(
            MotionMagicConfigs()
                .withMotionMagicAcceleration(100.0)
                .withMotionMagicCruiseVelocity(100.0)
                .withMotionMagicJerk(300.0)
        )
        .withCurrentLimits(
            CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(60.0)
        )
}
