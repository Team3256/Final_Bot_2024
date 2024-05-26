// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.ampbar

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue

object AmpBarConstants {
  const val kAmpBarVelocityThreshold: Double = 1.5

  const val kAmpBarCurrentThreshold: Double = 5.0
  const val kAmpBarAmpVoltage: Double = 16.0 // 8
  const val kAmpBarStowVoltage: Double = -16.0 // -4

  const val kAmpBarMotorID: Int = 10
  var kNumAmpBarMotors: Int = 1
  var kAmpBarMotorGearing: Double = 7.0 // i dont think we need this
  var jKgMetersSquared: Double = 0.1
  var kAmpBarLength: Double = 5.0
  var kAmpBarMinAngleDeg: Double = 0.0
  var kAmpBarMaxAngleDeg: Double = 90.0
  var kStallVelocityThreshold: Double = 0.1

  var updateFrequency: Double = 50.0

  val motorConfig: TalonFXConfiguration =
      TalonFXConfiguration()
          .withMotorOutput(
              MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withCurrentLimits(
              CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(20.0))
}
