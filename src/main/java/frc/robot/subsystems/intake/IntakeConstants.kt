// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake

import com.ctre.phoenix6.configs.*
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue

object IntakeConstants {
  /* CAN */
  const val kIntakeMotorID: Int = 33

  const val kPassthroughIntakeVoltage: Double = -8.0
  const val kIntakeIntakeVoltage: Double = 12.0

  // Time of Flight constants
  const val kBeamBreakDelayTime: Double = 0.0

  const val kIntakeBeamBreakDIO: Int = 1
  @JvmField var kPassthroughMotorID: Int = 35

  @JvmField var updateFrequency: Double = 50.0
  @JvmField var kIntakeMotionMagic: Boolean = false
  @JvmField var kPassthroughMotionMagic: Boolean = false

  @JvmField
  val intakeMotorConfig: TalonFXConfiguration =
      TalonFXConfiguration()
          .withSlot0(Slot0Configs().withKS(0.0).withKV(0.1).withKP(1.0).withKI(0.0).withKD(0.0))
          .withMotorOutput(
              MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withMotionMagic(
              MotionMagicConfigs()
                  .withMotionMagicAcceleration(120.0)
                  .withMotionMagicCruiseVelocity(60.0)
                  .withMotionMagicJerk(1200.0))
          .withCurrentLimits(
              CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(80.0))
  @JvmField
  val passthroughMotorConfig: TalonFXConfiguration =
      TalonFXConfiguration()
          .withSlot0(Slot0Configs().withKS(0.0).withKV(0.0).withKP(1.0).withKI(0.0).withKD(0.0))
          .withMotorOutput(
              MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withMotionMagic(
              MotionMagicConfigs()
                  .withMotionMagicAcceleration(120.0)
                  .withMotionMagicCruiseVelocity(60.0)
                  .withMotionMagicJerk(1200.0))
          .withCurrentLimits(
              CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(80.0))
}
