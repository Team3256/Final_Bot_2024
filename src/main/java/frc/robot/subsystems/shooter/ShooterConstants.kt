// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter

import com.ctre.phoenix6.configs.*
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue

object ShooterConstants {
  /* Misc */
  const val kUseShooterMotionMagic: Boolean = false
  const val kUseFOC: Boolean = true

  /* CAN */
  @JvmField var kShooterMotorID: Int = 11
  @JvmField var kShooterMotorFollowerID: Int = 23

  /* PID */
  // Shooter
  var motorOutputConfigs: MotorOutputConfigs =
      MotorOutputConfigs()
          .withNeutralMode(NeutralModeValue.Brake)
          .withInverted(InvertedValue.Clockwise_Positive)
  @JvmField
  var motorConfigs: TalonFXConfiguration =
      TalonFXConfiguration()
          .withSlot0(
              Slot0Configs()
                  .withKS(0.0)
                  .withKV(0.39) // Original 0.145
                  // .withKA(1.48)// Original 0 only for feedforward, might not use
                  .withKP(0.4)
                  .withKI(0.0)
                  .withKD(0.0))
          .withMotorOutput(motorOutputConfigs)
          .withMotionMagic(
              MotionMagicConfigs()
                  .withMotionMagicAcceleration(100.0)
                  .withMotionMagicCruiseVelocity(300.0)
                  .withMotionMagicJerk(1600.0))
          .withCurrentLimits(
              CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(60.0))
  @JvmField
  var followerMotorConfigs: TalonFXConfiguration =
      motorConfigs.withMotorOutput(
          motorOutputConfigs.withInverted(InvertedValue.CounterClockwise_Positive))

  var kShooterSpeakerRPS: Double = 42.0
  var kShooterFollowerSpeakerRPS: Double = 45.0 // really 80

  @JvmField var kShooterSubwooferRPS: Double = 60.0
  @JvmField var kShooterFollowerSubwooferRPS: Double = 70.0

  @JvmField var kShooterAmpRPS: Double = 22.5 // BEFORE: 1200/60
  @JvmField var kShooterFollowerAmpRPS: Double = 22.5

  var kShooterFeederRPS: Double = 42.0
  var kShooterFollowerFeederRPS: Double = 45.0

  /* Misc */
  var kShooterAngle: Double = 10.0 // The fixed angle for the shooter (in degrees)

  // before: 1800/6
  @JvmField var updateFrequency: Double = 50.0
  @JvmField var kUseMotionMagic: Boolean = false

  var neutralMode: NeutralModeValue = NeutralModeValue.Brake
  var shooterInverted: InvertedValue = InvertedValue.Clockwise_Positive
  var shooterFollowerInverted: InvertedValue = InvertedValue.CounterClockwise_Positive
}
