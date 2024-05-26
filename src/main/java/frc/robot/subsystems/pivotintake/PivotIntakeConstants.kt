// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.pivotintake

import com.ctre.phoenix6.configs.*
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.trajectory.TrapezoidProfile

object PivotIntakeConstants {
  /* CAN */
  const val kPivotMotorID: Int = 34

  /* PID */
  val kPivotProfileContraints: TrapezoidProfile.Constraints =
      TrapezoidProfile.Constraints(16.0, 16.0)

  /* Tolerance/threshold */
  const val kCurrentThreshold: Double = 10.0
  const val kPivotPositionToleranceDeg: Double = 1.0 // 5deg for the pivot.
  const val kStallVelocityThreshold: Double = 0.1

  /* Physics/geometry */
  const val kPivotMotorGearing: Double = 36.0 // 22 by 1
  const val kPivotGroundPos: Double = -5.6 / 16
  const val kPivotLength: Double = 0.2
  const val kPivotMinAngleDeg: Double = -90.0
  const val kPivotMaxAngleDeg: Double = 50.0
  const val jKgMetersSquared: Double = 0.1 // for sim

  /* Preset */
  const val kPivotSlamIntakeVoltage: Double = -5.0
  const val kPivotSlamShooterVoltage: Double = 4.0

  /* Misc */
  const val kUseFOC: Boolean = false
  var kUseMotionMagic: Boolean = false

  var updateFrequency: Double = 50.0

  var kPivotSlamStallCurrent: Double = 10.0

  val motorConfig: TalonFXConfiguration =
      TalonFXConfiguration()
          .withSlot0(Slot0Configs().withKS(0.0).withKV(0.05).withKP(1.0).withKI(0.0).withKD(0.0))
          .withMotorOutput(
              MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withMotionMagic(
              MotionMagicConfigs()
                  .withMotionMagicAcceleration(20.0)
                  .withMotionMagicCruiseVelocity(10.0)
                  .withMotionMagicJerk(100.0))
          .withCurrentLimits(
              CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(60.0))
}
