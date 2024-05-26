// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.pivotshooter

import com.ctre.phoenix6.configs.*
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.trajectory.TrapezoidProfile

object PivotShooterConstants {
  // public static final double kSubWooferPreset = (3.5 + 0.3) / 138.33; // idk if
  // this works
  const val kSubWooferPreset: Double = 3.4 / 138.33 // 3.2
  const val kFeederPreset: Double = 5.9 / 138.33
  const val kAmpPreset: Double = (4) / 138.33
  const val kWingNoteCenterPreset: Double = 5.8 / 138.33
  const val kWingNoteSidePreset: Double =
      5.5 / 138.33 // old value: 5.7 distance: -1.5 //old ish?: 5.4
  const val kWingNoteFarSidePreset: Double = 0 / 138.33
  const val kTrussSourceSidePreset: Double = 6.7 / 138.33 // -10.6875
  const val kHalfWingPodiumPreset: Double = 6.55 / 138.33 // old value: 6.7 distance: -11.5275
  const val kPodiumLeftPreset: Double = 6.5 / 138.33
  const val kPodiumRPreset: Double = 6 / 138.33

  const val kPivotMotorID: Int = 12

  /* PID */
  val kPivotProfileContraints: TrapezoidProfile.Constraints =
      TrapezoidProfile.Constraints(16.0, 16.0)

  /* Tolerance/threshold */
  const val kPivotPositionToleranceDeg: Double = 0.1 // 5deg for the pivot.
  const val kStallVelocityThreshold: Double = 0.1

  /* Physics/geometry */
  const val kPivotMotorGearing: Double = 138.333 // 22 by 1
  const val kPivotLength: Double = 0.2
  const val kPivotMinAngleDeg: Double = -90.0
  const val kPivotMaxAngleDeg: Double = 50.0
  const val kPivotStartingAngleDeg: Double = 0.0
  const val jKgMetersSquared: Double = 0.1 // for sim

  /* Preset */
  const val kPivotSlamIntakeVoltage: Double = -5.0
  const val kPivotSlamShooterVoltage: Double = -2.0

  /* Misc */
  const val kNumPivotMotors: Int = 1
  const val kUseFOC: Boolean = false
  var kUseMotionMagic: Boolean = true // idk
  var updateFrequency: Double = 50.0
  var kPivotSlamStallCurrent: Double = 50.0

  const val kSpeakerAprilTagRed: Int = 4
  const val kSpeakerAprilTagBlue: Int = 0

  const val kSpeakerBackupAprilTagRed: Int = 5
  const val kSpeakerBackupAprilTagBlue: Int = 1

  val motorConfigs: TalonFXConfiguration =
      TalonFXConfiguration()
          .withSlot0(
              Slot0Configs()
                  .withKS(0.0)
                  .withKV(0.05)
                  .withKP(25.0)
                  .withKI(0.0)
                  .withKD(0.0) // Original 0.145
              )
          .withMotorOutput(
              MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withMotionMagic(
              MotionMagicConfigs()
                  .withMotionMagicAcceleration(100.0)
                  .withMotionMagicCruiseVelocity(100.0)
                  .withMotionMagicJerk(420.0))
          .withCurrentLimits(
              CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(60.0))
}
