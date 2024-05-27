// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot

object Constants {
  // We do not include the AdvKit in the main FeatureFlags class - since this is
  // in Robot.java and we'd prefer not to
  // have things break.
  /* FeatureFlags that are used in Robot.java */
  const val kEnableAdvKit: Boolean = true
  const val kEnableOBlog: Boolean = false
  const val kDisableSubsystemsOnDisableInit: Boolean = true

  // CAREFUL!
  const val kOverrideBrownOutVoltage: Boolean = true
  const val kOverridenBrownOutVoltage: Double = 5.6

  /* General robot configruation */
  const val stickDeadband: Double = 0.1
  const val azimuthStickDeadband: Double = 0.3

  // Logging
  const val kLogLinesBeforeFlush: Int = 100

  object FeatureFlags {
    // subsystems
    const val kEasterEggEnabled: Boolean = false

    const val kIntakeEnabled: Boolean = true

    const val kShooterEnabled: Boolean = true

    const val kSwerveEnabled: Boolean = true

    const val kPivotIntakeEnabled: Boolean = true

    const val kClimbEnabled: Boolean = true
    const val kLEDEnabled: Boolean = true

    const val kAmpBarEnabled: Boolean = true

    // logging
    const val kDebugEnabled: Boolean = false
    const val DebugCommandEnabled: Boolean = false
    val kRobotVizEnabled: Boolean = true && !Robot.isReal()

    // features
    const val kAutoAlignEnabled: Boolean = false

    const val kLocalizationEnabled: Boolean = false

    // public static final boolean kSwerveAccelerationLimitingEnabled = true;
    const val kSwerveUseVisionForPoseEst: Boolean = false // ummm probably not disabling this
    const val kLocalizationDataCollectionMode: Boolean = false
    const val kLocalizationStdDistanceBased: Boolean = false

    // Make sure the pose is CORRECT WITHOUT VISION BEFORE ENAVLINGTHIS!!!!
    const val kSwerveVelocityLimitingEnabled: Boolean = false
    const val kIntakeAutoScoreDistanceSensorOffset: Boolean = false
    const val kShuffleboardLayoutEnabled: Boolean = true
    const val kGamePieceDetection: Boolean = false
    const val kUsePrefs: Boolean = true

    const val kPitRoutineEnabled: Boolean = false

    const val kCanTestEnabled: Boolean = false
    const val kResetHeadingOnZeroGyro: Boolean = true
    const val kQuadraticDrive: Boolean = false
    @JvmField var kPivotShooterEnabled: Boolean = true
  }

  object ShuffleboardConstants {
    const val kDriverTabName: String = "Driver"
    const val kOperatorTabName: String = "Operator"
    const val kIntakeLayoutName: String = "Intake"
    const val kSwerveLayoutName: String = "Swerve"
    const val kArmLayoutName: String = "Arm"
    const val kShooterLayoutName: String = "Shooter"
  }
}
