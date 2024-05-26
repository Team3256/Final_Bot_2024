// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.
package frc.robot.autos

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.trajectory.TrapezoidProfile
import frc.robot.subsystems.swerve.SwerveConstants

object AutoConstants {
    // specific robot
    const val kMaxSpeedMetersPerSecond: Double = 4.275
    @JvmField
    val kDriveBaseRadiusMeters: Double = SwerveConstants.driveBaseRadius
    const val kPathGenerationEndGoalVelocity: Double = 0.0
    const val kPathGenerationRotationalDelay: Double = 0.01 // how long it should travel before rotating
    const val kMaxAccelerationMetersPerSecondSquared: Double = 5.0
    const val kMaxAngularSpeedRadiansPerSecond: Double = Math.PI
    const val kMaxAngularSpeedRadiansPerSecondSquared: Double = Math.PI

    @JvmField
    val kRedSpeakerLocation: Pose2d = Pose2d(15.26, 5.56, Rotation2d.fromDegrees(-180.0))

    @JvmField
    val kBlueSpeakerLocation: Pose2d = Pose2d(1.27, 5.56, Rotation2d.fromDegrees(180.0))

    @JvmField
    val kRedAmpLocation: Pose2d = Pose2d(14.74, 7.48, Rotation2d.fromDegrees(90.0))

    @JvmField
    val kBlueAmpLocation: Pose2d = Pose2d(1.81, 7.57, Rotation2d.fromDegrees(90.0))

    const val kSpeakerAlignmentThreshold: Double = 0.2 // meters

    const val kPXController: Double = 1.0
    const val kPYController: Double = 1.0
    const val kPThetaController: Double = 1.0

    /* Constraint for the motion profilied robot angle controller */
    val kThetaControllerConstraints: TrapezoidProfile.Constraints = TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared
    )
}
