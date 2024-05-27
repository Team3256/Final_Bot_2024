// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve.helpers

import edu.wpi.first.math.geometry.Rotation2d

class SwerveModuleConstants
/**
 * Swerve Module Constants to be used when creating swerve modules.
 *
 * @param driveMotorID
 * @param angleMotorID
 * @param canCoderID
 * @param angleOffset
 */
(
    @JvmField val driveMotorID: Int,
    @JvmField val angleMotorID: Int,
    @JvmField val cancoderID: Int,
    @JvmField val angleOffset: Rotation2d
)
