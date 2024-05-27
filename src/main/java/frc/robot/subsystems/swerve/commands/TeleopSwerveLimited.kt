// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.
package frc.robot.subsystems.swerve.commands

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Translation2d
import frc.robot.Constants
import frc.robot.Constants.FeatureFlags.kQuadraticDrive
import frc.robot.helpers.DebugCommandBase
import frc.robot.subsystems.swerve.SwerveConstants
import frc.robot.subsystems.swerve.SwerveDrive
import frc.robot.subsystems.swerve.SwerveDrive.Companion.joystickToSpeed
import java.util.function.DoubleSupplier

class TeleopSwerveLimited(
    private val swerveDrive: SwerveDrive,
    translationSup: DoubleSupplier,
    strafeSup: DoubleSupplier,
    rotationSup: DoubleSupplier
) : DebugCommandBase() {
    private val translationSup: DoubleSupplier
    private val strafeSup: DoubleSupplier
    private val rotationSup: DoubleSupplier

    init {
        addRequirements(swerveDrive)

        this.translationSup = translationSup
        this.strafeSup = strafeSup
        this.rotationSup = rotationSup
    }

    override fun execute() {
        /* Get Values, Deadband */
        val translationVal =
            MathUtil.applyDeadband(-translationSup.asDouble, Constants.stickDeadband)
        val strafeVal = MathUtil.applyDeadband(-strafeSup.asDouble, Constants.stickDeadband)
        val rotationVal =
            MathUtil.applyDeadband(-rotationSup.asDouble, Constants.stickDeadband)

        /* Drive */
        if (kQuadraticDrive) {
            swerveDrive.drive(
                Translation2d(
                    joystickToSpeed(translationVal, SwerveConstants.slowMaxAngularVelocity),
                    joystickToSpeed(strafeVal, SwerveConstants.slowMaxTranslationalVelocity)
                ),
                joystickToSpeed(rotationVal, SwerveConstants.slowMaxAngularVelocity),
                true,
                true
            )
        } else {
            swerveDrive.drive(
                Translation2d(translationVal, strafeVal)
                    .times(SwerveConstants.slowMaxTranslationalVelocity),
                rotationVal * SwerveConstants.slowMaxAngularVelocity,
                true,
                true
            )
        }
    }
}
