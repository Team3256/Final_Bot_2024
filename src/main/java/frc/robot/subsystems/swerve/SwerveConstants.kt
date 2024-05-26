// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.
package frc.robot.subsystems.swerve

import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.util.Units
import frc.robot.Robot
import frc.robot.TrainingDataPoint
import frc.robot.subsystems.swerve.helpers.COTSTalonFXSwerveConstants
import frc.robot.subsystems.swerve.helpers.SwerveModuleConstants
import kotlin.math.hypot
import kotlin.math.sqrt

class SwerveConstants {
    // We KNOW the robot is a square, so we can assert this. REMOVE THIS IF ROBOT
    // CHANGES!!!
    init {
        assert(wheelBase == trackWidth)
    }

    /* Module Specific Constants */ /* Front Left Module - Module 0 */
    object Mod0 {
        const val driveMotorID: Int = 1
        const val angleMotorID: Int = 8
        const val canCoderID: Int = 2
        val angleOffset: Rotation2d = Rotation2d.fromDegrees(-154.072 + 180)
        val constants: SwerveModuleConstants =
            SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset)
    }

    /* Front Right Module - Module 1 */
    object Mod1 {
        const val driveMotorID: Int = 6
        const val angleMotorID: Int = 7
        const val canCoderID: Int = 9
        val angleOffset: Rotation2d = Rotation2d.fromDegrees(97.470 + 180)
        val constants: SwerveModuleConstants =
            SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset)
    }

    /* Back Left Module - Module 2 */
    object Mod2 {
        const val driveMotorID: Int = 2
        const val angleMotorID: Int = 4
        const val canCoderID: Int = 1
        val angleOffset: Rotation2d = Rotation2d.fromDegrees(-132.85 + 180)
        val constants: SwerveModuleConstants =
            SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset)
    }

    /* Back Right Module - Module 3 */
    object Mod3 {
        const val driveMotorID: Int = 5
        const val angleMotorID: Int = 0
        const val canCoderID: Int = 0
        val angleOffset: Rotation2d = Rotation2d.fromDegrees(-149.45 + 180)
        val constants: SwerveModuleConstants =
            SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset)
    }

    object AzimuthConstants {
        // Quick assignments
        private const val half = 0.5
        private val rootThreeTwo = (sqrt(3.0) / 2)
        private const val zero = 0.0
        private const val one = 1.0
        private const val pi = Math.PI

        // AMP
        const val ampXRed: Double = one
        const val ampXBlue: Double = -one
        const val ampY: Double = zero

        // SPEAKER
        const val subwooferFrontX: Double = zero
        const val subwooferLeftX: Double = -half
        const val subwooferRightX: Double = half
        const val subwooferFrontY: Double = -one
        val subwooferSideY: Double = -rootThreeTwo

        // SOURCE
        val sourceXRed: Double = -rootThreeTwo
        val sourceXBlue: Double = rootThreeTwo
        const val sourceY: Double = half

        // FEEDER
        const val feederXRed: Double = half
        const val feederXBlue: Double = -half
        val feederY: Double = -rootThreeTwo

        /* Angles */ // left is positive
        const val aziAmpRed: Double = 90.0 // (Math.atan2(ampXRed, ampY) * 180 / Math.PI);

        const val aziAmpBlue: Double = -aziAmpRed // (Math.atan2(ampXBlue, ampY) * 180 / Math.PI) + 180;

        const val aziSubwooferFront: Double =
            30.0 // (Math.atan2(subwooferFrontX, subwooferFrontY) * 180 / Math.PI) + 180;

        const val aziSubwooferLeft: Double = -30.0 // (Math.atan2(subwooferLeftX, subwooferSideY) * 180 / pi) + 180;

        const val aziSubwooferRight: Double = 0.0 // (Math.atan2(subwooferRightX, subwooferSideY) * 180 / pi) + 180;

        const val aziSourceRed: Double = 60.0

        // -(Math.atan2(sourceXRed, sourceY) * 180 / pi) + 10;
        const val aziSourceBlue: Double = -aziSourceRed // (Math.atan2(sourceXBlue, sourceY) * 180 / pi);

        const val feederRed: Double = 45.0 // -((Math.atan2(feederXRed, feederY) * 180 / pi) + 130);

        // -(Math.atan2(feederXRed, feederY) * 180 / pi) + 160;
        const val feederBlue: Double = -feederRed // (Math.atan2(feederXBlue, feederY) * 180 / pi) + 180;

        const val cleanUp: Double = 180.0 // (Math.atan2(ampXRed, ampY) * 180 / pi) + 90;

        /* Timeout */
        const val aziCommandTimeOut: Double = 1.5
    }

    companion object {
        // CAN
        const val pigeonID: Int = 24

        // Motor
        val chosenModule: COTSTalonFXSwerveConstants = COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(
            COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L3
        )

        // PID
        val angleKP: Double = chosenModule.angleKP
        val angleKI: Double = chosenModule.angleKI
        val angleKD: Double = chosenModule.angleKD
        const val driveKP: Double = 0.3 // 0.030 *12
        const val driveKI: Double = 0.0
        const val driveKD: Double = 0.0 // 0.00001*12
        const val driveKF: Double = 0.0
        const val aziDrivekP: Double = 0.1 // 0.135;
        const val aziDrivekI: Double = 0.0
        const val aziDrivekD: Double = 0.005 // 0.025; // 0.01 -> 0.005

        const val aDrivekF: Double = 0.00

        // Note drive
        const val noteTargetTurnX: Double = 0.5
        const val noteTargetTurnY: Double = 1.0
        const val noteTargetDriveX: Double = 0.5
        const val noteTargetDriveY: Double = 0.3

        const val noteAngleTolerance: Double = 5.0
        const val noteAngularVelocityTolerance: Double = 2.0
        const val noteDistTolerance: Double = 0.04
        const val noteVelocityTolerance: Double = 0.1

        const val noteDrivekP: Double = 0.7
        const val noteDrivekI: Double = 0.0
        const val noteDrivekD: Double = 0.0
        const val noteDrivekF: Double = 0.1
        const val noteTurnkP: Double = 0.025
        const val noteTurnkI: Double = 0.0
        const val noteTurnkD: Double = 0.0
        const val noteTurnkF: Double = 0.0

        /* Drive Motor Characterization Values From SysID */
        const val driveKS: Double = 0.0
        const val driveKV: Double = 2.08 // 0.13
        const val driveKA: Double = 0.21 // 0

        // Localization
        // Localization will run every [kLocalizationSeconds] seconds.
        const val kVisionSecondsBetweenLocalize: Double = Robot.defaultPeriodSecs * 10

        // Pose Estimation
        val kSwervePoseEstimatorStdData: List<TrainingDataPoint> = java.util.List.of(
            TrainingDataPoint(0.804213, 7.8637125, 3.167025, 0.157388),  // impossible location
            TrainingDataPoint(1.093281, 9.14025, 3.2641875, 0.167762),  // 1 foot
            TrainingDataPoint(1.431827, 4.66685, 2.0580375, 0.130962),  // 2 foot
            TrainingDataPoint(1.743492, 6.7381125, 3.246825, 0.158225),  // 3 foot
            TrainingDataPoint(2.028354, 6.0687, 3.2111875, 0.171350),  // 4 foot
            TrainingDataPoint(2.336431, 6.591775, 3.2745625, 0.166477),  // 5 foot
            TrainingDataPoint(2.665142, 7.11455, 3.798075, 0.167438),  // 6 foot
            TrainingDataPoint(2.953581, 7.4191, 3.9353125, 0.159327),  // 7 foot
            TrainingDataPoint(3.282424, 6.574225, 3.6709625, 0.161348),  // 8 foot
            TrainingDataPoint(3.560149, 6.7152875, 5.2717375, 0.186038),
            TrainingDataPoint(7.128826, 16.0333875, 30.8555875, 0.505372)
        ) // 9 foot

        const val kSwervePoseEstimatorMinValue: Double = 0.804213
        const val kSwervePoseEstimatorMaxValue: Double = 7.128826

        /* Drivetrain Constants */
        val trackWidth: Double = Units.inchesToMeters(20.75)
        val wheelBase: Double = Units.inchesToMeters(20.75)

        /*
   * We calculate the drive base radius using the Pythagorean theorem
   * Assuming we know the sides of the robot, which is square.
   * We can calculate the distance from the center of the robot to the corner.
   *
   *
   * wheelBase
   * <--------->
   * O---------O
   * | |
   * | | trackWidth is the top down distance - from the "O" to the "O"
   * | |
   * | |
   * O---------O
   * <--------->
   *
   * /|
   * / | trackWidth/2
   * / |
   * / |
   * C | driveBaseRadius
   * \ |
   * \ | wheelBase/2
   * \ |
   * \|
   *
   * The line from 'C' to any 'O' represents the hypotenuse of a right-angled
   * triangle
   * formed by 'trackWidth / 2' and 'wheelBase / 2'. Using the Pythagorean
   * theorem, we
   * can calculate the length of the hypotenuse, which is the 'driveBaseRadius'.
   */
        val driveBaseRadius: Double = hypot(trackWidth / 2, wheelBase / 2)
        val wheelCircumference: Double = Units.inchesToMeters(3.869) * Math.PI

        // public static final double LEGACY_wheelCircumference =
        // chosenModule.wheelCircumference
        /*
   * Swerve Kinematics
   * Don't change if using traditional 4 module swerve
   */
        val swerveKinematics: SwerveDriveKinematics = SwerveDriveKinematics(
            Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
        )

        /* Module Gear Ratios */
        val driveGearRatio: Double = chosenModule.driveGearRatio
        val angleGearRatio: Double = chosenModule.angleGearRatio

        /* Motor Inverts */
        val angleMotorInvert: InvertedValue = chosenModule.angleMotorInvert
        val driveMotorInvert: InvertedValue = chosenModule.driveMotorInvert

        /* Angle Encoder Invert */
        val cancoderInvert: SensorDirectionValue = chosenModule.cancoderInvert

        /* Swerve Current Limiting */
        const val angleCurrentLimit: Int = 25
        const val angleCurrentThreshold: Int = 40
        const val angleCurrentThresholdTime: Double = 0.1
        const val angleEnableCurrentLimit: Boolean = true
        const val driveCurrentLimit: Int = 50 // 35
        const val driveCurrentThreshold: Int = 90
        const val driveCurrentThresholdTime: Double = 0.1
        const val driveEnableCurrentLimit: Boolean = true

        /*
   * These values are used by the drive falcon to ramp in open loop and closed
   * loop driving.
   * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
   */
        const val openLoopRamp: Double = 0.25
        const val closedLoopRamp: Double = 0.0

        /* Swerve Profiling Values */
        const val maxTranslationalVelocity: Double = 4.6
        const val slowMaxTranslationalVelocity: Double = maxTranslationalVelocity * 0.2
        const val maxAngularVelocity: Double = 6.0
        const val slowMaxAngularVelocity: Double = maxAngularVelocity * 0.5

        /* Neutral Modes */
        val angleNeutralMode: NeutralModeValue = NeutralModeValue.Brake
        val driveNeutralMode: NeutralModeValue = NeutralModeValue.Brake
        var crosshairAngleKP: Double = 0.15
        var crosshairAngleKI: Double = 0.0
        var crosshairAngleKD: Double = 0.005
        var crosshairTurnTolerance: Double = 5.0
        var crosshairTurnToleranceVel: Double = 5.0

        var translationNoteKP: Double = 0.25
        var translationNoteKI: Double = 0.0
        var translationNoteKD: Double = 0.005
        var translationNoteTolerance: Double = 0.0
        var translationNoteToleranceVel: Double = 5.0

        var strafeNoteKP: Double = 0.2
        var strafeNoteKI: Double = 0.0
        var strafeNoteKD: Double = 0.005
        var strafeNoteTolerance: Double = 0.0
        var strafeNoteToleranceVel: Double = 5.0

        var updateFrequency: Double = 50.0

        const val kPixelToDegreesMagicNumber: Double = 69.0
    }
}
