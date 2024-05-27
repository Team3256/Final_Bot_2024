// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve.helpers

import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import edu.wpi.first.math.util.Units

/* Contains values and required settings for common COTS swerve modules. */
class COTSFalconSwerveConstants(
    val wheelDiameter: Double,
    val angleGearRatio: Double,
    val driveGearRatio: Double,
    val angleKP: Double,
    val angleKI: Double,
    val angleKD: Double,
    val driveMotorInvert: InvertedValue,
    val angleMotorInvert: InvertedValue,
    val cancoderInvert: SensorDirectionValue
) {
  val wheelCircumference: Double = wheelDiameter * Math.PI

  /* Drive Gear Ratios for all supported modules */
  object driveGearRatios {
    /* SDS MK3 */
    /** SDS MK3 - 8.16 : 1 */
    const val SDSMK3_Standard: Double = (8.16 / 1.0)

    /** SDS MK3 - 6.86 : 1 */
    const val SDSMK3_Fast: Double = (6.86 / 1.0)

    /* SDS MK4 */
    /** SDS MK4 - 8.14 : 1 */
    const val SDSMK4_L1: Double = (8.14 / 1.0)

    /** SDS MK4 - 6.75 : 1 */
    const val SDSMK4_L2: Double = (6.75 / 1.0)

    /** SDS MK4 - 6.12 : 1 */
    const val SDSMK4_L3: Double = (6.12 / 1.0)

    /** SDS MK4 - 5.14 : 1 */
    const val SDSMK4_L4: Double = (5.14 / 1.0)

    /* SDS MK4i */
    /** SDS MK4i - 8.14 : 1 */
    const val SDSMK4i_L1: Double = (8.14 / 1.0)

    /** SDS MK4i - 6.75 : 1 */
    const val SDSMK4i_L2: Double = (6.75 / 1.0)

    /** SDS MK4i - 6.12 : 1 */
    const val SDSMK4i_L3: Double = (6.12 / 1.0)
  }

companion object {
    /** Swerve Drive Specialties - MK3 Module */
    fun SDSMK3(driveGearRatio: Double): COTSFalconSwerveConstants {
      val wheelDiameter = Units.inchesToMeters(3.86)

      /** 12.8 : 1 */
      val angleGearRatio = (12.8 / 1.0)

      val angleKP = 4.8
      val angleKI = 0.0
      val angleKD = 0.0

      val driveMotorInvert = InvertedValue.CounterClockwise_Positive
      val angleMotorInvert = InvertedValue.CounterClockwise_Positive
      val cancoderInvert = SensorDirectionValue.CounterClockwise_Positive
      return COTSFalconSwerveConstants(
          wheelDiameter,
          angleGearRatio,
          driveGearRatio,
          angleKP,
          angleKI,
          angleKD,
          driveMotorInvert,
          angleMotorInvert,
          cancoderInvert)
    }

    /** Swerve Drive Specialties - MK4 Module */
    fun SDSMK4(driveGearRatio: Double): COTSFalconSwerveConstants {
      val wheelDiameter = Units.inchesToMeters(3.86)

      /** 12.8 : 1 */
      val angleGearRatio = (12.8 / 1.0)

      val angleKP = 4.8
      val angleKI = 0.0
      val angleKD = 0.0

      val driveMotorInvert = InvertedValue.CounterClockwise_Positive
      val angleMotorInvert = InvertedValue.CounterClockwise_Positive
      val cancoderInvert = SensorDirectionValue.CounterClockwise_Positive
      return COTSFalconSwerveConstants(
          wheelDiameter,
          angleGearRatio,
          driveGearRatio,
          angleKP,
          angleKI,
          angleKD,
          driveMotorInvert,
          angleMotorInvert,
          cancoderInvert)
    }

    /** Swerve Drive Specialties - MK4i Module */
    fun SDSMK4i(driveGearRatio: Double): COTSFalconSwerveConstants {
      val wheelDiameter = Units.inchesToMeters(3.86)

      /** (150 / 7) : 1 */
      val angleGearRatio = ((150.0 / 7.0) / 1.0)

      val angleKP = 100.0
      val angleKI = 0.0
      val angleKD = 0.0

      val driveMotorInvert = InvertedValue.CounterClockwise_Positive
      val angleMotorInvert = InvertedValue.Clockwise_Positive
      val cancoderInvert = SensorDirectionValue.CounterClockwise_Positive
      return COTSFalconSwerveConstants(
          wheelDiameter,
          angleGearRatio,
          driveGearRatio,
          angleKP,
          angleKI,
          angleKD,
          driveMotorInvert,
          angleMotorInvert,
          cancoderInvert)
    }
  }
}
