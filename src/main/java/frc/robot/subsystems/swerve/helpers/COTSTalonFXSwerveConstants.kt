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
class COTSTalonFXSwerveConstants(
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

    /** West Coast Products  */
    class WCP {
        /** West Coast Products - SwerveX Standard  */
        object SwerveXStandard {
            /** West Coast Products - SwerveX Standard (Falcon 500)  */
            fun Falcon500(driveGearRatio: Double): COTSTalonFXSwerveConstants {
                val wheelDiameter = Units.inchesToMeters(3.869)

                /** (396 / 35) : 1  */
                val angleGearRatio = ((396.0 / 35.0) / 1.0)

                val angleKP = 1.0 // old: 1
                val angleKI = 0.0
                val angleKD = 0.0

                val driveMotorInvert = InvertedValue.CounterClockwise_Positive
                val angleMotorInvert = InvertedValue.Clockwise_Positive
                val cancoderInvert = SensorDirectionValue.CounterClockwise_Positive
                return COTSTalonFXSwerveConstants(
                    wheelDiameter,
                    angleGearRatio,
                    driveGearRatio,
                    angleKP,
                    angleKI,
                    angleKD,
                    driveMotorInvert,
                    angleMotorInvert,
                    cancoderInvert
                )
            }

            /** West Coast Products - SwerveX Standard (Kraken X60)  */
            fun KrakenX60(driveGearRatio: Double): COTSTalonFXSwerveConstants {
                val wheelDiameter = Units.inchesToMeters(4.0)

                /** (396 / 35) : 1  */
                val angleGearRatio = ((396.0 / 35.0) / 1.0)

                val angleKP = 1.0
                val angleKI = 0.0
                val angleKD = 0.0

                val driveMotorInvert = InvertedValue.CounterClockwise_Positive
                val angleMotorInvert = InvertedValue.Clockwise_Positive
                val cancoderInvert = SensorDirectionValue.CounterClockwise_Positive
                return COTSTalonFXSwerveConstants(
                    wheelDiameter,
                    angleGearRatio,
                    driveGearRatio,
                    angleKP,
                    angleKI,
                    angleKD,
                    driveMotorInvert,
                    angleMotorInvert,
                    cancoderInvert
                )
            }

            object driveRatios {
                /** WCP SwerveX Standard X1 - 10 Tooth - (7.85 : 1)  */
                const val X1_10: Double = (7.85 / 1.0)

                /** WCP SwerveX Standard X1 - 11 Tooth - (7.13 : 1)  */
                const val X1_11: Double = (7.13 / 1.0)

                /** WCP SwerveX Standard X1 - 12 Tooth - (6.54 : 1)  */
                const val X1_12: Double = (6.54 / 1.0)

                /** WCP SwerveX Standard X2 - 10 Tooth - (6.56 : 1)  */
                const val X2_10: Double = (6.56 / 1.0)

                /** WCP SwerveX Standard X2 - 11 Tooth - (5.96 : 1)  */
                const val X2_11: Double = (5.96 / 1.0)

                /** WCP SwerveX Standard X2 - 12 Tooth - (5.46 : 1)  */
                const val X2_12: Double = (5.46 / 1.0)

                /** WCP SwerveX Standard X3 - 12 Tooth - (5.14 : 1)  */
                const val X3_12: Double = (5.14 / 1.0)

                /** WCP SwerveX Standard X3 - 13 Tooth - (4.75 : 1)  */
                const val X3_13: Double = (4.75 / 1.0)

                /** WCP SwerveX Standard X3 - 14 Tooth - (4.41 : 1)  */
                const val X3_14: Double = (4.41 / 1.0)
            }
        }

        /** West Coast Products - SwerveX Flipped  */
        object SwerveXFlipped {
            /** West Coast Products - SwerveX Flipped (Falcon 500)  */
            fun Falcon500(driveGearRatio: Double): COTSTalonFXSwerveConstants {
                val wheelDiameter = Units.inchesToMeters(4.0)

                /** (468 / 35) : 1  */
                val angleGearRatio = ((468.0 / 35.0) / 1.0)

                val angleKP = 1.0
                val angleKI = 0.0
                val angleKD = 0.0

                val driveMotorInvert = InvertedValue.CounterClockwise_Positive
                val angleMotorInvert = InvertedValue.Clockwise_Positive
                val cancoderInvert = SensorDirectionValue.CounterClockwise_Positive
                return COTSTalonFXSwerveConstants(
                    wheelDiameter,
                    angleGearRatio,
                    driveGearRatio,
                    angleKP,
                    angleKI,
                    angleKD,
                    driveMotorInvert,
                    angleMotorInvert,
                    cancoderInvert
                )
            }

            /** West Coast Products - SwerveX Flipped (Kraken X60)  */
            fun KrakenX60(driveGearRatio: Double): COTSTalonFXSwerveConstants {
                val wheelDiameter = Units.inchesToMeters(4.0)

                /** (468 / 35) : 1  */
                val angleGearRatio = ((468.0 / 35.0) / 1.0)

                val angleKP = 1.0
                val angleKI = 0.0
                val angleKD = 0.0

                val driveMotorInvert = InvertedValue.CounterClockwise_Positive
                val angleMotorInvert = InvertedValue.Clockwise_Positive
                val cancoderInvert = SensorDirectionValue.CounterClockwise_Positive
                return COTSTalonFXSwerveConstants(
                    wheelDiameter,
                    angleGearRatio,
                    driveGearRatio,
                    angleKP,
                    angleKI,
                    angleKD,
                    driveMotorInvert,
                    angleMotorInvert,
                    cancoderInvert
                )
            }

            object driveRatios {
                /** WCP SwerveX Flipped X1 - 10 Tooth - (8.10 : 1)  */
                const val X1_10: Double = (8.10 / 1.0)

                /** WCP SwerveX Flipped X1 - 11 Tooth - (7.36 : 1)  */
                const val X1_11: Double = (7.36 / 1.0)

                /** WCP SwerveX Flipped X1 - 12 Tooth - (6.75 : 1)  */
                const val X1_12: Double = (6.75 / 1.0)

                /** WCP SwerveX Flipped X2 - 10 Tooth - (6.72 : 1)  */
                const val X2_10: Double = (6.72 / 1.0)

                /** WCP SwerveX Flipped X2 - 11 Tooth - (6.11 : 1)  */
                const val X2_11: Double = (6.11 / 1.0)

                /** WCP SwerveX Flipped X2 - 12 Tooth - (5.60 : 1)  */
                const val X2_12: Double = (5.60 / 1.0)

                /** WCP SwerveX Flipped X3 - 10 Tooth - (5.51 : 1)  */
                const val X3_10: Double = (5.51 / 1.0)

                /** WCP SwerveX Flipped X3 - 11 Tooth - (5.01 : 1)  */
                const val X3_11: Double = (5.01 / 1.0)

                /** WCP SwerveX Flipped X3 - 12 Tooth - (4.59 : 1)  */
                const val X3_12: Double = (4.59 / 1.0)
            }
        }
    }

    /** Swerve Drive Specialities  */
    class SDS {
        /** Swerve Drive Specialties - MK3 Module  */
        object MK3 {
            /** Swerve Drive Specialties - MK3 Module (Falcon 500)  */
            fun Falcon500(driveGearRatio: Double): COTSTalonFXSwerveConstants {
                val wheelDiameter = Units.inchesToMeters(4.0)

                /** 12.8 : 1  */
                val angleGearRatio = (12.8 / 1.0)

                val angleKP = 1.0
                val angleKI = 0.0
                val angleKD = 0.0

                val driveMotorInvert = InvertedValue.CounterClockwise_Positive
                val angleMotorInvert = InvertedValue.CounterClockwise_Positive
                val cancoderInvert = SensorDirectionValue.CounterClockwise_Positive
                return COTSTalonFXSwerveConstants(
                    wheelDiameter,
                    angleGearRatio,
                    driveGearRatio,
                    angleKP,
                    angleKI,
                    angleKD,
                    driveMotorInvert,
                    angleMotorInvert,
                    cancoderInvert
                )
            }

            /** Swerve Drive Specialties - MK3 Module (Kraken X60)  */
            fun KrakenX60(driveGearRatio: Double): COTSTalonFXSwerveConstants {
                val wheelDiameter = Units.inchesToMeters(4.0)

                /** 12.8 : 1  */
                val angleGearRatio = (12.8 / 1.0)

                val angleKP = 1.0
                val angleKI = 0.0
                val angleKD = 0.0

                val driveMotorInvert = InvertedValue.CounterClockwise_Positive
                val angleMotorInvert = InvertedValue.CounterClockwise_Positive
                val cancoderInvert = SensorDirectionValue.CounterClockwise_Positive
                return COTSTalonFXSwerveConstants(
                    wheelDiameter,
                    angleGearRatio,
                    driveGearRatio,
                    angleKP,
                    angleKI,
                    angleKD,
                    driveMotorInvert,
                    angleMotorInvert,
                    cancoderInvert
                )
            }

            object driveRatios {
                /** SDS MK3 - (8.16 : 1)  */
                const val Standard: Double = (8.16 / 1.0)

                /** SDS MK3 - (6.86 : 1)  */
                const val Fast: Double = (6.86 / 1.0)
            }
        }

        /** Swerve Drive Specialties - MK4 Module  */
        object MK4 {
            /** Swerve Drive Specialties - MK4 Module (Falcon 500)  */
            fun Falcon500(driveGearRatio: Double): COTSTalonFXSwerveConstants {
                val wheelDiameter = Units.inchesToMeters(4.0)

                /** 12.8 : 1  */
                val angleGearRatio = (12.8 / 1.0)

                val angleKP = 1.0
                val angleKI = 0.0
                val angleKD = 0.0

                val driveMotorInvert = InvertedValue.CounterClockwise_Positive
                val angleMotorInvert = InvertedValue.CounterClockwise_Positive
                val cancoderInvert = SensorDirectionValue.CounterClockwise_Positive
                return COTSTalonFXSwerveConstants(
                    wheelDiameter,
                    angleGearRatio,
                    driveGearRatio,
                    angleKP,
                    angleKI,
                    angleKD,
                    driveMotorInvert,
                    angleMotorInvert,
                    cancoderInvert
                )
            }

            /** Swerve Drive Specialties - MK4 Module (Kraken X60)  */
            fun KrakenX60(driveGearRatio: Double): COTSTalonFXSwerveConstants {
                val wheelDiameter = Units.inchesToMeters(4.0)

                /** 12.8 : 1  */
                val angleGearRatio = (12.8 / 1.0)

                val angleKP = 1.0
                val angleKI = 0.0
                val angleKD = 0.0

                val driveMotorInvert = InvertedValue.CounterClockwise_Positive
                val angleMotorInvert = InvertedValue.CounterClockwise_Positive
                val cancoderInvert = SensorDirectionValue.CounterClockwise_Positive
                return COTSTalonFXSwerveConstants(
                    wheelDiameter,
                    angleGearRatio,
                    driveGearRatio,
                    angleKP,
                    angleKI,
                    angleKD,
                    driveMotorInvert,
                    angleMotorInvert,
                    cancoderInvert
                )
            }

            object driveRatios {
                /** SDS MK4 - (8.14 : 1)  */
                const val L1: Double = (8.14 / 1.0)

                /** SDS MK4 - (6.75 : 1)  */
                const val L2: Double = (6.75 / 1.0)

                /** SDS MK4 - (6.12 : 1)  */
                const val L3: Double = (6.12 / 1.0)

                /** SDS MK4 - (5.14 : 1)  */
                const val L4: Double = (5.14 / 1.0)
            }
        }

        /** Swerve Drive Specialties - MK4i Module  */
        object MK4i {
            /** Swerve Drive Specialties - MK4i Module (Falcon 500)  */
            fun Falcon500(driveGearRatio: Double): COTSTalonFXSwerveConstants {
                val wheelDiameter = Units.inchesToMeters(4.0)

                /** (150 / 7) : 1  */
                val angleGearRatio = ((150.0 / 7.0) / 1.0)

                val angleKP = 100.0
                val angleKI = 0.0
                val angleKD = 0.0

                val driveMotorInvert = InvertedValue.CounterClockwise_Positive
                val angleMotorInvert = InvertedValue.Clockwise_Positive
                val cancoderInvert = SensorDirectionValue.CounterClockwise_Positive
                return COTSTalonFXSwerveConstants(
                    wheelDiameter,
                    angleGearRatio,
                    driveGearRatio,
                    angleKP,
                    angleKI,
                    angleKD,
                    driveMotorInvert,
                    angleMotorInvert,
                    cancoderInvert
                )
            }

            /** Swerve Drive Specialties - MK4i Module (Kraken X60)  */
            fun KrakenX60(driveGearRatio: Double): COTSTalonFXSwerveConstants {
                val wheelDiameter = Units.inchesToMeters(4.0)

                /** (150 / 7) : 1  */
                val angleGearRatio = ((150.0 / 7.0) / 1.0)

                val angleKP = 1.0
                val angleKI = 0.0
                val angleKD = 0.0

                val driveMotorInvert = InvertedValue.CounterClockwise_Positive
                val angleMotorInvert = InvertedValue.Clockwise_Positive
                val cancoderInvert = SensorDirectionValue.CounterClockwise_Positive
                return COTSTalonFXSwerveConstants(
                    wheelDiameter,
                    angleGearRatio,
                    driveGearRatio,
                    angleKP,
                    angleKI,
                    angleKD,
                    driveMotorInvert,
                    angleMotorInvert,
                    cancoderInvert
                )
            }

            object driveRatios {
                /** SDS MK4i - (8.14 : 1)  */
                const val L1: Double = (8.14 / 1.0)

                /** SDS MK4i - (6.75 : 1)  */
                const val L2: Double = (6.75 / 1.0)

                /** SDS MK4i - (6.12 : 1)  */
                const val L3: Double =
                    (5.36 / 1) //        public static final double L3 = (double) 1 / ((16 / 50) * (28 / 16) * (15 / 45));
                // public static final double L3Dot5 = (double) 1 / ((16 / 50) * (28 / 16) * (15
                // / 45));
            }
        }
    }
}
