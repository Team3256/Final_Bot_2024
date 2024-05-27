// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.robotviz

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.util.Color8Bit

object RobotVizConstants {
  // --Visualization--
  // measurements are in meters
  // basic
  const val kLineWidth: Double = 5.0
  @JvmField val botLength: Double = Units.inchesToMeters(33.0) // exact
  val botHeight: Double = Units.inchesToMeters(33.0)
  @JvmField val kRobotSimWindowWidth: Double = Units.inchesToMeters(35.0)
  @JvmField val kRobotSimWindowHeight: Double = Units.inchesToMeters(35.0)
  const val kRootX: Double = 0.0
  const val kRootY: Double = 0.0

  // intake
  @JvmField val kIntakeRadius: Double = Units.inchesToMeters(2.0) // estimate

  // climber
  @JvmField val kClimberX: Double = Units.inchesToMeters(13.0) // exact
  @JvmField val kClimberSpacing: Double = Units.inchesToMeters(3.0) // spoof
  @JvmField val kClimberSpoutHeight: Double = Units.inchesToMeters(17.0) // estimate

  // shooter
  @JvmField val shooterAngle: Rotation2d = Rotation2d.fromDegrees((180 - 55).toDouble()) // exact
  @JvmField val shooterX: Double = Units.inchesToMeters(21.0) // estimate
  @JvmField val shooterOffset: Double = Units.inchesToMeters(23.0) // estimate
  @JvmField val shooterRadius: Double = Units.inchesToMeters(2.0) // estimate

  // feeder
  @JvmField val feederRadius: Double = Units.inchesToMeters(2.0) // estimate
  @JvmField val feederOffset: Double = Units.inchesToMeters(18.0) // estimate

  // arm
  @JvmField val armX: Double = Units.inchesToMeters(28.0) // estimate
  @JvmField val armY: Double = Units.inchesToMeters(5.0) // estimate
  @JvmField val armLength: Double = Units.inchesToMeters(10.0) // exact
  @JvmField val handLength: Double = Units.inchesToMeters(5.0) // exact

  // colors
  @JvmField val orange: Color8Bit = Color8Bit(235, 137, 52)
  @JvmField val red: Color8Bit = Color8Bit(255, 0, 0)
  val purple: Color8Bit = Color8Bit(255, 0, 255)
  @JvmField val blue: Color8Bit = Color8Bit(0, 0, 255)
  @JvmField val green: Color8Bit = Color8Bit(0, 255, 0)
  @JvmField val white: Color8Bit = Color8Bit(255, 255, 255)

  // --Spoof--
  // IRL motor RPM is very high and we won't be able to understand the data we see. So, we slow
  // down the visualization of the motor speeds by a large constant factor.
  const val shooterVelocitySimDamp: Double = 0.1
  const val intakeVelocitySimDamp: Double = 0.1
  const val feederVelocitySimDamp: Double = 0.1
}
