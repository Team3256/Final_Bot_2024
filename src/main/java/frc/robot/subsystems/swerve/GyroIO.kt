// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve

import edu.wpi.first.math.geometry.Rotation2d
import org.littletonrobotics.junction.AutoLog

interface GyroIO {
  @AutoLog
  open class GyroIOInputs {
    @JvmField var connected: Boolean = false
    @JvmField var yawPosition: Rotation2d = Rotation2d()
    @JvmField var yawVelocityRadPerSec: Double = 0.0
  }

  fun updateInputs(inputs: GyroIOInputs) {}

  fun setYaw(yaw: Double) {}
}
