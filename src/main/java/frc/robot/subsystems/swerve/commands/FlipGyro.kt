// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve.commands

import frc.robot.Constants
import frc.robot.helpers.DebugCommandBase
import frc.robot.subsystems.swerve.SwerveDrive

class FlipGyro(var swerveDrive: SwerveDrive) : DebugCommandBase() {
  override fun initialize() {
    super.initialize()
    swerveDrive.flipGyro()
    if (Constants.FeatureFlags.kResetHeadingOnZeroGyro) {
      println("kResetHeadingOnZeroGyro is TRUE -- resetting heading.")
      swerveDrive.zeroHeading()
    }
  }

  override fun isFinished(): Boolean {
    return true
  }
}
