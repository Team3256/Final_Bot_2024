// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve.commands

import frc.robot.helpers.DebugCommandBase
import frc.robot.subsystems.swerve.SwerveDrive

class ForceResetModulePositions(var swerveDrive: SwerveDrive) : DebugCommandBase() {
  override fun initialize() {
    super.initialize()
    swerveDrive.resetModulesToAbsolute()
  }

  override fun isFinished(): Boolean {
    return true
  }
}
