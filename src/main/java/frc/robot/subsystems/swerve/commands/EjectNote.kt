// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve.commands

import frc.robot.helpers.DebugCommandBase
import frc.robot.subsystems.intake.Intake
import frc.robot.subsystems.pivotintake.PivotIntake
import frc.robot.subsystems.swerve.SwerveDrive

class EjectNote(
    private val swerveSubsystem: SwerveDrive,
    private val pivot: PivotIntake,
    private val intake: Intake
) : DebugCommandBase() {
  override fun execute() {
    // new ParallelCommandGroup(new IntakeIn(intake), new PivotIntakeSlamAndVoltage(pivot));
    intake.intakeIn().withTimeout(2.0)
    swerveSubsystem.setAngularVelocity(10.0) // 10 rads/sec rotation
  }
}
