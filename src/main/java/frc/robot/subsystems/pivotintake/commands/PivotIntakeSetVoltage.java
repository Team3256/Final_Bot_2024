// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.pivotintake.commands;

import frc.robot.helpers.DebugCommandBase;
import frc.robot.subsystems.pivotintake.PivotIntake;

public class PivotIntakeSetVoltage extends DebugCommandBase {
  private PivotIntake pivotIntake;
  private final double voltage;

  public PivotIntakeSetVoltage(PivotIntake pivotIntake, double voltage) {
    this.pivotIntake = pivotIntake;
    this.voltage = voltage;
    addRequirements(pivotIntake);
  }

  public void initialize() {
    pivotIntake.setOutputVoltage(voltage);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
