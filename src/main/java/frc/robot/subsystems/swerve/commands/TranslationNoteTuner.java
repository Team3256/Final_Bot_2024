// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve.commands;

import static frc.robot.subsystems.swerve.SwerveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.helpers.DebugCommandBase;
import frc.robot.limelight.LimelightHelpers;
import frc.robot.subsystems.swerve.SwerveDrive;
import java.util.function.DoubleSupplier;

public class TranslationNoteTuner extends DebugCommandBase {
  private Translation2d translation;
  private boolean fieldRelative;
  private boolean openLoop;

  private SwerveDrive swerveSubsystem;

  private DoubleSupplier translationAxis;

  private PIDController translationPidController;

  /** Driver control */
  public TranslationNoteTuner(
      SwerveDrive swerveSubsystem,
      DoubleSupplier translationAxis,
      boolean fieldRelative,
      boolean openLoop) {
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);
    this.translationAxis = translationAxis;

    this.fieldRelative = fieldRelative;
    this.openLoop = openLoop;
    this.translationPidController =
        new PIDController(translationNoteKP, translationNoteKI, translationNoteKD);
    translationPidController.setTolerance(translationNoteTolerance, translationNoteToleranceVel);
    translationPidController.enableContinuousInput(-180, 180);
  }

  @Override
  public void execute() {
    System.out.println("SAM NO MORE TRANSLATION FOR YOU");
    // Obtains axis values for x and y for translation command

    // Safety area, insures that joystick movement will not be tracked within a
    // certain area,
    // prevents unintentional drifting

    // Converts from coordinates to angle, sets joystick forward input as 0,
    // converts angle to
    // degrees

    // PID controller takes current robot position (getYaw) and compares to the
    // azimuth angle to
    // calculate error
    double compensatedMaxVelocity =
        maxTranslationalVelocity * Math.abs(translationAxis.getAsDouble());

    double translationPIDOutput =
        translationPidController.calculate(LimelightHelpers.getTY("limelight-note"), 0.0);
    translationPIDOutput =
        MathUtil.clamp(translationPIDOutput, -compensatedMaxVelocity, compensatedMaxVelocity);
    translation = new Translation2d(translationPIDOutput, 0);

    swerveSubsystem.drive(translation, 0, fieldRelative, openLoop);
  }

  @Override
  public boolean isFinished() {
    // return manualRotating.getAsBoolean() || azimuthController.atSetpoint();
    // return rotateToSpeakerController.atSetpoint();
    return false;
  }
}
