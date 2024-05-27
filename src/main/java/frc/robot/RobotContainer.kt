// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.*
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.Constants.FeatureFlags
import frc.robot.autos.commands.*
import frc.robot.helpers.XboxStalker
import frc.robot.subsystems.ampbar.AmpBar
import frc.robot.subsystems.ampbar.AmpBarIOTalonFX
import frc.robot.subsystems.climb.Climb
import frc.robot.subsystems.climb.ClimbIOTalonFX
import frc.robot.subsystems.intake.Intake
import frc.robot.subsystems.intake.IntakeConstants
import frc.robot.subsystems.intake.IntakeIOTalonFX
import frc.robot.subsystems.led.LED
import frc.robot.subsystems.led.commands.CoordinatesButItsMultiple
import frc.robot.subsystems.pivotintake.PivotIntake
import frc.robot.subsystems.pivotintake.PivotIntakeConstants
import frc.robot.subsystems.pivotintake.PivotIntakeIOTalonFX
import frc.robot.subsystems.pivotshooter.PivotShooter
import frc.robot.subsystems.pivotshooter.PivotShooterConstants
import frc.robot.subsystems.pivotshooter.PivotShooterIOTalonFX
import frc.robot.subsystems.shooter.Shooter
import frc.robot.subsystems.shooter.ShooterConstants
import frc.robot.subsystems.shooter.ShooterIOTalonFX
import frc.robot.subsystems.swerve.GyroIOPigeon2
import frc.robot.subsystems.swerve.ModuleIOTalonFX
import frc.robot.subsystems.swerve.SwerveConstants
import frc.robot.subsystems.swerve.SwerveConstants.AzimuthConstants
import frc.robot.subsystems.swerve.SwerveDrive
import frc.robot.subsystems.swerve.commands.*
import frc.robot.subsystems.vision.Vision
import frc.robot.subsystems.vision.VisionIOLimelight
import frc.robot.utils.CommandQueue
import io.github.oblarg.oblog.annotations.Config

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
class RobotContainer {
  /* Controllers */
  private val driver = CommandXboxController(0)
  private val operator = CommandXboxController(1)

  // private final CommandXboxController tester = new CommandXboxController(2);
  /* Drive Controls */
  private val translationAxis = XboxController.Axis.kLeftY.value
  private val strafeAxis = XboxController.Axis.kLeftX.value
  private val rotationAxis = XboxController.Axis.kRightX.value
  private val secondaryAxis = XboxController.Axis.kRightY.value

  /* Swerve Helpers */
  private var isRed = true

  /* Subsystems */
  lateinit var swerveDrive: SwerveDrive
  lateinit var shooter: Shooter
  lateinit var intake: Intake
  lateinit var ampbar: AmpBar
  lateinit var pivotIntake: PivotIntake
  lateinit var climb: Climb
  private var commandQueue: CommandQueue

  private var vision: Vision

  private lateinit var pivotShooter: PivotShooter
  lateinit var led: LED

  @Config.Command(name = "Auto Score Speaker") private lateinit var autoScoreSpeaker: Command

  @Config.Command(name = "Auto Score Amp") private lateinit var autoScoreAmp: Command

  /* Auto */
  private var autoChooser: SendableChooser<Command>

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  init {
    // Cancel any previous commands running
    CommandScheduler.getInstance().cancelAll()

    commandQueue = CommandQueue()
    vision = Vision(VisionIOLimelight())

    // Setup subsystems & button-bindings
    if (FeatureFlags.kPivotShooterEnabled) {
      configurePivotShooter()
    }
    if (FeatureFlags.kShooterEnabled) {
      configureShooter()
    }

    if (FeatureFlags.kPivotIntakeEnabled) {
      configurePivotIntake()
    }
    if (FeatureFlags.kIntakeEnabled) {
      configureIntake()
    }
    if (FeatureFlags.kSwerveEnabled) {
      configureSwerve()
    }
    if (FeatureFlags.kClimbEnabled) {
      configureClimb()
    }
    // If all the subsystems are enabled, configure "operator" autos
    if (FeatureFlags.kIntakeEnabled &&
        FeatureFlags.kShooterEnabled &&
        FeatureFlags.kPivotIntakeEnabled) {
      configureOperatorAutos()
    }

    if (FeatureFlags.kLEDEnabled) {
      configureLED()
    }

    if (FeatureFlags.kSwerveEnabled &&
        FeatureFlags.kIntakeEnabled &&
        FeatureFlags.kShooterEnabled &&
        FeatureFlags.kPivotIntakeEnabled) {
      autoScoreSpeaker = AutoScoreSpeaker(swerveDrive, shooter, intake)
      autoScoreAmp = AutoScoreAmp(swerveDrive, shooter, intake)
    }
    // Named commands
    run {
      // Auto named commands
      NamedCommands.registerCommand("test intake", intake.setIntakeVoltage(12.0).withTimeout(1.0))

      // NamedCommands.registerCommand( // intake ground note, stow to feeder chamber
      // "intake sequence new",
      // new SequentialCommandGroup(
      // new ParallelCommandGroup(
      // new PivotSetAngle(pivotIntake, PivotConstants.kPivotGroundAngleDeg)
      // .withTimeout(0.75),
      // new IntakeIn(intake)),
      // new ParallelCommandGroup(
      // new PivotSlamAndVoltage(pivotIntake).withTimeout(0.75),
      // new ScheduleCommand(new ShootSpeaker(shooter)))));

      NamedCommands.registerCommand( // shoot preloaded note to speaker, use at match start
          "preload speaker",
          SequentialCommandGroup( // new PrintCommand("preload im outta blush"),
              pivotShooter.zero(),
              ParallelDeadlineGroup(
                  SequentialCommandGroup(
                      WaitCommand(0.5),
                      intake
                          .setVoltage(
                              IntakeConstants.kIntakeIntakeVoltage,
                              IntakeConstants.kPassthroughIntakeVoltage)
                          .withTimeout(0.7)),
                  pivotShooter.setPosition(PivotShooterConstants.kSubWooferPreset),
                  shooter.setVelocity(
                      ShooterConstants.kShooterSubwooferRPS,
                      ShooterConstants.kShooterFollowerSubwooferRPS))))

      NamedCommands.registerCommand( // shoot preloaded note to speaker, use at match start
          "preload speaker amp side",
          SequentialCommandGroup( // new PrintCommand("preload im outta blush"),
              pivotShooter.zero(),
              ParallelDeadlineGroup(
                  SequentialCommandGroup(
                      WaitCommand(0.8),
                      intake
                          .setVoltage(
                              IntakeConstants.kIntakeIntakeVoltage,
                              IntakeConstants.kPassthroughIntakeVoltage)
                          .withTimeout(0.7)),
                  pivotShooter.setPosition(PivotShooterConstants.kSubWooferPreset),
                  shooter.setVelocity(
                      ShooterConstants.kShooterSubwooferRPS,
                      ShooterConstants.kShooterFollowerSubwooferRPS)) // new
              // PivotShooterSlamAndVoltage(pivotShooter)));
              ))
      NamedCommands.registerCommand( // intake ground note, stow to feeder chamber
          "intake sequence",
          ParallelCommandGroup(
              pivotIntake.setPosition(PivotIntakeConstants.kPivotGroundPos),
              intake.intakeIn(), // new PivotShooterSlamAndVoltage(pivotShooter),
              // new PivotShootSubwoofer(pivotShooter),
              shooter.setVelocity(
                  ShooterConstants.kShooterSubwooferRPS,
                  ShooterConstants.kShooterFollowerSubwooferRPS)))
      NamedCommands.registerCommand( // outtake note to feeder
          "outtake speaker",
          SequentialCommandGroup( // new ScheduleCommand(new
              // PivotShootSubwoofer(pivotShooter)).asProxy(),
              ParallelCommandGroup(
                  intake
                      .setVoltage(
                          IntakeConstants.kIntakeIntakeVoltage,
                          IntakeConstants.kPassthroughIntakeVoltage)
                      .withTimeout(2.0),
                  shooter.setVelocity(
                      ShooterConstants.kShooterSubwooferRPS,
                      ShooterConstants.kShooterFollowerSubwooferRPS))))
      NamedCommands.registerCommand(
          "aim subwoofer", pivotShooter.setPosition(PivotShooterConstants.kSubWooferPreset))
      NamedCommands.registerCommand("shooter off", shooter.off())

      NamedCommands.registerCommand( // outtake note to feeder
          "safety",
          ParallelCommandGroup(
              intake.intakeIn().withTimeout(1.0),
              shooter.setVelocity(
                  ShooterConstants.kShooterAmpRPS, ShooterConstants.kShooterFollowerAmpRPS)))
      NamedCommands.registerCommand(
          "aim wing center",
          pivotShooter.setPosition(PivotShooterConstants.kWingNoteCenterPreset)) // wing note center
      NamedCommands.registerCommand(
          "aim wing side",
          pivotShooter.setPosition(PivotShooterConstants.kWingNoteSidePreset)) // wing note side
      NamedCommands.registerCommand(
          "aim wing far side",
          pivotShooter.setPosition(
              PivotShooterConstants.kWingNoteFarSidePreset)) // wing note far side
      NamedCommands.registerCommand(
          "aim truss",
          pivotShooter.setPosition(
              PivotShooterConstants.kTrussSourceSidePreset)) // truss source sid
      NamedCommands.registerCommand(
          "aim half truss wing",
          pivotShooter.setPosition(PivotShooterConstants.kHalfWingPodiumPreset)) // half wing podium
      NamedCommands.registerCommand("zero pivot shooter", pivotShooter.slamAndPID())

      NamedCommands.registerCommand( // rev shooter to speaker presets
          "rev speaker",
          shooter.setVelocity(
              ShooterConstants.kShooterSubwooferRPS, ShooterConstants.kShooterFollowerSubwooferRPS))
      NamedCommands.registerCommand( // rev shooter to amp presets
          "rev amp",
          shooter.setVelocity(
              ShooterConstants.kShooterAmpRPS, ShooterConstants.kShooterFollowerAmpRPS))
      NamedCommands.registerCommand( // modular pivot down, use for sabotage
          "pivot down",
          pivotIntake.setPosition(PivotIntakeConstants.kPivotGroundPos).withTimeout(0.75))
      NamedCommands.registerCommand("stow", pivotIntake.slamAndPID().withTimeout(0.75))
      NamedCommands.registerCommand( // intake with no stow, use for sabotage
          "intake", intake.intakeIn())
      NamedCommands.registerCommand( // shoot preloaded note to amp, use at match start
          "preload amp",
          SequentialCommandGroup(
              pivotIntake.zero(),
              ParallelDeadlineGroup(
                  SequentialCommandGroup(WaitCommand(0.8), intake.intakeIn().withTimeout(1.5)),
                  shooter.setVelocity(
                      ShooterConstants.kShooterAmpRPS, ShooterConstants.kShooterFollowerAmpRPS))))
      NamedCommands.registerCommand(
          "scheduled shoot speaker",
          ScheduleCommand(
              shooter.setVelocity(
                  ShooterConstants.kShooterSubwooferRPS,
                  ShooterConstants.kShooterFollowerSubwooferRPS)))
      NamedCommands.registerCommand(
          "align to note",
          SequentialCommandGroup(RotateToNote(swerveDrive), MoveToNote(swerveDrive, intake)))
      NamedCommands.registerCommand(
          "lmao",
          RepeatCommand(
              SequentialCommandGroup(
                  pivotIntake.setPosition(PivotIntakeConstants.kPivotGroundPos).withTimeout(0.75),
                  pivotIntake.slamAndPID())))
    }

    /* Run checks */
    configureCheeks()

    // operator.povLeft().onTrue(cancelCommand);
    // Configure the auto
    autoChooser =
        if (FeatureFlags.kSwerveEnabled) {
          AutoBuilder.buildAutoChooser()
        } else {
          SendableChooser()
        }
    // Autos
    SmartDashboard.putData("Auto Chooser", autoChooser)
  }

  private fun configurePivotShooter() {
    pivotShooter = PivotShooter(PivotShooterIOTalonFX())
    // operator.b().onTrue(new bruh(pivotShooter));
    // operator.x().onTrue(new SequentialCommandGroup(new
    // PivotShootSubwoofer(pivotShooter)));
    operator
        .x()
        .onTrue(
            SequentialCommandGroup(
                pivotShooter.setPosition(PivotShooterConstants.kSubWooferPreset)))
    operator
        .b()
        .onTrue(
            SequentialCommandGroup(
                pivotShooter.setPosition(PivotShooterConstants.kWingNoteSidePreset)))
  }

  private fun configureIntake() {
    intake = Intake(IntakeIOTalonFX())
    // intake.setDefaultCommand(new IntakeSetVoltage(intake, 0));
    // operator.rightBumper().whileTrue(new IntakeInOverride(intake));
    // We assume intake is already enabled, so if pivot is enabled as
    // use IntakeOutWithArm
    operator
        .rightBumper()
        .whileTrue(
            intake.setVoltage(
                IntakeConstants.kIntakeIntakeVoltage, IntakeConstants.kPassthroughIntakeVoltage))

    operator
        .leftBumper()
        .whileTrue(
            intake.setVoltage(
                -IntakeConstants.kIntakeIntakeVoltage, -IntakeConstants.kPassthroughIntakeVoltage))
    driver.rightTrigger().whileTrue(intake.intakeIn())

    // operator.povDown().onTrue(new IntakeOff(intake));
  }

  private fun configurePivotIntake() {
    pivotIntake = PivotIntake(PivotIntakeIOTalonFX())
    operator.povRight().onTrue(pivotIntake.setPosition(PivotIntakeConstants.kPivotGroundPos))
    operator.povLeft().onTrue(pivotIntake.slamAndPID())
  }

  private fun configureClimb() {
    climb = Climb(ClimbIOTalonFX())

    // zeroClimb = new ZeroClimb(climb); // NEED FOR SHUFFLEBOARD
    operator.povDown().onTrue(climb.zero())
    // new Trigger(() -> operator.getRawAxis(translationAxis) < -0.5).onTrue(new
    // UpClimb(climb));
    Trigger { operator.getRawAxis(translationAxis) > 0.5 }.onTrue(climb.retractClimber())
    Trigger { operator.getRawAxis(translationAxis) < -0.5 }
        .onTrue(
            Commands.sequence(
                ParallelCommandGroup(ampbar.setAmpPosition(), pivotShooter.setPosition(12 / 138.33))
                    .withTimeout(1.0),
                climb.extendClimber()))
    // Josh: HangSequence is broken and Rhea does not want to use it; we should
    // rmove this
    // later.
    // new ScheduleCommand(new HangSequence(climb, operator)).schedule();
    // operator.povDownLeft().onTrue(new TestClimbFlip(climb));
  }

  fun setAllianceCol(col: Boolean) {
    isRed = col
  }

  fun configureSwerve() {
    swerveDrive =
        SwerveDrive(
            GyroIOPigeon2(),
            ModuleIOTalonFX(0, SwerveConstants.Mod0.constants),
            ModuleIOTalonFX(1, SwerveConstants.Mod1.constants),
            ModuleIOTalonFX(2, SwerveConstants.Mod2.constants),
            ModuleIOTalonFX(3, SwerveConstants.Mod3.constants))

    swerveDrive.defaultCommand =
        TeleopSwerve(
            swerveDrive,
            { driver.getRawAxis(translationAxis) },
            { driver.getRawAxis(strafeAxis) },
            { driver.getRawAxis(rotationAxis) })

    driver
        .leftTrigger()
        .whileTrue(
            TeleopSwerveLimited(
                swerveDrive,
                { driver.getRawAxis(translationAxis) },
                { driver.getRawAxis(strafeAxis) },
                { driver.getRawAxis(rotationAxis) }))

    driver
        .rightTrigger()
        .whileTrue(
            NoMoreRotation(
                swerveDrive,
                { driver.getRawAxis(translationAxis) },
                { driver.getRawAxis(strafeAxis) },
                true,
                true))

    /* full reset */
    driver.y().onTrue(ZeroGyro(swerveDrive))

    // driver.y().onTrue(new ForceResetModulePositions(swerveDrive));
    if (isRed) /* RED ALLIANCE PRESETS */ {
      /* AMP */

      driver
          .a()
          .onTrue(
              Azimuth(
                      swerveDrive,
                      { driver.leftY },
                      { driver.leftX },
                      { AzimuthConstants.aziAmpRed },
                      { true },
                      true,
                      true)
                  .withTimeout(AzimuthConstants.aziCommandTimeOut))

      /* SOURCE */
      driver
          .rightBumper()
          .onTrue(
              Azimuth(
                      swerveDrive,
                      { driver.leftY },
                      { driver.leftX },
                      { AzimuthConstants.aziSourceRed },
                      { true },
                      true,
                      true)
                  .withTimeout(AzimuthConstants.aziCommandTimeOut))

      /* FEEDER */
      driver
          .povRight()
          .onTrue(
              Azimuth(
                  swerveDrive,
                  { driver.leftY },
                  { driver.leftX },
                  { AzimuthConstants.feederRed },
                  { true },
                  true,
                  true))
    } else /* BLUE ALLIANCE PRESETS */ {
      /* AMP */

      driver
          .a()
          .onTrue(
              Azimuth(
                      swerveDrive,
                      { driver.leftY },
                      { driver.leftX },
                      { AzimuthConstants.aziAmpBlue },
                      { true },
                      true,
                      true)
                  .withTimeout(AzimuthConstants.aziCommandTimeOut))

      /* SOURCE */
      driver
          .rightBumper()
          .onTrue(
              Azimuth(
                      swerveDrive,
                      { driver.leftY },
                      { driver.leftX },
                      { AzimuthConstants.aziSourceBlue },
                      { true },
                      true,
                      true)
                  .withTimeout(AzimuthConstants.aziCommandTimeOut))

      /* FEEDER */
      driver
          .povRight()
          .onTrue(
              Azimuth(
                  swerveDrive,
                  { driver.leftY },
                  { driver.leftX },
                  { AzimuthConstants.feederBlue },
                  { true },
                  true,
                  true))
    }

    /* SUBWOOFER FRONT */
    driver
        .leftBumper()
        .onTrue(
            Azimuth(
                    swerveDrive,
                    { driver.leftY },
                    { driver.leftX },
                    { AzimuthConstants.aziSubwooferFront },
                    { true },
                    true,
                    true)
                .withTimeout(AzimuthConstants.aziCommandTimeOut))

    /* SUBWOOFER RIGHT */
    driver
        .b()
        .onTrue(
            Azimuth(
                    swerveDrive,
                    { driver.leftY },
                    { driver.leftX },
                    { AzimuthConstants.aziSubwooferRight },
                    { true },
                    true,
                    true)
                .withTimeout(AzimuthConstants.aziCommandTimeOut))

    /* SUBWOOFER LEFT */
    driver
        .x()
        .onTrue(
            Azimuth(
                    swerveDrive,
                    { driver.leftY },
                    { driver.leftX },
                    { AzimuthConstants.aziSubwooferLeft },
                    { true },
                    true,
                    true)
                .withTimeout(AzimuthConstants.aziCommandTimeOut))

    /* CLEANUP */
    driver
        .povDown()
        .onTrue(
            Azimuth(
                    swerveDrive,
                    { driver.leftY },
                    { driver.leftX },
                    { AzimuthConstants.cleanUp },
                    { true },
                    true,
                    true)
                .withTimeout(AzimuthConstants.aziCommandTimeOut))
  }

  private fun configureShooter() {
    shooter = Shooter(ShooterIOTalonFX())
    // new Trigger(() -> Math.abs(shooter.getShooterRps() - 100) <= 5)
    // .onTrue(
    // new InstantCommand(
    // () -> {
    // operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 100);
    // }))
    // .onFalse(
    // new InstantCommand(
    // () -> {
    // operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
    // }));
    if (FeatureFlags.kAmpBarEnabled) {
      ampbar = AmpBar(AmpBarIOTalonFX())
      operator
          .rightTrigger()
          .onTrue(
              Commands.parallel(
                  shooter.setVelocity(
                      ShooterConstants.kShooterSubwooferRPS,
                      ShooterConstants.kShooterFollowerSubwooferRPS),
                  ampbar.setStowPosition()))
      operator
          .leftTrigger()
          .onTrue(
              ParallelCommandGroup(
                  shooter.setVelocity(
                      ShooterConstants.kShooterAmpRPS, ShooterConstants.kShooterFollowerAmpRPS),
                  ampbar.setAmpPosition(),
                  pivotShooter.setPosition(PivotShooterConstants.kAmpPreset)))
      operator
          .y()
          .onTrue(
              ParallelCommandGroup(
                  shooter.off(), ampbar.setStowPosition(), pivotShooter.slamAndPID()))
    } else {
      operator
          .rightTrigger()
          .onTrue(
              shooter.setVelocity(
                  ShooterConstants.kShooterSubwooferRPS,
                  ShooterConstants.kShooterFollowerSubwooferRPS))
      operator
          .leftTrigger()
          .onTrue(
              shooter.setVelocity(
                  ShooterConstants.kShooterAmpRPS, ShooterConstants.kShooterFollowerAmpRPS))
      operator.y().onTrue(ParallelCommandGroup(shooter.off(), pivotShooter.slamAndPID()))
    }
  }

  private fun configureOperatorAutos() {
    operator.a().onTrue(IntakeSequence(intake, pivotIntake, pivotShooter, shooter, ampbar))
    operator
        .povUp()
        .onTrue(
            ParallelCommandGroup(
                pivotShooter.setPosition(PivotShooterConstants.kFeederPreset),
                shooter.setVelocity(
                    ShooterConstants.kShooterFeederRPS,
                    ShooterConstants.kShooterFollowerFeederRPS)))
  }

  // operator.x().onTrue(new AutoScoreAmp(swerveDrive, shooter, intake));
  // operator.b().onTrue(new AutoScoreSpeaker(swerveDrive, shooter, intake));
  // }
  fun disableRumble() {
    driver.hid.setRumble(GenericHID.RumbleType.kBothRumble, 0.0)
    operator.hid.setRumble(GenericHID.RumbleType.kBothRumble, 0.0)
  }

  private fun configureLED() {
    val ledList = arrayOf(intArrayOf(2, 3), intArrayOf(1, 1))

    led = LED()
    led.defaultCommand = CoordinatesButItsMultiple(led, ledList, 100, 0, 0, 10)

    // led.setDefaultCommand(new SetLEDsFromBinaryString(led, LEDConstants.based,
    // 100, 0, 0, 5));

    /*
     * Intake LED, flashes RED while intake is down and running,
     * flashes GREEN on successful intake
     */
    if (FeatureFlags.kIntakeEnabled) {
      // Trigger intakeDetectedNote = new Trigger(intake::isBeamBroken);
      // // intakeDetectedNote.whileTrue(new SetSuccessfulIntake(led));

      // intakeDetectedNote
      // .onTrue(
      // new InstantCommand(
      // () -> {
      // operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 50);
      // driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 50);
      // }))
      // .onFalse(
      // new InstantCommand(
      // () -> {
      // operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
      // driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
      // }));

      // This boolean is true when velocity is LESS than 0.
      // Trigger intakeRunning = new Trigger(intake::isMotorSpinning);
      // intakeRunning.whileTrue(new SetGroundIntakeRunning(led));
    }

    /*
     * Shooter LED, solid ORANGE while shooter is running, flashes ORANGE if note
     * fed
     */
    // if (FeatureFlags.kShooterEnabled) {

    // Trigger shooterRunning = new Trigger(
    // () -> (shooter.getShooterFollowerRps() > 75 || shooter.getShooterRps() >
    // 75));
    // shooterRunning.whileTrue(new SetSpeakerScore(led));
    // }
    // if (FeatureFlags.kSwerveEnabled) {
    // if (DriverStation.isAutonomousEnabled() ||

    // FeatureFlags.kSwerveUseVisionForPoseEst) {
    // Trigger swerveSpeakerAligned = new Trigger(swerveDrive::isAlignedToSpeaker);
    // swerveSpeakerAligned.whileTrue(new SetRobotAligned(led));
    // }
    // if (DriverStation.isTeleopEnabled()) {
    // // if (DriverStation.isTeleopEnabled()) {
    // // Trigger azimuthRan =
    // // new Trigger(
    // // () ->
    // // (driver.leftBumper().getAsBoolean() ||
    // driver.rightBumper().getAsBoolean())
    // // || driver.x().getAsBoolean()
    // // || driver.b().getAsBoolean()
    // // || driver.a().getAsBoolean()
    // // || driver.povUp().getAsBoolean());
    // // azimuthRan.whileTrue(new SetAzimuthRan(led));
    // // }
    // }

    // if (FeatureFlags.kClimbEnabled) {
    // Trigger climbRunning =
    // new Trigger(
    // () -> ((climb.getLeftCurrent() > 0) || (operator.povDown().getAsBoolean())));
    // }
    // }
  }

  private fun configureCheeks() {
    // Here, put checks for the configuration of the robot
    if (DriverStation.isFMSAttached()) {
      // FMS-attached checks
      if (!FeatureFlags.kSwerveEnabled) {
        // Swerve is disabled, but the robot is FMS-attached. this *probably* shouldn't
        // happen!
        println(
            "Swerve is disabled, but the robot is FMS-attached. This probably shouldn't happen!")

        DriverStation.reportError(
            "Swerve is disabled, but the robot is FMS-attached. This probably shouldn't happen!",
            false)
        // You can forcibly disable the robot by throwing an exception here, or by
        // calling
        // DriverStationJNI.observeUserProgramDisabled();
      }
      if (FeatureFlags.kDebugEnabled) {
        System.err.println(
            "Robot is FMS-attached & debug mode is enabled. This is not recommended!")
        DriverStation.reportWarning(
            "Robot is FMS-attached & debug mode is enabled. This is not recommended.", false)
      }
    } else {}
  }

  val autonomousCommand: Command
    get() = autoChooser!!.selected

  /* Test Routines */
  fun CANTest(): Boolean {
    return swerveDrive.CANTest()
  }

  fun runPitTestRoutine() {
    // Command pitRoutine = new PitRoutine(swerveDrive, climb, intake, pivotIntake,
    // shooter);
    // pitRoutine.schedule();
  }

  fun periodic(dt: Double) {
    XboxStalker.stalk(driver, operator)

    // System.out.println(Limelight.getBotpose("limelight").length);
    // //
    // double ty = Limelight.getTY("limelight");
    // //
    // // // how many degrees back is your limelight rotated from perfectly
    // vertical?
    // //
    // double limelightMountAngleDegrees = 21.936;

    // // distance from the center of the Limelight lens to the floor
    // double limelightLensHeightInches = 15.601;

    // // distance from the target to the floor
    // double goalHeightInches = 56.375;

    // double angleToGoalDegrees = limelightMountAngleDegrees + ty;
    // double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    // // calculate distance
    // double distanceFromLimelightToGoalInches =
    // (goalHeightInches - limelightLensHeightInches) /
    // Math.tan(angleToGoalRadians);
    // LimelightHelpers.setPriorityTagID("limelight", 7);
    // System.out.println("Distance: " + ty);
    // System.out.println("Distance: " + distanceFromLimelightToGoalInches);
    // Optional<DriverStation.Alliance> ally = DriverStation.getAlliance();
    // if (ally.isPresent() && ally.get() == DriverStation.Alliance.Red) {
    // System.out.println("red");
    // } else if (ally.isPresent()) {
    // System.out.println("blue");
    // } else {
    // System.out.println("red");
    // }
  }
}
