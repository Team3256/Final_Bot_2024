// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.util.HolonomicPathFollowerConfig
import com.pathplanner.lib.util.PIDConstants
import com.pathplanner.lib.util.ReplanningConfig
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.*
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.FeatureFlags
import frc.robot.Robot
import frc.robot.autos.AutoConstants
import frc.robot.limelight.Limelight
import frc.robot.limelight.LimelightHelpers
import io.github.oblarg.oblog.Loggable
import io.github.oblarg.oblog.annotations.Config
import java.util.concurrent.locks.Lock
import java.util.concurrent.locks.ReentrantLock
import kotlin.math.hypot
import kotlin.math.sign
import org.apache.commons.math3.analysis.interpolation.LinearInterpolator
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger

class SwerveDrive(
    private val gyroIO: GyroIO,
    mod0: ModuleIO,
    mod1: ModuleIO,
    mod2: ModuleIO,
    mod3: ModuleIO
) : SubsystemBase(), Loggable {
  fun DEFAULT() {}

  private val gyroInputs: GyroIOInputsAutoLogged = GyroIOInputsAutoLogged()

  private val poseEstimator: SwerveDrivePoseEstimator
  private val limelightLocalizationField = Field2d()

  // private Pose2d PP_currentPose = new Pose2d();
  // private Pose2d PP_targetPose = new Pose2d();
  var swerveOdometry: SwerveDriveOdometry
  var mSwerveMods: Array<SwerveModule>

  @get:AutoLogOutput var chassisSpeeds: ChassisSpeeds = ChassisSpeeds()
  private val odometryThread: Thread
  private val distanceData: MutableList<Double> = ArrayList()
  private val poseXData: MutableList<Double> = ArrayList()
  private val poseYData: MutableList<Double> = ArrayList()
  private val poseThetaData: MutableList<Double> = ArrayList()
  private val lastLocalizeTime = 0.0

  var simPose: Pose2d = Pose2d()

  init {
    // gyro.getConfigurator().apply(new Pigeon2Configuration());
    val allianceBruh = DriverStation.getAlliance()
    if (allianceBruh.isPresent) {
      if (allianceBruh.get() == Alliance.Blue) {
        gyroIO.setYaw(180.0)
        LimelightHelpers.setPipelineIndex("limelight", 1)
      } else {
        gyroIO.setYaw(0.0)
        LimelightHelpers.setPipelineIndex("limelight", 0)
      }
    }
    mSwerveMods =
        arrayOf(
            SwerveModule(mod0, 0),
            SwerveModule(mod1, 1),
            SwerveModule(mod2, 2),
            SwerveModule(mod3, 3))

    /*
     * By pausing init for a second before setting module offsets, we avoid a bug
     * with inverting motors.
     * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
     */
    println("Waiting for one second before setting module offsets...")
    Timer.delay(1.0)
    resetModulesToAbsolute()

    swerveOdometry = SwerveDriveOdometry(SwerveConstants.swerveKinematics, gyroYaw, modulePositions)

    // old rotation 2 0 0.025
    AutoBuilder.configureHolonomic(
        { this.pose },
        { pose: Pose2d? -> this.pose = pose },
        { this.chassisSpeeds }, // Robot relative
        { chassisSpeeds: ChassisSpeeds -> // Robot relative
          drive(
              Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond),
              chassisSpeeds.omegaRadiansPerSecond,
              false, // Robot relative
              false) // Closed loop
        },
        HolonomicPathFollowerConfig(
            PIDConstants(5.0, 0.0, 0.0), // Translation PID constants OLD: 29 0 0
            PIDConstants(3.5, 0.0, 0.0), // Rotation PID constants OLD: 7.5 0 0.75
            AutoConstants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
            AutoConstants
                .kDriveBaseRadiusMeters, // Drive base radius in meters. Distance from robot center
            // to furthest module.
            ReplanningConfig(
                false, false) // Default path replanning config. See the API for the options
            // here
            ),
        {
          val alliance = DriverStation.getAlliance()
          if (alliance.isPresent) {
            return@configureHolonomic (alliance.get() ==
                Alliance.Blue // flip path if we are blue alliance since paths were
            )
            // drawn for red
          }
          false
        },
        this)

    // configureAutoBuilder(10, 0.075, 0.15, 0.1, 0, 0); // kPTrans: 5.75
    poseEstimator =
        SwerveDrivePoseEstimator(
            SwerveConstants.swerveKinematics,
            gyroYaw,
            arrayOf(
                mSwerveMods[0].position,
                mSwerveMods[1].position,
                mSwerveMods[2].position,
                mSwerveMods[3].position),
            Pose2d(),
            VecBuilder.fill(0.5, 0.5, 0.02),
            VecBuilder.fill(0.10, 0.10, 0.5))

    // new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.5, 0.5, 0.02), // Current state
    // X, Y,
    // theta.
    // new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.10, 0.10, 0.5));
    SmartDashboard.putData("Limelight Localization Field", limelightLocalizationField)

    odometryThread = Thread { this.localizeThreadCaller() }
    odometryThread.start()
  }

  fun drive(
      translation: Translation2d,
      rotation: Double,
      fieldRelative: Boolean,
      isOpenLoop: Boolean
  ) {
    chassisSpeeds =
        if (fieldRelative)
            ChassisSpeeds.fromFieldRelativeSpeeds(translation.x, translation.y, rotation, heading)
        else ChassisSpeeds(translation.x, translation.y, rotation)
    val swerveModuleStates = SwerveConstants.swerveKinematics.toSwerveModuleStates(chassisSpeeds)
    if (FeatureFlags.kSwerveVelocityLimitingEnabled) {
      SwerveDriveKinematics.desaturateWheelSpeeds(
          swerveModuleStates, SwerveConstants.maxTranslationalVelocity)
    }
    for (mod in mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop)
    }
  }

  // public void drive(Translation2d translation, double rotation, boolean
  // fieldRelative, boolean isOpenLoop) {
  // SwerveModuleState[] swerveModuleStates =
  // SwerveConstants.swerveKinematics.toSwerveModuleStates(
  // fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
  // translation.getX(),
  // translation.getY(),
  // rotation,
  // getHeading())
  // : new ChassisSpeeds(
  // translation.getX(),
  // translation.getY(),
  // rotation));
  // SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
  // SwerveConstants.maxTranslationalVelocity);
  // for (SwerveModule mod : mSwerveMods) {
  // mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
  // }
  // }
  @Config(name = "Set Angular Velocity")
  fun setAngularVelocity(radPerSec: Double) {
    println("setAngularVelocity:$radPerSec")
    drive(Translation2d(), radPerSec, true, true)
  }

  fun resetGyro() {
    gyroIO.setYaw(0.0)
    Timer.delay(0.125)
    println("Gyro reset!")
  }

  fun flipGyro() {
    gyroIO.setYaw(180.0)
    Timer.delay(0.125)
    println("Gyro flipped!")
  }

  fun CANTest(): Boolean {
    return true
  }

  val distanceToSpeaker: Unit
    get() {
      // System.out.println(Limelight.getBotpose("limelight").length);
      val ourPose = pose!!

      // double x = visionBotPose[0] -
      // Constants.AutoConstants.kBlueSpeakerLocation.getX();
      // double y = visionBotPose[1] -
      // Constants.AutoConstants.kBlueSpeakerLocation.getY();
      val scoringLocation: Pose2d
      val ally = DriverStation.getAlliance()
      scoringLocation =
          if (ally.isPresent && ally.get() == Alliance.Red) {
            AutoConstants.kRedSpeakerLocation
          } else if (ally.isPresent) {
            AutoConstants.kBlueSpeakerLocation
          } else {
            AutoConstants.kRedSpeakerLocation
          }
      val distance = ourPose.translation.getDistance(scoringLocation.translation)
      // double distance = Math.sqrt(x*x+y*y);
      println("dist to speaker:$distance")
      Logger.recordOutput("BruhPose", ourPose)
    }

  @AutoLogOutput
  fun stupidIdiotLimelightDumb(): Pose2d {
    val limelightPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight").pose
    return limelightPose
  }

  fun localize(networkTablesName: String) {
    // Check if we're running on the main thread
    // StaticThreadChecker.checkCurrentThread();
    if (!Limelight.hasValidTargets(networkTablesName)) return

    /*
     * From the Limelight docs about using WPILib's Pose
     * Estimator:
     * https://docs.limelightvision.io/docs/docs-limelight/pipeline-
     * apriltag/apriltag-robot-localization#using-wpilibs-pose-estimator
     * In 2024, most of the WPILib Ecosystem transitioned to a single-origin
     * coordinate system. In 2023, your coordinate system origin changed based on
     * your alliance color.
     *
     * For 2024 and beyond, the origin of your coordinate system should always be
     * the "blue" origin. FRC teams should always use botpose_wpiblue for
     * pose-related functionality
     */
    // LimelightHelpers.SetRobotOrientation(
    // networkTablesName,
    // poseEstimator.getEstimatedPosition().getRotation().getDegrees(),
    // 0,
    // 0,
    // 0,
    // 0,
    // 0);
    val tl = Limelight.getLatency_Pipeline(networkTablesName)
    val limelightPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(networkTablesName).pose

    val aprilTagLocation = Limelight.getTargetPose_RobotSpace(networkTablesName)
    val aprilTagDistance = Translation2d(aprilTagLocation[0], aprilTagLocation[2]).norm
    if (FeatureFlags.kLocalizationDataCollectionMode) {
      distanceData.add(aprilTagDistance)
      poseXData.add(limelightPose.x)
      poseYData.add(limelightPose.y)
      poseThetaData.add(limelightPose.rotation.radians)
    }

    if (FeatureFlags.kLocalizationStdDistanceBased) {
      // we should probably use std-devs for champs bc it helps filter out "bad" data,
      // but it needs to be done for EVERY SINGLE LIMELIGHT!
      if (FeatureFlags.kDebugEnabled) {
        SmartDashboard.putNumber("April Tag Distance", aprilTagDistance)
        SmartDashboard.putNumber("X std", getStdDevXTranslation(aprilTagDistance))
        SmartDashboard.putNumber("Y std", getStdDevYTranslation(aprilTagDistance))
        SmartDashboard.putNumber("Theta std", getStdDevAngle(aprilTagDistance))
      }

      // poseEstimator.setVisionMeasurementStdDevs(
      // VecBuilder.fill(
      // getStdDevXTranslation(aprilTagDistance),
      // getStdDevYTranslation(aprilTagDistance),
      // getStdDevAngle(aprilTagDistance)));
      poseEstimator.addVisionMeasurement(
          limelightPose, Timer.getFPGATimestamp() - Units.millisecondsToSeconds(tl))

      // poseEstimator.addVisionMeasurement(
      // limelightPose,
      // Timer.getFPGATimestamp() - Units.millisecondsToSeconds(tl),
      // new MatBuilder<>(Nat.N3(), Nat.N1())
      // .fill(
      // getStdDevXTranslation(aprilTagDistance),
      // getStdDevYTranslation(aprilTagDistance),
      // getStdDevAngle(aprilTagDistance)));
    } else {
      poseEstimator.addVisionMeasurement(
          limelightPose, Timer.getFPGATimestamp() - Units.millisecondsToSeconds(tl))
    }

    if (FeatureFlags.kDebugEnabled) {
      limelightLocalizationField.robotPose = limelightPose
      SmartDashboard.putNumber("Lime Light pose x $networkTablesName", limelightPose.x)
      SmartDashboard.putNumber("Lime Light pose y $networkTablesName", limelightPose.y)
      SmartDashboard.putNumber("Lime Light pose theta", limelightPose.rotation.degrees)
    }
  }

  fun localizeThreadCaller() {
    // localizeThreadCaller is called from the main thread:
    // new Thread(this::localizeThread).start();
    // so localize will update the pose estimator in a separate thread
    // and it'll reflect over to the main thread's poseEstimator
    // when the main thread calls getPose()// using while true instead of
    // periodic()ly creating a new thread!
    if (DriverStation.isTeleopEnabled()) {
      localize("limelight")
      // localize("limelight-left");
      // localize("limelight-right");
      // Timer.delay(SwerveConstants.kVisionSecondsBetweenLocalize);
    }
  }

  fun clampDistanceForInterpolation(distance: Double): Double {
    return MathUtil.clamp(
        distance,
        SwerveConstants.kSwervePoseEstimatorMinValue,
        SwerveConstants.kSwervePoseEstimatorMaxValue)
  }

  fun getStdDevXTranslation(distance: Double): Double {
    return distanceToStdDevXTranslation!!.value(clampDistanceForInterpolation(distance))
  }

  fun getStdDevYTranslation(distance: Double): Double {
    return distanceToStdDevYTranslation!!.value(clampDistanceForInterpolation(distance))
  }

  fun getStdDevAngle(distance: Double): Double {
    return distancetostdDevAngle!!.value(clampDistanceForInterpolation(distance))
  }

  var moduleStates: Array<SwerveModuleState>
    get() {
      return mSwerveMods.map { it.state }.toTypedArray()
    }
    /* Used by SwerveControllerCommand in Auto */ set(desiredStates) {
      SwerveDriveKinematics.desaturateWheelSpeeds(
          desiredStates, SwerveConstants.maxTranslationalVelocity)

      for (mod in mSwerveMods) {
        mod.setDesiredState(desiredStates[mod.moduleNumber], false)
      }
    }

  val modulePositions: Array<SwerveModulePosition?>
    get() {
      val positions = arrayOfNulls<SwerveModulePosition>(4)
      for (mod in mSwerveMods) {
        positions[mod.moduleNumber] = mod.position
      }
      return positions
    }

  @get:AutoLogOutput
  var pose: Pose2d?
    get() {
      if (Robot.isSimulation()) return simPose
      return if (FeatureFlags.kSwerveUseVisionForPoseEst) {
        poseEstimator.estimatedPosition
      } else {
        swerveOdometry.poseMeters
      }
    }
    set(pose) {
      if (FeatureFlags.kSwerveUseVisionForPoseEst) {
        poseEstimator.resetPosition(gyroYaw, modulePositions, pose)
      } else {
        swerveOdometry.resetPosition(gyroYaw, modulePositions, pose)
      }
    }

  fun updatePose(gyroYaw: Rotation2d?, swerveModulePositions: Array<SwerveModulePosition?>?) {
    if (Robot.isSimulation()) {
      val fieldRelative = ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds, this.gyroYaw)
      simPose =
          simPose.transformBy(
              Transform2d(
                  fieldRelative.vxMetersPerSecond * Robot.defaultPeriodSecs,
                  fieldRelative.vyMetersPerSecond * Robot.defaultPeriodSecs,
                  Rotation2d.fromRadians(
                      fieldRelative.omegaRadiansPerSecond * Robot.defaultPeriodSecs)))
    } else if (FeatureFlags.kSwerveUseVisionForPoseEst) {
      poseEstimator.update(this.gyroYaw, modulePositions)
    } else {
      swerveOdometry.update(this.gyroYaw, modulePositions)
    }
  }

  @get:AutoLogOutput(key = "robotVelocity")
  @set:Config(name = "Set Velocity")
  var velocity: Double
    get() {
      val something = SwerveConstants.swerveKinematics.toChassisSpeeds(*moduleStates)
      return hypot(something.vxMetersPerSecond, something.vyMetersPerSecond)
    }
    set(velocity) {
      println("setIntakeVelocity:$velocity")
      drive(Translation2d(velocity, 0.0), 0.0, false, true)
    }

  @get:AutoLogOutput(key = "robotRotationalVelocity")
  val rotationalVelocity: Double
    get() {
      val something = SwerveConstants.swerveKinematics.toChassisSpeeds(*moduleStates)
      return something.omegaRadiansPerSecond
    }

  fun zeroHeading() {
    if (FeatureFlags.kSwerveUseVisionForPoseEst) {
      poseEstimator.resetPosition(
          gyroYaw, modulePositions, Pose2d(pose!!.translation, Rotation2d()))
    } else {
      swerveOdometry.resetPosition(
          gyroYaw, modulePositions, Pose2d(pose!!.translation, Rotation2d()))
    }
  }

  @get:AutoLogOutput
  var heading: Rotation2d?
    get() = pose!!.rotation
    set(heading) {
      if (FeatureFlags.kSwerveUseVisionForPoseEst) {
        poseEstimator.resetPosition(gyroYaw, modulePositions, Pose2d(pose!!.translation, heading))
      } else {
        swerveOdometry.resetPosition(gyroYaw, modulePositions, Pose2d(pose!!.translation, heading))
      }
    }

  val gyroYaw: Rotation2d
    // @Log(name = "Gyro Yaw")
    get() {
      val yaw = if (Robot.isReal()) gyroInputs.yawPosition else simPose.rotation
      return yaw
    }

  fun resetModulesToAbsolute() {
    for (mod in mSwerveMods) {
      mod.resetToAbsolute()
    }
  }

  // Docs: https://pathplanner.dev/pplib-build-an-auto.html#configure-autobuilder
  fun configureAutoBuilder(
      translationKP: Double,
      translationKI: Double,
      translationKD: Double,
      rotationKP: Double,
      rotationKI: Double,
      rotationKD: Double
  ) {
    // Auto must drive in robot relative and closed loop mode
    AutoBuilder.configureHolonomic(
        { this.pose },
        { pose: Pose2d? -> this.pose = pose },
        { this.chassisSpeeds }, // Robot relative
        { chassisSpeeds: ChassisSpeeds -> // Robot relative
          drive(
              Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond),
              chassisSpeeds.omegaRadiansPerSecond,
              false, // Robot relative
              false) // Closed loop
        },
        HolonomicPathFollowerConfig(
            PIDConstants(
                translationKP, translationKI, translationKD, 0.0), // Translation PID constants
            PIDConstants(rotationKP, rotationKI, rotationKD), // Rotation PID constants
            AutoConstants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
            AutoConstants
                .kDriveBaseRadiusMeters, // Drive base radius in meters. Distance from robot center
            // to furthest module.
            ReplanningConfig() // Default path replanning config. See the API for the options
            // here
            ),
        {
          val alliance = DriverStation.getAlliance()
          if (alliance.isPresent) {
            return@configureHolonomic (alliance.get() ==
                Alliance.Blue // flip path if we are blue alliance since paths were
            )
            // drawn for red
          }
          false
        },
        this)

    // PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
    // PP_currentPose = pose;

    // });
    // PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
    // PP_targetPose = pose;
    // });
    // // PathPlannerLogging.setLogActivePathCallback((pose) -> {
    // // Logger.recordOutput("Auto-TargetPose", pose);
    // // })
    // // Pathfinding.setPathfinder(new LocalADStarAK());
    // PathPlannerLogging.setLogActivePathCallback(
    // (activePath) -> {
    // // Logger.recordOutput("AutoTrajectory", activePath.toArray(new
    // // Pose2d[activePath.size()]));
    // autoFieldLog.getObject("path").setPoses(activePath);
    // });
    // PathPlannerLogging.setLogTargetPoseCallback(
    // (targetPose) -> {
    // Logger.recordOutput("Auto/TrajectorySetpoint", targetPose);
    // });
  }

  val isAlignedToSpeaker: Boolean
    get() {
      // Get the current pose
      val currentPose = pose!!
      val alliance = DriverStation.getAlliance()
      if (alliance.isPresent) {
        val speakerPose =
            if (alliance.get() == Alliance.Red) {
              // Pose of speaker
              AutoConstants.kRedSpeakerLocation
            } else {
              // Pose of speaker
              AutoConstants.kBlueSpeakerLocation
            }
        if (speakerPose == null) {
          return false
        }
        // Get the distance to the speaker
        val distanceToSpeaker = currentPose.translation.getDistance(speakerPose.translation)

        // If the distance to the speaker is less than the threshold, return true
        return distanceToSpeaker < AutoConstants.kSpeakerAlignmentThreshold
      } else {
        return false
      }
    }

  fun off() {
    for (module in mSwerveMods) {
      module.off()
    }
  }

  override fun periodic() {
    // System.out.println(getGyroYaw().toString());
    // update last period values
    // ChassisSpeeds cs =
    // Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    // double actualVel = Math.hypot(cs.vxMetersPerSecond,cs.vyMetersPerSecond);
    // actualAccel = (actualVel - prevActualVel)/Robot.defaultPeriodSecs;
    // prevActualVel = actualVel;

    // Logger.recordOutput("currentPose", PP_currentPose);
    // Logger.recordOutput("targetPose", PP_targetPose);

    odometryLock.lock()
    gyroIO.updateInputs(gyroInputs)
    for (mod in mSwerveMods) {
      mod.updateInputs()
    }
    updatePose(gyroYaw, modulePositions)
    odometryLock.unlock()
    Logger.processInputs("Swerve/Gyro", gyroInputs)
    for (mod in mSwerveMods) {
      mod.periodic()
    }
    // if (Timer.getFPGATimestamp() - lastLocalizeTime >
    // SwerveConstants.kVisionSecondsBetweenLocalize) {
    // Logger.recordOutput("Last localize time", lastLocalizeTime);
    // localize("limelight-top");
    // lastLocalizeTime = Timer.getFPGATimestamp();
    // }
    for (mod in mSwerveMods) {
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.cANcoder.degrees)
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.position.angle.degrees)
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.state.speedMetersPerSecond)
    }
    // Is the odometry thread still alive?
    if (!odometryThread.isAlive) {
      if (FeatureFlags.kDebugEnabled) {
        println("Odometry thread is dead! Restarting...")
      }
      // odometryThread = new Thread(this::localizeThreadCaller);
      // odometryThread.start();
    }
    this.localizeThreadCaller()
  }

  companion object {
    private var distanceToStdDevXTranslation: PolynomialSplineFunction? = null
    private var distanceToStdDevYTranslation: PolynomialSplineFunction? = null
    private var distancetostdDevAngle: PolynomialSplineFunction? = null

    init {
      if (FeatureFlags.kLocalizationStdDistanceBased) {
        val trainDistance = DoubleArray(SwerveConstants.kSwervePoseEstimatorStdData.size)
        val trainStdDevXTranslation = DoubleArray(SwerveConstants.kSwervePoseEstimatorStdData.size)
        val trainStdDevYTranslation = DoubleArray(SwerveConstants.kSwervePoseEstimatorStdData.size)
        val trainStdDevAngle = DoubleArray(SwerveConstants.kSwervePoseEstimatorStdData.size)
        for (i in SwerveConstants.kSwervePoseEstimatorStdData.indices) {
          trainDistance[i] = SwerveConstants.kSwervePoseEstimatorStdData[i].distance
          trainStdDevXTranslation[i] =
              SwerveConstants.kSwervePoseEstimatorStdData[i].stdDevXTranslation
          trainStdDevYTranslation[i] =
              SwerveConstants.kSwervePoseEstimatorStdData[i].stdDevYTranslation
          trainStdDevAngle[i] = SwerveConstants.kSwervePoseEstimatorStdData[i].stdDevAngle
        }
        distanceToStdDevXTranslation =
            LinearInterpolator().interpolate(trainDistance, trainStdDevXTranslation)
        distanceToStdDevYTranslation =
            LinearInterpolator().interpolate(trainDistance, trainStdDevYTranslation)
        distancetostdDevAngle = LinearInterpolator().interpolate(trainDistance, trainStdDevAngle)
      }
    }

    val odometryLock: Lock = ReentrantLock()

    @JvmStatic
    fun joystickToSpeed(joystickValue: Double, maxValue: Double): Double {
      return if (FeatureFlags.kQuadraticDrive) {
        sign(joystickValue) * joystickValue * joystickValue * maxValue
      } else {
        joystickValue * maxValue
      }
    }
  }
}
