// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.
package frc.robot.subsystems.swerve

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.CANcoder
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import frc.robot.Robot
import frc.robot.drivers.MonitoredTalonFX
import frc.robot.helpers.Conversions
import frc.robot.subsystems.swerve.ModuleIO.ModuleIOInputs
import frc.robot.subsystems.swerve.helpers.SwerveModuleConstants
import frc.robot.utils.TalonUtil

class ModuleIOTalonFX(var moduleNumber: Int, moduleConstants: SwerveModuleConstants) : ModuleIO {
    private val angleOffset = moduleConstants.angleOffset

    private val mAngleMotor: MonitoredTalonFX
    private val mDriveMotor: MonitoredTalonFX
    private val angleEncoder = CANcoder(moduleConstants.cancoderID, "mani")

    private val driveFeedForward = SimpleMotorFeedforward(
        SwerveConstants.driveKS, SwerveConstants.driveKV, SwerveConstants.driveKA
    )

    private val driveDutyCycle: DutyCycleOut = DutyCycleOut(0.0).withEnableFOC(true)
    private val driveVelocity: VelocityVoltage = VelocityVoltage(0.0).withEnableFOC(true)

    private val anglePosition: PositionVoltage = PositionVoltage(0.0).withEnableFOC(true)

    private val angleMotorVoltage: StatusSignal<Double>
    private val angleMotorPosition: StatusSignal<Double>
    private val angleMotorStatorCurrent: StatusSignal<Double>
    private val angleMotorSupplyCurrent: StatusSignal<Double>
    private val angleMotorTemperature: StatusSignal<Double>
    private val angleMotorReferenceSlope: StatusSignal<Double>

    private val driveMotorVoltage: StatusSignal<Double>
    private val driveMotorVelocity: StatusSignal<Double>
    private val driveMotorPosition: StatusSignal<Double>
    private val driveMotorStatorCurrent: StatusSignal<Double>
    private val driveMotorSupplyCurrent: StatusSignal<Double>
    private val driveMotorTemperature: StatusSignal<Double>
    private val driveMotorReferenceSlope: StatusSignal<Double>

    init {
        angleEncoder.configurator.apply(Robot.ctreConfigs.swerveCANcoderConfig)

        mAngleMotor = MonitoredTalonFX(moduleConstants.angleMotorID, "mani")
        TalonUtil.applyAndCheckConfiguration(mAngleMotor, Robot.ctreConfigs.swerveAngleFXConfig)
        resetToAbsolute()

        mDriveMotor = MonitoredTalonFX(moduleConstants.driveMotorID, "mani")
        TalonUtil.applyAndCheckConfiguration(mDriveMotor, Robot.ctreConfigs.swerveDriveFXConfig)
        mDriveMotor.configurator.setPosition(0.0)

        angleMotorVoltage = mAngleMotor.motorVoltage
        angleMotorPosition = mAngleMotor.position
        angleMotorStatorCurrent = mAngleMotor.statorCurrent
        angleMotorSupplyCurrent = mAngleMotor.supplyCurrent
        angleMotorTemperature = mAngleMotor.deviceTemp
        angleMotorReferenceSlope = mAngleMotor.closedLoopReferenceSlope

        driveMotorVoltage = mDriveMotor.motorVoltage
        driveMotorVelocity = mDriveMotor.velocity
        driveMotorPosition = mDriveMotor.position
        driveMotorStatorCurrent = mDriveMotor.statorCurrent
        driveMotorSupplyCurrent = mDriveMotor.supplyCurrent
        driveMotorTemperature = mDriveMotor.deviceTemp
        driveMotorReferenceSlope = mDriveMotor.closedLoopReferenceSlope

        BaseStatusSignal.setUpdateFrequencyForAll(
            SwerveConstants.updateFrequency,
            angleMotorVoltage,
            angleMotorPosition,
            angleMotorStatorCurrent,
            angleMotorSupplyCurrent,
            angleMotorTemperature,
            angleMotorReferenceSlope,
            driveMotorVoltage,
            driveMotorVelocity,
            driveMotorPosition,
            driveMotorStatorCurrent,
            driveMotorSupplyCurrent,
            driveMotorTemperature,
            driveMotorReferenceSlope
        )

        mAngleMotor.optimizeBusUtilization()
        mDriveMotor.optimizeBusUtilization()
    }

    override fun updateInputs(inputs: ModuleIOInputs) {
        BaseStatusSignal.refreshAll(
            angleMotorVoltage,
            angleMotorPosition,
            angleMotorStatorCurrent,
            angleMotorSupplyCurrent,
            angleMotorTemperature,
            angleMotorReferenceSlope,
            driveMotorVoltage,
            driveMotorVelocity,
            driveMotorPosition,
            driveMotorStatorCurrent,
            driveMotorSupplyCurrent,
            driveMotorTemperature,
            driveMotorReferenceSlope
        )
        inputs.angleMotorVoltage = angleMotorVoltage.valueAsDouble
        inputs.angleMotorPosition = angleMotorPosition.valueAsDouble
        inputs.angleMotorStatorCurrent = angleMotorStatorCurrent.valueAsDouble
        inputs.angleMotorSupplyCurrent = angleMotorSupplyCurrent.valueAsDouble
        inputs.angleMotorTemperature = angleMotorTemperature.valueAsDouble
        inputs.angleMotorReferenceSlope = angleMotorReferenceSlope.valueAsDouble

        inputs.driveMotorVoltage = driveMotorVoltage.valueAsDouble
        inputs.driveMotorVelocity = driveMotorVelocity.valueAsDouble
        inputs.driveMotorPosition = driveMotorPosition.valueAsDouble
        inputs.driveMotorStatorCurrent = driveMotorStatorCurrent.valueAsDouble
        inputs.driveMotorSupplyCurrent = driveMotorSupplyCurrent.valueAsDouble
        inputs.driveMotorTemperature = driveMotorTemperature.valueAsDouble
        inputs.driveMotorReferenceSlope = driveMotorReferenceSlope.valueAsDouble

        inputs.absolutePosition = cANcoder
    }

    override fun setDriveVelocity(velocity: Double, isOpenLoop: Boolean) { // meters per second
        if (isOpenLoop) {
            driveDutyCycle.Output = velocity / SwerveConstants.maxTranslationalVelocity
            mDriveMotor.setControl(driveDutyCycle)
        } else {
            driveVelocity.Velocity = Conversions.MPSToRPS(velocity, SwerveConstants.wheelCircumference)
            driveVelocity.FeedForward = driveFeedForward.calculate(velocity)
            mDriveMotor.setControl(driveVelocity)
        }
    }

    override fun setAnglePosition(position: Double) {
        anglePosition.Position = position
        mAngleMotor.setControl(anglePosition)
    }

    override val cANcoder: Rotation2d
        get() = Rotation2d.fromDegrees(angleEncoder.absolutePosition.value)

    override fun resetToAbsolute() {
        val absolutePosition = cANcoder.rotations - angleOffset.rotations
        mAngleMotor.setPosition(absolutePosition)
    }
}
