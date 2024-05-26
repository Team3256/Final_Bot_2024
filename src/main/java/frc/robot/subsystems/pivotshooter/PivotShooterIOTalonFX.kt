// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.
package frc.robot.subsystems.pivotshooter

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.NeutralOut
import com.ctre.phoenix6.controls.PositionVoltage
import frc.robot.drivers.MonitoredTalonFX
import frc.robot.subsystems.pivotshooter.PivotShooterIO.PivotShooterIOInputs
import frc.robot.utils.PhoenixUtil
import frc.robot.utils.TalonUtil

class PivotShooterIOTalonFX : PivotShooterIO {
    private val pivotShooterMotor = MonitoredTalonFX(PivotShooterConstants.kPivotMotorID)
    val positionRequest: PositionVoltage = PositionVoltage(0.0).withSlot(0)
    val motionMagicRequest: MotionMagicVoltage = MotionMagicVoltage(0.0).withSlot(0)

    private val pivotShooterMotorVoltage: StatusSignal<Double> = pivotShooterMotor.motorVoltage
    private val pivotShooterMotorVelocity: StatusSignal<Double> = pivotShooterMotor.velocity
    private val pivotShooterMotorPosition: StatusSignal<Double> = pivotShooterMotor.position
    private val pivotShooterMotorStatorCurrent: StatusSignal<Double> = pivotShooterMotor.statorCurrent
    private val pivotShooterMotorSupplyCurrent: StatusSignal<Double> = pivotShooterMotor.supplyCurrent
    private val pivotShooterMotorTemperature: StatusSignal<Double> = pivotShooterMotor.deviceTemp
    private val pivotShooterMotorReferenceSlope: StatusSignal<Double> = pivotShooterMotor.closedLoopReferenceSlope

    init {
        val motorConfig = PivotShooterConstants.motorConfigs
        PhoenixUtil.checkErrorAndRetry { pivotShooterMotor.configurator.refresh(motorConfig) }
        TalonUtil.applyAndCheckConfiguration(pivotShooterMotor, motorConfig)

        BaseStatusSignal.setUpdateFrequencyForAll(
            PivotShooterConstants.updateFrequency,
            pivotShooterMotorVoltage,
            pivotShooterMotorVelocity,
            pivotShooterMotorPosition,
            pivotShooterMotorStatorCurrent,
            pivotShooterMotorSupplyCurrent,
            pivotShooterMotorTemperature,
            pivotShooterMotorReferenceSlope
        )
        pivotShooterMotor.optimizeBusUtilization()
    }

    override fun updateInputs(inputs: PivotShooterIOInputs) {
        BaseStatusSignal.refreshAll(
            pivotShooterMotorVoltage,
            pivotShooterMotorVelocity,
            pivotShooterMotorPosition,
            pivotShooterMotorStatorCurrent,
            pivotShooterMotorSupplyCurrent,
            pivotShooterMotorTemperature,
            pivotShooterMotorReferenceSlope
        )
        inputs.pivotShooterMotorVoltage = pivotShooterMotorVoltage.valueAsDouble
        inputs.pivotShooterMotorVelocity = pivotShooterMotorVelocity.valueAsDouble
        inputs.pivotShooterMotorPosition = pivotShooterMotorPosition.valueAsDouble
        inputs.pivotShooterMotorDegrees =
            (inputs.pivotShooterMotorPosition / PivotShooterConstants.kPivotMotorGearing) * 360
        inputs.pivotShooterMotorStatorCurrent = pivotShooterMotorStatorCurrent.valueAsDouble
        inputs.pivotShooterMotorSupplyCurrent = pivotShooterMotorSupplyCurrent.valueAsDouble
        inputs.pivotShooterMotorTemperature = pivotShooterMotorTemperature.valueAsDouble
        inputs.pivotShooterMotorReferenceSlope = pivotShooterMotorReferenceSlope.valueAsDouble
    }

    override fun setPosition(position: Double) {
        if (PivotShooterConstants.kUseMotionMagic) {
            pivotShooterMotor.setControl(motionMagicRequest.withPosition(position))
        } else {
            pivotShooterMotor.setControl(positionRequest.withPosition(position))
        }
    }

    override fun setVoltage(voltage: Double) {
        pivotShooterMotor.setVoltage(voltage)
    }

    override fun off() {
        pivotShooterMotor.setControl(NeutralOut())
    }

    override fun zero() {
        pivotShooterMotor.setPosition(0.0)
    }
}
