// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.
package frc.robot.subsystems.pivotintake

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.NeutralOut
import com.ctre.phoenix6.controls.PositionVoltage
import frc.robot.drivers.MonitoredTalonFX
import frc.robot.subsystems.pivotintake.PivotIntakeIO.PivotIntakeIOInputs
import frc.robot.utils.PhoenixUtil
import frc.robot.utils.TalonUtil

class PivotIntakeIOTalonFX : PivotIntakeIO {
    private val pivotIntakeMotor = MonitoredTalonFX(PivotIntakeConstants.kPivotMotorID)
    val positionRequest: PositionVoltage = PositionVoltage(0.0).withSlot(0)
    val motionMagicRequest: MotionMagicVoltage = MotionMagicVoltage(0.0).withSlot(0)

    private val pivotIntakeMotorVoltage: StatusSignal<Double> = pivotIntakeMotor.motorVoltage
    private val pivotIntakeMotorVelocity: StatusSignal<Double> = pivotIntakeMotor.velocity
    private val pivotIntakeMotorPosition: StatusSignal<Double> = pivotIntakeMotor.position
    private val pivotIntakeMotorStatorCurrent: StatusSignal<Double> = pivotIntakeMotor.statorCurrent
    private val pivotIntakeMotorSupplyCurrent: StatusSignal<Double> = pivotIntakeMotor.supplyCurrent
    private val pivotIntakeMotorTemperature: StatusSignal<Double> = pivotIntakeMotor.deviceTemp
    private val pivotIntakeMotorReferenceSlope: StatusSignal<Double> = pivotIntakeMotor.closedLoopReferenceSlope

    init {
        val motorConfig = PivotIntakeConstants.motorConfig
        PhoenixUtil.checkErrorAndRetry { pivotIntakeMotor.configurator.refresh(motorConfig) }
        TalonUtil.applyAndCheckConfiguration(pivotIntakeMotor, motorConfig)

        BaseStatusSignal.setUpdateFrequencyForAll(
            PivotIntakeConstants.updateFrequency,
            pivotIntakeMotorVoltage,
            pivotIntakeMotorVelocity,
            pivotIntakeMotorPosition,
            pivotIntakeMotorStatorCurrent,
            pivotIntakeMotorSupplyCurrent,
            pivotIntakeMotorTemperature,
            pivotIntakeMotorReferenceSlope
        )
        pivotIntakeMotor.optimizeBusUtilization()
    }

    override fun updateInputs(inputs: PivotIntakeIOInputs) {
        BaseStatusSignal.refreshAll(
            pivotIntakeMotorVoltage,
            pivotIntakeMotorVelocity,
            pivotIntakeMotorPosition,
            pivotIntakeMotorStatorCurrent,
            pivotIntakeMotorSupplyCurrent,
            pivotIntakeMotorTemperature,
            pivotIntakeMotorReferenceSlope
        )
        inputs.pivotIntakeMotorVoltage = pivotIntakeMotorVoltage.valueAsDouble
        inputs.pivotIntakeMotorVelocity = pivotIntakeMotorVelocity.valueAsDouble
        inputs.pivotIntakeMotorPosition = pivotIntakeMotorPosition.valueAsDouble
        inputs.pivotIntakeMotorDegrees =
            (inputs.pivotIntakeMotorPosition / PivotIntakeConstants.kPivotMotorGearing) * 360
        inputs.pivotIntakeMotorStatorCurrent = pivotIntakeMotorStatorCurrent.valueAsDouble
        inputs.pivotIntakeMotorSupplyCurrent = pivotIntakeMotorSupplyCurrent.valueAsDouble
        inputs.pivotIntakeMotorTemperature = pivotIntakeMotorTemperature.valueAsDouble
        inputs.pivotIntakeMotorReferenceSlope = pivotIntakeMotorReferenceSlope.valueAsDouble
    }

    override fun setPosition(position: Double) {
        if (PivotIntakeConstants.kUseMotionMagic) {
            pivotIntakeMotor.setControl(motionMagicRequest.withPosition(position))
        } else {
            pivotIntakeMotor.setControl(positionRequest.withPosition(position))
        }
    }

    override fun setVoltage(voltage: Double) {
        pivotIntakeMotor.setVoltage(voltage)
    }

    override fun off() {
        pivotIntakeMotor.setControl(NeutralOut())
    }

    override fun zero() {
        pivotIntakeMotor.setPosition(0.0)
    }
}
