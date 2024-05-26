// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.
package frc.robot.subsystems.intake

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage
import com.ctre.phoenix6.controls.NeutralOut
import com.ctre.phoenix6.controls.VelocityVoltage
import edu.wpi.first.wpilibj.DigitalInput
import frc.robot.drivers.MonitoredTalonFX
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs
import frc.robot.utils.PhoenixUtil
import frc.robot.utils.TalonUtil

class IntakeIOTalonFX : IntakeIO {
    private val intakeMotor = MonitoredTalonFX(IntakeConstants.kIntakeMotorID)
    val intakeRequest: VelocityVoltage = VelocityVoltage(0.0).withSlot(0)
    val motionMagicIntakeRequest: MotionMagicVelocityVoltage = MotionMagicVelocityVoltage(0.0).withSlot(0)

    private val intakeMotorVoltage: StatusSignal<Double> = intakeMotor.motorVoltage
    private val intakeMotorVelocity: StatusSignal<Double> = intakeMotor.velocity
    private val intakeMotorStatorCurrent: StatusSignal<Double> = intakeMotor.statorCurrent
    private val intakeMotorSupplyCurrent: StatusSignal<Double> = intakeMotor.supplyCurrent
    private val intakeMotorTemperature: StatusSignal<Double> = intakeMotor.deviceTemp
    private val intakeMotorReferenceSlope: StatusSignal<Double> = intakeMotor.closedLoopReferenceSlope

    private val passthroughMotor = MonitoredTalonFX(IntakeConstants.kPassthroughMotorID)
    val passthroughRequest: VelocityVoltage = VelocityVoltage(0.0).withSlot(0)
    val motionMagicPassthroughRequest: MotionMagicVelocityVoltage = MotionMagicVelocityVoltage(0.0).withSlot(0)

    private val passthroughMotorVoltage: StatusSignal<Double> = passthroughMotor.motorVoltage
    private val passthroughMotorVelocity: StatusSignal<Double> = passthroughMotor.velocity
    private val passthroughMotorStatorCurrent: StatusSignal<Double> = passthroughMotor.statorCurrent
    private val passthroughMotorSupplyCurrent: StatusSignal<Double> = passthroughMotor.supplyCurrent
    private val passthroughMotorTemperature: StatusSignal<Double> = passthroughMotor.deviceTemp
    private val passthroughMotorReferenceSlope: StatusSignal<Double> = passthroughMotor.closedLoopReferenceSlope

    private val beamBreakInput = DigitalInput(IntakeConstants.kIntakeBeamBreakDIO)

    init {
        val motorConfig = IntakeConstants.intakeMotorConfig
        PhoenixUtil.checkErrorAndRetry { intakeMotor.configurator.refresh(motorConfig) }
        TalonUtil.applyAndCheckConfiguration(intakeMotor, motorConfig)

        val passthroughConfig = IntakeConstants.passthroughMotorConfig
        PhoenixUtil.checkErrorAndRetry { passthroughMotor.configurator.refresh(passthroughConfig) }
        TalonUtil.applyAndCheckConfiguration(passthroughMotor, passthroughConfig)

        BaseStatusSignal.setUpdateFrequencyForAll(
            IntakeConstants.updateFrequency,
            intakeMotorVoltage,
            intakeMotorVelocity,
            intakeMotorStatorCurrent,
            intakeMotorSupplyCurrent,
            intakeMotorTemperature,
            intakeMotorReferenceSlope,
            passthroughMotorVoltage,
            passthroughMotorVelocity,
            passthroughMotorStatorCurrent,
            passthroughMotorSupplyCurrent,
            passthroughMotorTemperature,
            passthroughMotorReferenceSlope
        )
        intakeMotor.optimizeBusUtilization()
        passthroughMotor.optimizeBusUtilization()
    }

    override fun updateInputs(inputs: IntakeIOInputs?) {
        BaseStatusSignal.refreshAll(
            intakeMotorVoltage,
            intakeMotorVelocity,
            intakeMotorStatorCurrent,
            intakeMotorSupplyCurrent,
            intakeMotorTemperature,
            intakeMotorReferenceSlope,
            passthroughMotorVoltage,
            passthroughMotorVelocity,
            passthroughMotorStatorCurrent,
            passthroughMotorSupplyCurrent,
            passthroughMotorTemperature,
            passthroughMotorReferenceSlope
        )
        inputs!!.intakeMotorVoltage = intakeMotorVoltage.valueAsDouble
        inputs.intakeMotorVelocity = intakeMotorVelocity.valueAsDouble
        inputs.intakeMotorStatorCurrent = intakeMotorStatorCurrent.valueAsDouble
        inputs.intakeMotorSupplyCurrent = intakeMotorSupplyCurrent.valueAsDouble
        inputs.intakeMotorTemperature = intakeMotorTemperature.valueAsDouble
        inputs.intakeMotorReferenceSlope = intakeMotorReferenceSlope.valueAsDouble

        inputs.passthroughMotorVoltage = passthroughMotorVoltage.valueAsDouble
        inputs.passthroughMotorVelocity = passthroughMotorVelocity.valueAsDouble
        inputs.passthroughMotorStatorCurrent = passthroughMotorStatorCurrent.valueAsDouble
        inputs.passthroughMotorSupplyCurrent = passthroughMotorSupplyCurrent.valueAsDouble
        inputs.passthroughMotorTemperature = passthroughMotorTemperature.valueAsDouble
        inputs.passthroughMotorReferenceSlope = passthroughMotorReferenceSlope.valueAsDouble

        inputs.isBeamBroken = !beamBreakInput.get()
    }

    override fun setIntakeVoltage(voltage: Double) {
        intakeMotor.setVoltage(voltage)
    }

    override fun setIntakeVelocity(velocity: Double) {
        if (IntakeConstants.kIntakeMotionMagic) {
            intakeMotor.setControl(motionMagicIntakeRequest.withVelocity(velocity))
        } else {
            intakeMotor.setControl(intakeRequest.withVelocity(velocity))
        }
    }

    override fun setPassthroughVoltage(voltage: Double) {
        passthroughMotor.setVoltage(voltage)
    }

    override fun setPassthroughVelocity(velocity: Double) {
        if (IntakeConstants.kPassthroughMotionMagic) {
            passthroughMotor.setControl(motionMagicPassthroughRequest.withVelocity(velocity))
        } else {
            passthroughMotor.setControl(passthroughRequest.withVelocity(velocity))
        }
    }

    override fun off() {
        intakeMotor.setControl(NeutralOut())
        passthroughMotor.setControl(NeutralOut())
    }

    override val isBeamBroken: Boolean
        get() = !beamBreakInput.get()
}
