// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.climb

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.NeutralOut
import com.ctre.phoenix6.controls.PositionVoltage
import frc.robot.drivers.MonitoredTalonFX
import frc.robot.subsystems.climb.ClimbIO.ClimbIOInputs
import frc.robot.utils.PhoenixUtil
import frc.robot.utils.TalonUtil

class ClimbIOTalonFX : ClimbIO {
  private val climbMotor = MonitoredTalonFX(ClimbConstants.kLeftClimbMotorID)
  val positionRequest: PositionVoltage = PositionVoltage(0.0).withSlot(0)
  val motionMagicRequest: MotionMagicVoltage = MotionMagicVoltage(0.0).withSlot(0)

  private val climbMotorVoltage: StatusSignal<Double> = climbMotor.motorVoltage
  private val climbMotorVelocity: StatusSignal<Double> = climbMotor.velocity
  private val climbMotorPosition: StatusSignal<Double> = climbMotor.position
  private val climbMotorStatorCurrent: StatusSignal<Double> = climbMotor.statorCurrent
  private val climbMotorSupplyCurrent: StatusSignal<Double> = climbMotor.supplyCurrent
  private val climbMotorTemperature: StatusSignal<Double> = climbMotor.deviceTemp
  private val climbMotorReferenceSlope: StatusSignal<Double> = climbMotor.closedLoopReferenceSlope

  init {
    val motorConfig = ClimbConstants.motorConfig
    PhoenixUtil.checkErrorAndRetry { climbMotor.configurator.refresh(motorConfig) }
    TalonUtil.applyAndCheckConfiguration(climbMotor, motorConfig)

    BaseStatusSignal.setUpdateFrequencyForAll(
        ClimbConstants.updateFrequency,
        climbMotorVoltage,
        climbMotorVelocity,
        climbMotorPosition,
        climbMotorStatorCurrent,
        climbMotorSupplyCurrent,
        climbMotorTemperature,
        climbMotorReferenceSlope)
    climbMotor.optimizeBusUtilization()
  }

  override fun updateInputs(inputs: ClimbIOInputs) {
    BaseStatusSignal.refreshAll(
        climbMotorVoltage,
        climbMotorVelocity,
        climbMotorPosition,
        climbMotorStatorCurrent,
        climbMotorSupplyCurrent,
        climbMotorTemperature,
        climbMotorReferenceSlope)
    inputs!!.climbMotorVoltage = climbMotorVoltage.valueAsDouble
    inputs.climbMotorVelocity = climbMotorVelocity.valueAsDouble
    inputs.climbMotorPosition = climbMotorPosition.valueAsDouble
    inputs.climbMotorStatorCurrent = climbMotorStatorCurrent.valueAsDouble
    inputs.climbMotorSupplyCurrent = climbMotorSupplyCurrent.valueAsDouble
    inputs.climbMotorTemperature = climbMotorTemperature.valueAsDouble
    inputs.climbMotorReferenceSlope = climbMotorReferenceSlope.valueAsDouble
  }

  override fun setPosition(position: Double) {
    if (ClimbConstants.kUseMotionMagic) {
      climbMotor.setControl(motionMagicRequest.withPosition(position))
    } else {
      climbMotor.setControl(positionRequest.withPosition(position))
    }
  }

  override fun setVoltage(voltage: Double) {
    climbMotor.setVoltage(voltage)
  }

  override fun off() {
    climbMotor.setControl(NeutralOut())
  }

  override fun zero() {
    climbMotor.setPosition(0.0)
  }
}
