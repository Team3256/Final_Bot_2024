// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage
import com.ctre.phoenix6.controls.NeutralOut
import com.ctre.phoenix6.controls.VelocityVoltage
import frc.robot.drivers.MonitoredTalonFX
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs
import frc.robot.utils.PhoenixUtil
import frc.robot.utils.TalonUtil

class ShooterIOTalonFX : ShooterIO {
  private val shooterMotor = MonitoredTalonFX(ShooterConstants.kShooterMotorID)
  val velocityRequest: VelocityVoltage = VelocityVoltage(0.0).withSlot(0)
  val motionMagicRequest: MotionMagicVelocityVoltage = MotionMagicVelocityVoltage(0.0).withSlot(0)

  private val shooterMotorVoltage: StatusSignal<Double> = shooterMotor.motorVoltage
  private val shooterMotorVelocity: StatusSignal<Double> = shooterMotor.velocity
  private val shooterMotorStatorCurrent: StatusSignal<Double> = shooterMotor.statorCurrent
  private val shooterMotorSupplyCurrent: StatusSignal<Double> = shooterMotor.supplyCurrent
  private val shooterMotorTemperature: StatusSignal<Double> = shooterMotor.deviceTemp
  private val shooterMotorReferenceSlope: StatusSignal<Double> =
      shooterMotor.closedLoopReferenceSlope

  private val shooterMotorFollower = MonitoredTalonFX(ShooterConstants.kShooterMotorFollowerID)
  val velocityRequestFollower: VelocityVoltage = VelocityVoltage(0.0).withSlot(0)
  val motionMagicRequestFollower: MotionMagicVelocityVoltage =
      MotionMagicVelocityVoltage(0.0).withSlot(0)

  private val shooterMotorFollowerVoltage: StatusSignal<Double> = shooterMotorFollower.motorVoltage
  private val shooterMotorFollowerVelocity: StatusSignal<Double> = shooterMotorFollower.velocity
  private val shooterMotorFollowerStatorCurrent: StatusSignal<Double> =
      shooterMotorFollower.statorCurrent
  private val shooterMotorFollowerSupplyCurrent: StatusSignal<Double> =
      shooterMotorFollower.supplyCurrent
  private val shooterMotorFollowerTemperature: StatusSignal<Double> =
      shooterMotorFollower.deviceTemp
  private val shooterMotorFollowerReferenceSlope: StatusSignal<Double> =
      shooterMotorFollower.closedLoopReferenceSlope

  init {
    val motorConfig = ShooterConstants.motorConfigs
    PhoenixUtil.checkErrorAndRetry { shooterMotor.configurator.refresh(motorConfig) }
    TalonUtil.applyAndCheckConfiguration(shooterMotor, motorConfig)

    val motorConfigFollower = ShooterConstants.followerMotorConfigs
    PhoenixUtil.checkErrorAndRetry {
      shooterMotorFollower.configurator.refresh(motorConfigFollower)
    }
    TalonUtil.applyAndCheckConfiguration(shooterMotorFollower, motorConfigFollower)

    BaseStatusSignal.setUpdateFrequencyForAll(
        ShooterConstants.updateFrequency,
        shooterMotorVoltage,
        shooterMotorVelocity,
        shooterMotorStatorCurrent,
        shooterMotorSupplyCurrent,
        shooterMotorTemperature,
        shooterMotorReferenceSlope,
        shooterMotorFollowerVoltage,
        shooterMotorFollowerVelocity,
        shooterMotorFollowerStatorCurrent,
        shooterMotorFollowerSupplyCurrent,
        shooterMotorFollowerTemperature,
        shooterMotorFollowerReferenceSlope)
    shooterMotor.optimizeBusUtilization()
    shooterMotorFollower.optimizeBusUtilization()
  }

  override fun updateInputs(inputs: ShooterIOInputs) {
    BaseStatusSignal.refreshAll(
        shooterMotorVoltage,
        shooterMotorVelocity,
        shooterMotorStatorCurrent,
        shooterMotorSupplyCurrent,
        shooterMotorTemperature,
        shooterMotorReferenceSlope,
        shooterMotorFollowerVoltage,
        shooterMotorFollowerVelocity,
        shooterMotorFollowerStatorCurrent,
        shooterMotorFollowerSupplyCurrent,
        shooterMotorFollowerTemperature,
        shooterMotorFollowerReferenceSlope)
    inputs.shooterMotorVoltage = shooterMotorVoltage.valueAsDouble
    inputs.shooterMotorVelocity = shooterMotorVelocity.valueAsDouble
    inputs.shooterMotorStatorCurrent = shooterMotorStatorCurrent.valueAsDouble
    inputs.shooterMotorSupplyCurrent = shooterMotorSupplyCurrent.valueAsDouble
    inputs.shooterMotorTemperature = shooterMotorTemperature.valueAsDouble
    inputs.shooterMotorReferenceSlope = shooterMotorReferenceSlope.valueAsDouble

    inputs.shooterMotorFollowerVoltage = shooterMotorFollowerVoltage.valueAsDouble
    inputs.shooterMotorFollowerVelocity = shooterMotorFollowerVelocity.valueAsDouble
    inputs.shooterMotorFollowerStatorCurrent = shooterMotorFollowerStatorCurrent.valueAsDouble
    inputs.shooterMotorFollowerSupplyCurrent = shooterMotorFollowerSupplyCurrent.valueAsDouble
    inputs.shooterMotorFollowerTemperature = shooterMotorFollowerTemperature.valueAsDouble
    inputs.shooterMotorFollowerReferenceSlope = shooterMotorFollowerReferenceSlope.valueAsDouble
  }

  override fun setShooterVoltage(voltage: Double) {
    shooterMotor.setVoltage(voltage)
  }

  override fun setShooterVelocity(velocity: Double) {
    if (ShooterConstants.kUseMotionMagic) {
      shooterMotor.setControl(motionMagicRequest.withVelocity(velocity))
    } else {
      shooterMotor.setControl(velocityRequest.withVelocity(velocity))
    }
  }

  override fun setShooterFollowerVoltage(voltage: Double) {
    shooterMotorFollower.setVoltage(voltage)
  }

  override fun setShooterFollowerVelocity(velocity: Double) {
    if (ShooterConstants.kUseMotionMagic) {
      shooterMotorFollower.setControl(motionMagicRequestFollower.withVelocity(velocity))
    } else {
      shooterMotorFollower.setControl(velocityRequestFollower.withVelocity(velocity))
    }
  }

  override fun off() {
    shooterMotor.setControl(NeutralOut())
    shooterMotorFollower.setControl(NeutralOut())
  }
}
