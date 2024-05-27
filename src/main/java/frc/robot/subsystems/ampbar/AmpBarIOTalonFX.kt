// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.ampbar

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.controls.NeutralOut
import frc.robot.drivers.MonitoredTalonFX
import frc.robot.subsystems.ampbar.AmpBarIO.AmpBarIOInputs
import frc.robot.utils.PhoenixUtil
import frc.robot.utils.TalonUtil

class AmpBarIOTalonFX : AmpBarIO {
  private val ampBarMotor = MonitoredTalonFX(AmpBarConstants.kAmpBarMotorID)

  private val ampBarMotorVoltage: StatusSignal<Double> = ampBarMotor.motorVoltage
  private val ampBarMotorVelocity: StatusSignal<Double> = ampBarMotor.velocity
  private val ampBarMotorPosition: StatusSignal<Double> = ampBarMotor.position
  private val ampBarMotorStatorCurrent: StatusSignal<Double> = ampBarMotor.statorCurrent
  private val ampBarMotorSupplyCurrent: StatusSignal<Double> = ampBarMotor.supplyCurrent
  private val ampBarMotorTemperature: StatusSignal<Double> = ampBarMotor.deviceTemp

  init {
    val motorConfig = AmpBarConstants.motorConfig
    PhoenixUtil.checkErrorAndRetry { ampBarMotor.configurator.refresh(motorConfig) }
    TalonUtil.applyAndCheckConfiguration(ampBarMotor, motorConfig)

    BaseStatusSignal.setUpdateFrequencyForAll(
        AmpBarConstants.updateFrequency,
        ampBarMotorVoltage,
        ampBarMotorVelocity,
        ampBarMotorPosition,
        ampBarMotorStatorCurrent,
        ampBarMotorSupplyCurrent,
        ampBarMotorTemperature)
    ampBarMotor.optimizeBusUtilization()
  }
  override fun updateInputs(inputs: AmpBarIOInputs) {
    BaseStatusSignal.refreshAll(
        ampBarMotorVoltage,
        ampBarMotorVelocity,
        ampBarMotorPosition,
        ampBarMotorStatorCurrent,
        ampBarMotorSupplyCurrent,
        ampBarMotorTemperature)
    inputs.ampBarMotorVoltage = ampBarMotorVoltage.valueAsDouble
    inputs.ampBarMotorVelocity = ampBarMotorVelocity.valueAsDouble
    inputs.ampBarMotorPosition = ampBarMotorPosition.valueAsDouble
    inputs.ampBarMotorStatorCurrent = ampBarMotorStatorCurrent.valueAsDouble
    inputs.ampBarMotorSupplyCurrent = ampBarMotorSupplyCurrent.valueAsDouble
    inputs.ampBarMotorTemperature = ampBarMotorTemperature.valueAsDouble
  }

  override fun setVoltage(voltage: Double) {
    ampBarMotor.setVoltage(voltage)
  }

  override fun off() {
    ampBarMotor.setControl(NeutralOut())
  }

  override public val isCurrentSpiking
    get() = ampBarMotor.statorCurrent.valueAsDouble > AmpBarConstants.kAmpBarCurrentThreshold
}
