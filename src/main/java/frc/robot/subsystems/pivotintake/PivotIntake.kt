// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.
package frc.robot.subsystems.pivotintake

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.StartEndCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger

class PivotIntake(private val pivotIntakeIO: PivotIntakeIO) : SubsystemBase() {
    private val pivotIntakeIOAutoLogged: PivotIntakeIOInputsAutoLogged = PivotIntakeIOInputsAutoLogged()

    override fun periodic() {
        pivotIntakeIO.updateInputs(pivotIntakeIOAutoLogged)
        Logger.processInputs(this.javaClass.name, pivotIntakeIOAutoLogged)
    }

    fun setPosition(position: Double): Command {
        return StartEndCommand(
            { pivotIntakeIO.setPosition(position * PivotIntakeConstants.kPivotMotorGearing) },
            {},
            this
        )
    }

    fun setVoltage(voltage: Double): Command {
        return StartEndCommand(
            { pivotIntakeIO.setVoltage(voltage) }, { pivotIntakeIO.setVoltage(0.0) }, this
        )
    }

    fun off(): Command {
        return StartEndCommand({ pivotIntakeIO.off() }, {}, this)
    }

    fun slamZero(): Command {
        return object : Command() {
            override fun initialize() {
                pivotIntakeIO.setVoltage(PivotIntakeConstants.kPivotSlamShooterVoltage)
            }

            override fun end(interrupted: Boolean) {
                pivotIntakeIO.off()
                if (!interrupted) {
                    pivotIntakeIO.zero()
                }
            }

            override fun isFinished(): Boolean {
                return (pivotIntakeIOAutoLogged.pivotIntakeMotorStatorCurrent
                        > PivotIntakeConstants.kPivotSlamStallCurrent)
            }
        }
    }

    fun slamAndPID(): Command {
        return SequentialCommandGroup(this.setPosition(0.0), this.slamZero())
    }

    fun zero(): Command {
        return StartEndCommand({ pivotIntakeIO.zero() }, {}, this)
    }
}
