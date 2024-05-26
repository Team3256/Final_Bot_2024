// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.
package frc.robot.subsystems.led

import com.ctre.phoenix.led.CANdle
import com.ctre.phoenix.led.CANdleConfiguration
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import io.github.oblarg.oblog.Loggable

class LED : SubsystemBase(), Loggable {
    var currentAnimation: IndicatorAnimation = IndicatorAnimation.Default
    private val candle = CANdle(LEDConstants.kLEDCANID, "rio")

    fun DEFAULT() {}

    init {
        val config = CANdleConfiguration()
        config.stripType = CANdle.LEDStripType.RGB
        config.brightnessScalar = 0.25
        config.v5Enabled = false
        candle.configAllSettings(config)

        // candle.animate(new LarsonAnimation(255, 0, 255, 1, 0.54, 64,
        // LarsonAnimation.BounceMode.Center, 6));

        // RainbowAnimation rainbowAnim = new RainbowAnimation(1, 0.67, 64);
        // TwinkleAnimation twinkleAnim = new TwinkleAnimation(255, 0, 255);
        // StrobeAnimation strobeAnim = new StrobeAnimation(255, 255, 255, 255, 0.1,
        // 64);
        this.animate(currentAnimation)
    }

    fun reset() {
        currentAnimation = IndicatorAnimation.Default
        this.animate(currentAnimation)
    }

    fun animate(animation: IndicatorAnimation) {
        currentAnimation = animation
        currentAnimation.value.accept(candle)
        // candle.animate(IndicatorAnimation.Default.value);
    }

    fun solidAnim(r: Int, g: Int, b: Int): Command {
        return InstantCommand({ candle.setLEDs(r, g, b) })
    }

    fun setLedColor(r: Int, g: Int, b: Int, w: Int, index: Int, length: Int) {
        candle.setLEDs(r, g, b, w, index, length)
    }
}
