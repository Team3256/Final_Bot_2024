// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.
package frc.robot.subsystems.led

import com.ctre.phoenix.ErrorCode
import com.ctre.phoenix.led.*
import frc.robot.Constants
import java.util.function.Consumer

enum class IndicatorAnimation {
    /*
   * DefaultConfig(
   * new LarsonAnimation(
   * 255,
   * 0,
   * 255,
   * 1,
   * Constants.LEDConstants.kLEDSpeed,
   * Constants.LEDConstants.kLEDLength,
   * LarsonAnimation.BounceMode.Center,
   * 6)),
   * IntakeIsOpenAndRunningConfig(
   * new SingleFadeAnimation(
   * 255, 150, 0, 1, Constants.LEDConstants.kLEDSpeed / 2,
   * Constants.LEDConstants.kLEDLength)),
   */
    RobotAligned(RainbowAnimation()),
    Default(LarsonAnimation(255, 0, 255, 1, 0.50, 264, LarsonAnimation.BounceMode.Center, 6)),

    // Default(new RainbowAnimation(0.78, 0.24, 264)),
    IntakeIsOpenAndRunning(StrobeAnimation(255, 0, 0, 127, 0.1, 264)),
    NoteIntaken(StrobeAnimation(0, 255, 0, 127, 0.1, 264)),
    SpeakerScore(StrobeAnimation(255, 165, 0, 127, 0.1, 264)),
    AmpScore(StrobeAnimation(255, 165, 0, 127, 0.1, 264)),
    AzimuthRan(StrobeAnimation(255, 255, 0, 127, 0.3, 264)),
    BeamBreakTriggered(SingleFadeAnimation(0, 100, 0, 5, 0.5, 264));

    val value: Consumer<CANdle?>

    constructor(animation: Consumer<CANdle?>) {
        this.value = animation
    }

    constructor(animation: Animation) {
        this.value =
            Consumer { candle: CANdle? ->
                val error = candle!!.animate(animation)
                if (error != ErrorCode.OK && Constants.FeatureFlags.kDebugEnabled) {
                    println("CANdle Error (Unable to set animation). Error Code: $error")
                    println("Traceback (most recent call last)")
                    for (ste in Thread.currentThread().stackTrace) {
                        println(ste.toString() + "\n")
                    }
                }
            }
    }
}
