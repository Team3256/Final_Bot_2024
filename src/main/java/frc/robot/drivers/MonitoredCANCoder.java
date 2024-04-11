// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.drivers;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.utils.WBLogger;

public class MonitoredCANCoder {
  private int canDeviceID;
  private String canDeviceBus;
  private CANcoder canCoder;
  private final double kCancoderBootAllowanceSeconds = 10.0; // 254 also uses 10 seconds

  public MonitoredCANCoder(int deviceId, String canbus) {
    canCoder = new CANcoder(deviceId, canbus);
    this.canDeviceID = deviceId;
    this.canDeviceBus = canbus;
    // if (initialize()) {
    // WBLogger.getInstance().info("CAN device " + deviceId + " has been
    // successfully initialized");
    // } else {
    // WBLogger.getInstance()
    // .info("CAN device " + deviceId + " has NOT been successfully initialized");
    // }
  }

  public WBConfigurator<CANcoderConfiguration> getConfigurator() {
    return new WBConfigurator<CANcoderConfiguration>(
        (CANcoderConfiguration x) -> canCoder.getConfigurator().apply(x));
  }

  public MonitoredCANCoder(int deviceId) {
    canCoder = new CANcoder(deviceId);
    // Our DriveTrain can is on the "mani" bus.
    System.out.println(
        "MonitoredCANCoder: Using default canDeviceBus of 'rio' on device " + deviceId);
    this.canDeviceID = deviceId;
    this.canDeviceBus = "rio";
    // if (initialize()) {
    // WBLogger.getInstance().info("CAN device " + deviceId + " has been
    // successfully initialized");
    // } else {
    // WBLogger.getInstance()
    // .info("CAN device " + deviceId + " has NOT been successfully initialized");
    // }
  }

  // public boolean initialize() {
  //   double startInitTs = Timer.getFPGATimestamp();
  //   WBLogger.getInstance().info("* Starting to init cancoders at ts " + startInitTs);
  //   CANTimestampObserver observer = new CANTimestampObserver(canCoder);
  //   while (Timer.getFPGATimestamp() - startInitTs < kCancoderBootAllowanceSeconds) {
  //     if (observer.hasUpdate()) {
  //       return true;
  //     }
  //     Timer.delay(0.1);
  //   }
  //   return false;
  // }

  public StatusSignal<Double> getAbsolutePosition() {
    return WBLogger.getInstance()
        .monitorStatusSignal(
            canCoder.getAbsolutePosition(),
            canDeviceID,
            canDeviceBus,
            "MonitoredCANCoder",
            "MonitoredCANCoder.getAbsolutePosition()");
  }
}
