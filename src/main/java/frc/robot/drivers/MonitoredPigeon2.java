// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.drivers;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.utils.WBLogger;

public class MonitoredPigeon2 {
  private int canDeviceID;
  private String canDeviceBus;
  private Pigeon2 orig;

  public MonitoredPigeon2(int deviceId, String canbus) {
    orig = new Pigeon2(deviceId, canbus);
    this.canDeviceID = deviceId;
    this.canDeviceBus = canbus;
  }

  public MonitoredPigeon2(int deviceId) {
    orig = new Pigeon2(deviceId);
    // Our Drivetrain can is on the "mani" bus which includes the Pigeon2.
    System.out.println(
        "MonitoredPigeon2: Using default canDeviceBus of 'rio' on device " + deviceId);
    this.canDeviceID = deviceId;
    this.canDeviceBus = "rio";
  }

  public WBConfigurator<Pigeon2Configuration> getConfigurator() {
    return new WBConfigurator<Pigeon2Configuration>(
        (Pigeon2Configuration x) -> orig.getConfigurator().apply(x));
  }

  public void setYaw(double n) {
    this.orig.setYaw(n);
  }

  public StatusSignal<Double> getYaw() {
    return WBLogger.getInstance()
        .monitorStatusSignal(
            orig.getYaw(),
            canDeviceID,
            canDeviceBus,
            "MonitoredPidgeon2",
            "MonitoredPigeon2.getYaw()");
  }
}
