// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.drivers;

import com.ctre.phoenix6.StatusCode;
import frc.robot.utils.WBLogger;
import java.util.function.Function;

public class WBConfigurator<T> {
  private final Function<T, StatusCode> orig;
  private final int kNumTries = 5;

  public WBConfigurator(Function<T, StatusCode> org) {
    this.orig = org;
  }

  public StatusCode apply(T config) {
    return apply(config, kNumTries);
  }

  public StatusCode apply(T config, int numTries) {
    for (int i = 0; i < numTries; i++) {
      StatusCode code;
      if ((code = StatusCodeChecker.checkErrorAndRetry(() -> orig.apply(config)))
          == StatusCode.OK) {
        // API says we applied config, lets make sure it's right
        if (true) {
          // if (readAndVerifyConfiguration(talon, config)) {
          return code;
        } else {
          // WBLogger.getInstance().warn("Failed to verify config for talon [" +
          // talon.getDescription()
          // + "] (attempt " + (i + 1) + " of " + numTries + ")");
        }
      } else {
        WBLogger.getInstance()
            .warn("Failed to apply config. (attempt " + (i + 1) + " of " + numTries + ")");
      }
    }
    WBLogger.getInstance()
        .error("Failed to apply config for talon after " + numTries + " attempts");
    return StatusCode.GeneralError;
  }
}
