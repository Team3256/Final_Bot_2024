// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.drivers;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix6.StatusCode;
import frc.robot.utils.WBLogger;
import java.util.function.Supplier;

// Stolen from 254
public class StatusCodeChecker {
  /**
   * checks the specified error code for issues
   *
   * @param errorCode error code
   * @param message message to print if error happens
   */
  public static void checkError(ErrorCode errorCode, String message) {
    if (errorCode != ErrorCode.OK) {
      WBLogger.getInstance().error(message + " " + errorCode);
    }
  }

  public static StatusCode checkErrorAndRetry(Supplier<StatusCode> function, int numTries) {
    StatusCode code = function.get();
    int tries = 0;
    while (code != StatusCode.OK && tries < numTries) {
      WBLogger.getInstance().warn("Retrying CTRE Device Config " + code.getName());
      code = function.get();
      tries++;
    }
    if (code != StatusCode.OK) {
      WBLogger.getInstance()
          .error("Failed to execute phoenix pro api call after " + numTries + " attempts");
      return code;
    }
    return code;
  }

  /**
   * checks the specified error code and throws an exception if there are any issues
   *
   * @param errorCode error code
   * @param message message to print if error happens
   */
  public static void checkErrorWithThrow(ErrorCode errorCode, String message) {
    if (errorCode != ErrorCode.OK) {
      throw new RuntimeException(message + " " + errorCode);
    }
  }

  public static StatusCode checkErrorAndRetry(Supplier<StatusCode> function) {
    return checkErrorAndRetry(function, 5);
  }
}
