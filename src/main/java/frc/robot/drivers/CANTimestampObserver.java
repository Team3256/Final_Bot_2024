// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.drivers;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import java.util.Optional;
/// Copied from 254:
// https://github.com/Team254/FRC-2023-Public/blob/3856510c06ea67407be9bad08557ddba35b62c71/src/main/java/com/team254/frc2023/subsystems/Cancoders.java#L32-L55

public class CANTimestampObserver {
  private final CANcoder cancoder;
  private Optional<Double> lastTs = Optional.empty();
  private int validUpdates = 0;
  private static final int kRequiredValidTimestamps = 10;

  public CANTimestampObserver(CANcoder cancoder) {
    this.cancoder = cancoder;
  }

  public boolean hasUpdate() {
    StatusSignal<Double> absolutePositionSignal =
        cancoder.getAbsolutePosition(); // Need to call this to update ts
    double ts = absolutePositionSignal.getTimestamp().getTime();
    if (lastTs.isEmpty()) {
      lastTs = Optional.of(ts);
    }
    if (ts > lastTs.get()) {
      validUpdates++;
      lastTs = Optional.of(ts);
    }
    return validUpdates > kRequiredValidTimestamps;
  }
}
