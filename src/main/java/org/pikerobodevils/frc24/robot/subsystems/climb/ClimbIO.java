package org.pikerobodevils.frc24.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;
import org.pikerobodevils.frc24.robot.Constants;
import org.pikerobodevils.frc24.robot.Constants.Mode;

public interface ClimbIO {

  @AutoLog
  public static class ClimbIOInputs {
    public double encoderPos;
    public boolean atSetpoint;
  }

  /** default Values and inputs from motors encoders and such here then overide in real */
  public default void updateInputs(ClimbIOInputs inputs) {}

  /** default Methods here then overide in real */
  public default void setPosition(double pos) {}

  public default void setSpeed(double speed) {}

  public default void setDistance(double distance) {}

  public default void resetEncoders() {}

  // for sim and shit
  public static ClimbIO isReal() {
    if (Constants.currentMode == Mode.REAL) {
      return new REALClimb();
    } else {
      return new SIMClimb();
    }
  }
}
