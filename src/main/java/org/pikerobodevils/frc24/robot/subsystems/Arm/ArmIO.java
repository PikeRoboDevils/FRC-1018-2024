package org.pikerobodevils.frc24.robot.subsystems.Arm;

import org.littletonrobotics.junction.AutoLog;
import org.pikerobodevils.frc24.robot.Constants;
import org.pikerobodevils.frc24.robot.Constants.Mode;

public interface ArmIO {

  @AutoLog
  public static class ArmIOInputs {

    public double Position;
    public double Angle;
    public boolean Goal;
  }

  /** default Methods here then overide in real */
  public default void updateInputs(ArmIOInputs inputs) {}

  public default void reset() {}

  public default void updatePositionController() {}

  public default void updatePositionController(double asDouble) {}

  public default void setGoal(double valueRadians) {}

  public default void setSpeed(double speed) {}

  public default void setVoltage(double volts) {}

  public default void holdPositionCommand() {}

  public static ArmIO isReal() {
    if (Constants.currentMode == Mode.REAL) {
      return new REALArm();
    } else {
      return new SIMArm();
    }
  }
}
