package org.pikerobodevils.frc24.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ControlBoard {
  public final CommandXboxController driver = new CommandXboxController(0);

  public final CommandXboxController operator = new CommandXboxController(1);

  public double getSpeed() {
    double leftY = driver.getLeftY();
    return Math.signum(leftY) * Math.pow(MathUtil.applyDeadband(leftY, .04), 2);
  }

  public double getTurn() {
    double rightX = -driver.getRightX()*.80;
    return Math.signum(rightX) * Math.pow(MathUtil.applyDeadband(rightX, .04), 2);
  }

  public double getIntake()
  {
    return -driver.getLeftTriggerAxis() + driver.getRightTriggerAxis();
  }

  public void rumble(CommandXboxController controller){
    controller.getHID().setRumble(RumbleType.kBothRumble, .5);
  }

  
}