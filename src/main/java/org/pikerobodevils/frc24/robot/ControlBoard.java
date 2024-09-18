package org.pikerobodevils.frc24.robot;

import org.pikerobodevils.frc24.robot.Constants.DrivetrainConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ControlBoard {
  
  public final CommandXboxController driver = new CommandXboxController(0);
  public final CommandXboxController operator = new CommandXboxController(1);

 //Creates a SlewRateLimiter that limits the rate of change of the signal to 0.5 units per second
  SlewRateLimiter filter = new SlewRateLimiter(DrivetrainConstants.SlewRateLimiter);

  //reordered for clarity but....
  public double getSpeed() {
    double leftY = driver.getLeftY();//not negated because common sense 
    return Math.signum(leftY) * //Signum is for 0 or not 0 drift 
      filter.calculate( //SlewRateLimiter cant go faster than that rate 
       Math.pow(
        MathUtil.applyDeadband(leftY, .04), 2));
  }
  public double getSpeedRIGHT() {
    double rightY = driver.getRightY();
    return Math.signum(rightY) * //Signum is for 0 or not 0 drift 
      filter.calculate( //SlewRateLimiter cant go faster than that rate 
       Math.pow(
        MathUtil.applyDeadband(rightY, .04), 2));
  }

  public double getTurn() {
    double rightX = driver.getRightX()*.80; //not negated because common sense 
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