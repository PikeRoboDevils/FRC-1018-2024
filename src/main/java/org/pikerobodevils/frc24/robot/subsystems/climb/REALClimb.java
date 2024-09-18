
package org.pikerobodevils.frc24.robot.subsystems.climb;


import static org.pikerobodevils.frc24.robot.Constants.ClimbConstants.*;

import org.pikerobodevils.frc24.lib.vendor.SparkMax;
import org.pikerobodevils.frc24.lib.vendor.SparkMaxUtils;
import org.pikerobodevils.frc24.robot.subsystems.drive.DriveIO.DriveIOInputs;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;


public class REALClimb implements ClimbIO {

  // Difines we have A motor and spark max that is linked
  private final CANSparkMax ClimbLead = new CANSparkMax(LMOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax ClimbFollower = new CANSparkMax(RMOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
  private final PIDController pid = new PIDController (KP, 0, 0);
  private final RelativeEncoder encoder = ClimbLead.getEncoder();





  public REALClimb() {
    //intiialize
    ClimbLead.restoreFactoryDefaults();
    ClimbLead.setSmartCurrentLimit(80);
    ClimbLead.burnFlash();
   
    ClimbFollower.restoreFactoryDefaults();
    ClimbFollower.setSmartCurrentLimit(80);
    ClimbFollower.follow(ClimbLead);
    ClimbFollower.setInverted(false);
    ClimbFollower.burnFlash();
   
    encoder.setPosition(0);
}


    @Override
    public void updateInputs(ClimbIOInputs inputs) {
      inputs.encoderPos = encoder.getPosition();
      inputs.atSetpoint = pid.atSetpoint();
  }

  @Override
  public void setPosition(double pos) {
    encoder.setPosition(pos);
  }
  @Override
  // this sets the speed of the motor
  public void setSpeed(double speed) {
    ClimbLead.set(speed);
  }
@Override
public void setDistance(double distance){
  ClimbLead.set(pid.calculate(encoder.getPosition(),distance));
}
@Override
public void resetEncoders(){
  encoder.setPosition(0);
}




}
