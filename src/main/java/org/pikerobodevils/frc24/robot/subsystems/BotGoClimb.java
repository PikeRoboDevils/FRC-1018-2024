// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.pikerobodevils.frc24.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

import static org.pikerobodevils.frc24.robot.Constants.ClimbConstants.*;

import java.util.function.DoubleSupplier;

import org.pikerobodevils.frc24.robot.RobotContainer;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BotGoClimb extends SubsystemBase {
  // Difines we have A motor and spark max that is linked
  private final CANSparkMax ClimbLead = new CANSparkMax(LMOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax ClimbFollower = new CANSparkMax(RMOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
  private final PIDController pid = new PIDController (KP, 0, 0);
  private final RelativeEncoder encoder = ClimbLead.getEncoder();
  private final MechanismLigament2d m_climber;



      //sim mechanisms
    // the main mechanism object
    private final Mechanism2d mech = new Mechanism2d(0,0);
    // the mechanism root node
    private final MechanismRoot2d root = mech.getRoot("BotGoClimb", -0.5, 0.1524); 
    {

     
    // MechanismLigament2d objects represent each "section"/"stage" of the mechanism, and are based
    // off the root node or another ligament object
    m_climber = root.append(new MechanismLigament2d("climber", 10, 90));

    // post the mechanism to the dashboard
    SmartDashboard.putData("Climber", mech);}

  /** Creates a new BotGoClimb. */
  // Creates a new Climb.
  public BotGoClimb() {
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
  public double getPosition(){
    return encoder.getPosition();

  }
  // this sets the speed of the motor
  public void setSpeed(double speed) {
    ClimbLead.set(speed);
  }
public void setDistance(double distance){
  ClimbLead.set(pid.calculate(encoder.getPosition(),distance));
}

public void resetEncoders(){
  encoder.setPosition(0);
}
  public Command climberUp() {
    return run(() -> setDistance(CLIMB_DISTANCE))
    .until(()-> pid.atSetpoint())
    .finallyDo(() -> setSpeed(0));
  }
  public Command climbOverride(DoubleSupplier speed) {
    return run(() -> setSpeed(speed.getAsDouble())).finallyDo(() -> setSpeed(0));
  }
  
  public Command climberDown() {
    return run(() -> setDistance(0))
    .until(()-> pid.atSetpoint())
    .finallyDo(() -> setSpeed(0));
  }
  

  @Override
  public void periodic() {
   // This method will be called once per scheduler run
  m_climber.setLength(encoder.getPosition() + 0.4);
  }
}
