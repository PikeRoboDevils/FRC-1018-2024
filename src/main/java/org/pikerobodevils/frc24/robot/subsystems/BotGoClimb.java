// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.pikerobodevils.frc24.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.pikerobodevils.frc24.robot.Constants.ClimbConstants.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BotGoClimb extends SubsystemBase {
  // Difines we have A motor and spark max that is linked
  private final CANSparkMax ClimbLead = new CANSparkMax(MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
  private final PIDController pid = new PIDController (KP, 0, 0);
  private final RelativeEncoder encoder = ClimbLead.getEncoder();

  /** Creates a new BotGoClimb. */
  // Creates a new Climb.
  public BotGoClimb() {
    ClimbLead.restoreFactoryDefaults();
    ClimbLead.burnFlash();

  }

  // this sets the speed of the motor
  public void setSpeed(double speed) {
    ClimbLead.set(speed);
  }
public void setDistance(double distance){
  ClimbLead.set(pid.calculate(encoder.getPosition(),distance));
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
  }
}
