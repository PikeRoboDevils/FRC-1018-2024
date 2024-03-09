// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.pikerobodevils.frc24.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import static org.pikerobodevils.frc24.robot.Constants.IntakeConstants.*;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final CANSparkMax intakeMotor = new CANSparkMax(MOTOR_ID,CANSparkLowLevel.MotorType.kBrushless);
  private final DigitalInput irDetector = new DigitalInput(IR_PORT);

  public Intake() {
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setInverted(true);
    intakeMotor.setSmartCurrentLimit(40);
    intakeMotor.burnFlash();
  }

  //Set Speed of intake motor
  public void setSpeed(double speed){
    intakeMotor.set(speed);
  } 
  //gets value of intake ir sensor
  public boolean hasNote(){
    return !irDetector.get();
  }

  public Command runIntake(double speed){
    return run(
      ()->{
        setSpeed(speed);
      })
      .until(()->hasNote())      
      .finallyDo(()->{
      setSpeed(0);
    });
  }

  public Command runOuttake(){
    return run(
      ()->{
        setSpeed(DEFAULT_OUTTAKE);
      })
      .finallyDo(()->{setSpeed(0);
      });
  }

  public Command shoot(){
    return run(
      ()->{
        setSpeed(SHOOT_SPEED);
      })
      .finallyDo(()->{setSpeed(0);
      });
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
