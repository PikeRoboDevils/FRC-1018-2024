// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final CANSparkMax intakeMotor = new CANSparkMax(9,CANSparkLowLevel.MotorType.kBrushless);
  private final DigitalInput irDetector = new DigitalInput(9);

  public Intake() {
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setInverted(true);
    //intakeMotor.setSmartCurrentLimit(CURRENT_LIMIT);
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

  public Command runIntake(){
    return run(
      ()->{
        setSpeed(.25);
      })
      .until(()->hasNote())      
      .finallyDo(()->{
      setSpeed(0);
    });
  }

  public Command runOuttake(){
    return run(
      ()->{
        setSpeed(-.25);
      })
      .finallyDo(()->{setSpeed(0);
      });
  }

  public Command shoot(){
    return run(
      ()->{
        setSpeed(.5);
      })
      .finallyDo(()->{setSpeed(0);
      });
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
