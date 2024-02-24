// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;


public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final CANSparkMax intakeMotor = new CANSparkMax(9,CANSparkLowLevel.MotorType.kBrushless);

  public Intake() {
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setInverted(false);
    //intakeMotor.setSmartCurrentLimit(CURRENT_LIMIT);
    intakeMotor.burnFlash();
  }

  //Set Speed of intake motor
  public void setSpeed(double speed){
    intakeMotor.set(speed);
  } 

  public Command runIntake(){
    return run(
      ()->{
        setSpeed(.25);
      }
    ).finallyDo(()->{
      setSpeed(0);
    });
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
