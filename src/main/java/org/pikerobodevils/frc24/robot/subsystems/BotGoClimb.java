// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.pikerobodevils.frc24.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.pikerobodevils.frc24.robot.Constants.ClimbConstants.*;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase; 

public class BotGoClimb extends SubsystemBase {
  //Difines we have A motor and spark max that is linked
  private final CANSparkMax ClimbLead = new CANSparkMax(MOTOR_ID,CANSparkLowLevel.MotorType.kBrushless);
  /** Creates a new BotGoClimb. */
  // Creates a new Climb. 
  public BotGoClimb() {
    ClimbLead.restoreFactoryDefaults();
    ClimbLead.burnFlash();
   
    
 
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
