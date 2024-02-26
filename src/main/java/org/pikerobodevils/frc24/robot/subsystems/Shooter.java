// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.pikerobodevils.frc24.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final CANSparkMax shooterLead = new CANSparkMax(13,CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax shooterFollow = new CANSparkMax(14,CANSparkLowLevel.MotorType.kBrushless);

  
  /** Creates a new Shooter. */
  public Shooter() {
    shooterLead.restoreFactoryDefaults();
    shooterLead.burnFlash();
    shooterFollow.restoreFactoryDefaults();

    Timer.delay(.1);
    shooterFollow.follow(shooterLead, false);
    Timer.delay(0.1);
    shooterFollow.burnFlash();
    Timer.delay(.1);
  }
  public void setSpeed(double speed){
    shooterLead.set(speed);
  }

  public Command spinUp(){
    return run(
      ()->{
        setSpeed(.75);
      })
      .finallyDo(()->{setSpeed(.15);
      });
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
