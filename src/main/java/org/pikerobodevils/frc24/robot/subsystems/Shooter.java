// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.pikerobodevils.frc24.robot.subsystems;

import static org.pikerobodevils.frc24.robot.Constants.ShooterConstants.*;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class Shooter extends SubsystemBase {
  private final CANSparkMax shooterLead = new CANSparkMax(10,CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax shooterFollow = new CANSparkMax(11,CANSparkLowLevel.MotorType.kBrushless);

  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(KS, KV, KA);
  private final RelativeEncoder encoder = shooterLead.getEncoder();
  

   // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Angle> m_angle = mutable(Rotations.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));

  // Create a new SysId routine for characterizing the shooter.
  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motor(s).
              (Measure<Voltage> volts) -> {
                shooterLead.setVoltage(volts.in(Volts));
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the shooter motor.
                log.motor("shooter-wheel")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            shooterLead.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
                    .angularPosition(m_angle.mut_replace(encoder.getPosition(), Rotations))
                    .angularVelocity(
                        m_velocity.mut_replace(encoder.getVelocity()/60, RotationsPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("shooter")
              this));
  /** Creates a new Shooter. */
  public Shooter() {
    shooterLead.restoreFactoryDefaults();
    shooterLead.setSmartCurrentLimit(30);
    shooterLead.burnFlash();
    shooterFollow.restoreFactoryDefaults();
    shooterFollow.setSmartCurrentLimit(30);

    Timer.delay(.1);
    shooterFollow.follow(shooterLead, false);
    Timer.delay(0.1);
    shooterFollow.burnFlash();
    Timer.delay(.1);

   // setDefaultCommand(spin());
  }
  public void setSpeed(double speed){
    shooterLead.set(speed);
  }

  public double getVelocity(){
    return encoder.getVelocity();
  }

  public boolean shootReady(){
    return encoder.getVelocity()>=SHOOT_SPEED;
  }


  public Command spin()
  {
   return run(()->setSpeed(feedforward.calculate(CONSTANT_VELOCITY)));
  }

  public Command spinUpAmp(){
        return run(
      ()->{
        setSpeed(feedforward.calculate(SHOOT_SPEED));
      })
      .finallyDo(()->{setSpeed(feedforward.calculate(CONSTANT_VELOCITY));
      });
  }

  public Command spinUpPodium(){
    return run(
      ()->{
        setSpeed(feedforward.calculate(SHOOT_SPEED));
      })
      .finallyDo(()->{setSpeed(feedforward.calculate(CONSTANT_VELOCITY));
      });
  }

  public Command spinUp(){
    return run(
      ()->{
        setSpeed(feedforward.calculate(SHOOT_SPEED));
      })
      .finallyDo(()->{setSpeed(feedforward.calculate(CONSTANT_VELOCITY));
      });
  }

    public Command spinUp(double speed){
    return run(
      ()->{
        setSpeed(feedforward.calculate(speed));
      })
      .finallyDo(()->{setSpeed(feedforward.calculate(CONSTANT_VELOCITY));
      });
  }

   /**
   * Returns a command that will execute a quasistatic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
