// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.pikerobodevils.frc24.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import static org.pikerobodevils.frc24.robot.Constants.ArmConstants.*;

// import io.github.oblarg.oblog.Loggable;
// import io.github.oblarg.oblog.annotations.Log;
import java.util.function.DoubleSupplier;


import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SUBArm extends SubsystemBase {

  public enum ArmPosition {
    STOW(87),
    AMP(110),
    SUBWOOFER(30),
    INTAKE(0),
    PODIUM(45),
    HPODIUM(47),
    FPODIUM(50),
    SAFE(56);
    ArmPosition(double angleDegrees) {
      this.valueRadians = Units.degreesToRadians(angleDegrees);
    }

    public final double valueRadians;
  }

  private final MechanismLigament2d m_arm;
  
      //sim mechanisms
    // the main mechanism object
    @AutoLogOutput
    private final Mechanism2d mech = new Mechanism2d(1,1);
    
    // the mechanism root node
    private final MechanismRoot2d root = mech.getRoot("Arm", 0.3, 0.254);

    // AdvantageKit inputs
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  /** Creates a new arm. */
  public SUBArm(ArmIO io) {
    this.io = io;

    setDefaultCommand(holdPositionCommand().withName("Default Hold Position"));
    io.setGoal(ArmPosition.SUBWOOFER.valueRadians);
    io.reset();

    m_arm = root.append(new MechanismLigament2d("arm", 0.889, 0));


  }

  /**
   * Sets the voltage applied to the arm motors
   *
   * @param volts voltage to apply.
   */




  /**
   * Return whether the arm is at the desired goal.
   *
   * @return true if the arm is at the goal otherwise false.
   */
  public boolean atGoal() {
    return inputs.Goal;
  }




  public Command armOverride(DoubleSupplier speed){
    return run(()->io.setSpeed(speed.getAsDouble())).finallyDo(()->io.setSpeed(0));
  }


  public Command holdPositionCommand() {
    return run(()->io.updatePositionController());
  }

  public Command setGoalCommand(double goalPosition) {
    return runOnce(
            () -> {
              io.setGoal(goalPosition);
            })
        .andThen(holdPositionCommand())
        .until(()->inputs.Goal);
  }

  public Command setGoalCommand(ArmPosition goalPosition) {
    return runOnce(
            () -> {
              io.setGoal(goalPosition.valueRadians);
            })
        .andThen(holdPositionCommand())
        .until(this::atGoal);
  }
    public Command setGoalCommand(ArmPosition goalPosition, boolean hold) {
    return runOnce(
            () -> {
              io.setGoal(goalPosition.valueRadians);
            })
        .andThen(holdPositionCommand());
  }

  public Command continuousGoalCommand(DoubleSupplier goalSupplier) {
    return run(
        () -> {
          io.updatePositionController(goalSupplier.getAsDouble());
        });
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    if (!DriverStation.isEnabled()) {
      io.reset();
    }

    m_arm.setAngle(inputs.Angle);
    Logger.recordOutput("ARM", mech);
  }

  public double getPositionDeg() {
   return inputs.Angle;
  }
}
