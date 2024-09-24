// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.pikerobodevils.frc24.robot.subsystems.climb;

import static org.pikerobodevils.frc24.robot.Constants.ClimbConstants.*;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class BotGoClimb extends SubsystemBase {

  // sim mechanisms
  // the main mechanism object
  private final MechanismLigament2d m_climber;
  private final Mechanism2d mech = new Mechanism2d(0, 0);
  // the mechanism root node
  private final MechanismRoot2d root = mech.getRoot("BotGoClimb", -0.5, 0.1524);

  {

    // MechanismLigament2d objects represent each "section"/"stage" of the mechanism, and are based
    // off the root node or another ligament object
    m_climber = root.append(new MechanismLigament2d("climber", 10, 90));

    // post the mechanism to the dashboard
    SmartDashboard.putData("Climber", mech);
  }

  // advantage stuff
  private final ClimbIO io;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  /** Creates a new BotGoClimb. */
  // Creates a new Climb.
  public BotGoClimb(ClimbIO io) {
    this.io = io;
  }

  public Command climberUp() {
    return run(() -> io.setDistance(CLIMB_DISTANCE))
        .until(() -> inputs.atSetpoint)
        .finallyDo(() -> io.setSpeed(0));
  }

  public Command climbOverride(DoubleSupplier speed) {
    return run(() -> io.setSpeed(speed.getAsDouble())).finallyDo(() -> io.setSpeed(0));
  }

  public Command climberDown() {
    return run(() -> io.setDistance(0))
        .until(() -> inputs.atSetpoint)
        .finallyDo(() -> io.setSpeed(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_climber.setLength(inputs.encoderPos + 0.4);
  }

  public double getPosition() {
    return inputs.encoderPos;
  }
}
