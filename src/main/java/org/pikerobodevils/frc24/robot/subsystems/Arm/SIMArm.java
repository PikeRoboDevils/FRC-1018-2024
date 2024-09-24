// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package org.pikerobodevils.frc24.robot.subsystems.Arm;

import static org.pikerobodevils.frc24.robot.Constants.ArmConstants.CONSTRAINTS;
import static org.pikerobodevils.frc24.robot.Constants.ArmConstants.KA;
import static org.pikerobodevils.frc24.robot.Constants.ArmConstants.KG;
import static org.pikerobodevils.frc24.robot.Constants.ArmConstants.KS;
import static org.pikerobodevils.frc24.robot.Constants.ArmConstants.KV;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class SIMArm implements ArmIO {
  private final SingleJointedArmSim arm =
      new SingleJointedArmSim(
          DCMotor.getNEO(2),
          3, // I guessed
          SingleJointedArmSim.estimateMOI(0.889, 30), // no idea moment of inertia yap
          0.889,
          0,
          Math.PI * 2,
          true,
          0);
  ArmFeedforward feedforward = new ArmFeedforward(KS, KG, KV, KA);
  ProfiledPIDController controller = new ProfiledPIDController(3, 1, 100, CONSTRAINTS);

  @Override
  public void updateInputs(ArmIOInputs inputs) {

    arm.update(.02);
    inputs.Position = MathUtil.angleModulus(arm.getAngleRads());
    inputs.Angle = Units.radiansToDegrees(inputs.Position);
    inputs.Goal = controller.atGoal();
  }

  @Override
  public void updatePositionController() {

    controller.disableContinuousInput();
    // Update controller with new measurement
    // This updates the controllers setpoint internally.
    var feedbackOutput = controller.calculate(MathUtil.angleModulus(arm.getAngleRads()));
    // use the newly updated setpoint to calculate a feedforward.
    var setpoint = controller.getSetpoint();
    var feedforwardOutput =
        feedforward.calculate(MathUtil.angleModulus(arm.getAngleRads()), setpoint.velocity);
    var totalOutputVolts = feedbackOutput + feedforwardOutput;
    setVoltage(feedbackOutput);
  }

  @Override
  public void updatePositionController(double setpoint) {
    setGoal(setpoint);
    updatePositionController();
  }

  @Override
  public void setGoal(double goal) {
    controller.setGoal(goal);
  }

  @Override
  public void setVoltage(double volts) {
    arm.setInputVoltage(volts);
  }

  @Override
  public void setSpeed(double speed) {
    arm.setState(arm.getAngleRads(), speed);
  }

  @Override
  public void reset() {
    controller.reset(MathUtil.angleModulus(arm.getAngleRads()));
  }

  @Override
  public void holdPositionCommand() {
    arm.setInputVoltage(0);
    arm.setState(arm.getAngleRads(), 0);
  }
}
