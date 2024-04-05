// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.pikerobodevils.frc24.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;

import static org.pikerobodevils.frc24.robot.Constants.ArmConstants.*;

// import io.github.oblarg.oblog.Loggable;
// import io.github.oblarg.oblog.annotations.Log;
import java.util.function.DoubleSupplier;

import org.pikerobodevils.frc24.lib.vendor.SparkMax;
import org.pikerobodevils.frc24.lib.vendor.SparkMaxUtils;

public class Arm extends SubsystemBase {

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

  SparkMax leftController =
      new SparkMax(LEFT_CONTROLLER_ID, CANSparkLowLevel.MotorType.kBrushless);
  SparkMax rightController =
      new SparkMax(RIGHT_CONTROLLER_ID, CANSparkLowLevel.MotorType.kBrushless);

  Encoder encoder = new Encoder(ENCODER_QUAD_A, ENCODER_QUAD_B, true, CounterBase.EncodingType.k4X);
  DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(ENCODER_ABS_DIO);
  ArmFeedforward feedforward = new ArmFeedforward(KS, KG, KV, KA);
  ProfiledPIDController controller = new ProfiledPIDController(KP, KI, KD, CONSTRAINTS);

  // @Log(name = "Arm Simulation")
  private final Mechanism2d m_mech2d = new Mechanism2d(60, 60);

  private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 30, 30);
  private final MechanismLigament2d m_armTower =
      m_armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
  private final MechanismLigament2d m_arm =
      m_armPivot.append(new MechanismLigament2d("Arm", 30, 0, 6, new Color8Bit(Color.kYellow)));

  public Arm() {
    rightController.withInitializer(
        (spark, isInit) -> {
          int errors = 0;
          errors += SparkMaxUtils.check(spark.setIdleMode(CANSparkMax.IdleMode.kBrake));
          spark.setInverted(true);
          return errors == 0;
        });
    leftController.withInitializer(
        (spark, isInit) -> {
          int errors = 0;
          errors += SparkMaxUtils.check(spark.setIdleMode(CANSparkMax.IdleMode.kBrake));
          return errors == 0;
        });
        
    leftController.setSmartCurrentLimit(40);
    rightController.setSmartCurrentLimit(40);

    encoder.setDistancePerPulse(RAD_PER_QUAD_TICK);
    absoluteEncoder.setDistancePerRotation(RAD_PER_ENCODER_ROTATION);
    absoluteEncoder.setPositionOffset(ENCODER_OFFSET);

    setDefaultCommand(holdPositionCommand().withName("Default Hold Position"));

    controller.reset(getPosition());
    setGoal(ArmPosition.SUBWOOFER.valueRadians);

    m_armTower.setColor(new Color8Bit(Color.kBlue));
  }

  /**
   * Sets the voltage applied to the arm motors
   *
   * @param volts voltage to apply.
   */
  public void setVoltage(double volts) {
    leftController.setVoltage(volts);
    rightController.setVoltage(volts);
  }

    public void setSpeed(double speed) {
    leftController.set(speed/2);
    rightController.set(speed/2);
  }

  // @Log(name = "Voltage")
  public double getVoltage() {
    if (RobotBase.isReal()) {
      return leftController.getAppliedOutput() * leftController.getBusVoltage();
    } else {
      return leftController.getAppliedOutput();
    }
  }

  // @Log(name = "Left Current")
  public double getLeftCurrent() {
    return leftController.getOutputCurrent();
  }

  // @Log(name = "Right Current")
  public double getRightCurrent() {
    return rightController.getOutputCurrent();
  }

  public double getPosition() {
    return MathUtil.angleModulus(absoluteEncoder.getDistance());
  }

  // @Log(name = "Position")
  public double getPositionDeg() {
    return Units.radiansToDegrees(getPosition());
  }

  // @Log
  public double getQuadPositionDeg() {
    return Units.radiansToDegrees(encoder.getDistance());
  }

  /**
   * Returns the velocity of the arm in radians / second.
   *
   * @return velocity of the arm in radians / second
   */
  public double getVelocity() {
    return encoder.getRate();
  }

  // @Log(name = "Velocity")
  public double getVelocityDeg() {
    return Units.radiansToDegrees(getVelocity());
  }

  /**
   * Return whether the arm is at the desired goal.
   *
   * @return true if the arm is at the goal otherwise false.
   */
  public boolean atGoal() {
    return controller.atGoal();
  }

  /**
   * Sets the goal position of the arm controller, with a velocity goal of 0 rad/s. Does not update
   * motor outputs.
   *
   * @param goal goal position of the arm in radians.
   */
  public void setGoal(double goal) {
    controller.setGoal(goal);
  }

  // @Log(name = "Setpoint Position")
  public double getSetpointPosition() {
    return Units.radiansToDegrees(controller.getSetpoint().position);
  }

  public double getGoalPosition() {
    return controller.getGoal().position;
  }

  // @Log(name = "Setpoint Velocity")
  public double getSetpointVelocity() {
    return Units.radiansToDegrees(controller.getSetpoint().velocity);
  }

  public void updatePositionController(double setpoint) {
    setGoal(setpoint);
    updatePositionController();
  }

  public void updatePositionController() {
    // Update controller with new measurement
    // This updates the controllers setpoint internally.
    var feedbackOutput = controller.calculate(getPosition());
    // use the newly updated setpoint to calculate a feedforward.
    var setpoint = controller.getSetpoint();
    var feedforwardOutput = feedforward.calculate(getPosition(), setpoint.velocity);
    var totalOutputVolts = feedbackOutput + feedforwardOutput;
    setVoltage(-totalOutputVolts);
  }

  public Command armOverride(DoubleSupplier speed){
    return run(()->setSpeed(speed.getAsDouble())).finallyDo(()->setSpeed(0));
  }


  public Command holdPositionCommand() {
    return run(this::updatePositionController);
  }

  public Command setGoalCommand(double goalPosition) {
    return runOnce(
            () -> {
              setGoal(goalPosition);
            })
        .andThen(holdPositionCommand())
        .until(this::atGoal);
  }

  public Command setGoalCommand(ArmPosition goalPosition) {
    return runOnce(
            () -> {
              setGoal(goalPosition.valueRadians);
            })
        .andThen(holdPositionCommand())
        .until(this::atGoal);
  }
    public Command setGoalCommand(ArmPosition goalPosition, boolean hold) {
    return runOnce(
            () -> {
              setGoal(goalPosition.valueRadians);
            })
        .andThen(holdPositionCommand());
  }

  public Command continuousGoalCommand(DoubleSupplier goalSupplier) {
    return run(
        () -> {
          updatePositionController(goalSupplier.getAsDouble());
        });
  }

  @Override
  public void periodic() {
    if (!DriverStation.isEnabled()) {
      controller.reset(getPosition());
    }
    m_arm.setAngle(Units.radiansToDegrees(getPosition()));
  }
}
