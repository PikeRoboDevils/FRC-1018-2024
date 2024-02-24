/* Copyright 2024 Pike RoboDevils, FRC Team 1018
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE.md file or
 * at https://opensource.org/licenses/MIT. */

package frc.robot.subsystems;


import static frc.robot.Constants.DrivetrainConstants.*;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import io.github.oblarg.oblog.Loggable;
// import io.github.oblarg.oblog.annotations.Log;
import java.util.function.DoubleSupplier;
import frc.lib.vendor.SparkMax;

public class Drivetrain extends SubsystemBase{

  private final SparkMax leftLeader = new SparkMax(LEFT_LEADER_ID, MotorType.kBrushless);
  private final SparkMax leftFollowerOne = new SparkMax(LEFT_FOLLOWER_ONE_ID, MotorType.kBrushless);
  private final SparkMax leftFollowerTwo = new SparkMax(LEFT_FOLLOWER_TWO_ID, MotorType.kBrushless);

  private final SparkMax rightLeader = new SparkMax(RIGHT_LEADER_ID, MotorType.kBrushless);
  private final SparkMax rightFollowerOne =
      new SparkMax(RIGHT_FOLLOWER_ONE_ID, MotorType.kBrushless);
  private final SparkMax rightFollowerTwo =
      new SparkMax(RIGHT_FOLLOWER_TWO_ID, MotorType.kBrushless);

  private final AHRS navX = new AHRS();

  LinearFilter pitchRate = LinearFilter.backwardFiniteDifference(1, 2, 0.02);

  double currentPitchRate = 0;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    leftLeader.restoreFactoryDefaults();
    leftLeader.setIdleMode(IDLE_MODE);
    leftLeader.setSmartCurrentLimit(40);

    leftFollowerOne.restoreFactoryDefaults();
    leftFollowerOne.setIdleMode(IDLE_MODE);
    leftFollowerOne.follow(leftLeader);
    leftFollowerTwo.setSmartCurrentLimit(CURRENT_LIMIT);

    leftFollowerTwo.restoreFactoryDefaults();
    leftFollowerTwo.setIdleMode(IDLE_MODE);
    leftFollowerTwo.follow(leftLeader);
    leftFollowerTwo.setSmartCurrentLimit(CURRENT_LIMIT);

    rightLeader.restoreFactoryDefaults();
    rightLeader.setIdleMode(IDLE_MODE);
    rightLeader.setInverted(true);
    rightLeader.setSmartCurrentLimit(CURRENT_LIMIT);

    rightFollowerOne.restoreFactoryDefaults();
    rightFollowerOne.setIdleMode(IDLE_MODE);
    rightFollowerOne.follow(rightLeader);
    rightFollowerTwo.setSmartCurrentLimit(CURRENT_LIMIT);

    rightFollowerTwo.restoreFactoryDefaults();
    rightFollowerTwo.setIdleMode(IDLE_MODE);
    rightFollowerTwo.follow(rightLeader);
    rightFollowerTwo.setSmartCurrentLimit(CURRENT_LIMIT);
  }

  public void setLeftRight(double left, double right) {
    leftLeader.set(left);
    rightLeader.set(right);
  }

  public void setLeftRightVoltage(double left, double right) {
    leftLeader.setVoltage(left);
    rightLeader.setVoltage(right);
  }

  public void setIdleMode(CANSparkMax.IdleMode mode) {
    leftLeader.setIdleMode(mode);
    leftFollowerOne.setIdleMode(mode);
    leftFollowerTwo.setIdleMode(mode);

    rightLeader.setIdleMode(mode);
    rightFollowerOne.setIdleMode(mode);
    rightFollowerTwo.setIdleMode(mode);
  }

//   @Log(name = "Yaw")
  public double getYaw() {
    return navX.getYaw();
  }

//   @Log(name = "Pitch")
  public double getPitch() {
    return navX.getPitch();
  }

//   @Log(name = "Pitch Rate")
  public double getPitchRate() {
    return currentPitchRate;
  }

//   @Log(name = "Roll")
  public double getRoll() {
    return navX.getRoll();
  }

//   @Log
  public double getLeftVoltage() {
    return leftLeader.getAppliedOutput() * leftLeader.getBusVoltage();
  }

//   @Log
  public double getRightVoltage() {
    return leftLeader.getAppliedOutput() * leftLeader.getBusVoltage();
  }

  public void arcadeDrive(double speed, double rotation) {
    DifferentialDrive.WheelSpeeds speeds = DifferentialDrive.arcadeDriveIK(speed, rotation, false);
    setLeftRight(speeds.left, speeds.right);
  }

  public Command arcadeDriveCommand(DoubleSupplier speed, DoubleSupplier rotation) {
    return run(() -> arcadeDrive(speed.getAsDouble(), rotation.getAsDouble()));
  }

  /*public Command driveTrajectoryCommand(Trajectory trajectory) {
    return driveTrajectoryCommand(() -> trajectory);
  }*/

  /*public Command driveTrajectoryCommand(Supplier<Trajectory> trajectory) {
    RamseteController ramsete = new RamseteController();
    PIDController leftController = new PIDController(0,0,0);
    PIDController rightController = new PIDController(0,0,0);
    return run(() -> {

    })
  }*/
  public Command setLeftRightVoltageCommand(double leftVoltage, double rightVoltage) {
    return run(() -> {
          setLeftRightVoltage(leftVoltage, rightVoltage);
        })
        .finallyDo(
            (interrupted) -> {
              setLeftRightVoltage(0, 0);
            });
  }


  @Override
  public void periodic() {
    currentPitchRate = pitchRate.calculate(getPitch());
  }
}
