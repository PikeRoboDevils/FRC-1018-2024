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

package org.pikerobodevils.frc24.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static org.pikerobodevils.frc24.robot.Constants.DrivetrainConstants.*;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SUBDrive extends SubsystemBase {

  private final DriveIO io;
  private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
  private final DifferentialDriveOdometry odometry;
  private final Pose2d pose;
  private final DifferentialDriveKinematics kinematics =
      kDriveKinematics;
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(KS, KV);
  private final SysIdRoutine sysId;

  public static final double MotorKV = KP; //473;//??????

  /** Creates a new Drive. */
  public SUBDrive(DriveIO io) {
    this.io = io;
    
     pose = new Pose2d(0, 0, new Rotation2d());


    odometry = new DifferentialDriveOdometry(io.getRotation2d(),
  getLeftPositionMeters(),
  getRightPositionMeters(),
    pose);

    // // Configure AutoBuilder for PathPlanner
    // AutoBuilder.configureLTV(
    //     this::getPose,
    //     this::setPose,
    //     () ->
    //         kinematics.toChassisSpeeds(
    //             new DifferentialDriveWheelSpeeds(
    //                 getLeftVelocityMetersPerSec(), getRightVelocityMetersPerSec())),

    //     (speeds) -> {
    //       var wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    //       driveVelocity(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
    //     },

    //     0.02,
    //     new ReplanningConfig(true,true),
    //     () ->
    //         DriverStation.getAlliance().isPresent()
    //             && DriverStation.getAlliance().get() == Alliance.Red,
    //     this);

    AutoBuilder.configureLTV(this::getPose, this::setPose, () -> kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(inputs.leftVelocity, inputs.rightVelocity)), this::runVelocity,0.02, new ReplanningConfig(), () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red, this);

    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> driveVolts(voltage.in(Volts), voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive", inputs);

    // Update odometry
    odometry.update(inputs.gyroYaw, getLeftPositionMeters(), getRightPositionMeters());
  }

  /** Run open loop at the specified voltage. */
  public void driveVolts(double leftVolts, double rightVolts) {
    io.setVoltage(leftVolts, rightVolts);
  }
    public void runVelocity(ChassisSpeeds speeds) {
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    // 5600 * 0.0762 = M
    //5600 = 473 * 12
    //(473 * 12) * 0.0762 = M
    //12 = M / (473 * 0.0762)
    //manually divide by the max set speed to achieve percentage.
    io.setVoltage(wheelSpeeds.leftMetersPerSecond* GEAR_RATIO/(MotorKV * WHEEL_RADIUS*2), wheelSpeeds.rightMetersPerSecond * GEAR_RATIO/(MotorKV * WHEEL_RADIUS*2));
  }

  // /** Run closed loop at the specified voltage. */
  // public void driveVelocity(double leftMetersPerSec, double rightMetersPerSec) {
  //   Logger.recordOutput("Drive/LeftVelocitySetpointMetersPerSec", leftMetersPerSec);
  //   Logger.recordOutput("Drive/RightVelocitySetpointMetersPerSec", rightMetersPerSec);
  //   double leftRadPerSec = leftMetersPerSec / WHEEL_RADIUS;
  //   double rightRadPerSec = rightMetersPerSec / WHEEL_RADIUS;
  //   io.setVelocity(
  //       leftRadPerSec,
  //       rightRadPerSec,
  //       feedforward.calculate(leftRadPerSec),
  //       feedforward.calculate(rightRadPerSec));
  //   //io.setVoltage(feedforward.calculate(leftRadPerSec), feedforward.calculate(rightRadPerSec));
  // }

  /** Stops the drive. */
  public void stop() {
    io.setVoltage(0.0, 0.0);
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /** Returns the current odometry pose in meters. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /** SETS the current odometry pose. */
  public void setPose(Pose2d pose) {
    odometry.resetPosition(pose.getRotation(), getLeftPositionMeters(), getRightPositionMeters(), pose);
  }
/** Resets the current odometry pose. */
  public void resetPose(Pose2d pose) {
odometry.resetPosition(inputs.gyroYaw, getLeftPositionMeters(), getRightPositionMeters(), getPose());
  }
  /** Returns the position of the left wheels in meters. */
  @AutoLogOutput
  public double getLeftPositionMeters() {
    return inputs.leftPosition;
  }

  /** Returns the position of the right wheels in meters. */
  @AutoLogOutput
  public double getRightPositionMeters() {
    return inputs.rightPosition;
  }

  /** Returns the velocity of the left wheels in meters/second. */
  @AutoLogOutput
  public double getLeftVelocityMetersPerSec() {
    return inputs.leftVelocity;
  }

  /** Returns the velocity of the right wheels in meters/second. */
  @AutoLogOutput
  public double getRightVelocityMetersPerSec() {
    return inputs.rightVelocity;
  }
  // /** Returns the average velocity in radians/second. */
  // public double getCharacterizationVelocity() {
  //   return (inputs.leftVelocityRadPerSec + inputs.rightVelocityRadPerSec) / 2.0;
  // }

  public void Brake() {
    io.Brake();
  }

  public Command arcadeDriveCommand(DoubleSupplier speed, DoubleSupplier rotation) {
    return run(() -> arcadeDrive(speed.getAsDouble(), rotation.getAsDouble()));
  }

  public void arcadeDrive(double speed, double rotation) {
    var speeds = DifferentialDrive.arcadeDriveIK(speed, rotation, true);
    io.setVoltage(speeds.left * 12.0, speeds.right * 12.0);
   
  }

  public Command tankDriveCommand(DoubleSupplier speedL, DoubleSupplier speedR){
    return run(()-> tankDrive(speedL.getAsDouble(), speedR.getAsDouble()));
  }
  

  public void tankDrive(double left, double right) {
    var speeds = DifferentialDrive.tankDriveIK(left, right, true);
    io.setVoltage(speeds.left * 12.0, speeds.right * 12.0);
  }

  public Command idkDriveCommand(DoubleSupplier speed, DoubleSupplier rotation) {
    return run(() -> idkDrive(speed.getAsDouble(), rotation.getAsDouble()));
  }
  
  /* Pretty much same as arcade drive
  */
  public void idkDrive(double speed, double rotation) {
    var speeds = DifferentialDrive.curvatureDriveIK(speed, rotation, true);
    io.setVoltage(speeds.left * 12.0, speeds.right * 12.0);
  }

  
}