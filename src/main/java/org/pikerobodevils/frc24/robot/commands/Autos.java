// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.pikerobodevils.frc24.robot.commands;

import java.util.List;

import static org.pikerobodevils.frc24.robot.Constants.DrivetrainConstants.*;
import static org.pikerobodevils.frc24.robot.Constants.ShooterConstants.SHOOT_SPEED;

import org.pikerobodevils.frc24.robot.subsystems.Arm.ArmPosition;
import org.pikerobodevils.frc24.robot.subsystems.Arm;
import org.pikerobodevils.frc24.robot.subsystems.Drivetrain;
import org.pikerobodevils.frc24.robot.subsystems.ExampleSubsystem;
import org.pikerobodevils.frc24.robot.subsystems.Intake;
import org.pikerobodevils.frc24.robot.subsystems.Shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */
  // public static Command exampleAuto(ExampleSubsystem subsystem) {
  //   return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  // }
  
  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

    public static Command ShootSubwooferAuto(Shooter shooterSubsystem, Arm arm, Intake intakeSubsystem){
    return Commands.runOnce(()->intakeSubsystem.runIntake(.25))
    .andThen(arm.setGoalCommand(ArmPosition.INTAKE))
      .andThen(shooterSubsystem.spinUp().raceWith(arm.setGoalCommand(ArmPosition.SUBWOOFER)))
      .withTimeout(2)
      .andThen(intakeSubsystem.shoot());
  }

  public static Command getAutonomousCommand(Drivetrain m_robotDrive) {
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                KS,
                KV,
                KA),
                kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                3,
                1)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(-75))),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(.25, -.25)),
            // End 3 meters straight ahead of where we started, facing forw/ard
            new Pose2d(5, -.5, new Rotation2d(0)),
            // Pass config
            config);

                // An example trajectory to follow. All units in meters.
    Trajectory driveBack =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(1, 0, new Rotation2d(Units.degreesToRadians(0))),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(.25, 0)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass config
            config);
        

    

    // Reset odometry to the initial pose of the trajectory, run path following
    // command, then stop at the end.
    return Commands.runOnce(()->m_robotDrive.resetEncoders())
    .andThen(Commands.runOnce(()->m_robotDrive.resetGyro()))
    .andThen(Commands.runOnce(() -> m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose())));
  }
}
