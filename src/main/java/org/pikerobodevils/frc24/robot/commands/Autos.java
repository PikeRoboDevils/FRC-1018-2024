// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.pikerobodevils.frc24.robot.commands;

import java.util.List;

import static org.pikerobodevils.frc24.robot.Constants.DrivetrainConstants.*;
import static org.pikerobodevils.frc24.robot.Constants.ShooterConstants.SHOOT_SPEED;

import org.pikerobodevils.frc24.robot.subsystems.Arm.ArmPosition;

import com.fasterxml.jackson.databind.introspect.AnnotatedMethodCollector;

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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */
  // public static Command exampleAuto(ExampleSubsystem subsystem) {
  //   return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  // }
  
  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
public static Command DriveBack(Drivetrain drivetrain, Double speed, Double time ){
  return Commands.runOnce(()->drivetrain.resetEncoders()).andThen(new RunCommand(()->drivetrain.arcadeDriveCommand(()->-.2, ()->0.0)));
}

public static Command DriveBack(Drivetrain drivetrain, Double speed ){
  return new RunCommand(()->drivetrain.arcadeDrive(-.2, 0.0));
}

    public static Command ShootSubwooferAuto(Shooter shooterSubsystem, Arm arm, Intake intakeSubsystem){
    return Commands.runOnce(()->intakeSubsystem.runIntake(.25))
    .andThen(arm.setGoalCommand(ArmPosition.SUBWOOFER)).withTimeout(.25)
      .andThen(arm.setGoalCommand(ArmPosition.INTAKE).raceWith(shooterSubsystem.spinUp())) 
      .andThen(arm.setGoalCommand(ArmPosition.SUBWOOFER))
      .withTimeout(2)
      .andThen(intakeSubsystem.shoot())
      .withTimeout(4)
      .andThen(arm.setGoalCommand(ArmPosition.INTAKE));
  }

  public static Command getAutonomousCommand(Drivetrain drivetrain, Shooter shooter, Arm arm, Intake intake) {
    return ShootSubwooferAuto(shooter, arm, intake).withTimeout(5)
      .andThen(DriveBack(drivetrain, -.2));
}

public static Command twoNoteDrive(Shooter shooter,Drivetrain drivetrain, Arm arm, Intake intake){
  return ShootSubwooferAuto(shooter, arm, intake)
   .andThen(DriveBack(drivetrain, -.2).raceWith(arm.setGoalCommand(ArmPosition.INTAKE))
   .raceWith(intake.runIntake(.75)))
   .andThen(DriveBack(drivetrain, .2));
   
}

public static Command ampSide(Shooter shooter,Drivetrain drivetrain, Arm arm, Intake intake) {
 
  return Commands.runOnce(()->drivetrain.resetGyro()) .andThen(ShootSubwooferAuto(shooter, arm, intake))
  .andThen(DriveBack(drivetrain, -.2, 1.0))
  .andThen(drivetrain.turntoAngle(-45))
   .andThen(intake.runIntake(.75).raceWith(DriveBack(drivetrain, -.2))).withTimeout(4.)
   .andThen(DriveBack(drivetrain, .2));
}

public static Command sourceSide(Shooter shooter,Drivetrain drivetrain, Arm arm, Intake intake) {
  return Commands.runOnce(()->drivetrain.resetGyro()) .andThen(ShootSubwooferAuto(shooter, arm, intake))  
  .withTimeout(5)
  .andThen(DriveBack(drivetrain, -.2, 6.0))
  .andThen(drivetrain.turntoAngle(45))
  .andThen(intake.runIntake(.75).raceWith(DriveBack(drivetrain, -.2)))
   .andThen(DriveBack(drivetrain, .2));
}

public static Command justDrive(Drivetrain drivetrain){
  Trajectory exampleTrajectory =
  TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(0, 0, new Rotation2d(-45)),
      // Pass through these two interior waypoints, making an 's' curve path
      List.of( new Translation2d(.25, .5)),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(1, 1, new Rotation2d(0)),
      // Pass config
      drivetrain.config);
  return Commands.runOnce(()->drivetrain.resetGyro()) 
  .andThen(drivetrain.getAutonomousCommand(()->exampleTrajectory)); 
}
}
