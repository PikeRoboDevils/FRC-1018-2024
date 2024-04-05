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
      .withTimeout(3)
      .andThen(arm.setGoalCommand(ArmPosition.INTAKE));
  }

      public static Command ShootStageAuto(Shooter shooterSubsystem, Arm arm, Intake intakeSubsystem){
    return Commands.runOnce(()->intakeSubsystem.runIntake(.25))
      .andThen(arm.setGoalCommand(ArmPosition.PODIUM)) 
      .withTimeout(2)
      .andThen(intakeSubsystem.shoot())
      .withTimeout(3)
      .andThen(arm.setGoalCommand(ArmPosition.INTAKE));
      //to stop shooter
     //.andThen(shooterSubsystem.spin());
  }

  public static Command getAutonomousCommand(Drivetrain drivetrain, Shooter shooter, Arm arm, Intake intake) {
    return ShootSubwooferAuto(shooter, arm, intake).withTimeout(5)
      .andThen(DriveBack(drivetrain, -.2));
}

public static Command twoNoteDrive(Shooter shooter,Drivetrain drivetrain, Arm arm, Intake intake){

   return Commands.runOnce(()->drivetrain.resetGyro()).andThen(()->drivetrain.resetEncoders()) 
   .andThen(ShootSubwooferAuto(shooter, arm, intake)).andThen(intake.runIntake(.75)
   .alongWith(drivetrain.DriveDist( 1)))
  .andThen(ShootStageAuto(shooter, arm, intake));
 //.finallyDo(()->shooter.spin()); // doesnt do what i want; breaks everything
  //.finallyDo(()->drivetrain.arcadeDriveCommand(()->0, ()->0)); // doesnt stop anything
}


public static Command threeNote(Shooter shooter,Drivetrain drivetrain, Arm arm, Intake intake) {
// should work but make sure encoders are plugged in!!! 
  return  Commands.runOnce(()->drivetrain.resetGyro()).andThen(()->drivetrain.resetEncoders()) 
  .andThen(ShootSubwooferAuto(shooter, arm, intake)) .andThen(justDrive(drivetrain, 1.5).raceWith(intake.runIntake(.5)))
  .andThen(ShootStageAuto(shooter, arm, intake))
  .andThen(arm.setGoalCommand(ArmPosition.INTAKE).andThen(drivetrain.turntoAngle(90))
  .andThen(justDrive(drivetrain, .25).raceWith(intake.runIntake(.5))))
  .andThen(drivetrain.turntoAngle(45)).andThen(ShootStageAuto(shooter, arm, intake))
  ;
}

 public static Command ampSide(Shooter shooter,Drivetrain drivetrain, Arm arm, Intake intake) { 
  return  Commands.runOnce(()->drivetrain.resetGyro()).andThen(()->drivetrain.resetEncoders()) 
  .andThen(ShootSubwooferAuto(shooter, arm, intake))
  .andThen(()->drivetrain.resetEncoders())
  .andThen(justDrive(drivetrain, .35))
  .andThen(drivetrain.turntoAngle(-70))
  .andThen(()->drivetrain.resetEncoders())
 .andThen(intake.runIntake(.75).alongWith(justDrive(drivetrain, 1.5)))
 .andThen(ShootStageAuto(shooter, arm, intake).raceWith(drivetrain.turntoAngle(-30)));
}

public static Command sourceSide(Shooter shooter,Drivetrain drivetrain, Arm arm, Intake intake) {
  return Commands.runOnce(()->drivetrain.resetGyro()).andThen(()->drivetrain.resetEncoders()).andThen(ShootSubwooferAuto(shooter, arm, intake))  
  .withTimeout(5).andThen(justDrive(drivetrain, .25))
 .andThen(drivetrain.turntoAngle(25)).andThen(justDrive(drivetrain, 2.5))
 .andThen(drivetrain.turntoAngle(15)).andThen(justDrive(drivetrain, 4.0));
}

public static Command justDrive(Drivetrain drivetrain, Double distance){
//    Trajectory exampleTrajectory =
//    TrajectoryGenerator.generateTrajectory(
//      // Start at the origin facing the +X direction
//     new Pose2d(0, 0, new Rotation2d(0)),
//     // Pass through these two interior waypoints, making an 's' curve path
//     List.of( new Translation2d(distance/2, 0)),
//    // End 3 meters straight ahead of where we started, facing forward
//    new Pose2d(distance, 0, new Rotation2d(0)),
//    // Pass config
//    drivetrain.config);
      
  return drivetrain.DriveDist(distance);
}}