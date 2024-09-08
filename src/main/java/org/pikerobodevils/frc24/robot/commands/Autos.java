// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.pikerobodevils.frc24.robot.commands;

import java.util.List;

import javax.print.attribute.standard.MediaSize.JIS;

import static org.pikerobodevils.frc24.robot.Constants.DrivetrainConstants.*;
import static org.pikerobodevils.frc24.robot.Constants.ShooterConstants.SHOOT_SPEED;

import org.pikerobodevils.frc24.robot.subsystems.Arm.ArmPosition;
import org.pikerobodevils.frc24.robot.subsystems.Shooter.SUBShooter;
import org.pikerobodevils.frc24.robot.subsystems.drive.SUBDrive;

import com.fasterxml.jackson.databind.introspect.AnnotatedMethodCollector;

import org.pikerobodevils.frc24.robot.subsystems.Arm;
import org.pikerobodevils.frc24.robot.subsystems.ExampleSubsystem;
import org.pikerobodevils.frc24.robot.subsystems.Intake;

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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
public static Command DriveBack(SUBDrive drivetrain, Double speed, Double time ){
  return Commands.runOnce(()->drivetrain.resetEncoders()).andThen(new RunCommand(()->drivetrain.arcadeDriveCommand(()->-.2, ()->0.0)));
}

public static Command DriveBack(SUBDrive drivetrain, Double speed ){
  return new RunCommand(()->drivetrain.arcadeDrive(-.2, 0.0));
}

    public static Command ShootSubwooferAuto(SUBShooter shooterSubsystem, Arm arm, Intake intakeSubsystem){
    return Commands.runOnce(()->intakeSubsystem.runIntake(.25))
    .andThen(arm.setGoalCommand(ArmPosition.SUBWOOFER)).withTimeout(.125)
      .andThen(arm.setGoalCommand(ArmPosition.INTAKE).raceWith(shooterSubsystem.spinUp())) 
      .andThen(arm.setGoalCommand(ArmPosition.SUBWOOFER))
      .withTimeout(2)
      .andThen(intakeSubsystem.shoot())
      .withTimeout(3)
      .andThen(arm.setGoalCommand(ArmPosition.INTAKE)).andThen(shooterSubsystem.spin());
  }

      public static Command ShootStageAuto(SUBShooter shooterSubsystem, Arm arm, Intake intakeSubsystem){
    return Commands.runOnce(()->intakeSubsystem.runIntake(.25)).raceWith(shooterSubsystem.spinUp())
      .andThen(arm.setGoalCommand(ArmPosition.PODIUM))
      .withTimeout(2)
      .andThen(intakeSubsystem.shoot())
      .withTimeout(3)
      .andThen(arm.setGoalCommand(ArmPosition.INTAKE));
      //to stop shooter
   //  .andThen(shooterSubsystem.spin());
  }

  public static Command getAutonomousCommand(SUBDrive drivetrain, SUBShooter shooter, Arm arm, Intake intake) {
    return ShootSubwooferAuto(shooter, arm, intake).withTimeout(5)
      .andThen(DriveBack(drivetrain, -.2));
}

//uses depreciated Reset gyros

//public static Command twoNoteDrive(SUBShooter shooter,SUBDrive drivetrain, Arm arm, Intake intake){
//
//   return Commands.runOnce(()->drivetrain.resetGyro()).andThen(()->drivetrain.resetEncoders()) 
//   .andThen(ShootSubwooferAuto(shooter, arm, intake))
//   .andThen(intake.runIntake(.75).raceWith(drivetrain.DriveDist( 1)))
//  .andThen(ShootStageAuto(shooter, arm, intake));
// //.finallyDo(()->shooter.spin()); // doesnt do what i want; breaks everything
//  //.finallyDo(()->drivetrain.arcadeDriveCommand(()->0, ()->0)); // doesnt stop anything
//}
//
//
//public static Command threeNote(SUBShooter shooter,SUBDrive drivetrain, Arm arm, Intake intake) {
//  return  Commands.runOnce(()->drivetrain.resetGyro()).andThen(()->drivetrain.resetEncoders()) 
//  .andThen(ShootSubwooferAuto(shooter, arm, intake)) .andThen(justDrive(drivetrain, 1.5).alongWith(intake.runIntake(.5)))
//  .andThen(ShootStageAuto(shooter, arm, intake)).andThen(drivetrain.turntoAngle(90) // mid note
//  .alongWith(arm.setGoalCommand(ArmPosition.INTAKE))).andThen(justDrive(drivetrain, 3.1))
//  .andThen(drivetrain.turntoAngle(64.77)).andThen(ShootStageAuto(shooter, arm, intake))// right note
//  .andThen(drivetrain.turntoAngle(0)).andThen(justDrive(drivetrain, 6.3))
//  .andThen(drivetrain.turntoAngle(-26.7)).andThen(justDrive(drivetrain, 9.26))
//  .andThen(drivetrain.turntoAngle(0)).andThen(justDrive(drivetrain,6.5))
//  .andThen(ShootSubwooferAuto(shooter, arm, intake)).andThen(justDrive(drivetrain,7.305)) //pass mid right note
//  .andThen(drivetrain.turntoAngle(46.8)).andThen(intake.runIntake(.5).raceWith(justDrive(drivetrain, 9.9875)))//gets far right
//  .andThen(drivetrain.turntoAngle(0)).andThen(justDrive(drivetrain,7.22))
//  .andThen(drivetrain.turntoAngle(66.8)).andThen(justDrive(drivetrain, 5.633))
//  .andThen(ShootStageAuto(shooter, arm, intake))//shoots note
//  ;
//}
//
//
// public static Command ampRSide(SUBShooter shooter,SUBDrive drivetrain, Arm arm, Intake intake) {
//  return Commands.runOnce(()->drivetrain.resetGyro()).andThen(()->drivetrain.resetEncoders())
//  .andThen(ShootSubwooferAuto(shooter, arm, intake))
//  .andThen(justDrive(drivetrain, .25)).andThen(drivetrain.turntoAngle(-65))
//  .andThen(intake.runIntake(.5).raceWith(justDrive(drivetrain,2.0)))
//  .andThen(drivetrain.turntoAngle(-25)).andThen(ShootStageAuto(shooter, arm, intake));
//
// }
//  public static Command ampBSide(SUBShooter shooter,SUBDrive drivetrain, Arm arm, Intake intake) {
//  return Commands.runOnce(()->drivetrain.resetGyro()).andThen(()->drivetrain.resetEncoders())
//  .andThen(ShootSubwooferAuto(shooter, arm, intake))
//  .andThen(justDrive(drivetrain, .25)).andThen(drivetrain.turntoAngle(65))
//  .andThen(intake.runIntake(.5).raceWith(justDrive(drivetrain,2.0)))
//  .andThen(drivetrain.turntoAngle(25)).andThen(ShootStageAuto(shooter, arm, intake));
// }
//
//public static Command sourceRSide(SUBShooter shooter,SUBDrive drivetrain, Arm arm, Intake intake) {
//  return Commands.runOnce(()->drivetrain.resetGyro()).andThen(()->drivetrain.resetEncoders()).andThen(ShootSubwooferAuto(shooter, arm, intake))  
// .andThen(justDrive(drivetrain, .25))
// .andThen(drivetrain.turntoAngle(25)).andThen(justDrive(drivetrain, 2.5))
// .andThen(drivetrain.turntoAngle(15)).andThen(justDrive(drivetrain, 4.0));
//}
//public static Command sourceBSide(SUBShooter shooter,SUBDrive drivetrain, Arm arm, Intake intake) {
//  return Commands.runOnce(()->drivetrain.resetGyro()).andThen(()->drivetrain.resetEncoders()).andThen(ShootSubwooferAuto(shooter, arm, intake))  
//  .withTimeout(5).andThen(justDrive(drivetrain, .25))
// .andThen(drivetrain.turntoAngle(-25)).andThen(justDrive(drivetrain, 2.5))
// .andThen(drivetrain.turntoAngle(-15)).andThen(justDrive(drivetrain, 4.0));
//}

public static Command justDrive(SUBDrive drivetrain, Double distance){
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
}
  public static Command turbo(SUBShooter armsShooter, SUBDrive drivetrain) {
    return drivetrain.arcadeDriveCommand(() -> 0.0, () -> .1).withTimeout(1).andThen(drivetrain.arcadeDriveCommand(()->0.0, ()->0.0));
  }
} 