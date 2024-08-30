// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.pikerobodevils.frc24.robot;

import static org.pikerobodevils.frc24.robot.Constants.ShooterConstants.SHOOT_SPEED;

import org.pikerobodevils.frc24.robot.Constants.OperatorConstants;
import org.pikerobodevils.frc24.robot.commands.Autos;
import org.pikerobodevils.frc24.robot.subsystems.Arm;
import org.pikerobodevils.frc24.robot.subsystems.Arm.ArmPosition;
import org.pikerobodevils.frc24.robot.subsystems.BotGoClimb;
import org.pikerobodevils.frc24.robot.subsystems.Drivetrain;
import org.pikerobodevils.frc24.robot.subsystems.Intake;
import org.pikerobodevils.frc24.robot.subsystems.Shooter;
import org.pikerobodevils.frc24.robot.subsystems.Vision;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ControlBoard controlboard = new ControlBoard();
  private final Intake intakeSubsystem = new Intake(controlboard);
  private final Drivetrain drivetrain = new Drivetrain();
  private final Shooter shooterSubsystem = new Shooter();
  private final BotGoClimb climber = new BotGoClimb(); 
  private final Arm arm = new Arm();
  private final Vision vision = new Vision();
  private final ShuffleboardTab shuffleboard = Shuffleboard.getTab("Driver Dashboard");
  SendableChooser<Command> autoChooser = new SendableChooser<>();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
  
    drivetrain.setDefaultCommand(
        drivetrain
            .arcadeDriveCommand(controlboard::getSpeed, controlboard::getTurn)
            .withName("Default Drive Command"));



    //for tracing data idk if itll work or not edit: It works
    SmartDashboard.putData("Field", drivetrain.m_field);



    shuffleboard.addBoolean("Has Note",()->intakeSubsystem.hasNote());
    shuffleboard.addDouble("right velocity", ()->drivetrain.getRightVelocity());
    shuffleboard.addDouble("left velocity", ()->drivetrain.getLeftVelocity());
    shuffleboard.addDouble("left distance", ()->drivetrain.getLeftDistance() );
    shuffleboard.addDouble("right distance", ()->drivetrain.getRightDistance());
    shuffleboard.addDouble("Arm Deg", ()->arm.getPositionDeg());
    shuffleboard.addBoolean("At Arm Goal", ()->arm.atGoal());
    shuffleboard.addDouble("Climb Position", ()->climber.getPosition());
    shuffleboard.addDouble("Shooter Velocity", ()->shooterSubsystem.getVelocity());
    shuffleboard.addDouble("Rotation", ()->drivetrain.getYaw());
    shuffleboard.addDouble("PoseX", ()->drivetrain.getPose().getX());
   shuffleboard.addDouble("PoseY", ()->drivetrain.getPose().getY());
shuffleboard.addDouble("rotation2d", ()->drivetrain.getPose().getRotation().getDegrees());

    // Another option that allows you to specify the default auto by its name
    autoChooser.addOption("shoot move", Autos.getAutonomousCommand(drivetrain, shooterSubsystem, arm, intakeSubsystem));
     autoChooser.addOption("shoot no move", Autos.ShootSubwooferAuto(shooterSubsystem, arm, intakeSubsystem));
   //   autoChooser.addOption("move", Autos.DriveBack(drivetrain, .2));
          autoChooser.addOption("Middle High", Autos.twoNoteDrive(shooterSubsystem, drivetrain, arm, intakeSubsystem));
    autoChooser.addOption("Source RED", Autos.sourceRSide(shooterSubsystem, drivetrain, arm, intakeSubsystem));
    autoChooser.addOption("Source BLUE", Autos.sourceBSide(shooterSubsystem, drivetrain, arm, intakeSubsystem));
    autoChooser.addOption("three note", Autos.threeNote(shooterSubsystem, drivetrain, arm, intakeSubsystem));
     autoChooser.addOption("Amp RED", Autos.ampRSide(shooterSubsystem, drivetrain, arm, intakeSubsystem));
     autoChooser.addOption("Amp BLUE", Autos.ampBSide(shooterSubsystem, drivetrain, arm, intakeSubsystem));
    autoChooser.addOption("DRIVE", Autos.justDrive(drivetrain, 1.0));
        autoChooser.addOption("tubro", Autos.turbo(shooterSubsystem,drivetrain));

    autoChooser.addOption("TESTSTAGE", Autos.ShootStageAuto(shooterSubsystem, arm, intakeSubsystem));
    shuffleboard.add("Auto Chooser", autoChooser);
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

   // Sys ID buttons:
    // controlboard.driver.a().whileTrue((shooterSubsystem.sysIdQuasistatic(Direction.kForward)).finallyDo(()->shooterSubsystem.setSpeed(0)));
    // controlboard.driver.b().whileTrue((shooterSubsystem.sysIdQuasistatic(Direction.kReverse)).finallyDo(()->shooterSubsystem.setSpeed(0)));
    // controlboard.driver.x().whileTrue((shooterSubsystem.sysIdDynamic(Direction.kForward)).finallyDo(()->shooterSubsystem.setSpeed(0)));
    // controlboard.driver.y().whileTrue((shooterSubsystem.sysIdDynamic(Direction.kReverse)).finallyDo(()->shooterSubsystem.setSpeed(0)));

    controlboard.driver.leftBumper().whileTrue(shooterSubsystem.spinUp(SHOOT_SPEED));
    controlboard.driver.rightBumper().whileTrue(arm.setGoalCommand(ArmPosition.SAFE));
    controlboard.driver.a().onTrue(shooterSubsystem.spinUpAmp(()->!intakeSubsystem.hasNote()));
    controlboard.driver.x().whileTrue(intakeSubsystem.shoot());
    controlboard.driver.leftTrigger().whileTrue(intakeSubsystem.runOuttake());
    controlboard.driver.rightTrigger().whileTrue(intakeSubsystem.runIntake(.75).andThen(intakeSubsystem.stopInake()));
    

    controlboard.operator.rightBumper().whileTrue(arm.armOverride(controlboard.operator::getLeftY));
    controlboard.operator.leftBumper().whileTrue(climber.climbOverride(()->controlboard.operator.getRawAxis(1)));

    controlboard.operator.leftTrigger(.75).whileTrue(arm.setGoalCommand( ArmPosition.FPODIUM));
    controlboard.operator.rightTrigger(.75).whileTrue((arm.setGoalCommand(ArmPosition.HPODIUM)));
    controlboard.operator.y().whileTrue(arm.setGoalCommand(ArmPosition.PODIUM));
    

    controlboard.operator.a().whileTrue(arm.setGoalCommand(ArmPosition.INTAKE));
    controlboard.operator.b().whileTrue(arm.setGoalCommand(ArmPosition.AMP));
    controlboard.operator.x().whileTrue(arm.setGoalCommand(ArmPosition.SUBWOOFER));
    controlboard.operator.povUp().onTrue(climber.climberUp());
    controlboard.operator.povDown().onTrue(climber.climberDown());
    controlboard.operator.povLeft().onTrue(arm.setGoalCommand(ArmPosition.AMP).andThen(()->climber.resetEncoders()));
    controlboard.operator.povRight().onTrue(arm.setGoalCommand(ArmPosition.STOW));
    controlboard.operator.start().onTrue(Commands.runOnce(()->climber.resetEncoders()));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return Autos.ShootSubwooferAuto(s);
      return autoChooser.getSelected();
     
      // .andThen(shooterSubsystem.spinUp())
      // .alongWith(arm.setGoalCommand(ArmPosition.SUBWOOFER))
      // .alongWith(intakeSubsystem.shoot())
      // .onlyWhile(()->shooterSubsystem.shootReady());
  }
  public void robotPeriodic() {
    // update the dashboard mechanism's state
   //m_arm.setAngle(arm.getPositionDeg());
  }

}
