// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.pikerobodevils.frc24.robot;

import static org.pikerobodevils.frc24.robot.Constants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.pikerobodevils.frc24.robot.Constants.OperatorConstants;
import org.pikerobodevils.frc24.robot.subsystems.Arm.ArmIO;
import org.pikerobodevils.frc24.robot.subsystems.Arm.SUBArm;
import org.pikerobodevils.frc24.robot.subsystems.Arm.SUBArm.ArmPosition;
import org.pikerobodevils.frc24.robot.subsystems.Intake;
import org.pikerobodevils.frc24.robot.subsystems.Shooter;
import org.pikerobodevils.frc24.robot.subsystems.Camera;
import org.pikerobodevils.frc24.robot.subsystems.climb.BotGoClimb;
import org.pikerobodevils.frc24.robot.subsystems.climb.ClimbIO;
import org.pikerobodevils.frc24.robot.subsystems.drive.DriveIO;
import org.pikerobodevils.frc24.robot.subsystems.drive.SUBDrive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
// The robot's subsystems and commands are defined here...
public class RobotContainer {

  private final SUBDrive drivetrain = new SUBDrive(DriveIO.isReal());
  private final ControlBoard controlboard = new ControlBoard();
  private final Intake intakeSubsystem = new Intake(controlboard);
  private final Shooter shooterSubsystem = new Shooter();
  private final BotGoClimb climber = new BotGoClimb(ClimbIO.isReal());
  private final SUBArm arm = new SUBArm(ArmIO.isReal());
  private final Camera vision = new Camera();
  private final ShuffleboardTab shuffleboard = Shuffleboard.getTab("Driver Dashboard");

  // private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser("JustDrive");

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // CONTROLLER INPUT HERE
    drivetrain.setDefaultCommand(
        //  drivetrain
        //      .tankDriveCommand(controlboard::getSpeed, controlboard::getSpeedRIGHT));

        drivetrain
            .arcadeDriveCommand(controlboard::getSpeed, controlboard::getTurn)
            .withName("Default Drive Command"));

    // drivetrain
    //        .carDriveCommand(controlboard::getSpeed, controlboard::getTurn)
    //         .withName("Default Drive Command"));

    NamedCommands.registerCommand("INTAKE", intakeSubsystem.runIntakeLIMIT());
    NamedCommands.registerCommand("SPIN", shooterSubsystem.spinUp(ShooterConstants.SHOOT_SPEED));
    NamedCommands.registerCommand("SHOOT", intakeSubsystem.shoot());

    NamedCommands.registerCommand("ArmINTAKE", arm.setGoalCommand(ArmPosition.INTAKE));
    NamedCommands.registerCommand("ArmSUBWOOFER", arm.setGoalCommand(ArmPosition.SUBWOOFER));
    NamedCommands.registerCommand("ArmPODIUM", arm.setGoalCommand(ArmPosition.PODIUM));
    NamedCommands.registerCommand("ArmFARPODIUM", arm.setGoalCommand(ArmPosition.FPODIUM));
    NamedCommands.registerCommand("ArmAMP", arm.setGoalCommand(ArmPosition.AMP));

    shuffleboard.addBoolean("Has Note", () -> intakeSubsystem.hasNote());
    // shuffleboard.addDouble("right velocity", ()->drivetrain.getRightVelocity());
    // shuffleboard.addDouble("left velocity", ()->drivetrain.getLeftVelocity());
    // shuffleboard.addDouble("left distance", ()->drivetrain.getLeftDistance() );
    // shuffleboard.addDouble("right distance", ()->drivetrain.getRightDistance());
    shuffleboard.addDouble("Arm Deg", () -> arm.getPositionDeg());
    shuffleboard.addBoolean("At Arm Goal", () -> arm.atGoal());
    shuffleboard.addDouble("Climb Position", () -> climber.getPosition());
    shuffleboard.addDouble("Shooter Velocity", () -> shooterSubsystem.getVelocity());
    // shuffleboard.addDouble("Rotation", ()->drivetrain.getYaw());
    shuffleboard.addDouble("PoseX", () -> drivetrain.getPose().getX());
    shuffleboard.addDouble("PoseY", () -> drivetrain.getPose().getY());
    shuffleboard.addDouble("rotation2d", () -> drivetrain.getPose().getRotation().getDegrees());

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

    controlboard
        .driver
        .leftBumper()
        .whileTrue(shooterSubsystem.spinUp(ShooterConstants.SHOOT_SPEED));
    controlboard.driver.rightBumper().whileTrue(arm.setGoalCommand(ArmPosition.SAFE));
    controlboard.driver.a().onTrue(shooterSubsystem.spinUpAmp(() -> !intakeSubsystem.hasNote()));
    controlboard.driver.x().whileTrue(intakeSubsystem.shoot());
    controlboard.driver.leftTrigger().whileTrue(intakeSubsystem.runOuttake());
    controlboard.driver.rightTrigger().whileTrue(intakeSubsystem.runIntake(.75));

controlboard.driver.rightBumper().whileTrue(arm.setGoalCommand(ArmPosition.SUBWOOFER));
controlboard.driver.rightBumper().whileFalse(arm.setGoalCommand(ArmPosition.INTAKE).withTimeout(3));


    controlboard.operator.rightBumper().whileTrue(arm.armOverride(controlboard.operator::getLeftY));
    controlboard
        .operator
        .leftBumper()
        .whileTrue(climber.climbOverride(() -> controlboard.operator.getRawAxis(1)));

    controlboard.operator.leftTrigger(.75).whileTrue(arm.setGoalCommand(ArmPosition.FPODIUM));
    controlboard.operator.rightTrigger(.75).whileTrue((arm.setGoalCommand(ArmPosition.HPODIUM)));
    controlboard.operator.y().whileTrue(arm.setGoalCommand(ArmPosition.PODIUM));

    controlboard.operator.a().whileTrue(arm.setGoalCommand(ArmPosition.INTAKE));
    controlboard.operator.b().whileTrue(arm.setGoalCommand(ArmPosition.AMP));
    controlboard.operator.x().whileTrue(arm.setGoalCommand(ArmPosition.SUBWOOFER));
    controlboard.operator.povUp().onTrue(climber.climberUp());
    controlboard.operator.povDown().onTrue(climber.climberDown());
    controlboard.operator.povLeft().onTrue(arm.setGoalCommand(ArmPosition.AMP));
    controlboard.operator.povRight().onTrue(arm.setGoalCommand(ArmPosition.STOW));
    // controlboard.operator.start().onTrue(Commands.runOnce(()->climber.resetEncoders()));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.ShootSubwooferAuto(s);

    return autoChooser.getSelected();

    // .andThen(shooterSubsystem.spinUp())
    // .alongWith(arm.setGoalCommand(ArmPosition.SUBWOOFER))
    // .alongWith(intakeSubsystem.shoot())
    // .onlyWhile(()->shooterSubsystem.shootReady());
  }

  public void robotPeriodic() {
    // update the dashboard mechanism's state
    // m_arm.setAngle(arm.getPositionDeg());
  }

  public void safety() {
    drivetrain.Brake();
  }
}
