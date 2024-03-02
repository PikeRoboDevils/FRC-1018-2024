// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.pikerobodevils.frc24.robot;

import org.pikerobodevils.frc24.robot.Constants.OperatorConstants;
import org.pikerobodevils.frc24.robot.commands.Autos;
import org.pikerobodevils.frc24.robot.subsystems.BotGoClimb;
import org.pikerobodevils.frc24.robot.subsystems.Drivetrain;
import org.pikerobodevils.frc24.robot.subsystems.ExampleSubsystem;
import org.pikerobodevils.frc24.robot.subsystems.Intake;
import org.pikerobodevils.frc24.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Intake intakeSubsystem = new Intake();
  private final Drivetrain drivetrain = new Drivetrain();
  private final Shooter shooterSubsystem = new Shooter();
  private final BotGoClimb climber= new BotGoClimb(); 

  private final ShuffleboardTab shuffleboard = Shuffleboard.getTab("Driver Dashboard");
  private final ControlBoard controlboard = new ControlBoard();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drivetrain.setDefaultCommand(
        drivetrain
            .arcadeDriveCommand(controlboard::getSpeed, controlboard::getTurn)
            .withName("Default Drive Command"));
    shuffleboard.addBoolean("Has Note",()->intakeSubsystem.hasNote());
    shuffleboard.addDouble("right velocity", ()->drivetrain.getRightVelocity());
    shuffleboard.addDouble("left velocity", ()->drivetrain.getLeftVelocity());
    shuffleboard.addDouble("left distance", ()->drivetrain.getLeftDistance() );
    shuffleboard.addDouble("right distance", ()->drivetrain.getRightDistance());
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

    //Sys ID buttons:
    controlboard.driver.a().whileTrue((drivetrain.runQuasiSysId(Direction.kForward)).finallyDo(()->drivetrain.setLeftRightVoltageCommand(0, 0)));
    controlboard.driver.b().whileTrue((drivetrain.runQuasiSysId(Direction.kReverse)).finallyDo(()->drivetrain.setLeftRightVoltageCommand(0, 0)));
    controlboard.driver.x().whileTrue((drivetrain.runDynamicSysId(Direction.kForward)).finallyDo(()->drivetrain.setLeftRightVoltageCommand(0, 0)));
    controlboard.driver.y().whileTrue((drivetrain.runDynamicSysId(Direction.kReverse)).finallyDo(()->drivetrain.setLeftRightVoltageCommand(0, 0)));


    controlboard.operator.a().onTrue(climber.climberDown());
    controlboard.operator.b().onTrue(climber.climberUp());
    controlboard.operator.x().whileTrue(intakeSubsystem.shoot());
    controlboard.operator.leftBumper().whileTrue(shooterSubsystem.spinUp());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.getAutonomousCommand(drivetrain);
  }
}
