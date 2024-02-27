/* Copyright 2024 Pike RoboDevils, FRC Team 1018
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE.md file or
 * at https://opensource.org/licenses/MIT. */

package org.pikerobodevils.frc24.robot.subsystems;


import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.pikerobodevils.frc24.robot.Constants.DrivetrainConstants.*;

import java.util.List;
// import io.github.oblarg.oblog.Loggable;
// import io.github.oblarg.oblog.annotations.Log;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.pikerobodevils.frc24.lib.vendor.SparkMax;
import org.pikerobodevils.frc24.lib.vendor.SparkMaxUtils;
import org.pikerobodevils.frc24.lib.vendor.SparkMaxUtils.UnitConversions;

public class Drivetrain extends SubsystemBase{

  private final SparkMax leftLeader = new SparkMax(LEFT_LEADER_ID, MotorType.kBrushless);
  private final SparkMax leftFollowerOne = new SparkMax(LEFT_FOLLOWER_ONE_ID, MotorType.kBrushless);
  private final SparkMax leftFollowerTwo = new SparkMax(LEFT_FOLLOWER_TWO_ID, MotorType.kBrushless);

  private final SparkMax rightLeader = new SparkMax(RIGHT_LEADER_ID, MotorType.kBrushless);
  private final SparkMax rightFollowerOne =
      new SparkMax(RIGHT_FOLLOWER_ONE_ID, MotorType.kBrushless);
  private final SparkMax rightFollowerTwo =
      new SparkMax(RIGHT_FOLLOWER_TWO_ID, MotorType.kBrushless);

  private final RelativeEncoder leftEncoder = leftLeader.getEncoder();
  private final RelativeEncoder rightEncoder = rightLeader.getEncoder();
  private final AHRS navX = new AHRS();
  private DifferentialDriveOdometry m_Odometry;
  LinearFilter pitchRate = LinearFilter.backwardFiniteDifference(1, 2, 0.02);

  double currentPitchRate = 0;
  private Pose2d m_Pose;
  private final DifferentialDrive m_drive =

  new DifferentialDrive(leftLeader::set, rightLeader::set);
  /** Creates a new Drivetrain. */
  public Drivetrain() {

    m_Pose = new Pose2d(5.0, 13.5, new Rotation2d());
    leftLeader.restoreFactoryDefaults();
    leftLeader.setIdleMode(IDLE_MODE);
    leftLeader.setSmartCurrentLimit(CURRENT_LIMIT);

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
        UnitConversions.setDegreesFromGearRatio(leftEncoder, GEAR_RATIO);
    UnitConversions.setDegreesFromGearRatio(rightEncoder, GEAR_RATIO);
    leftEncoder.setPositionConversionFactor(Math.PI*6/GEAR_RATIO);
    rightEncoder.setPositionConversionFactor(Math.PI*6/GEAR_RATIO);

    m_Odometry = new DifferentialDriveOdometry(navX.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition(),
    m_Pose);
  }


  public void resetOdometry(Pose2d pose) {

    m_Odometry.resetPosition(

        navX.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition(), pose);

  }
  public void setLeftRight(double left, double right) {
    leftLeader.set(left);
    rightLeader.set(right);
  }

  public void setLeftRightVoltage(double left, double right) {
    leftLeader.setVoltage(left);
    rightLeader.setVoltage(right);
  }
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {

    return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());

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

  // public Command driveTrajectoryCommand(Trajectory trajectory) {
  //   return driveTrajectoryCommand(() -> trajectory);
  // }

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
    m_Pose = m_Odometry.update(
      navX.getRotation2d(),
      leftEncoder.getPosition(),
      rightEncoder.getPosition()
    );
  }

   public Command getAutonomousCommand(Supplier<Trajectory> trajectory) {

    // Create a voltage constraint to ensure we don't accelerate too fast

    var autoVoltageConstraint =

        new DifferentialDriveVoltageConstraint(

            new SimpleMotorFeedforward(

                DriveConstants.ksVolts,

                DriveConstants.kvVoltSecondsPerMeter,

                DriveConstants.kaVoltSecondsSquaredPerMeter),

            DriveConstants.kDriveKinematics,

            10);


    // Create config for trajectory

    TrajectoryConfig config =

        new TrajectoryConfig(

                AutoConstants.kMaxSpeedMetersPerSecond,

                AutoConstants.kMaxAccelerationMetersPerSecondSquared)

            // Add kinematics to ensure max speed is actually obeyed

            .setKinematics(DriveConstants.kDriveKinematics)

            // Apply the voltage constraint

            .addConstraint(autoVoltageConstraint);


    RamseteCommand ramseteCommand =

        new RamseteCommand(

            trajectory,

            m_Pose,

            new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),

            new SimpleMotorFeedforward(

                DriveConstants.ksVolts,

                DriveConstants.kvVoltSecondsPerMeter,

                DriveConstants.kaVoltSecondsSquaredPerMeter),

            DriveConstants.kDriveKinematics,

           this.getWheelSpeeds(),

            new PIDController(kPDriveVel, 0, 0),

            new PIDController(DriveConstants.kPDriveVel, 0, 0),

            // RamseteCommand passes volts to the callback

            this::setLeftRightVoltage,

            this);


    // Reset odometry to the initial pose of the trajectory, run path following

    // command, then stop at the end.

    return Commands.runOnce(() -> this.resetOdometry(trajectory.get().getInitialPose()))

        .andThen(ramseteCommand)

        .andThen(Commands.runOnce(() -> this.setLeftRightVoltage(0, 0)));

  }

}

