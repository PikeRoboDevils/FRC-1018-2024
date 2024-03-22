/* Copyright 2024 Pike RoboDevils, FRC Team 1018
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE.md file or
 * at https://opensource.org/licenses/MIT. */

package org.pikerobodevils.frc24.robot.subsystems;


import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.proto.ChassisSpeedsProto;
import edu.wpi.first.math.proto.Kinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import static org.pikerobodevils.frc24.robot.Constants.DrivetrainConstants.*;

import java.nio.channels.Pipe;
import java.util.List;
// import io.github.oblarg.oblog.Loggable;
// import io.github.oblarg.oblog.annotations.Log;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.pikerobodevils.frc24.lib.vendor.SparkMax;
import org.pikerobodevils.frc24.lib.vendor.SparkMaxUtils;
import org.pikerobodevils.frc24.lib.vendor.SparkMaxUtils.UnitConversions;

public class Drivetrain extends SubsystemBase{
  private final ShuffleboardTab dashboard = Shuffleboard.getTab("Driver");
  private final SparkMax leftLeader = new SparkMax(LEFT_LEADER_ID, MotorType.kBrushless);
  private final SparkMax leftFollowerOne = new SparkMax(LEFT_FOLLOWER_ONE_ID, MotorType.kBrushless);
  //private final SparkMax leftFollowerTwo = new SparkMax(LEFT_FOLLOWER_TWO_ID, MotorType.kBrushless);

  private final SparkMax rightLeader = new SparkMax(RIGHT_LEADER_ID, MotorType.kBrushless);
  private final SparkMax rightFollowerOne =
      new SparkMax(RIGHT_FOLLOWER_ONE_ID, MotorType.kBrushless);
  // private final SparkMax rightFollowerTwo =
  //     new SparkMax(RIGHT_FOLLOWER_TWO_ID, MotorType.kBrushless);

  private final Encoder leftEncoder = new Encoder(LEFT_ENCODER_A,LEFT_ENCODER_B,true, CounterBase.EncodingType.k4X);
  private final Encoder rightEncoder = new Encoder(RIGHT_ENCODER_A,RIGHT_ENCODER_B,true, CounterBase.EncodingType.k4X);

  private final AHRS navX = new AHRS();
  private DifferentialDriveOdometry m_Odometry;
  LinearFilter pitchRate = LinearFilter.backwardFiniteDifference(1, 2, 0.02);

  double currentPitchRate = 0;
  private Pose2d m_Pose;
  public TrajectoryConfig config;

  private final PIDController leftDrivePid = new PIDController(KP, 0, 0);
  private final PIDController rightDrivePid = new PIDController(KP, 0, 0);
 private final PIDController turnDrivePid = new PIDController(KP, 0, 0);
    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));
 
  /** Creates a new Drivetrain. */
  public Drivetrain() {
    leftEncoder.setDistancePerPulse((Math.PI*.1524)/2048);
    rightEncoder.setDistancePerPulse((Math.PI*.1524)/2048);
    m_Pose = new Pose2d(0, 0, new Rotation2d());
    leftLeader.restoreFactoryDefaults();
    leftLeader.setIdleMode(IDLE_MODE);
    leftLeader.setSmartCurrentLimit(CURRENT_LIMIT);

    leftFollowerOne.restoreFactoryDefaults();
    leftFollowerOne.setIdleMode(IDLE_MODE);
    leftFollowerOne.follow(leftLeader);
    // leftFollowerTwo.setSmartCurrentLimit(CURRENT_LIMIT);

    // leftFollowerTwo.restoreFactoryDefaults();
    // leftFollowerTwo.setIdleMode(IDLE_MODE);
    // leftFollowerTwo.follow(leftLeader);
    // leftFollowerTwo.setSmartCurrentLimit(CURRENT_LIMIT);

    rightLeader.restoreFactoryDefaults();
    rightLeader.setIdleMode(IDLE_MODE);
    rightLeader.setInverted(true);
    rightLeader.setSmartCurrentLimit(CURRENT_LIMIT);

    rightFollowerOne.restoreFactoryDefaults();
    rightFollowerOne.setIdleMode(IDLE_MODE);
    rightFollowerOne.follow(rightLeader);
    // rightFollowerTwo.setSmartCurrentLimit(CURRENT_LIMIT);

    // rightFollowerTwo.restoreFactoryDefaults();
    // rightFollowerTwo.setIdleMode(IDLE_MODE);
    // rightFollowerTwo.follow(rightLeader);
    // rightFollowerTwo.setSmartCurrentLimit(CURRENT_LIMIT);
    // UnitConversions.setDegreesFromGearRatio(leftEncoder, GEAR_RATIO);
    // UnitConversions.setDegreesFromGearRatio(rightEncoder, GEAR_RATIO);
    // leftEncoder.setPositionConversionFactor(Math.PI*6);
    // rightEncoder.setPositionConversionFactor(Math.PI*6);

    // leftEncoder.setVelocityConversionFactor(1/GEAR_RATIO);
    // rightEncoder.setVelocityConversionFactor(1/GEAR_RATIO);
    var autoVoltageConstraint =

        new DifferentialDriveVoltageConstraint(

            new SimpleMotorFeedforward(

                KS,

                KV,

                KA),

           kDriveKinematics,

            10);

    config =

        new TrajectoryConfig(

                1,

                1)

            // Add kinematics to ensure max speed is actually obeyed

            .setKinematics(kDriveKinematics)

            // Apply the voltage constraint

            .addConstraint(autoVoltageConstraint);


            

    m_Odometry = new DifferentialDriveOdometry(navX.getRotation2d(), getLeftDistance(), getRightDistance(),
    m_Pose);

        AutoBuilder.configureRamsete(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getWheelSpeeds, // Current ChassisSpeeds supplier
            this::drive, // Method that will drive the robot given ChassisSpeeds
            new ReplanningConfig(), // Default path replanning config. See the API for the options here
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return true;
            },
            this // Reference to this subsystem to set requirements
    );
  }
  public boolean isRed() {
                  var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              } return false;
  }


 private final SysIdRoutine sysId =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motors.
              (Measure<Voltage> volts) -> {
                leftLeader.setVoltage(volts.in(Volts));
                rightLeader.setVoltage(volts.in(Volts));
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the left motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-left")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            leftLeader.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(this.getLeftDistance(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(this.getLeftVelocity(), MetersPerSecond));
                // Record a frame for the right motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-right")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            rightLeader.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(this.getRightDistance(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(this.getRightVelocity(), MetersPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("drive")
              this));    

  public void resetOdometry(Pose2d pose) {

    m_Odometry.resetPosition(

        navX.getRotation2d(), getLeftDistance(), getRightDistance(), pose);

  }
  public void drive(ChassisSpeeds speeds)
  {
    DifferentialDriveWheelSpeeds wheelSpeeds = kDriveKinematics.toWheelSpeeds(speeds);
    
    leftLeader.set(-leftDrivePid.calculate(getLeftVelocity(),wheelSpeeds.leftMetersPerSecond));
    rightLeader.set(-rightDrivePid.calculate(getRightVelocity(),wheelSpeeds.rightMetersPerSecond));
  }
  public void resetEncoders(){
    leftEncoder.reset();
    rightEncoder.reset();

  }
  public void resetGyro(){
    navX.reset();
  }
  public void setLeftRight(double left, double right) {
    leftLeader.set(left);
    rightLeader.set(right);
  }

  public void setLeftRightVoltage(double left, double right) {
    leftLeader.setVoltage(-left);
    rightLeader.setVoltage(-right);
  }
    public ChassisSpeeds getWheelSpeeds() {


    return new ChassisSpeeds(getLeftVelocity(), getRightVelocity(), Units.degreesToRadians(navX.getRate()));

  }

  public DifferentialDriveWheelSpeeds getDiffWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(getLeftVelocity(),getRightVelocity());
  }

  public void setIdleMode(CANSparkMax.IdleMode mode) {
    leftLeader.setIdleMode(mode);
    leftFollowerOne.setIdleMode(mode);
    //leftFollowerTwo.setIdleMode(mode);

    rightLeader.setIdleMode(mode);
    rightFollowerOne.setIdleMode(mode);
    // rightFollowerTwo.setIdleMode(mode);
  }
  public double getLeftVelocity(){
    return leftEncoder.getRate();
  }
  public double getRightVelocity(){
    return rightEncoder.getRate();
  }

  public double getLeftDistance(){
    return leftEncoder.getDistance();
  }

  public double getRightDistance(){
    return rightEncoder.getDistance();
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

  public Pose2d getPose() {
    return m_Pose;
  }

  public void arcadeDrive(double speed, double rotation) {
    DifferentialDrive.WheelSpeeds speeds = DifferentialDrive.arcadeDriveIK(speed, rotation, false);
    setLeftRight(speeds.left, speeds.right);
  }

  public Command arcadeDriveCommand(DoubleSupplier speed, DoubleSupplier rotation) {
    return run(() -> arcadeDrive(speed.getAsDouble(), rotation.getAsDouble()));
  }

  public Command turnToTx(double tx)
  {
    double error = 0 - tx;
    double output  = KP*error;
    return run(()-> arcadeDriveCommand(()->0, ()->output));
  }

  public Command runQuasiSysId(SysIdRoutine.Direction direction){
    return sysId.quasistatic(direction);
  }
  public Command runDynamicSysId(SysIdRoutine.Direction direction){
    return sysId.dynamic(direction);
  }
  // public Command driveTrajectoryCommand(Trajectory trajectory) {
  //   return driveTrajectoryCommand(() -> trajectory);
  // }

  // public Command driveTrajectoryCommand(Supplier<Trajectory> trajectory) {
  //   RamseteController ramsete = new RamseteController();
  //   PIDController leftController = new PIDController(0,0,0);
  //   PIDController rightController = new PIDController(0,0,0);
  //   return run(() -> {

  //   })
  
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
      getLeftDistance(),
      getRightDistance()
        );

   
  }

  public Command turntoAngle(double angle) {
   
     return arcadeDriveCommand(()->0, ()->-turnDrivePid.calculate(navX.getAngle(),angle)).withTimeout(1);
  }

  
  public Command DriveDist(double distance) {
    double error = distance - getLeftDistance();
    double output = KP * error;
     return run(() -> arcadeDriveCommand(()->output, ()->0));
  }
  

   public Command getAutonomousCommand(Supplier<Trajectory> trajectory) {

    // Create a voltage constraint to ensure we don't accelerate too fast



    // Create config for trajectory

    
            RamseteCommand ramseteCommand =
            new RamseteCommand(
                trajectory.get(),
                this::getPose,
                new RamseteController(2, .7),
                new SimpleMotorFeedforward(
                    KS,
                    KV,
                    KA),
                kDriveKinematics,
                this::getDiffWheelSpeeds,
                new PIDController(KP, 0, 0),
                new PIDController(KP, 0, 0),
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

