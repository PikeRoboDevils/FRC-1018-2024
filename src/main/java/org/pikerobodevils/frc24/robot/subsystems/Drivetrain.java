/* Copyright 2024 Pike RoboDevils, FRC Team 1018
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE.md file or
 * at https://opensource.org/licenses/MIT. */

package org.pikerobodevils.frc24.robot.subsystems;


import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static org.pikerobodevils.frc24.robot.Constants.DrivetrainConstants.CURRENT_LIMIT;
import static org.pikerobodevils.frc24.robot.Constants.DrivetrainConstants.IDLE_MODE;
import static org.pikerobodevils.frc24.robot.Constants.DrivetrainConstants.KA;
import static org.pikerobodevils.frc24.robot.Constants.DrivetrainConstants.KP;
import static org.pikerobodevils.frc24.robot.Constants.DrivetrainConstants.KS;
import static org.pikerobodevils.frc24.robot.Constants.DrivetrainConstants.KV;
import static org.pikerobodevils.frc24.robot.Constants.DrivetrainConstants.LEFT_ENCODER_A;
import static org.pikerobodevils.frc24.robot.Constants.DrivetrainConstants.LEFT_ENCODER_B;
import static org.pikerobodevils.frc24.robot.Constants.DrivetrainConstants.LEFT_FOLLOWER_ONE_ID;
import static org.pikerobodevils.frc24.robot.Constants.DrivetrainConstants.LEFT_LEADER_ID;
import static org.pikerobodevils.frc24.robot.Constants.DrivetrainConstants.RIGHT_ENCODER_A;
import static org.pikerobodevils.frc24.robot.Constants.DrivetrainConstants.RIGHT_ENCODER_B;
import static org.pikerobodevils.frc24.robot.Constants.DrivetrainConstants.RIGHT_FOLLOWER_ONE_ID;
import static org.pikerobodevils.frc24.robot.Constants.DrivetrainConstants.RIGHT_LEADER_ID;
import static org.pikerobodevils.frc24.robot.Constants.DrivetrainConstants.VOLTRAMP;
import static org.pikerobodevils.frc24.robot.Constants.DrivetrainConstants.kDriveKinematics;
import static org.pikerobodevils.frc24.robot.Constants.DrivetrainConstants.kTrackwidthMeters;

// import io.github.oblarg.oblog.Loggable;
// import io.github.oblarg.oblog.annotations.Log;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.pikerobodevils.frc24.lib.vendor.SparkMax;
import org.pikerobodevils.frc24.robot.Robot;
import org.pikerobodevils.frc24.robot.subsystems.drive.DriveIO;
import org.pikerobodevils.frc24.robot.subsystems.drive.DriveIOInputsAutoLogged;
import org.pikerobodevils.frc24.robot.subsystems.drive.LocalADStarAK;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Drivetrain extends SubsystemBase{
  private final ShuffleboardTab dashboard = Shuffleboard.getTab("Driver");
  //for advantagescope tracking
  public final Field2d m_field = new Field2d();
  private final SparkMax leftLeader = new SparkMax(LEFT_LEADER_ID, MotorType.kBrushless);
  private final SparkMax leftFollowerOne = new SparkMax(LEFT_FOLLOWER_ONE_ID, MotorType.kBrushless);
  //private final SparkMax leftFollowerTwo = new SparkMax(LEFT_FOLLOWER_TWO_ID, MotorType.kBrushless);

  private final SparkMax rightLeader = new SparkMax(RIGHT_LEADER_ID, MotorType.kBrushless);
  private final SparkMax rightFollowerOne = new SparkMax(RIGHT_FOLLOWER_ONE_ID, MotorType.kBrushless);
  // private final SparkMax rightFollowerTwo =
  //     new SparkMax(RIGHT_FOLLOWER_TWO_ID, MotorType.kBrushless);

  private final Encoder leftEncoder = new Encoder(LEFT_ENCODER_A,LEFT_ENCODER_B,true, CounterBase.EncodingType.k4X);
  private final Encoder rightEncoder = new Encoder(RIGHT_ENCODER_A,RIGHT_ENCODER_B,true, CounterBase.EncodingType.k4X);
  
  // AdvantageKit inputs
  private final DriveIO io;
  private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
  private final AHRS navX = new AHRS();
  private DifferentialDriveOdometry m_Odometry;

  LinearFilter pitchRate = LinearFilter.backwardFiniteDifference(1, 2, 0.02);
  double currentPitchRate = 0;

  @AutoLogOutput
  private Pose2d m_Pose;
  public TrajectoryConfig config;

  private final PIDController leftDrivePid = new PIDController(KP, 0, 0);
  private final PIDController rightDrivePid = new PIDController(KP, 0, 0);
 private final PIDController turnDrivePid = new PIDController(KP, 0, 0);
 private final PIDController drivePID = new PIDController(.75, 0, 0);
    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));
 
  //no idea what this is - wisdom
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(KS, KV, KA);
  // now this i can assume but still idk shi- wisdom
  private final DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(kTrackwidthMeters);

  // I know what this is not how to use it -wisdom
  private final SysIdRoutine sysId;



  /** Creates a new Drivetrain. */
  public Drivetrain(DriveIO io) {
    this.io = io;
    
    // Adds robot to "Field"
    SmartDashboard.putData("Field", m_field);
    // Encoder and Motor Setup
    leftEncoder.setDistancePerPulse((Math.PI*.1524)/2048);
    rightEncoder.setDistancePerPulse((Math.PI*.1524)/2048);
    m_Pose = new Pose2d(0, 0, new Rotation2d());
    leftLeader.restoreFactoryDefaults();
    leftLeader.setIdleMode(IDLE_MODE);
    leftLeader.setSmartCurrentLimit(CURRENT_LIMIT);
    leftLeader.setOpenLoopRampRate(VOLTRAMP);

    leftFollowerOne.restoreFactoryDefaults();
    leftFollowerOne.setIdleMode(IDLE_MODE);
    leftFollowerOne.follow(leftLeader);
    leftFollowerOne.setOpenLoopRampRate(VOLTRAMP);
    // leftFollowerTwo.setSmartCurrentLimit(CURRENT_LIMIT);

    // leftFollowerTwo.restoreFactoryDefaults();
    // leftFollowerTwo.setIdleMode(IDLE_MODE);
    // leftFollowerTwo.follow(leftLeader);
    // leftFollowerTwo.setSmartCurrentLimit(CURRENT_LIMIT);

    rightLeader.restoreFactoryDefaults();
    rightLeader.setIdleMode(IDLE_MODE);
    rightLeader.setInverted(true);
    rightLeader.setSmartCurrentLimit(CURRENT_LIMIT);
    rightLeader.setOpenLoopRampRate(VOLTRAMP);

    rightFollowerOne.restoreFactoryDefaults();
    rightFollowerOne.setIdleMode(IDLE_MODE);
    rightFollowerOne.follow(rightLeader);
    rightFollowerOne.setOpenLoopRampRate(VOLTRAMP);
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

   drivePID.setTolerance(.1);  
    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configureRamsete(
        this::getPose,
        this::setOdometry,
        () ->
            kinematics.toChassisSpeeds(
                new DifferentialDriveWheelSpeeds(
                    getLeftVelocity(), getRightVelocity())),
        (speeds) -> {
          var wheelSpeeds = kinematics.toWheelSpeeds(speeds);
          driveVelocity(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
        },
        new ReplanningConfig(),
        () ->
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK()); //LocalADStarAK is used to made logging work with advantagekit or so im told 
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

  public boolean isRed() {
                  var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              } return false;
  }

  public void resetOdometry(Pose2d pose) {

    m_Odometry.resetPosition(

        navX.getRotation2d(), getLeftDistance(), getRightDistance(), pose);

  }
   /** Resets the current odometry pose.
    * made for pathplanner it say reset but it gives a value so i gotta make sure
    */
   public void setOdometry(Pose2d pose) {
    m_Odometry.resetPosition(inputs.gyroYaw, getLeftDistance(), getRightDistance(), pose);
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
  @AutoLogOutput
  public double getLeftVelocity(){
    return inputs.leftVelocityRadPerSec * kTrackwidthMeters;
  }
  @AutoLogOutput
  public double getRightVelocity(){
    return inputs.rightVelocityRadPerSec * kTrackwidthMeters;
  }
  @AutoLogOutput
  public double getLeftDistance(){
    return inputs.leftPositionRad * kTrackwidthMeters;
  }
  @AutoLogOutput
  public double getRightDistance(){
    return inputs.rightPositionRad * kTrackwidthMeters;
  }

  @AutoLogOutput
  public double getYaw() {
    return navX.getYaw();
  }

  @AutoLogOutput
  public double getPitch() {
    return navX.getPitch();
  }
  @AutoLogOutput
  public double getPitchRate() {
    return currentPitchRate;
  }

  @AutoLogOutput
  public double getRoll() {
    return navX.getRoll();
  }

  @AutoLogOutput
  public double getLeftVoltage() {
    return leftLeader.getAppliedOutput() * leftLeader.getBusVoltage();
  }

  @AutoLogOutput
  public double getRightVoltage() {
    return leftLeader.getAppliedOutput() * leftLeader.getBusVoltage();
  }
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return m_Pose;
  }

   public void arcadeDrive(double speed, double rotation) {
    var speeds = DifferentialDrive.arcadeDriveIK(speed, rotation, true);
    io.setVoltage(speeds.left * 12.0, speeds.right * 12.0);
   
  }

  // doesnt completly stop
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
    io.updateInputs(inputs);
    Logger.processInputs("Drive", inputs);

    currentPitchRate = pitchRate.calculate(getPitch());
    m_Pose = m_Odometry.update(
      inputs.gyroYaw,
      getLeftDistance(),
      getRightDistance());

    m_field.setRobotPose(m_Odometry.getPoseMeters());
    }

  public Command turntoAngle(double angle) {
   double setpoint = angle ;//(isRed()? angle:-angle);
     return arcadeDriveCommand(()->0, ()->-turnDrivePid.calculate(navX.getAngle(),setpoint)).withTimeout(1);
  }

  
  public Command DriveDist(double distance) {

     return arcadeDriveCommand(()->drivePID.calculate(getRightDistance(),-distance), ()->0).until(()->drivePID.atSetpoint())
     .finallyDo(()->arcadeDrive(0, 0)); // only arcade drive stops

  }

  /** Run open loop at the specified voltage. */
  public void driveVolts(double leftVolts, double rightVolts) {
    io.setVoltage(leftVolts, rightVolts);
  }

   /** Run closed loop at the specified voltage. */
   public void driveVelocity(double leftMetersPerSec, double rightMetersPerSec) {
    Logger.recordOutput("Drive/LeftVelocitySetpointMetersPerSec", leftMetersPerSec);
    Logger.recordOutput("Drive/RightVelocitySetpointMetersPerSec", rightMetersPerSec);
    double leftRadPerSec = leftMetersPerSec / kTrackwidthMeters;
    double rightRadPerSec = rightMetersPerSec / kTrackwidthMeters;
    io.setVelocity(
        leftRadPerSec,
        rightRadPerSec,
        feedforward.calculate(leftRadPerSec),
        feedforward.calculate(rightRadPerSec));
  }
}

