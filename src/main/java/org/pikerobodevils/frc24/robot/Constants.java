// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.pikerobodevils.frc24.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import com.revrobotics.CANSparkBase.IdleMode;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class ArmConstants {
    public static final int LEFT_CONTROLLER_ID = 7;
    public static final int RIGHT_CONTROLLER_ID = 8;

    public static final int ENCODER_ABS_DIO = 4;
    public static final int ENCODER_QUAD_A = 5;
    public static final int ENCODER_QUAD_B = 6;

    public static final double QUAD_COUNTS_PER_REV = 2048;

    // Encoder pulley teeth / arm pulley teeth;
    public static final double ARM_TO_ENCODER_RATIO = 1;

    public static final double RAD_PER_ENCODER_ROTATION = 2 * Math.PI * ARM_TO_ENCODER_RATIO;
    public static final double RAD_PER_QUAD_TICK = RAD_PER_ENCODER_ROTATION / QUAD_COUNTS_PER_REV;

    public static final double OFFSET_DEGREES = 74;
    public static final double ENCODER_OFFSET = MathUtil.inputModulus(
        Units.degreesToRadians(OFFSET_DEGREES) / RAD_PER_ENCODER_ROTATION, 0, 1);

    public static final double KP = 5.3731; // 5.2103;
    public static final double KI = 0;
    public static final double KD = 0; // 1.6179;

    public static final double KS = 0.19677;
    public static final double KG = .5;
    public static final double KV = 2.114;
    public static final double KA = 0.12894;

    /**
     * SIM GAINS public static final double KP = 5; public static final double KI =
     * 0; public static
     * final double KD = 1;
     *
     * <p>
     * public static final double KS = 0; public static final double KG = 1.3;
     * public static
     * final double KV = 2.22; public static final double KA = 0.04;
     */
    public static final double MAX_VELO = Math.PI;
    // Rad / s / s
    public static final double MAX_ACCEL = Math.PI * 3;
    public static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(MAX_VELO,
        MAX_ACCEL);

    // The reduction between the motors and the arm.
    public static final double ARM_REDUCTION = 5 * 5 * (64.0 / 14);

    public static final double COM_DISTANCE = Units.inchesToMeters(25);
    public static final double MASS = Units.lbsToKilograms(15);

    public static final double MOI_KG_M_SQUARED = MASS * (Math.pow(COM_DISTANCE, 2));
  }

  public static class IntakeConstants {

    public static final int MOTOR_ID = 9;

    public static final int CURRENT_LIMIT = 20; // Amps

    public static final double INTAKE_SPEED = .5;

    public static final double SHOOT_SPEED = .75;

    public static final double DEFAULT_OUTTAKE = -.5;
  }

  public static class ShooterConstants {
    public static final int MAIN_MOTOR_ID = 13;
    public static final int FOLLOWER_MOTOR_ID = 14;

    public static final double SHOOT_SPEED = .75;
    public static final double CONSTANT_VELOCITY = .15;
  }

  public static class DrivetrainConstants {
    public static final int LEFT_LEADER_ID = 1;
    public static final int LEFT_FOLLOWER_ONE_ID = 3;

    public static final int RIGHT_LEADER_ID = 2;
    public static final int RIGHT_FOLLOWER_ONE_ID = 4;

    public static final IdleMode IDLE_MODE = IdleMode.kBrake;

    public static final int CURRENT_LIMIT = 50;
    public static final double GEAR_RATIO = 10.86;

    public static final double KS = 0.21709;
    public static final double KA = 0.69281;
    public static final double KV = 2.8975;
    public static final double KP = 0.65671;
    public static final double kTrackwidthMeters = 0.6;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
        kTrackwidthMeters);

  }
public static class ClimbConstants{
  // Setup by Cannon "heyyyy"
  //Sets contants for variables such as motor ID, dictance and speedin the the robot. 
    public static final int MOTOR_ID = 21;
    public static final double CLIMB_SPEED = 0.5;
    public static final int CLIMB_DISTANCE = 18; // cms 
    public static final double KP = 0.01;
}







}
