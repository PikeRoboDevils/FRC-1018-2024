// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package org.pikerobodevils.frc24.robot.subsystems.drive;

import static org.pikerobodevils.frc24.robot.Constants.DrivetrainConstants.*;

import org.littletonrobotics.junction.AutoLogOutput;
import org.pikerobodevils.frc24.lib.vendor.SparkMax;
import org.pikerobodevils.frc24.robot.Constants.DrivetrainConstants;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class REALDriveIO implements DriveIO {
  private static final double KP = DrivetrainConstants.KP; 
  private static final double KD = DrivetrainConstants.KD;

  private final CANSparkMax leftLeader = new SparkMax(DrivetrainConstants.LEFT_LEADER_ID, MotorType.kBrushless);
  private final CANSparkMax rightLeader = new CANSparkMax(DrivetrainConstants.RIGHT_LEADER_ID, MotorType.kBrushless);
  private final CANSparkMax leftFollower = new CANSparkMax(DrivetrainConstants.LEFT_FOLLOWER_ONE_ID, MotorType.kBrushless);
  private final CANSparkMax rightFollower = new CANSparkMax(DrivetrainConstants.RIGHT_FOLLOWER_ONE_ID, MotorType.kBrushless);

  private final Encoder leftEncoder = new Encoder(LEFT_ENCODER_A,LEFT_ENCODER_B,false, CounterBase.EncodingType.k4X);
  private final Encoder rightEncoder = new Encoder(RIGHT_ENCODER_A,RIGHT_ENCODER_B,false, CounterBase.EncodingType.k4X);

  private final AHRS navx = new AHRS();

  public REALDriveIO() {
    leftLeader.restoreFactoryDefaults();
    rightLeader.restoreFactoryDefaults();
    leftFollower.restoreFactoryDefaults();
    rightFollower.restoreFactoryDefaults();

    leftLeader.setCANTimeout(250);
    rightLeader.setCANTimeout(250);
    leftFollower.setCANTimeout(250);
    rightFollower.setCANTimeout(250);

    leftLeader.setInverted(true);
    rightLeader.setInverted(false);
    leftFollower.follow(leftLeader, false);
    rightFollower.follow(rightLeader, false);

    //leftLeader.enableVoltageCompensation(12.0); //bad for flapjack :(
    //rightLeader.enableVoltageCompensation(12.0); //bad for flapjack :(
    leftLeader.setSmartCurrentLimit(DrivetrainConstants.CURRENT_LIMIT);
    rightLeader.setSmartCurrentLimit(DrivetrainConstants.CURRENT_LIMIT);

    leftLeader.burnFlash();
    rightLeader.burnFlash();
    leftFollower.burnFlash();
    rightFollower.burnFlash();

    leftEncoder.setDistancePerPulse((Math.PI*.1524)/2048);//IS THIS RIGHT???
    rightEncoder.setDistancePerPulse((Math.PI*.1524)/2048);


    navx.zeroYaw();
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    inputs.leftPosition =leftEncoder.getDistance();
    inputs.leftVelocity = Units.rotationsPerMinuteToRadiansPerSecond(leftEncoder.getRate());

    inputs.leftVoltage = leftLeader.getAppliedOutput() * leftLeader.getBusVoltage();
    inputs.leftAppliedVolts = leftLeader.getAppliedOutput();
    inputs.leftCurrentAmps = new double[] {leftLeader.getOutputCurrent(), leftFollower.getOutputCurrent()};

    inputs.rightPosition = rightEncoder.getDistance();
    inputs.rightVelocity = Units.rotationsPerMinuteToRadiansPerSecond(rightEncoder.getRate());

    inputs.rightVoltage = rightLeader.getAppliedOutput() * rightLeader.getBusVoltage();
    inputs.rightAppliedVolts = rightLeader.getAppliedOutput();
    inputs.rightCurrentAmps = new double[] {rightLeader.getOutputCurrent(), rightFollower.getOutputCurrent()};
    
    inputs.angle = navx.getAngle();
    inputs.rate = navx.getRate();
    inputs.gryoAngle = navx.getRotation2d();
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    leftLeader.setVoltage(leftVolts);
    rightLeader.setVoltage(rightVolts);
  }

  @Override
    public void set(double left, double right) {
      leftLeader.set(left);
     rightLeader.set(right);
}

  @Override
  public void Brake() {
    leftLeader.setIdleMode(IdleMode.kBrake);
    rightLeader.setIdleMode(IdleMode.kBrake);
    leftFollower.setIdleMode(IdleMode.kBrake);
    rightFollower.setIdleMode(IdleMode.kBrake);
  }

}
