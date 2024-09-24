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

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;
import org.pikerobodevils.frc24.robot.Constants;
import org.pikerobodevils.frc24.robot.Constants.Mode;

public interface DriveIO {
  @AutoLog
  public static class DriveIOInputs {
    public double leftPosition;
    public double leftVelocity;

    public double leftVoltage;
    public double leftAppliedVolts;
    public double[] leftCurrentAmps = new double[] {};

    public double rightPosition;
    public double rightVelocity;
    // public double rightPositionRad = 0.0;
    // public double rightVelocityRadPerSec = 0.0;

    public double rightVoltage;
    public double rightAppliedVolts;
    public double[] rightCurrentAmps = new double[] {};

    public double angle;
    public double rate;
    public Rotation2d gryoAngle = new Rotation2d();
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(DriveIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double leftVolts, double rightVolts) {}

  /** Run closed loop at the specified velocity. */
  public default void setVelocity(
      double leftRadPerSec, double rightRadPerSec, double leftFFVolts, double rightFFVolts) {}

  /** ADDED FOR EASY DRIVETRAIN REPLACEMENT */
  public default void set(double left, double right) {}

  public default void Brake() {}

  public static DriveIO isReal() {
    if (Constants.currentMode == Mode.REAL) {
      return new REALDriveIO();
    } else {
      return new SIMDriveIO();
    }
  }
}
