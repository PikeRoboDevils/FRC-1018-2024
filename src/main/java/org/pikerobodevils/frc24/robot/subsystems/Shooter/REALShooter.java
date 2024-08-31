
package org.pikerobodevils.frc24.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLogOutput;
import org.pikerobodevils.frc24.lib.vendor.SparkMax;
import org.pikerobodevils.frc24.robot.Constants.DrivetrainConstants;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class REALShooter implements ShooterIO {
  private static final double GEAR_RATIO = DrivetrainConstants.GEAR_RATIO;
  private static final double KP = DrivetrainConstants.KP; // TODO: MUST BE TUNED, consider using REV Hardware Client
  private static final double KD = 0.0; // TODO: MUST BE TUNED, consider using REV Hardware Client

  private final CANSparkMax leftLeader = new SparkMax(DrivetrainConstants.LEFT_LEADER_ID, MotorType.kBrushless);
  private final CANSparkMax rightLeader = new CANSparkMax(DrivetrainConstants.RIGHT_LEADER_ID, MotorType.kBrushless);
  private final CANSparkMax leftFollower = new CANSparkMax(DrivetrainConstants.LEFT_FOLLOWER_ONE_ID, MotorType.kBrushless);
  private final CANSparkMax rightFollower = new CANSparkMax(DrivetrainConstants.RIGHT_FOLLOWER_ONE_ID, MotorType.kBrushless);

  private final RelativeEncoder leftEncoder = leftLeader.getEncoder();
  private final RelativeEncoder rightEncoder = rightLeader.getEncoder();

  private final SparkPIDController leftPID = leftLeader.getPIDController();
  private final SparkPIDController rightPID = rightLeader.getPIDController();

  private final AHRS navx = new AHRS();
  private final float yaw = navx.getYaw();


  public REALShooter() {
    //intiialize
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    //inputs.x = x
  }
//see REALDrive for more
}
