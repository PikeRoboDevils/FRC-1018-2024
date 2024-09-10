
package org.pikerobodevils.frc24.robot.subsystems.Arm;

import static org.pikerobodevils.frc24.robot.Constants.ArmConstants.*;

import org.pikerobodevils.frc24.lib.vendor.SparkMax;
import org.pikerobodevils.frc24.lib.vendor.SparkMaxUtils;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class REALArm implements ArmIO {

  SparkMax leftController =
      new SparkMax(LEFT_CONTROLLER_ID, CANSparkLowLevel.MotorType.kBrushless);
  SparkMax rightController =
      new SparkMax(RIGHT_CONTROLLER_ID, CANSparkLowLevel.MotorType.kBrushless);

  //Encoder encoder = new Encoder(ENCODER_QUAD_A, ENCODER_QUAD_B, true, CounterBase.EncodingType.k4X);
  DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(ENCODER_ABS_DIO);
  ArmFeedforward feedforward = new ArmFeedforward(KS, KG, KV, KA);
  ProfiledPIDController controller = new ProfiledPIDController(KP, KI, KD, CONSTRAINTS);




  public REALArm() {
    //intiialize
        rightController.withInitializer(
        (spark, isInit) -> {
          int errors = 0;
          errors += SparkMaxUtils.check(spark.setIdleMode(CANSparkMax.IdleMode.kBrake));
          spark.setInverted(true);
          return errors == 0;
        });
    leftController.withInitializer(
        (spark, isInit) -> {
          int errors = 0;
          errors += SparkMaxUtils.check(spark.setIdleMode(CANSparkMax.IdleMode.kBrake));
          return errors == 0;
        });
        
    leftController.setSmartCurrentLimit(40);
    rightController.setSmartCurrentLimit(40);

    //encoder.setDistancePerPulse(RAD_PER_QUAD_TICK);
    absoluteEncoder.setDistancePerRotation(RAD_PER_ENCODER_ROTATION);
    absoluteEncoder.setPositionOffset(ENCODER_OFFSET);

    

    
    

  }
  @Override
  public void updatePositionController() {
    // Update controller with new measurement
    // This updates the controllers setpoint internally.
    var feedbackOutput = controller.calculate(MathUtil.angleModulus(absoluteEncoder.getDistance()));
    // use the newly updated setpoint to calculate a feedforward.
    var setpoint = controller.getSetpoint();
    var feedforwardOutput = feedforward.calculate(MathUtil.angleModulus(absoluteEncoder.getDistance()), setpoint.velocity);
    var totalOutputVolts = feedbackOutput + feedforwardOutput;
    setVoltage(-totalOutputVolts);
  }
  @Override
  public void updatePositionController(double setpoint) {
    setGoal(setpoint);
    updatePositionController();
  }
  @Override
  public void setGoal(double goal) {
    controller.setGoal(goal);
  }
  @Override
  public void setVoltage(double volts) {
    leftController.setVoltage(volts);
    rightController.setVoltage(volts);
  }
  @Override
  public void setSpeed(double speed) {
  leftController.set(speed/2);
  rightController.set(speed/2);
  }
  @Override
  public void reset() {
    controller.reset(MathUtil.angleModulus(absoluteEncoder.getDistance()));
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    //where logging output values happens & get methods
    //inputs.x = x
    inputs.Position = MathUtil.angleModulus(absoluteEncoder.getDistance());
    inputs.Angle =  Units.radiansToDegrees(inputs.Position);
  }

}
