package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Rotations;

import badgerutils.motor.MotorConfigUtils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;

public class HoodConstants {
  public static final double KP = 200;
  public static final double KD = 3;

  public static final double KS = 3;
  public static final double KG = 3;

  public static final Angle ZERO_POSITION = Rotations.of(0.02);
  public static final Angle MAX_ANGLE = Rotations.of(.88);

  public static final double ROTOR_TO_SENSOR_RATIO = 150.462963;

  public static final TalonFXConfiguration HOOD_MOTOR_CONFIGS =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withSupplyCurrentLimit(40)
                  .withSupplyCurrentLimitEnable(true))
          .withMotorOutput(
              MotorConfigUtils.createMotorOutputConfig(
                  InvertedValue.CounterClockwise_Positive, NeutralModeValue.Brake))
          .withSlot0(
              MotorConfigUtils.createPidConfig(
                  KP, 0, KD, KS, 0, KG, 0, GravityTypeValue.Elevator_Static))
          .withFeedback(
              new FeedbackConfigs()
                  .withFeedbackRemoteSensorID(Constants.CanIds.HOOD_ENCODER_ID)
                  .withRotorToSensorRatio(ROTOR_TO_SENSOR_RATIO)
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder));
}
