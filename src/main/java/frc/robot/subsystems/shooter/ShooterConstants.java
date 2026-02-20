package frc.robot.subsystems.shooter;

import badgerutils.motor.MotorConfigUtils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

public class ShooterConstants {
  public static final double KP = 15;
  public static final double KI = 0;
  public static final double KD = 0;

  public static final double KV = 0.028795;
  public static final double KS = 0.058037;

  public static final double ROTOR_TO_SENSOR_RATIO = 1.5;

  public static final double SUPPLY_CURRENT_LIMIT = 60;

  public static final double ERROR_THRESHOLD = 2;

  // CONFIGS
  public static final TalonFXConfiguration CW_SHOOTER_MOTOR_CONFIGS =
      new TalonFXConfiguration()
          .withSlot0(
              MotorConfigUtils.createPidConfig(
                  KP, KI, KD, KS, KV, 0, 0, GravityTypeValue.Arm_Cosine))
          .withFeedback(
              new FeedbackConfigs()
                  .withFeedbackRemoteSensorID(Constants.CanIds.SHOOTER_ENCODER_ID)
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.SyncCANcoder)
                  .withRotorToSensorRatio(ROTOR_TO_SENSOR_RATIO))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(false)
                  .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
                  .withSupplyCurrentLimitEnable(true))
          .withMotorOutput(
              MotorConfigUtils.createMotorOutputConfig(
                  InvertedValue.Clockwise_Positive, NeutralModeValue.Coast));

  public static final TalonFXConfiguration CCW_SHOOTER_MOTOR_CONFIGS =
      CW_SHOOTER_MOTOR_CONFIGS
          .clone()
          .withMotorOutput(
              MotorConfigUtils.createMotorOutputConfig(
                  InvertedValue.CounterClockwise_Positive, NeutralModeValue.Coast));
}
