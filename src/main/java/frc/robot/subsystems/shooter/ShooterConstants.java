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
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ShooterConstants {

  @AutoLogOutput
  private static final LoggedNetworkNumber KP_SUPPLIER =
      new LoggedNetworkNumber("/Tuning/Shooter KP", 0);

  @AutoLogOutput
  private static final LoggedNetworkNumber KI_SUPPLIER =
      new LoggedNetworkNumber("/Tuning/Shooter KI", 0);

  @AutoLogOutput
  private static final LoggedNetworkNumber KD_SUPPLIER =
      new LoggedNetworkNumber("/Tuning/Shooter KD", 0);

  @AutoLogOutput
  private static final LoggedNetworkNumber KV_SUPPLIER =
      new LoggedNetworkNumber("/Tuning/Shooter KV", 0);

  public static final double KP = 0;
  public static final double KI = 0;
  public static final double KD = 0;

  public static final double KV = 0;
  public static final double ROTOR_TO_SENSOR_RATIO = 1;

  public static final double SUPPLY_CURRENT_LIMIT = 60;

  public static final double ERROR_THRESHOLD = 25;

  // CONFIGS
  public static final TalonFXConfiguration CW_SHOOTER_MOTOR_CONFIGS =
      new TalonFXConfiguration()
          .withSlot0(
              MotorConfigUtils.createPidConfig(
                  KP, KI, KD, 0, KV, 0, 0, GravityTypeValue.Elevator_Static))
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
      CW_SHOOTER_MOTOR_CONFIGS.withMotorOutput(
          MotorConfigUtils.createMotorOutputConfig(
              InvertedValue.CounterClockwise_Positive, NeutralModeValue.Coast));
}
