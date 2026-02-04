package frc.robot.subsystems.shooter;

import badgerutils.motor.MotorConfigUtils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;

public class ShooterConstants {

  public static final double KP = 0;
  public static final double KI = 0;
  public static final double KD = 0;

  public static final double KV = 0;
  public static final double ROTOR_TO_SENSOR_RATIO = 1;

  public static final double SUPPLY_CURRENT_LIMIT = 60;

  public static final int LEFT_TOP_MOTOR_ID = 13;
  public static final int LEFT_BOTTOM_MOTOR_ID = 14;

  public static final int RIGHT_TOP_MOTOR_ID = 15;
  public static final int RIGHT_BOTTOM_MOTOR_ID = 16;

  public static final int ENCODER_ID = 17;

  public static final double ERROR_THRESHOLD = 25;

  public static final Slot0Configs PID_CONFIGS =
      MotorConfigUtils.createPidConfig(
          KP, KI, KD, 0, KV, 0, 0, GravityTypeValue.Elevator_Static); // gravity doesn't matter
  public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS =
      new CurrentLimitsConfigs()
          .withStatorCurrentLimitEnable(false)
          .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
          .withSupplyCurrentLimitEnable(true);
}
