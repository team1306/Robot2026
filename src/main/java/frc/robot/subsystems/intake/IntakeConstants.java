package frc.robot.subsystems.intake;

import badgerutils.motor.MotorConfigUtils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeConstants {
  // Tuning doens't need to be perfect. We're only deploying once per match.
  public static final double KP = 0;
  public static final double KI = 0;
  public static final double KD = 0;

  // CONFIG
  public static final TalonFXConfiguration CW_INTAKE_MOTOR_CONFIGS =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(false)
                  .withSupplyCurrentLimitEnable(false))
          .withMotorOutput(
              MotorConfigUtils.createMotorOutputConfig(
                  InvertedValue.Clockwise_Positive, NeutralModeValue.Brake));

  public static final TalonFXConfiguration CCW_INTAKE_MOTOR_CONFIGS =
      CW_INTAKE_MOTOR_CONFIGS
          .clone()
          .withMotorOutput(
              MotorConfigUtils.createMotorOutputConfig(
                  InvertedValue.CounterClockwise_Positive, NeutralModeValue.Brake));

  public static final TalonFXConfiguration DEPLOYER_MOTOR_CONFIGS =
      new TalonFXConfiguration()
          .withMotorOutput(
              MotorConfigUtils.createMotorOutputConfig(
                  InvertedValue.Clockwise_Positive, NeutralModeValue.Coast))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(false)
                  .withSupplyCurrentLimitEnable(false))
          .withSlot0(
              MotorConfigUtils.createPidConfig(
                  KP, KI, KD, 0, 0, 0, 0, GravityTypeValue.Arm_Cosine));
}
