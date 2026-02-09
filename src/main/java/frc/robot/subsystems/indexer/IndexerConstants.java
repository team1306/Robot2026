package frc.robot.subsystems.indexer;

import badgerutils.motor.MotorConfigUtils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IndexerConstants {
  // CONFIGS
  public static final TalonFXConfiguration CW_INDEXER_MOTOR_CONFIGS =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(false)
                  .withSupplyCurrentLimitEnable(false))
          .withMotorOutput(
              MotorConfigUtils.createMotorOutputConfig(
                  InvertedValue.Clockwise_Positive, NeutralModeValue.Brake));
  // feedback and pid configs don't matter because we aren't using them

  public static final TalonFXConfiguration CCW_INDEXER_MOTOR_CONFIGS =
      CW_INDEXER_MOTOR_CONFIGS.withMotorOutput(
          MotorConfigUtils.createMotorOutputConfig(
              InvertedValue.CounterClockwise_Positive, NeutralModeValue.Brake));
}
