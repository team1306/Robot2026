package frc.robot.subsystems.booster;

import badgerutils.motor.MotorConfigUtils;
import frc.robot.Constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class BoosterConstants {
  // CONFIGS
  public static final TalonFXConfiguration BOOSTER_MOTOR_CONFIGS =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(false)
                  .withSupplyCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(Constants.CurrentLimits.BOOSTER_SUPPLY))
          .withMotorOutput(
              MotorConfigUtils.createMotorOutputConfig(
                  InvertedValue.Clockwise_Positive, NeutralModeValue.Coast));
}
