package frc.robot.subsystems.leds;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.signals.StripTypeValue;

public class LedsConstants {
  // 0-7 are onboard, 8-399 are an external strip.
  public static final int STRIP_START_INDEX = 8;
  public static final int STRIP_END_INDEX = STRIP_START_INDEX + 26;
  public static final LEDConfigs CANDLE_CONFIG =
      new CANdleConfiguration().LED.withStripType(StripTypeValue.GRB).withBrightnessScalar(0.5);
}
