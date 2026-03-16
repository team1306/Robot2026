package frc.robot.subsystems.Leds;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import frc.robot.Constants;

public class LedsReal implements LedsIO {

  private final CANdle candle = new CANdle(Constants.CanIds.CANDLE_ID);

  public void LEDIOCANdle() {

    candle.getConfigurator().apply(LedsConstants.CANDLE_CONFIG);
  }

  @Override
  public void setSolid(int red, int blue, int green) {
    candle.setControl(
        new SolidColor(LedsConstants.STRIP_START_INDEX, LedsConstants.STRIP_END_INDEX)
            .withColor(new RGBWColor(red, green, blue)));
  }

  @Override
  public void setBlink(int red, int blue, int green, int speed) {
    candle.setControl(
        new StrobeAnimation(LedsConstants.STRIP_START_INDEX, LedsConstants.STRIP_END_INDEX)
            .withSlot(0)
            .withColor(new RGBWColor(red, green, blue))
            .withFrameRate(speed));
  }
}
