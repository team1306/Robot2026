package frc.robot.subsystems.leds;

import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;
import frc.robot.Constants;

public class LedsReal implements LedsIO {

  private final CANdle candle = new CANdle(Constants.CanIds.CANDLE_ID);

  public LedsReal() {

    candle.getConfigurator().apply(LedsConstants.CANDLE_CONFIG);
  }

  @Override
  public void setSolid(int red, int green, int blue) {
    clearAnimation();
    candle.setControl(
        new SolidColor(LedsConstants.STRIP_START_INDEX, LedsConstants.STRIP_END_INDEX)
            .withColor(new RGBWColor(red, green, blue)));
  }

  @Override
  public void setBlink(int red, int green, int blue, int speed) {

    candle.setControl(
        new StrobeAnimation(LedsConstants.STRIP_START_INDEX, LedsConstants.STRIP_END_INDEX)
            .withColor(new RGBWColor(red, green, blue))
            .withFrameRate(speed));
  }

  @Override
  public void setRainbow(int speed) {

    candle.setControl(
        new RainbowAnimation(LedsConstants.STRIP_START_INDEX, LedsConstants.STRIP_END_INDEX)
            .withDirection(AnimationDirectionValue.Forward)
            .withFrameRate(speed));
  }

  @Override
  public void setBounce(int red, int green, int blue, int speed) {

    candle.setControl(
        new LarsonAnimation(LedsConstants.STRIP_START_INDEX, LedsConstants.STRIP_END_INDEX)
            .withColor(new RGBWColor(red, green, blue))
            .withFrameRate(speed));
  }

  @Override
  public void clearAnimation() {
    candle.setControl(new EmptyAnimation(0));
  }
}
