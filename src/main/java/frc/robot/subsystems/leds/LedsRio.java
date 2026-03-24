package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class LedsRio implements LedsIO {

  private final AddressableLED ledStrip = new AddressableLED(0);
  private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(18);

  public LedsRio() {
    ledStrip.setLength(18);
    ledStrip.setData(ledBuffer);
    ledStrip.start();
  }

  @Override
  public void setSolid(int red, int green, int blue) {
    LEDPattern color = LEDPattern.solid(new Color(red, green, blue));
    color.applyTo(ledBuffer);
    ledStrip.setData(ledBuffer);
  }

  @Override
  public void setBlink(int red, int green, int blue, int speed) {
    LEDPattern color = LEDPattern.solid(new Color(red, green, blue));
    LEDPattern blink = color.blink(Seconds.of(0.25));
    blink.applyTo(ledBuffer);
    ledStrip.setData(ledBuffer);
  }
}
