package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;

public class LedsRio implements LedsIO {

  private final AddressableLED ledStrip = new AddressableLED(0);
  private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(18);
  
  public LedsRio() {

    
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
    LEDPattern blink = color.blink(Seconds.of(1/speed));
    blink.applyTo(ledBuffer);
    ledStrip.setData(ledBuffer);
  }
}
