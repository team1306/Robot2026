package frc.robot.subsystems.Leds;

/** * LEDIO interface where you update the red, green, and blue LEDIO inputs */
public interface LedsIO {
  /** Sets the LEDs to a red, green, and blue value */
  default void setSolid(int red, int green, int blue) {}

  default void setBlink(int red, int green, int blue, int speed) {}

  default void clearAnimation() {}
}
