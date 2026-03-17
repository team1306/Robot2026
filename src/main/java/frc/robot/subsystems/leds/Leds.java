package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Leds extends SubsystemBase {

  public final LedsIO LedsIO;
  public boolean isDisabled = false;
  public boolean isShooting = false;
  public boolean isInShootingTolerance = false;
  public Color currentColor = new Color(0, 0, 0);

  public Leds(LedsIO LedsIO) {
    this.LedsIO = LedsIO;
  }

  @Override
  public void periodic() {
    if (isDisabled) {
      LedsIO.setBlink(255, 0, 0, 2);
      currentColor = new Color(255, 20, 0);
    } else if (isShooting && !isInShootingTolerance) {
      LedsIO.setSolid(0, 255, 0);
      currentColor = new Color(0, 255, 0);
    } else if (isShooting) {
      LedsIO.setBlink(0, 255, 0, 4);
      currentColor = new Color(0, 0, 255);
    } else {
      LedsIO.setSolid(255, 0, 0);
      currentColor = new Color(255, 0, 0);
    }

    Logger.recordOutput("Leds/isDisabled", isDisabled);
    Logger.recordOutput("Leds/isShooting", isShooting);
    Logger.recordOutput("Leds/isInShootingTolerance", isInShootingTolerance);
    Logger.recordOutput("Leds/color", currentColor);
  }
}
