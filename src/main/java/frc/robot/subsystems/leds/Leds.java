package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Leds extends SubsystemBase {

  public final LedsIO LedsIO;
  public boolean isDisabled = false;
  public boolean isShooting = false;
  public boolean isInShootingTolerance = false;

  public Leds(LedsIO LedsIO) {
    this.LedsIO = LedsIO;
  }

  @Override
  public void periodic() {
    if (isShooting && !isInShootingTolerance) {
      LedsIO.setSolid(0, 255, 0);
    } else if (isShooting && isInShootingTolerance) {
      LedsIO.setBlink(0, 255, 0, 4);
    }
    if (isDisabled) {
      LedsIO.setSolid(255, 0, 0);
    }
    Logger.recordOutput("Leds/isDisabled", isDisabled);
    Logger.recordOutput("Leds/isShooting", isShooting);
    Logger.recordOutput("Leds/isInShootingTolerance", isInShootingTolerance);
  }
}
