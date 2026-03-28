package frc.robot.subsystems.leds;

import badgerutils.triggers.AllianceTriggers;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Leds extends SubsystemBase {

  public final LedsIO LedsIO;
  public boolean isShooting = false;
  public boolean isInShootingTolerance = false;

  public Leds(LedsIO LedsIO) {
    this.LedsIO = LedsIO;
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {

      LedsIO.setSolid(
          AllianceTriggers.isRedAlliance() ? 150 : 0,
          0,
          AllianceTriggers.isRedAlliance() ? 0 : 150);

    } else if (DriverStation.isAutonomous()) {
      LedsIO.setSolid(150, 0, 150);
    } else if (isShooting && !isInShootingTolerance) {

      LedsIO.setSolid(0, 150, 0);
    } else if (isShooting) {
      LedsIO.setBlink(0, 150, 0, 4);
    } else {
      LedsIO.setSolid(
          AllianceTriggers.isRedAlliance() ? 150 : 0,
          0,
          AllianceTriggers.isRedAlliance() ? 0 : 150);
    }

    Logger.recordOutput("Leds/isShooting", isShooting);
    Logger.recordOutput("Leds/isInShootingTolerance", isInShootingTolerance);
  }
}
