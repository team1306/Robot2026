package unittests;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.booster.Booster;
import frc.robot.subsystems.booster.BoosterIO;
import org.junit.jupiter.api.Test;

class BoosterTest {
  private static final double DELTA = 1e-9;

  @Test
  void delegatesPeriodicOutputAndCommandLifecycle() {
    UnitTestBoosterIO io = new UnitTestBoosterIO();
    Booster booster = new Booster(io);

    booster.periodic();
    booster.setDutyCycle(0.2);
    assertEquals(1, io.updateCount);
    assertEquals(0.2, io.dutyCycle, DELTA);

    Command command = booster.boostCommand(0.75);
    command.initialize();
    assertEquals(0.75, io.dutyCycle, DELTA);
    command.end(true);
    assertEquals(0.0, io.dutyCycle, DELTA);
  }

  private static class UnitTestBoosterIO implements BoosterIO {
    double dutyCycle;
    int updateCount;

    @Override
    public void updateInputs(BoosterIOInputs inputs) {
      updateCount++;
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
      this.dutyCycle = dutyCycle;
    }
  }
}
