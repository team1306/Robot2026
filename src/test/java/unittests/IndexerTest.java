package unittests;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import org.junit.jupiter.api.Test;

class IndexerTest {
  private static final double DELTA = 1e-9;

  @Test
  void commandsSetRequestedOutputAndStopOnEnd() {
    UnitTestIndexerIO io = new UnitTestIndexerIO();
    Indexer indexer = new Indexer(io);
    Command command = indexer.indexUntilCancelledCommand(-0.4);

    command.initialize();
    assertEquals(-0.4, io.dutyCycle, DELTA);
    command.end(true);
    assertEquals(0.0, io.dutyCycle, DELTA);

    indexer.periodic();
    assertEquals(1, io.updateCount);
  }

  private static class UnitTestIndexerIO implements IndexerIO {
    double dutyCycle;
    int updateCount;

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
      updateCount++;
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
      this.dutyCycle = dutyCycle;
    }
  }
}
