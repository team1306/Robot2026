package frc.robot.tests;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotSimHarness;
import frc.robot.RobotSimulationExtension;
import frc.robot.SimFixture;
import frc.robot.SimFixtures;
import frc.robot.subsystems.indexer.Indexer;
import java.util.concurrent.TimeUnit;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;
import org.junit.jupiter.api.extension.ExtendWith;

@ExtendWith(RobotSimulationExtension.class)
class IndexerCommandsIntegrationTest {
  /** Comfortably above sensor noise, far below the current from a full-duty-cycle command. */
  private static final double MIN_COMMAND_AMPS = 1.0;

  private static final int MAX_LOOPS = 25;

  private final RobotSimHarness harness;
  private final SimFixture fixture;
  private final Indexer indexer;

  IndexerCommandsIntegrationTest() {
    harness = RobotSimHarness.getInstance();
    fixture = SimFixtures.createIndexerSimFixture(harness);
    indexer = harness.robotContainer().TESTONLY_getIndexer();
  }

  @Test
  @Timeout(value = 120, unit = TimeUnit.SECONDS)
  void indexUntilCancelledWithConstantSpeedControlsAllMotors() {
    Command command = indexer.indexUntilCancelledCommand(1.0);

    startCommand(command);
    assertMotorsRunning("all indexer motors commanded to run at a constant speed");

    CommandScheduler.getInstance().cancel(command);
    assertMotorsStopped("all indexer motors stopped after constant-speed command cancellation");
  }

  @Test
  @Timeout(value = 120, unit = TimeUnit.SECONDS)
  void indexUntilCancelledWithSupplierControlsAllMotors() {
    Command command = indexer.indexUntilCancelledCommand(() -> 1.0);

    startCommand(command);
    assertMotorsRunning("all indexer motors commanded to run from a speed supplier");

    CommandScheduler.getInstance().cancel(command);
    assertMotorsStopped("all indexer motors stopped after supplier command cancellation");
  }

  @Test
  @Timeout(value = 120, unit = TimeUnit.SECONDS)
  void jumbleIndexerControlsAllMotors() {
    // NOTE: The way jumble indexer is currently written would make it very jank to properly test the fluctuating nature of it.
    Command command = indexer.jumbleIndexer(() -> 1.0);

    startCommand(command);
    assertMotorsRunning("all indexer motors commanded to run by jumbleIndexer");

    CommandScheduler.getInstance().cancel(command);
    assertMotorsStopped("all indexer motors stopped after jumbleIndexer cancellation");
  }

  @Test
  @Timeout(value = 120, unit = TimeUnit.SECONDS)
  void indexForTimeControlsAndStopsAllMotors() {
    startCommand(indexer.indexForTime(Seconds.of(0.1), 1.0));

    assertMotorsRunning("all indexer motors commanded to run by indexForTime");
    assertMotorsStopped("all indexer motors stopped when indexForTime expires");
  }

  private void startCommand(Command command) {
    CommandScheduler.getInstance().cancelAll();
    harness.enableTeleop();
    CommandScheduler.getInstance().schedule(command);
  }

  private void assertMotorsRunning(String failureMessage) {
    fixture.checkMotorCondition(
        harness,
        failureMessage,
        motor -> Math.abs(fixture.torqueCurrentAmps(motor)) > MIN_COMMAND_AMPS,
        MAX_LOOPS);
  }

  private void assertMotorsStopped(String failureMessage) {
    fixture.checkMotorCondition(
        harness,
        failureMessage,
        motor -> Math.abs(fixture.torqueCurrentAmps(motor)) <= MIN_COMMAND_AMPS,
        MAX_LOOPS);
  }
}
