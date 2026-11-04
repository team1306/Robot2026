package frc.robot.tests;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.simulation.DriverStationSim;
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

    fixture.startCommand(harness, command);
    fixture.assertMotorsRunning(
        harness, "all indexer motors commanded to run at a constant speed", MAX_LOOPS);

    CommandScheduler.getInstance().cancel(command);
    fixture.assertMotorsStopped(
        harness, "all indexer motors stopped after constant-speed command cancellation", MAX_LOOPS);
  }

  @Test
  @Timeout(value = 120, unit = TimeUnit.SECONDS)
  void indexUntilCancelledWithSupplierControlsAllMotors() {
    Command command = indexer.indexUntilCancelledCommand(() -> 1.0);

    fixture.startCommand(harness, command);
    fixture.assertMotorsRunning(
        harness, "all indexer motors commanded to run from a speed supplier", MAX_LOOPS);

    CommandScheduler.getInstance().cancel(command);
    fixture.assertMotorsStopped(
        harness, "all indexer motors stopped after supplier command cancellation", MAX_LOOPS);
  }

  @Test
  @Timeout(value = 120, unit = TimeUnit.SECONDS)
  void jumbleIndexerControlsAllMotors() {
    // NOTE: The way jumble indexer is currently written would make it very jank to properly test
    // the fluctuating nature of it.
    Command command = indexer.jumbleIndexer(() -> 1.0);

    fixture.startCommand(harness, command);
    fixture.assertMotorsRunning(
        harness, "all indexer motors commanded to run by jumbleIndexer", MAX_LOOPS);

    CommandScheduler.getInstance().cancel(command);
    fixture.assertMotorsStopped(
        harness, "all indexer motors stopped after jumbleIndexer cancellation", MAX_LOOPS);
  }

  @Test
  @Timeout(value = 120, unit = TimeUnit.SECONDS)
  void indexForTimeControlsAndStopsAllMotors() {
    fixture.startCommand(harness, indexer.indexForTime(Seconds.of(0.1), 1.0));

    fixture.assertMotorsRunning(
        harness, "all indexer motors commanded to run by indexForTime", MAX_LOOPS);
    fixture.assertMotorsStopped(
        harness, "all indexer motors stopped when indexForTime expires", MAX_LOOPS);
  }

  @Test
  @Timeout(value = 120, unit = TimeUnit.SECONDS)
  void operatorRightBumperRunsTheIndexer() {
    CommandScheduler.getInstance().cancelAll();
    harness.enableTeleop();

    fixture.assertMotorsStopped(
        harness, "all indexer motors idle before the operator right bumper is pressed", MAX_LOOPS);

    harness.operator().setRightBumperButton(true);
    DriverStationSim.notifyNewData();

    fixture.assertMotorsRunning(
        harness, "all indexer motors commanded by the operator right bumper", MAX_LOOPS);

    harness.operator().setRightBumperButton(false);
    DriverStationSim.notifyNewData();

    fixture.assertMotorsStopped(
        harness,
        "all indexer motors stopped after the operator right bumper is released",
        MAX_LOOPS);
  }
}
