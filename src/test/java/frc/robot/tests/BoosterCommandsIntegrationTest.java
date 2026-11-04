package frc.robot.tests;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotSimHarness;
import frc.robot.RobotSimulationExtension;
import frc.robot.SimFixture;
import frc.robot.SimFixtures;
import frc.robot.subsystems.booster.Booster;
import java.util.concurrent.TimeUnit;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;
import org.junit.jupiter.api.extension.ExtendWith;

@ExtendWith(RobotSimulationExtension.class)
class BoosterCommandsIntegrationTest {
  private static final int MAX_LOOPS = 25;

  private final RobotSimHarness harness;
  private final SimFixture fixture;
  private final Booster booster;

  BoosterCommandsIntegrationTest() {
    harness = RobotSimHarness.getInstance();
    fixture = SimFixtures.createBoosterSimFixture(harness);
    booster = harness.robotContainer().TESTONLY_getBooster();
  }

  @Test
  @Timeout(value = 120, unit = TimeUnit.SECONDS)
  void boostCommandWithConstantDutyCycleControlsTheMotor() {
    Command command = booster.boostCommand(1.0);

    fixture.startCommand(harness, command);
    fixture.assertMotorsRunning(
        harness, "booster motor commanded at a constant duty cycle", MAX_LOOPS);

    CommandScheduler.getInstance().cancel(command);
    fixture.assertMotorsStopped(
        harness, "booster motor stopped after constant-duty-cycle command cancellation", MAX_LOOPS);
  }

  @Test
  @Timeout(value = 120, unit = TimeUnit.SECONDS)
  void boostCommandWithDutyCycleSupplierControlsTheMotor() {
    Command command = booster.boostCommand(() -> 1.0);

    fixture.startCommand(harness, command);
    fixture.assertMotorsRunning(
        harness, "booster motor commanded from a duty-cycle supplier", MAX_LOOPS);

    CommandScheduler.getInstance().cancel(command);
    fixture.assertMotorsStopped(
        harness, "booster motor stopped after supplier command cancellation", MAX_LOOPS);
  }
}
