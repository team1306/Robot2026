package frc.robot.tests;

import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotSimHarness;
import frc.robot.RobotSimulationExtension;
import frc.robot.SimFixture;
import frc.robot.SimFixtures;
import frc.robot.subsystems.intake.Intake;
import java.util.concurrent.TimeUnit;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;
import org.junit.jupiter.api.extension.ExtendWith;

@ExtendWith(RobotSimulationExtension.class)
public class IntakeCommandsIntegrationTest {
  private static final int MAX_LOOPS = 25;

  private final RobotSimHarness harness;
  private final SimFixture fixture;
  private final Intake intake;

  public IntakeCommandsIntegrationTest() {
    // Common test startup
    harness = RobotSimHarness.getInstance();
    fixture = SimFixtures.createIntakeSimFixture(harness);
    intake = harness.robotContainer().TESTONLY_getIntake();
  }

  @Test
  @Timeout(value = 120, unit = TimeUnit.SECONDS)
  void intakeAtDutyCycleControlsAllMotors() {
    fixture.startCommand(harness, intake.intakeAtDutyCycleCommand(1));

    fixture.assertMotorsRunning(harness, "all intake motors commanded to spin", MAX_LOOPS);

    intake.setDutyCycle(0);
  }

  @Test
  @Timeout(value = 120, unit = TimeUnit.SECONDS)
  void leftTriggerRunsIntake() {
    CommandScheduler.getInstance().cancelAll();
    harness.enableTeleop();

    fixture.assertMotorsStopped(
        harness, "all intake motors idle before the trigger is pressed", MAX_LOOPS);

    harness.driver().setLeftTriggerAxis(1);
    DriverStationSim.notifyNewData();

    fixture.assertMotorsRunning(
        harness, "all intake motors commanded to spin from the left trigger", MAX_LOOPS);
  }

  @Test
  @Timeout(value = 120, unit = TimeUnit.SECONDS)
  void intakeUntilInterruptedCommandControlsAllMotors() {
    Command command = intake.intakeUntilInterruptedCommand(1);

    CommandScheduler.getInstance().cancelAll();
    harness.enableTeleop();

    fixture.assertMotorsStopped(harness, "all intake motors idle before command starts", MAX_LOOPS);

    fixture.startCommand(harness, command);

    fixture.assertMotorsRunning(harness, "all intake motors commanded to spin", MAX_LOOPS);

    CommandScheduler.getInstance().cancel(command);

    fixture.assertMotorsStopped(
        harness, "all intake motors stopped after command cancellation", MAX_LOOPS);
  }
}
