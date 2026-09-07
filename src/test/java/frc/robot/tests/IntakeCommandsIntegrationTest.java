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
  /** Comfortably above sensor noise, far below the ~170 A the spool command actually produces. */
  private static final double MIN_COMMAND_AMPS = 1.0;

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
    CommandScheduler.getInstance().cancelAll();
    harness.enableTeleop();

    CommandScheduler.getInstance().schedule(intake.intakeAtDutyCycleCommand(1));

    fixture.checkMotorCondition(
        harness,
        "all intake motors commanded to spin",
        motor -> Math.abs(fixture.torqueCurrentAmps(motor)) > MIN_COMMAND_AMPS,
        MAX_LOOPS);

    intake.setDutyCycle(0);
  }

  @Test
  @Timeout(value = 120, unit = TimeUnit.SECONDS)
  void leftTriggerRunsIntake() {
    CommandScheduler.getInstance().cancelAll();
    harness.enableTeleop();

    fixture.checkMotorCondition(
        harness,
        "all intake motors idle before the trigger is pressed",
        motor -> Math.abs(fixture.torqueCurrentAmps(motor)) <= MIN_COMMAND_AMPS,
        MAX_LOOPS);

    harness.driver().setLeftTriggerAxis(1);
    DriverStationSim.notifyNewData();

    fixture.checkMotorCondition(
        harness,
        "all intake motors commanded to spin from the left trigger",
        motor -> Math.abs(fixture.torqueCurrentAmps(motor)) > MIN_COMMAND_AMPS,
        MAX_LOOPS);
  }

  @Test
  @Timeout(value = 120, unit = TimeUnit.SECONDS)
  void intakeUntilInterruptedCommandControlsAllMotors() {
    Command command = intake.intakeUntilInterruptedCommand(1);

    CommandScheduler.getInstance().cancelAll();
    harness.enableTeleop();

    fixture.checkMotorCondition(
        harness,
        "all intake motors idle before command starts",
        motor -> Math.abs(fixture.torqueCurrentAmps(motor)) <= MIN_COMMAND_AMPS,
        MAX_LOOPS);

    CommandScheduler.getInstance().schedule(command);

    fixture.checkMotorCondition(
        harness,
        "all intake motors commanded to spin",
        motor -> Math.abs(fixture.torqueCurrentAmps(motor)) > MIN_COMMAND_AMPS,
        MAX_LOOPS);

    CommandScheduler.getInstance().cancel(command);

    fixture.checkMotorCondition(
        harness,
        "all intake motors stopped after command cancellation",
        motor -> Math.abs(fixture.torqueCurrentAmps(motor)) <= MIN_COMMAND_AMPS,
        MAX_LOOPS);
  }
}
