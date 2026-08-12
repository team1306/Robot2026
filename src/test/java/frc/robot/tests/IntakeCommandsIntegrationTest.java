package frc.robot.tests;

import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.hardware.TalonFX;
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

  @Test
  @Timeout(value = 120, unit = TimeUnit.SECONDS)
  void intakeAtDutyCycleControlsAllMotors() {
    RobotSimHarness harness = RobotSimHarness.getInstance();
    SimFixture fixture = SimFixtures.createIntakeSimFixture(harness);
    Intake intake = harness.robotContainer().intake;

    harness.enableTeleop();

    CommandScheduler.getInstance().schedule(intake.intakeAtDutyCycleCommand(1));

    harness.stepUntilOrFail(
        "all four intake motors commanded to spin",
        () -> {
          for (TalonFX motor : fixture.motors()) {
            if (Math.abs(fixture.torqueCurrentAmps(motor)) <= MIN_COMMAND_AMPS) return false;
          }
          return true;
        },
        MAX_LOOPS);

    for (TalonFX motor : fixture.motors()) {
      assertTrue(
          Math.abs(fixture.torqueCurrentAmps(motor)) > MIN_COMMAND_AMPS,
          () -> "motor was not commanded: " + fixture.describe(motor));
    }

    intake.setDutyCycle(0);
  }

  @Test
  @Timeout(value = 120, unit = TimeUnit.SECONDS)
  void leftTriggerRunsIntake() {
    RobotSimHarness harness = RobotSimHarness.getInstance();
    SimFixture intake = SimFixtures.createIntakeSimFixture(harness);

    harness.enableTeleop();

    for (TalonFX motor : intake.motors()) {
      assertTrue(
          Math.abs(intake.torqueCurrentAmps(motor)) <= MIN_COMMAND_AMPS,
          () -> "expected idle before the trigger was pressed: " + intake.describe(motor));
    }

    harness.driver().setLeftTriggerAxis(1);
    DriverStationSim.notifyNewData();

    harness.stepUntilOrFail(
        "all four intake motors commanded to spin",
        () -> {
          for (TalonFX motor : intake.motors()) {
            if (Math.abs(intake.torqueCurrentAmps(motor)) <= MIN_COMMAND_AMPS) return false;
          }
          return true;
        },
        MAX_LOOPS);

    for (TalonFX motor : intake.motors()) {
      assertTrue(
          Math.abs(intake.torqueCurrentAmps(motor)) > MIN_COMMAND_AMPS,
          () -> "motor was not commanded: " + intake.describe(motor));
    }
  }

  @Test
  @Timeout(value = 120, unit = TimeUnit.SECONDS)
  void intakeUntilInterruptedCommandControlsAllMotors() {
    RobotSimHarness harness = RobotSimHarness.getInstance();
    SimFixture fixture = SimFixtures.createIntakeSimFixture(harness);
    Intake intake = harness.robotContainer().intake;
    Command command = intake.intakeUntilInterruptedCommand(1);

    harness.enableTeleop();
    CommandScheduler.getInstance().cancelAll();

    for (TalonFX motor : fixture.motors()) {
      assertTrue(
          Math.abs(fixture.torqueCurrentAmps(motor)) <= MIN_COMMAND_AMPS,
          () -> "expected idle before command started: " + fixture.describe(motor));
    }

    CommandScheduler.getInstance().schedule(command);

    harness.stepUntilOrFail(
        "Expected all motors to spin",
        () -> {
          for (TalonFX motor : fixture.motors()) {
            if (Math.abs(fixture.torqueCurrentAmps(motor)) <= MIN_COMMAND_AMPS) return false;
          }
          return true;
        },
        MAX_LOOPS);

    CommandScheduler.getInstance().cancel(command);

    harness.stepUntilOrFail(
        "Expected all motors to stop",
        () -> {
          for (TalonFX motor : fixture.motors()) {
            if (Math.abs(fixture.torqueCurrentAmps(motor)) > MIN_COMMAND_AMPS) return false;
          }
          return true;
        },
        MAX_LOOPS);

    CommandScheduler.getInstance().unregisterSubsystem(intake);
  }
}
