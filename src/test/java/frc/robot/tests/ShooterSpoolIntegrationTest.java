package frc.robot.tests;

import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.RobotSimHarness;
import frc.robot.RobotSimulationExtension;
import frc.robot.SimFixture;
import frc.robot.SimFixtures;
import java.util.concurrent.TimeUnit;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;
import org.junit.jupiter.api.extension.ExtendWith;

/**
 * End-to-end check that the "Spool Shooter" control reaches the hardware: holding the operator's
 * right trigger must command all four shooter motors to spin.
 *
 * <p>This exercises the real path rather than a shortcut. The trigger axis is set through the
 * Driver Station simulation, so the binding in {@code CompetitionControllerMapping} is what
 * schedules the command, the real {@code ShooterIOReal} issues the CTRE control requests, and the
 * assertions read those requests back out of Phoenix's simulated devices.
 *
 * <p>At the default pose the robot is 6.1 m from the blue hub, which interpolates to a setpoint of
 * 28 rot/s from {@code ShooterCommands.HUB_SETPOINTS}. The closed-loop reference assertion below
 * would catch a future setpoint-table change that silently zeroed the command.
 */
@ExtendWith(RobotSimulationExtension.class)
class ShooterSpoolIntegrationTest {

  /** Comfortably above sensor noise, far below the ~170 A the spool command actually produces. */
  private static final double MIN_COMMAND_AMPS = 1.0;

  private static final int MAX_LOOPS = 25;

  @Test
  @Timeout(value = 120, unit = TimeUnit.SECONDS)
  void operatorRightTriggerSpoolsAllFourShooterMotors() {
    RobotSimHarness harness = RobotSimHarness.getInstance();
    SimFixture shooter = SimFixtures.createShooterSimFixture(harness);

    harness.enableTeleop();

    for (TalonFX motor : shooter.motors()) {
      assertTrue(
          Math.abs(shooter.torqueCurrentAmps(motor)) <= MIN_COMMAND_AMPS,
          () -> "expected idle before the trigger was pressed: " + shooter.describe(motor));
    }

    harness.operator().setRightTriggerAxis(1.0);
    DriverStationSim.notifyNewData();

    harness.stepUntilOrFail(
        "all four shooter motors commanded to spin",
        () -> {
          for (TalonFX motor : shooter.motors()) {
            if (Math.abs(shooter.torqueCurrentAmps(motor)) <= MIN_COMMAND_AMPS) return false;
          }
          return true;
        },
        MAX_LOOPS);

    for (TalonFX motor : shooter.motors()) {
      assertTrue(
          Math.abs(shooter.torqueCurrentAmps(motor)) > MIN_COMMAND_AMPS,
          () -> "motor was not commanded: " + shooter.describe(motor));
      assertTrue(
          Math.abs(shooter.closedLoopReferenceRps(motor)) > 0.0,
          () -> "motor had a zero velocity setpoint: " + shooter.describe(motor));
    }
  }

  @Test
  @Timeout(value = 120, unit = TimeUnit.SECONDS)
  void releasingTheTriggerReturnsTheShooterToIdle() {
    RobotSimHarness harness = RobotSimHarness.getInstance();
    SimFixture shooter = SimFixtures.createShooterSimFixture(harness);

    harness.enableTeleop();
    harness.operator().setRightTriggerAxis(1.0);
    DriverStationSim.notifyNewData();

    harness.stepUntilOrFail(
        "shooter spooling",
        () -> {
          for (TalonFX motor : shooter.motors()) {
            if (Math.abs(shooter.torqueCurrentAmps(motor)) <= MIN_COMMAND_AMPS) return false;
          }
          return true;
        },
        MAX_LOOPS);

    harness.operator().setRightTriggerAxis(0.0);
    DriverStationSim.notifyNewData();

    harness.stepUntilOrFail(
        "all four shooter motors released to neutral",
        () -> {
          for (TalonFX motor : shooter.motors()) {
            if (Math.abs(shooter.torqueCurrentAmps(motor)) > MIN_COMMAND_AMPS) return false;
          }
          return true;
        },
        MAX_LOOPS);
  }
}
