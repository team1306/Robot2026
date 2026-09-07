package frc.robot.tests;

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

  private static final int MAX_LOOPS = 25;

  @Test
  @Timeout(value = 120, unit = TimeUnit.SECONDS)
  void operatorRightTriggerSpoolsAllFourShooterMotors() {
    RobotSimHarness harness = RobotSimHarness.getInstance();
    SimFixture shooter = SimFixtures.createShooterSimFixture(harness);

    harness.enableTeleop();

    shooter.assertMotorsStopped(
        harness, "all shooter motors idle before the trigger is pressed", MAX_LOOPS);

    harness.operator().setRightTriggerAxis(1.0);
    DriverStationSim.notifyNewData();

    shooter.assertMotorsRunning(harness, "all shooter motors commanded to spin", MAX_LOOPS);

    shooter.checkMotorCondition(
        harness,
        "all shooter motors have a nonzero velocity setpoint",
        motor -> Math.abs(shooter.closedLoopReferenceRps(motor)) > 0.0,
        MAX_LOOPS);
  }

  @Test
  @Timeout(value = 120, unit = TimeUnit.SECONDS)
  void releasingTheTriggerReturnsTheShooterToIdle() {
    RobotSimHarness harness = RobotSimHarness.getInstance();
    SimFixture shooter = SimFixtures.createShooterSimFixture(harness);

    harness.enableTeleop();
    harness.operator().setRightTriggerAxis(1.0);
    DriverStationSim.notifyNewData();

    shooter.assertMotorsRunning(harness, "all shooter motors commanded to spin", MAX_LOOPS);

    harness.operator().setRightTriggerAxis(0.0);
    DriverStationSim.notifyNewData();

    shooter.assertMotorsStopped(harness, "all shooter motors released to neutral", MAX_LOOPS);
  }
}
