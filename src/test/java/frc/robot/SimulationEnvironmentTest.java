package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.unmanaged.Unmanaged;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import java.util.concurrent.TimeUnit;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

/**
 * Verifies that the simulation environment itself works, independently of any robot code.
 *
 * <p>This is deliberately the smallest possible test that touches the native simulation layer, and
 * it depends on none of the harness classes. When it fails, the problem is the build or native
 * wiring rather than the robot code, which makes every other integration test's failure
 * interpretable.
 *
 * <p>It proves four things in one shot: the WPILib HAL and Phoenix 6 {@code swsim} natives resolve
 * on the test JVM's library path, the HAL initializes, a simulated TalonFX exists and can be
 * driven, and the Driver Station enable state reaches Phoenix's actuator gate.
 *
 * <p>Like {@link RobotSimHarness}, this runs the simulation clock free rather than stepping it.
 * {@code SimHooks.stepTiming} blocks until every pending notifier alarm is serviced, so once any
 * other test in the JVM has constructed the robot and left notifier threads parked, stepping would
 * deadlock in a non-interruptible native call. Real-time waits keep this test independent of the
 * order tests happen to run in.
 */
class SimulationEnvironmentTest {

  /**
   * A CAN ID no subsystem uses, so this test cannot perturb a real device's simulation state. Must
   * stay within Phoenix's valid device ID range of 0-62; the highest ID in {@code Constants.CanIds}
   * is 30.
   */
  private static final int UNUSED_DEVICE_ID = 40;

  private static final double SUPPLY_VOLTAGE = 12.0;
  private static final double COMMANDED_VOLTAGE = 6.0;
  private static final long PERIOD_MILLIS = 20;
  private static final int SETTLE_LOOPS = 20;

  /** Phoenix brings its simulated CAN bus up asynchronously in real time; bound the wait. */
  private static final long DEVICE_REGISTRATION_TIMEOUT_MS = 10_000;

  @Test
  @Timeout(value = 120, unit = TimeUnit.SECONDS)
  void halAndPhoenixSimulationNativesLoad() {
    assertTrue(HAL.initialize(500, 0), "HAL.initialize failed");
    SimHooks.setProgramStarted();

    // Phoenix only actuates when the robot is enabled. feedEnable covers the non-FRC path in case
    // the Driver Station enable state does not propagate.
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(false);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    Unmanaged.feedEnable(1000);

    try (TalonFX talon = new TalonFX(UNUSED_DEVICE_ID)) {
      TalonFXSimState simState = talon.getSimState();

      // Phoenix brings its simulated CAN network up on a background thread in real time, so the
      // device is briefly absent right after the TalonFX is constructed.
      StatusCode supplyStatus = StatusCode.SimDeviceNotFound;
      long deadline = System.currentTimeMillis() + DEVICE_REGISTRATION_TIMEOUT_MS;
      while (supplyStatus != StatusCode.OK && System.currentTimeMillis() < deadline) {
        supplyStatus = simState.setSupplyVoltage(SUPPLY_VOLTAGE);
        if (supplyStatus != StatusCode.OK) {
          sleep(PERIOD_MILLIS);
        }
      }
      assertEquals(
          StatusCode.OK,
          supplyStatus,
          "simulated TalonFX never registered; sim natives may be stale");

      VoltageOut request = new VoltageOut(COMMANDED_VOLTAGE);

      // The control request must be re-sent every loop, exactly as a running command would: Phoenix
      // reverts a device to neutral if it stops receiving control frames.
      StatusCode lastControlStatus = StatusCode.OK;
      for (int i = 0; i < SETTLE_LOOPS; i++) {
        Unmanaged.feedEnable(1000);
        simState.setSupplyVoltage(SUPPLY_VOLTAGE);
        lastControlStatus = talon.setControl(request);
        HAL.simPeriodicBefore();
        HAL.simPeriodicAfter();
        sleep(PERIOD_MILLIS);
      }
      assertEquals(StatusCode.OK, lastControlStatus, "setControl");

      System.out.println(
          "[sim env] motorVoltage="
              + simState.getMotorVoltage()
              + " torqueCurrent="
              + simState.getTorqueCurrent()
              + " deviceEnabled="
              + Unmanaged.getEnableState()
              + " dsEnabled="
              + DriverStation.isEnabled());

      assertNotEquals(
          0.0,
          simState.getMotorVoltage(),
          1e-6,
          "Phoenix simulation produced no motor output for a VoltageOut request");
    }
  }

  private static void sleep(long millis) {
    try {
      Thread.sleep(millis);
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
      throw new IllegalStateException("interrupted while waiting for Phoenix simulation", e);
    }
  }
}
