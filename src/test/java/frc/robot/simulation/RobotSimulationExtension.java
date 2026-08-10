package frc.robot.simulation;

import org.junit.jupiter.api.extension.AfterEachCallback;
import org.junit.jupiter.api.extension.BeforeAllCallback;
import org.junit.jupiter.api.extension.BeforeEachCallback;
import org.junit.jupiter.api.extension.ExtensionContext;
import org.junit.jupiter.api.extension.ExtensionContext.Namespace;

/**
 * Boots {@link RobotSimHarness} once for the whole test run and returns controls to neutral around
 * every test.
 *
 * <p>Apply it explicitly with {@code @ExtendWith(RobotSimulationExtension.class)}. It is
 * deliberately not registered for automatic detection: the build enables JUnit's extension
 * autodetection, so a service-loader registration would drag HAL initialization and full robot
 * construction into the existing pure-logic unit tests that have no need for either.
 */
public final class RobotSimulationExtension
    implements BeforeAllCallback, BeforeEachCallback, AfterEachCallback {

  @Override
  public void beforeAll(ExtensionContext context) {
    // Stored on the ROOT context so the robot is built once per JVM rather than once per class.
    context
        .getRoot()
        .getStore(Namespace.GLOBAL)
        .getOrComputeIfAbsent(RobotSimHarness.class, key -> RobotSimHarness.getInstance());
  }

  @Override
  public void beforeEach(ExtensionContext context) {
    RobotSimHarness.getInstance().resetForTest();
  }

  @Override
  public void afterEach(ExtensionContext context) {
    RobotSimHarness.getInstance().resetForTest();
  }
}
