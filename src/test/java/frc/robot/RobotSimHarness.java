package frc.robot;

import com.ctre.phoenix6.unmanaged.Unmanaged;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

/**
 * Boots the real {@link frc.robot.Robot} in simulation and drives it one loop at a time, so tests
 * can inject controller input and observe what the robot actually commands.
 *
 * <p><b>To write a test:</b> annotate the class with
 * {@code @ExtendWith(RobotSimulationExtension.class)}, then call {@link #getInstance()}. The
 * extension builds the robot once and resets input state between tests.
 *
 * <p><b>Why this is a singleton.</b> A great deal of the robot's startup is one-shot per JVM:
 * {@code Logger.start()}, {@code NamedCommands.registerCommand} (called six times by {@code
 * Autos}), the static {@code Controls.persistentTriggers} set, and {@code AutoBuilder.configure}.
 * Constructing a second {@code RobotContainer} would double-register all of it. Gradle runs the
 * whole test task in one JVM, so a static instance is genuinely process-wide.
 */
public final class RobotSimHarness {

  public static final double PERIOD_SECONDS = 0.02;

  private static final long PERIOD_MILLIS = (long) (PERIOD_SECONDS * 1000);

  public static final int DRIVER_PORT = 0;
  public static final int OPERATOR_PORT = 1;

  private static final int AXIS_COUNT = 6;
  private static final int BUTTON_COUNT = 10;
  private static final int POV_COUNT = 1;

  /** A POV of -1 means "not pressed"; 0 would read as D-pad UP. */
  private static final int POV_RELEASED = -1;

  /**
   * Loops run after construction, before any test touches a control. The controls selector fires
   * its change callback a second time on the first logger cycle (its previous value starts as an
   * empty string), which re-runs {@code clear()} then {@code bind()}. Settling here means a held
   * input can never be swallowed by that re-bind.
   */
  private static final int SETTLE_LOOPS = 5;

  private static RobotSimHarness instance;

  private final SimulatedRobot robot;
  private final XboxControllerSim driver;
  private final XboxControllerSim operator;
  private final List<Runnable> preLoopHooks = new CopyOnWriteArrayList<>();

  /** Returns the process-wide harness, booting the robot on first call. */
  public static synchronized RobotSimHarness getInstance() {
    if (instance == null) {
      instance = new RobotSimHarness();
    }
    return instance;
  }

  private RobotSimHarness() {
    if (!HAL.initialize(500, 0)) {
      throw new IllegalStateException("HAL.initialize failed; simulation natives may be missing");
    }
    SimHooks.setProgramStarted();
    SimHooks.restartTiming();
    // Deliberately NOT paused. SimHooks.stepTiming() blocks until every pending notifier alarm has
    // been serviced, and the robot leaves notifier threads parked (the IterativeRobotBase watchdog
    // among them), so stepping deadlocks. Phoenix also brings its simulated CAN bus up on a
    // real-time background thread. Running the clock free and pacing loops in real time is both
    // simpler and closer to how desktop simulation actually behaves.
    SimHooks.resumeTiming();

    // Reset first: the XboxControllerSim constructors set axis/button/POV counts, and resetData
    // would wipe them.
    DriverStationSim.resetData();
    driver = new XboxControllerSim(DRIVER_PORT);
    operator = new XboxControllerSim(OPERATOR_PORT);

    // Neutralize inputs BEFORE the robot is constructed, so no binding sees a spurious rising edge
    // as it is installed.
    neutralizeHid();
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setFmsAttached(false);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(false);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();

    // AdvantageKit's console capture buffers stdout unboundedly, which is pure overhead in tests.
    Logger.disableConsoleCapture();

    // Constructing Robot runs Logger.start() and builds RobotContainer, which installs bindings.
    robot = new SimulatedRobot();

    // Mirror what LoggedRobot.startCompetition() does after robotInit, so @AutoLogOutput fields are
    // registered and the cycle opened by Logger.start() is closed.
    AutoLogOutputManager.addObject(robot);
    Logger.AdvancedHooks.invokePeriodicAfterUser(0, 0);

    step(SETTLE_LOOPS);
  }

  /** Runs a single robot loop. */
  public void step() {
    step(1);
  }

  /** Runs {@code loops} robot loops, one robot period apart. */
  public void step(int loops) {
    for (int i = 0; i < loops; i++) {
      // Belt-and-braces: keeps Phoenix devices actuating even if the DS enable path lapses.
      Unmanaged.feedEnable((int) (PERIOD_MILLIS * 10));
      preLoopHooks.forEach(Runnable::run);

      Logger.AdvancedHooks.invokePeriodicBeforeUser();
      long userCodeStart = RobotController.getFPGATime();
      robot.runLoopOnce();
      long userCodeEnd = RobotController.getFPGATime();
      Logger.AdvancedHooks.invokePeriodicAfterUser(userCodeEnd - userCodeStart, 0);

      // The clock free-runs, so pace loops in real time to give the HAL, Phoenix's device models,
      // and the notifier threads a period's worth of wall clock between cycles.
      sleep(PERIOD_MILLIS);
    }
  }

  private static void sleep(long millis) {
    try {
      Thread.sleep(millis);
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
      throw new IllegalStateException("interrupted while running the robot simulation", e);
    }
  }

  /** Steps until {@code condition} holds, up to {@code maxLoops}. Returns whether it held. */
  public boolean stepUntil(BooleanSupplier condition, int maxLoops) {
    for (int i = 0; i < maxLoops; i++) {
      if (condition.getAsBoolean()) {
        return true;
      }
      step();
    }
    return condition.getAsBoolean();
  }

  /**
   * Steps until {@code condition} holds, throwing a described {@link AssertionError} if it never
   * does.
   */
  public void stepUntilOrFail(String description, BooleanSupplier condition, int maxLoops) {
    if (!stepUntil(condition, maxLoops)) {
      throw new AssertionError(
          "Timed out after " + maxLoops + " loops waiting for: " + description);
    }
  }

  /** Puts the Driver Station into enabled teleop and runs the transition loops. */
  public void enableTeleop() {
    setMode(true, false, false);
  }

  /** Disables the robot and runs the transition loops. */
  public void disable() {
    setMode(false, false, false);
  }

  private void setMode(boolean enabled, boolean autonomous, boolean test) {
    DriverStationSim.setAutonomous(autonomous);
    DriverStationSim.setTest(test);
    DriverStationSim.setEnabled(enabled);
    DriverStationSim.notifyNewData();
    // Loop 1 refreshes DS data and runs the mode's init; loop 2 is the first full cycle in it.
    step(2);
  }

  public XboxControllerSim driver() {
    return driver;
  }

  public XboxControllerSim operator() {
    return operator;
  }

  /** Sets the operator's right trigger axis. {@code rightTrigger()} activates above 0.5. */
  public void setOperatorRightTrigger(double value) {
    operator.setRightTriggerAxis(value);
    DriverStationSim.notifyNewData();
  }

  /**
   * Registers a callback run before each loop, used by fixtures to feed simulated device inputs
   * (supply voltage, sensor readings) the way a physical robot would.
   */
  public void addPreLoopHook(Runnable hook) {
    preLoopHooks.add(hook);
  }

  /** Returns all controls to neutral and cancels running commands. Called between tests. */
  void resetForTest() {
    neutralizeHid();
    DriverStationSim.notifyNewData();
    // Disabling lets any whileTrue command see its falling edge and run its end() action.
    disable();
    CommandScheduler.getInstance().cancelAll();
    step();
  }

  private void neutralizeHid() {
    for (int port : new int[] {DRIVER_PORT, OPERATOR_PORT}) {
      DriverStationSim.setJoystickAxisCount(port, AXIS_COUNT);
      DriverStationSim.setJoystickButtonCount(port, BUTTON_COUNT);
      DriverStationSim.setJoystickPOVCount(port, POV_COUNT);
      for (int axis = 0; axis < AXIS_COUNT; axis++) {
        DriverStationSim.setJoystickAxis(port, axis, 0.0);
      }
      DriverStationSim.setJoystickButtons(port, 0);
      DriverStationSim.setJoystickPOV(port, 0, POV_RELEASED);
    }
  }
}
