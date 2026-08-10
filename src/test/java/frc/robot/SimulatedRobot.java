package frc.robot;

/**
 * Test-only subclass of {@link Robot} that exposes {@code IterativeRobotBase}'s protected loop so
 * an integration test can drive the robot one cycle at a time instead of surrendering the thread to
 * {@code startCompetition()}.
 *
 * <p>This class must stay in package {@code frc.robot}: AdvantageKit's {@code AutoLogOutputManager}
 * seeds its package allowlist from the package of the object passed to {@code addObject}, so moving
 * it elsewhere would silently stop every {@code @AutoLogOutput} field under {@code frc.robot} from
 * being logged, making the simulation diverge from the real robot.
 */
public class SimulatedRobot extends Robot {

  /**
   * Runs exactly one robot loop: Driver Station refresh, mode transitions, the mode's periodic, and
   * the HAL simulation callbacks that Phoenix uses to advance its device models.
   */
  public void runLoopOnce() {
    loopFunc();
  }
}
