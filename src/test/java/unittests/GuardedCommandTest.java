package unittests;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.GuardedCommand;
import java.util.concurrent.atomic.AtomicBoolean;
import org.junit.jupiter.api.Test;

class GuardedCommandTest {
  @Test
  void doesNotStartOrEndChildWhileGuardIsFalse() {
    RecordingCommand child = new RecordingCommand();
    GuardedCommand guarded = new GuardedCommand(child, () -> false);

    guarded.initialize();
    guarded.execute();
    guarded.execute();
    guarded.end(true);

    assertEquals(0, child.initializeCount);
    assertEquals(0, child.executeCount);
    assertEquals(0, child.endCount);
  }

  @Test
  void startsStopsAndRestartsChildAsGuardChanges() {
    AtomicBoolean condition = new AtomicBoolean(false);
    RecordingCommand child = new RecordingCommand();
    GuardedCommand guarded = new GuardedCommand(child, condition::get);

    guarded.initialize();
    condition.set(true);
    guarded.execute();
    guarded.execute();

    assertEquals(1, child.initializeCount);
    assertEquals(2, child.executeCount);

    condition.set(false);
    guarded.execute();
    guarded.execute();

    assertEquals(1, child.endCount);
    assertEquals(true, child.lastInterrupted);

    condition.set(true);
    guarded.execute();

    assertEquals(2, child.initializeCount);
    assertEquals(3, child.executeCount);
  }

  @Test
  void endingGuardPropagatesOnlyToAnActiveChild() {
    RecordingCommand child = new RecordingCommand();
    GuardedCommand guarded = new GuardedCommand(child, () -> true);

    guarded.initialize();
    guarded.end(false);
    guarded.end(true);

    assertEquals(1, child.initializeCount);
    assertEquals(1, child.endCount);
    assertEquals(false, child.lastInterrupted);
  }

  private static class RecordingCommand extends Command {
    int initializeCount;
    int executeCount;
    int endCount;
    boolean lastInterrupted;

    @Override
    public void initialize() {
      initializeCount++;
    }

    @Override
    public void execute() {
      executeCount++;
    }

    @Override
    public void end(boolean interrupted) {
      endCount++;
      lastInterrupted = interrupted;
    }
  }
}
