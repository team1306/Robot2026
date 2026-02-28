package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;

public class GuardedCommand extends Command {

  private final Command command;
  private final BooleanSupplier condition;

  private boolean initialized;

  /**
   * Constructs a new Guarded command, which initializes and runs the {@code command} when all the
   * {@code conditions} are true, and cancels it when any are false. This may reinitialize a command
   * multiple times in the lifetime of this command
   *
   * @param command the command to be guarded against
   * @param conditions the guards to put in place
   */
  public GuardedCommand(Command command, BooleanSupplier... conditions) {
    this.command = command;

    condition =
        () -> {
          for (BooleanSupplier condition : conditions) {
            if (!condition.getAsBoolean()) {
              return false;
            }
          }
          return true;
        };

    addRequirements(command.getRequirements());
  }

  @Override
  public void initialize() {
    if (condition.getAsBoolean()) {
      command.initialize();
      initialized = true;
    }
  }

  @Override
  public void execute() {
    if (!initialized) {
      command.initialize();
      initialized = true;
    }

    if (condition.getAsBoolean()) {
      command.execute();
    } else {
      initialized = false;
      command.end(true);
    }
  }

  @Override
  public void end(boolean interrupted) {
    command.end(interrupted);
  }
}
