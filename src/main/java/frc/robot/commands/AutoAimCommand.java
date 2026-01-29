package frc.robot.commands;

import badgerutils.triggers.AllianceTriggers;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;

public class AutoAimCommand extends Command {

  private final Drive drive;

  private Translation3d target;

  public AutoAimCommand(Drive drive) {
    this.drive = drive;
  }

  @Override
  public void initialize() {
    target =
        AllianceTriggers.isBlueAlliance()
            ? Constants.Locations.blueHub
            : Constants.Locations.redHub;
  }

  @Override
  public void execute() {}
}