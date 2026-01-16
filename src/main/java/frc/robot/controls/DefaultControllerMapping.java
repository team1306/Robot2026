package frc.robot.controls;

import badgerutils.commands.CommandUtils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;

public class DefaultControllerMapping extends ControllerMapping {
  
  private final Drive drive;
  
  public DefaultControllerMapping(
          CommandXboxController driverController, CommandXboxController operatorController, Drive drive) {
    super(driverController, operatorController);
    this.drive = drive;
  }

  @Override
  public void bind() {
    drive.setDefaultCommand(DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX())
    );

    driverController.start()
            .onTrue(
                    Commands.runOnce(
                           () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)), drive
                    ).ignoringDisable(true));
  }

  @Override
  public void clear() {
    super.clear();
    CommandUtils.removeAndCancelDefaultCommand(drive);
  }
}
