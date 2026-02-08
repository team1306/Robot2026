package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.drive.Drive;

public class SysIdControllerMapping extends ControllerMapping {

  private final Drive drive;

  public SysIdControllerMapping(
      CommandXboxController driverController,
      CommandXboxController operatorController,
      Drive drive) {
    super(driverController, operatorController);
    this.drive = drive;
  }

  @Override
  public void bind() {
    driverController.a().whileTrue(drive.sysIdDynamic(Direction.kForward));
    driverController.b().whileTrue(drive.sysIdDynamic(Direction.kReverse));
    driverController.x().whileTrue(drive.sysIdQuasistatic(Direction.kForward));
    driverController.y().whileTrue(drive.sysIdQuasistatic(Direction.kReverse));
  }

  @Override
  public void clear() {
    super.clear();
  }
}
