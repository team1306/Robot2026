package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class SysIdControllerMapping extends ControllerMapping {

  public SysIdControllerMapping(
      CommandXboxController driverController, CommandXboxController operatorController) {
    super(driverController, operatorController);
  }

  @Override
  public void bind() {}

  @Override
  public void clear() {
    super.clear();
  }
}
