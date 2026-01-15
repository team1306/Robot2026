package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class DefaultControllerMapping extends ControllerMapping {
  public DefaultControllerMapping(
      CommandXboxController driverController, CommandXboxController operatorController) {
    super(driverController, operatorController);
  }

  @Override
  public void bind() {}
}
