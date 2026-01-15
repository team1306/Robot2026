package frc.robot.controls;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;
import java.util.EnumMap;
import java.util.function.Consumer;

public class Controls {

  // ============================== CONTROLLERS ===================================
  private final CommandXboxController driverController;
  private final CommandXboxController operatorController;

  // ============================== SUBSYSTEMS ====================================
  private final Drive drivetrain;

  private final EnumMap<ControlStates, ControllerMapping> mappings =
      new EnumMap<>(ControlStates.class);

  private ControlStates currentState = ControlStates.DEFAULT;

  public Controls(Drive drivetrain) {
    this.drivetrain = drivetrain;

    DriverStation.silenceJoystickConnectionWarning(true);
    driverController = new CommandXboxController(0);
    operatorController = new CommandXboxController(1);

    mappings.put(
        ControlStates.DEFAULT, new DefaultControllerMapping(driverController, operatorController));
    mappings.put(
        ControlStates.OTHER, new OtherControllerMapping(driverController, operatorController));

    Consumer<ControlStates> onChange =
        (nextState) -> {
          mappings.get(currentState).clear();
          mappings.get(nextState).bind();
          currentState = nextState;
        };

    // waiting on pull request from badger-utils
    //        BadgerLog.createSelectorFromEnum(
    //                "Controls/Controller Mode",
    //                ControlStates.class,
    //                ControlStates.DEFAULT,
    //                value -> mappings.get((ControlStates) value).run());
  }
}
