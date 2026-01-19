package frc.robot.controls;

import badgerutils.networktables.LoggedNetworkTablesBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.Drive;
import java.util.EnumMap;
import java.util.HashSet;
import java.util.Set;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class Controls {

  // ============================== CONTROLLERS ===================================
  private final CommandXboxController driverController;
  private final CommandXboxController operatorController;

  // ============================== SUBSYSTEMS ====================================
  private final Drive drivetrain;

  private final EnumMap<ControlStates, ControllerMapping> mappings =
      new EnumMap<>(ControlStates.class);

  private ControlStates currentState = ControlStates.DEFAULT;

  private static final Set<Supplier<Trigger>> persistentTriggers = new HashSet<>();

  public Controls(Drive drivetrain) {
    this.drivetrain = drivetrain;

    DriverStation.silenceJoystickConnectionWarning(true);
    driverController = new CommandXboxController(0);
    operatorController = new CommandXboxController(1);

    mappings.put(
        ControlStates.DEFAULT,
        new DefaultControllerMapping(driverController, operatorController, drivetrain));
    mappings.put(
        ControlStates.OTHER, new OtherControllerMapping(driverController, operatorController));

    Consumer<Enum<ControlStates>> onChange =
        (nextState) -> {
          ControlStates actualState = (ControlStates) nextState;
          mappings.get(currentState).clear();
          mappings.get(actualState).bind();

          persistentTriggers.forEach(Supplier::get);
          currentState = actualState;
        };

    LoggedNetworkTablesBuilder.createSelectorFromEnum(
        "Controls/Controller Mode", ControlStates.class, ControlStates.DEFAULT, onChange);
  }

  public static void addPersistentTrigger(Supplier<Trigger> triggerSupplier) {
    persistentTriggers.add(triggerSupplier);
  }
}
