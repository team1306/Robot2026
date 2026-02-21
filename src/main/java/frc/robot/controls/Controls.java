package frc.robot.controls;

import badgerutils.networktables.LoggedNetworkTablesBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.fueldetection.FuelDetection;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import java.util.EnumMap;
import java.util.HashSet;
import java.util.Set;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class Controls {

  // ============================== CONTROLLERS ===================================
  private final CommandXboxController driverController;
  private final CommandXboxController operatorController;

  private final EnumMap<ControlStates, ControllerMapping> mappings =
      new EnumMap<>(ControlStates.class);

  private ControlStates currentState = ControlStates.COMPETITION;

  private static final Set<Supplier<Trigger>> persistentTriggers = new HashSet<>();

  public Controls(
      Drive drivetrain,
      Intake intake,
      Shooter shooter,
      Indexer indexer,
      FuelDetection fuelDetection) {
    DriverStation.silenceJoystickConnectionWarning(true);
    driverController = new CommandXboxController(0);
    operatorController = new CommandXboxController(1);

    mappings.put(
        ControlStates.COMPETITION,
        new CompetitionControllerMapping(
            driverController,
            operatorController,
            drivetrain,
            intake,
            shooter,
            indexer,
            fuelDetection));
    mappings.put(
        ControlStates.TEST_ONLY_REMOVE_ME,
        new RemoveMeControllerMapping(driverController, operatorController));
    mappings.put(
        ControlStates.SYSID,
        new SysIdControllerMapping(driverController, operatorController, drivetrain));

    Consumer<Enum<ControlStates>> onChange =
        (nextState) -> {
          ControlStates actualState = (ControlStates) nextState;
          if (!mappings.containsKey(actualState)) {
            System.out.println("Invalid State: " + actualState);
            return;
          }

          mappings.get(currentState).clear();
          mappings.get(actualState).bind();

          persistentTriggers.forEach(Supplier::get);
          currentState = actualState;
        };

    LoggedNetworkTablesBuilder.createSelectorFromEnum(
        "Controls/Controller Mode", ControlStates.class, ControlStates.COMPETITION, onChange);
  }

  public static void addPersistentTrigger(Supplier<Trigger> triggerSupplier) {
    persistentTriggers.add(triggerSupplier);
  }
}
