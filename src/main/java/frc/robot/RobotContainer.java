package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controls.Controls;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOPrototype;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Shooter shooter;

  private final Controls controls;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        shooter = new Shooter(new ShooterIOPrototype() {});
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations

        shooter = new Shooter(new ShooterIOPrototype() {});
        break;

      default:
        // Replayed robot, disable IO implementations

        shooter = new Shooter(new ShooterIO() {});
        break;
    }

    controls = new Controls(shooter);
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
