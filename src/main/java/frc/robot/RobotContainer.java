package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controls.Controls;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOReal;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Intake intake;
  private final Indexer indexer;
  private final Shooter shooter;

  private final Controls controls;
  private final Autos autos;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        intake = new Intake(new IntakeIOReal());
        indexer = new Indexer(new IndexerIOReal());
        shooter = new Shooter(new ShooterIOReal());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        intake = new Intake(new IntakeIOSim());
        indexer = new Indexer(new IndexerIOSim());
        shooter = new Shooter(new ShooterIOReal());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        intake = new Intake(new IntakeIO() {});
        indexer = new Indexer(new IndexerIO() {});
        shooter = new Shooter(new ShooterIO() {});
        break;
    }

    controls = new Controls(drive, intake);
    autos = new Autos(drive);

    vision = initializeVision();
  }

  private Vision initializeVision() {
    drive.setVisionStdDevs(VecBuilder.fill(1, 1, 1));
    switch (Constants.currentMode) {
      case REAL:
        return new Vision(
            drive::addVisionMeasurement,
            new VisionIOPhotonVision(
                "frontLeft",
                new Transform3d(
                    0.307325,
                    0.307325,
                    0.215781,
                    new Rotation3d(0, Units.degreesToRadians(-45), Units.degreesToRadians(45)))),
            new VisionIOPhotonVision(
                "frontRight",
                new Transform3d(
                    0.307325,
                    -0.307325,
                    0.215781,
                    new Rotation3d(0, Units.degreesToRadians(-45), Units.degreesToRadians(-45)))),
            new VisionIOPhotonVision(
                "backLeft",
                new Transform3d(
                    -0.307325,
                    0.307325,
                    0.215781,
                    new Rotation3d(0, Units.degreesToRadians(-45), Units.degreesToRadians(135)))),
            new VisionIOPhotonVision(
                "backRight",
                new Transform3d(
                    -0.307325,
                    -0.307325,
                    0.215781,
                    new Rotation3d(0, Units.degreesToRadians(-45), Units.degreesToRadians(-135)))));
      case SIM:
        return new Vision(
            drive::addVisionMeasurement,
            new VisionIOPhotonVisionSim(
                "frontLeft",
                new Transform3d(
                    0.307325,
                    0.307325,
                    0.215781,
                    new Rotation3d(0, Units.degreesToRadians(-45), Units.degreesToRadians(45))),
                () -> drive.getPose()),
            new VisionIOPhotonVisionSim(
                "frontRight",
                new Transform3d(
                    0.307325,
                    -0.307325,
                    0.215781,
                    new Rotation3d(0, Units.degreesToRadians(-45), Units.degreesToRadians(-45))),
                () -> drive.getPose()),
            new VisionIOPhotonVisionSim(
                "backLeft",
                new Transform3d(
                    -0.307325,
                    0.307325,
                    0.215781,
                    new Rotation3d(0, Units.degreesToRadians(-45), Units.degreesToRadians(135))),
                () -> drive.getPose()),
            new VisionIOPhotonVisionSim(
                "backRight",
                new Transform3d(
                    -0.307325,
                    -0.307325,
                    0.215781,
                    new Rotation3d(0, Units.degreesToRadians(-45), Units.degreesToRadians(-135))),
                () -> drive.getPose()));

      default:
        return new Vision(
            drive::addVisionMeasurement,
            new VisionIO() {},
            new VisionIO() {},
            new VisionIO() {},
            new VisionIO() {});
    }
  }

  public Command getAutonomousCommand() {
    return autos.createCommandFromSelectedAuto();
  }
}
