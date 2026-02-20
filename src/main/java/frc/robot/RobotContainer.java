package frc.robot;

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
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
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

  // Initializing constants for tuning
  private final ShooterConstants shooterConstants = new ShooterConstants();

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
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    VisionConstants.LEFT_CAMERA_NAME, VisionConstants.LEFT_CAMERA_POSITION),
                new VisionIOPhotonVision(
                    VisionConstants.BACK_LEFT_CAMERA_NAME,
                    VisionConstants.BACK_LEFT_CAMERA_POSITION),
                new VisionIOPhotonVision(
                    VisionConstants.BACK_RIGHT_CAMERA_NAME,
                    VisionConstants.BACK_RIGHT_CAMERA_POSITION),
                new VisionIOPhotonVision(
                    VisionConstants.RIGHT_CAMERA_NAME, VisionConstants.RIGHT_CAMERA_POSITION));
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
        intake = new Intake(new IntakeIOReal());
        indexer = new Indexer(new IndexerIOReal());
        shooter = new Shooter(new ShooterIOReal());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.LEFT_CAMERA_NAME,
                    VisionConstants.LEFT_CAMERA_POSITION,
                    () -> drive.getPose()),
                new VisionIOPhotonVisionSim(
                    VisionConstants.BACK_LEFT_CAMERA_NAME,
                    VisionConstants.BACK_LEFT_CAMERA_POSITION,
                    () -> drive.getPose()),
                new VisionIOPhotonVisionSim(
                    VisionConstants.BACK_RIGHT_CAMERA_NAME,
                    VisionConstants.BACK_RIGHT_CAMERA_POSITION,
                    () -> drive.getPose()),
                new VisionIOPhotonVisionSim(
                    VisionConstants.RIGHT_CAMERA_NAME,
                    VisionConstants.RIGHT_CAMERA_POSITION,
                    () -> drive.getPose()));
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
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {});
        break;
    }

    controls = new Controls(drive, intake, indexer, shooter);
    autos = new Autos(drive, indexer, intake, shooter);
  }

  public Command getAutonomousCommand() {
    return autos.createCommandFromSelectedAuto();
  }
}