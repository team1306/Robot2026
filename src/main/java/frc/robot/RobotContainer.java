package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controls.Controls;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.booster.Booster;
import frc.robot.subsystems.booster.BoosterIO;
import frc.robot.subsystems.booster.BoosterIOReal;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.fueldetection.FuelDetection;
import frc.robot.subsystems.fueldetection.FuelDetectionIO;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOReal;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.leds.LedsIO;
import frc.robot.subsystems.leds.LedsReal;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;

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
  private final Booster booster;
  private final FuelDetection fuelDetection;
  private final Leds leds;

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
        booster = new Booster(new BoosterIOReal());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    VisionConstants.LEFT_BACK_CAMERA_NAME,
                    VisionConstants.LEFT_BACK_CAMERA_POSITION),
                new VisionIOPhotonVision(
                    VisionConstants.LEFT_SIDE_CAMERA_NAME,
                    VisionConstants.LEFT_SIDE_CAMERA_POSITION),
                new VisionIOPhotonVision(
                    VisionConstants.RIGHT_BACK_CAMERA_NAME,
                    VisionConstants.RIGHT_BACK_CAMERA_POSITION),
                new VisionIOPhotonVision(
                    VisionConstants.RIGHT_SIDE_CAMERA_NAME,
                    VisionConstants.RIGHT_SIDE_CAMERA_POSITION));
        fuelDetection = new FuelDetection(new FuelDetectionIO() {});
        leds = new Leds(new LedsReal());

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
        booster = new Booster(new BoosterIOReal());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {});

        fuelDetection = new FuelDetection(new FuelDetectionIO() {});
        leds = new Leds(new LedsReal());
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
        booster = new Booster(new BoosterIO() {});
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {});
        fuelDetection = new FuelDetection(new FuelDetectionIO() {});
        leds = new Leds(new LedsIO() {});
        break;
    }

    controls = new Controls(drive, intake, shooter, indexer, booster, fuelDetection, leds);
    autos = new Autos(drive, indexer, intake, shooter, booster, leds);
  }

  public Command getAutonomousCommand() {
    return autos.createCommandFromSelectedAuto();
  }
}
