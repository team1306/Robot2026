
// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

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
import frc.robot.subsystems.vision.Vision;
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
        break;
    }

    controls = new Controls(drive);
    autos = new Autos(drive);

    vision = initializeVision();
  }

  private Vision initializeVision() {
    drive.setVisionStdDevs(VecBuilder.fill(1, 1, 1));

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
                new Rotation3d(0, Units.degreesToRadians(-45), Units.degreesToRadians(-135))))
    );
  }

  public Command getAutonomousCommand() {
    return autos.createCommandFromSelectedAuto();
  }
}