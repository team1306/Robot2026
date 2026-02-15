package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveAtAngleCommand extends Command {
  private static double ANGLE_KP = 12;
  private static double ANGLE_KI = 0.00;
  private static double ANGLE_KD = 0.15;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final Rotation2d INITIAL_TOLERANCE = Rotation2d.fromDegrees(1);
  private static final Rotation2d ADJUSTMENT_TOLERANCE = Rotation2d.fromDegrees(3);

  private final Drive drive;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final Supplier<Rotation2d> rotationSupplier;
  private final ProfiledPIDController angleController;

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public DriveAtAngleCommand(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {
    this.drive = drive;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.rotationSupplier = rotationSupplier;

    angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            ANGLE_KI,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    angleController.reset(drive.getRotation().getRadians());
    angleController.setTolerance(INITIAL_TOLERANCE.getRadians());
  }

  @Override
  public void execute() {
    // Get linear velocity
    Translation2d linearVelocity =
        DriveCommands.getLinearVelocityFromJoysticks(
            xSupplier.getAsDouble(), ySupplier.getAsDouble());

    // Calculate angular speed
    double omega =
        angleController.calculate(
            drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

    // Warning: PID error and setpoints are not correct when outputting, but it works, so it's good.
    Logger.recordOutput("Drive/At Angle Setpoint", angleController.atSetpoint());

    // If not moving and at desired angle
    if (linearVelocity.getX() == 0 && linearVelocity.getY() == 0 && angleController.atSetpoint()) {
      drive.stopWithX();
      angleController.setTolerance(ADJUSTMENT_TOLERANCE.getRadians());
      return;
    }
    angleController.setTolerance(INITIAL_TOLERANCE.getRadians());

    // Convert to field relative speeds & send command
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
            omega);
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
  }
}
