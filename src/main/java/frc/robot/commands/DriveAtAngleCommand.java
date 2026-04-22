package frc.robot.commands;

import badgerutils.triggers.AllianceTriggers;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedNetworkNumberPlus;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class DriveAtAngleCommand extends Command {
  private static double ANGLE_KP = 12;
  private static double ANGLE_KI = 0.00;
  private static double ANGLE_KD = 0.15;

  @AutoLogOutput
  private static final LoggedNetworkNumberPlus KP_SUPPLIER =
      new LoggedNetworkNumberPlus("/Tuning/Angle KP", ANGLE_KP);

  @AutoLogOutput
  private static final LoggedNetworkNumberPlus KD_SUPPLIER =
      new LoggedNetworkNumberPlus("/Tuning/Angle KD", ANGLE_KD);

  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final Rotation2d INITIAL_TOLERANCE = Rotation2d.fromDegrees(1);
  public static final Rotation2d ADJUSTMENT_TOLERANCE = Rotation2d.fromDegrees(2);

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

    this.addRequirements(drive);

    KP_SUPPLIER.addSubscriber(value -> angleController.setP(value));
    KD_SUPPLIER.addSubscriber(value -> angleController.setD(value));
  }

  @Override
  public void initialize() {
    resetPID();
  }

  @Override
  public void execute() {
    // Get linear velocity
    Translation2d linearVelocity =
        DriveCommands.getLinearVelocityFromJoysticks(
            xSupplier.getAsDouble(), ySupplier.getAsDouble());

    // Calculate angular speed
    double omega = getPIDOutput(false);

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
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            AllianceTriggers.isRedAlliance()
                ? drive.getRotation().plus(new Rotation2d(Math.PI))
                : drive.getRotation()));
  }

  public void resetPID() {
    angleController.reset(drive.getRotation().getRadians());
    angleController.setTolerance(INITIAL_TOLERANCE.getRadians());
  }

  public double getPIDOutput(boolean flipped) {
    Logger.recordOutput("Drive/Angle Setpoint Error", angleController.getPositionError());
    

    return angleController.calculate(
        drive.getRotation().getRadians(),
        flipped
            ? rotationSupplier.get().getRadians() + Math.PI
            : rotationSupplier.get().getRadians());
  }
}
