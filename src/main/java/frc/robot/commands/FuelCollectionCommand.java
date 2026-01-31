package frc.robot.commands;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.fueldetection.FuelDetection;
import frc.robot.subsystems.fueldetection.ObjectTarget;

/**
 * Uses the fuel detection camera to detect, rotate to, and then drive towards the largest clump of
 * fuel as detected by the camera
 */
public class FuelCollectionCommand extends Command {

  private final Drive drive;
  private final FuelDetection fuelDetection;

  private ObjectTarget target;

  private double lastTargetSeenTime = 0;

  private static final double LOSE_TIME_SECONDS = 1;

  private static final LinearVelocity forwardSpeed = MetersPerSecond.of(2);

  private final PIDController pidController = new PIDController(0, 0, 0);

  public FuelCollectionCommand(Drive drive, FuelDetection fuelDetection) {
    this.drive = drive;
    this.fuelDetection = fuelDetection;

    pidController.setSetpoint(0);
  }

  @Override
  public void execute() {
    ObjectTarget bestTarget = fuelDetection.getBestTarget();
    boolean isRealTarget = bestTarget.area() != -1;

    if (lastTargetSeenTime - Timer.getFPGATimestamp() > LOSE_TIME_SECONDS) {
      return;
    }

    if (isRealTarget) {
      lastTargetSeenTime = Timer.getFPGATimestamp();
      target = bestTarget;

      double value = pidController.calculate(target.yaw());
      driveWithAngularVelocity(DegreesPerSecond.of(value));
    } else {
      double yaw = target.yaw() + drive.getPose().getRotation().getDegrees();

      double value = pidController.calculate(yaw);
      driveWithAngularVelocity(DegreesPerSecond.of(value));
    }

    driveWithForwardVelocity(forwardSpeed);
  }

  private void driveWithAngularVelocity(AngularVelocity velocity) {
    drive.runVelocity(new ChassisSpeeds(MetersPerSecond.of(0), MetersPerSecond.of(0), velocity));
  }

  private void driveWithForwardVelocity(LinearVelocity velocity) {
    drive.runVelocity(new ChassisSpeeds(velocity, MetersPerSecond.of(0), RotationsPerSecond.of(0)));
  }

  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(new ChassisSpeeds(0, 0, 0));
  }
}
