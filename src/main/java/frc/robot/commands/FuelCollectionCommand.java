package frc.robot.commands;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;

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
    addRequirements(drive, fuelDetection);
  }

  @Override
  public void execute() {
    ObjectTarget bestTarget = fuelDetection.getBestTarget();
    boolean isRealTarget = bestTarget.area() != -1;

    if (lastTargetSeenTime - Timer.getFPGATimestamp() > LOSE_TIME_SECONDS) {
      return;
    }

    AngularVelocity angularVelocity;
    if (isRealTarget) {
      lastTargetSeenTime = Timer.getFPGATimestamp();
      target = bestTarget;

      double value = pidController.calculate(target.yaw());
      angularVelocity = DegreesPerSecond.of(value);
    } else if (target == null) {
      return;
    } else {
      double yaw = target.yaw() + drive.getPose().getRotation().getDegrees();

      double value = pidController.calculate(yaw);
      angularVelocity = DegreesPerSecond.of(value);
    }

    drive.runVelocity(new ChassisSpeeds(forwardSpeed, MetersPerSecond.of(0), angularVelocity));
  }

  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(new ChassisSpeeds(0, 0, 0));
  }
}
