package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.LocationUtils;

public class ShootOnTheMoveCommand extends ParallelCommandGroup {
    public ShootOnTheMoveCommand(Drive drive, Shooter shooter, DoubleSupplier xSupplier, DoubleSupplier ySupplier, Translation2d target) {
        Supplier<Translation2d> targetLead = () -> calculateLeadTarget(drive);

        addCommands(
            ShooterCommands.shootAtDistanceCommand(shooter, () -> LocationUtils.getDistanceToLocation(drive.getPose().getTranslation(), targetLead.get())),
            DriveCommands.driveAimLockedCommand(drive, xSupplier, ySupplier, targetLead)
        );
    }

    private Translation2d calculateLeadTarget(Drive drive) {
        Translation2d robotPos = drive.getPose().getTranslation();
        Translation2d targetPos = Constants.Locations.blueHub.toTranslation2d();
        Translation2d velocityVec = new Translation2d(drive.getChassisSpeeds().vxMetersPerSecond, drive.getChassisSpeeds().vyMetersPerSecond);

        // Initial estimate
        Distance distance = LocationUtils.getDistanceToLocation(robotPos, targetPos);
        Time timeOfFlight = ShooterCommands.interpolateSetpoints(ShooterCommands.SETPOINTS, distance).time();

        Translation2d aimPoint = new Translation2d();
        // Iterate 3 times
        for (int i = 0; i < 3; i++) {
            Translation2d motionOffset = velocityVec.times(timeOfFlight.in(Seconds));
            aimPoint = targetPos.minus(motionOffset);

            Distance newDistance = LocationUtils.getDistanceToLocation(aimPoint, robotPos);
            timeOfFlight = ShooterCommands.interpolateSetpoints(ShooterCommands.SETPOINTS, newDistance).time();
        }

        return aimPoint;
    }
}