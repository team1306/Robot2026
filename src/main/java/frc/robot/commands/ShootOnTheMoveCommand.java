package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LocationUtils;

public class ShootOnTheMoveCommand extends Command {
    private Drive drive;

    // Copied AI psudo-code
    private Rotation2d calculateAimDirection() {
        Translation2d robotPos = drive.getPose().getTranslation();
        Translation2d targetPos = Constants.Locations.blueHub.toTranslation2d();
        Translation2d velocity = new Translation2d(drive.getChassisSpeeds().vxMetersPerSecond, drive.getChassisSpeeds().vyMetersPerSecond);

        // Initial estimate
        Distance distance = LocationUtils.getDistanceToLocation(robotPos, targetPos);
        Time timeOfFlight = getTOF(distance);

        Translation2d aimPoint = new Translation2d();
        // Iterate 3 times
        for (int i = 0; i < 3; i++) {
            Translation2d motionOffset = velocity.times(timeOfFlight.in(Seconds));
            aimPoint = targetPos.minus(motionOffset);

            Distance newDistance = LocationUtils.getDistanceToLocation(aimPoint, robotPos);
            timeOfFlight = getTOF(newDistance);
        }

        return LocationUtils.getDirectionToLocation(robotPos, aimPoint);
    }

    private Time getTOF(Distance distance) {return Seconds.of(0);};
}