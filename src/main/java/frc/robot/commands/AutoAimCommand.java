package frc.robot.commands;

import badgerutils.triggers.AllianceTriggers;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;

public class AutoAimCommand extends ParallelCommandGroup {

  private final Drive drive;
  private final Shooter shooter;


  public AutoAimCommand(Drive drive, Shooter shooter) {
    this.drive = drive;
    this.shooter = shooter;

    addCommands();
  }

  public Vector<N2> getResultantVector() {
      Translation3d target3d = AllianceTriggers.isBlueAlliance() ? Constants.Locations.blueHub : Constants.Locations.redHub;
      Translation2d target = target3d.toTranslation2d();

      ChassisSpeeds chassisSpeeds = drive.getChassisSpeeds();

      Vector<N2> targetVector = target.toVector();
      Vector<N2> velocityVector = VecBuilder.fill(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);

      return targetVector.minus(velocityVector);
  }
}
