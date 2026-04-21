package frc.robot.commands;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.Interpolation;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ShooterCommands {

  public static final ShooterSetpoint[] SETPOINTS =
      Arrays.stream(
              new ShooterSetpoint[] {
                new ShooterSetpoint(Meters.of(3.07), RotationsPerSecond.of(35), Seconds.of(0.881)),
                new ShooterSetpoint(Meters.of(2.41), RotationsPerSecond.of(33), Seconds.of(0.763)),
                new ShooterSetpoint(Meters.of(3.88), RotationsPerSecond.of(38), Seconds.of(1.013)),
                new ShooterSetpoint(Meters.of(4.85), RotationsPerSecond.of(42), Seconds.of(1.156)),
                new ShooterSetpoint(Meters.of(4.42), RotationsPerSecond.of(40.5), Seconds.of(1.094)),
                new ShooterSetpoint(Meters.of(5.61), RotationsPerSecond.of(55), Seconds.of(1.261)),
              })
          .sorted()
          .toArray(ShooterSetpoint[]::new);

  public static ShooterSetpoint interpolateSetpoints(
      ShooterSetpoint[] setpoints, Distance distance) {
    Optional<ShooterSetpoint> firstSetpointOptional =
        Arrays.stream(setpoints)
            .filter(setpoint -> setpoint.distance.lt(distance))
            .reduce((a, b) -> b);

    Optional<ShooterSetpoint> secondSetpointOptional =
        Arrays.stream(setpoints).filter(setpoint -> setpoint.distance.gte(distance)).findFirst();

    if (firstSetpointOptional.isEmpty() && secondSetpointOptional.isEmpty())
      return new ShooterSetpoint(Meters.of(0), RotationsPerSecond.of(0), Seconds.of(0));
    else if (firstSetpointOptional.isEmpty()) return secondSetpointOptional.get();
    else if (secondSetpointOptional.isEmpty()) return firstSetpointOptional.get();

    ShooterSetpoint firstSetpoint = firstSetpointOptional.get();
    ShooterSetpoint secondSetpoint = secondSetpointOptional.get();

    double t =
        Interpolation.inverseLerp(
            firstSetpoint.distance.in(Meters),
            secondSetpoint.distance.in(Meters),
            distance.in(Meters));
    double lerpedVelocity =
        Interpolation.lerp(
            firstSetpoint.velocity.in(RotationsPerSecond),
            secondSetpoint.velocity.in(RotationsPerSecond),
            t);

    double lerpedTime =
        Interpolation.lerp(firstSetpoint.time.in(Seconds), secondSetpoint.time.in(Seconds), t);

    return new ShooterSetpoint(
        distance, RotationsPerSecond.of(lerpedVelocity), Seconds.of(lerpedTime));
  }

  public static Command shootAtSpeedCommand(Shooter shooter, AngularVelocity velocity) {
    return Commands.startEnd(() -> shooter.setVelocity(velocity), shooter::setIdle, shooter);
  }

  public static Command shootAtSpeedCommand(Shooter shooter, Supplier<AngularVelocity> velocity) {
    return Commands.runEnd(() -> shooter.setVelocity(velocity.get()), shooter::setIdle, shooter);
  }

  public static Command shootAtDistanceCommand(Shooter shooter, Supplier<Distance> distance) {
    return shootAtSpeedCommand(
        shooter,
        () -> {
          Logger.recordOutput("Shooter/Distance to Target", distance.get().in(Feet));
          return interpolateSetpoints(SETPOINTS, distance.get()).velocity;
        });
  }

  public static Command shootForTimeCommand(
      Shooter shooter, Supplier<Distance> distanceSupplier, Time time) {
    return new ParallelDeadlineGroup(
        new WaitCommand(time), shootAtDistanceCommand(shooter, distanceSupplier));
  }

  public record ShooterSetpoint(Distance distance, AngularVelocity velocity, Time time)
      implements Comparable<ShooterSetpoint> {
    @Override
    public int compareTo(ShooterSetpoint o) {
      return distance.compareTo(o.distance);
    }
  }
}
