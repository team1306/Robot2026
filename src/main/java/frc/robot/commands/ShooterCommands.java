package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.Interpolation;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.Supplier;

public class ShooterCommands {

  public static final ShooterSetpoint[] SETPOINTS =
      Arrays.stream(
              new ShooterSetpoint[] {
                new ShooterSetpoint(Meters.of(0), RotationsPerSecond.of(0), Seconds.of(0)),
                new ShooterSetpoint(Meters.of(5), RotationsPerSecond.of(5), Seconds.of(2.5)),
                new ShooterSetpoint(Meters.of(10), RotationsPerSecond.of(10), Seconds.of(3.75)),
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

    double time =
        Interpolation.inverseLerp(
            firstSetpoint.distance.in(Meters),
            secondSetpoint.distance.in(Meters),
            distance.in(Meters));
    double lerpedVelocity =
        Interpolation.lerp(
            firstSetpoint.velocity.in(RotationsPerSecond),
            secondSetpoint.velocity.in(RotationsPerSecond),
            time);

    double lerpedTime =
        Interpolation.lerp(firstSetpoint.time.in(Seconds), secondSetpoint.time.in(Seconds), time);

    return new ShooterSetpoint(
        distance, RotationsPerSecond.of(lerpedVelocity), Seconds.of(lerpedTime));
  }

  public static Command getShootSpeedCommand(Shooter shooter, AngularVelocity velocity) {
    return Commands.run(() -> shooter.setVelocity(velocity), shooter);
  }

  public static Command getShootSpeedCommand(Shooter shooter, Supplier<AngularVelocity> velocity) {
    return Commands.run(() -> shooter.setVelocity(velocity.get()), shooter);
  }

  public static Command getShootSpeedDistanceRelativeCommand(
      Shooter shooter, Supplier<Distance> distance) {
    return getShootSpeedCommand(
        shooter, () -> interpolateSetpoints(SETPOINTS, distance.get()).velocity);
  }

  public record ShooterSetpoint(Distance distance, AngularVelocity velocity, Time time)
      implements Comparable<ShooterSetpoint> {
    @Override
    public int compareTo(ShooterSetpoint o) {
      return distance.compareTo(o.distance);
    }
  }
}
