package frc.robot.commands;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Angle;
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
import org.littletonrobotics.junction.Logger;

public class ShooterCommands {

  // Starter setpoints; not well done
  public static final ShooterSetpoint[] HUB_SETPOINTS =
      Arrays.stream(
              new ShooterSetpoint[] {
                new ShooterSetpoint(
                    Meters.of(2.44),
                    Rotations.of(0),
                    RotationsPerSecond.of(22),
                    Seconds.of(0.7824)),
                new ShooterSetpoint(
                    Meters.of(2.86),
                    Rotations.of(0),
                    RotationsPerSecond.of(22.5),
                    Seconds.of(0.8694)),
                new ShooterSetpoint(
                    Meters.of(3.15),
                    Rotations.of(0),
                    RotationsPerSecond.of(23.25),
                    Seconds.of(0.897)),
                new ShooterSetpoint(
                    Meters.of(3.34),
                    Rotations.of(0),
                    RotationsPerSecond.of(23.5),
                    Seconds.of(0.9162)),
                new ShooterSetpoint(
                    Meters.of(3.82),
                    Rotations.of(0),
                    RotationsPerSecond.of(24.5),
                    Seconds.of(1.016)),
                new ShooterSetpoint(
                    Meters.of(4.23),
                    Rotations.of(0),
                    RotationsPerSecond.of(25.75),
                    Seconds.of(1.0426)),
                new ShooterSetpoint(
                    Meters.of(4.69), Rotations.of(0), RotationsPerSecond.of(27), Seconds.of(1.096)),
                new ShooterSetpoint(
                    Meters.of(5),
                    Rotations.of(0.1),
                    RotationsPerSecond.of(27.25),
                    Seconds.of(1.1946)),
                new ShooterSetpoint(
                    Meters.of(5.42),
                    Rotations.of(0.2),
                    RotationsPerSecond.of(28.25),
                    Seconds.of(1.1894)),
              })
          .sorted()
          .toArray(ShooterSetpoint[]::new);

  public static final ShooterSetpoint[] PASSING_SETPOINTS =
      Arrays.stream(
              new ShooterSetpoint[] {
                new ShooterSetpoint(
                    Meters.of(6.3),
                    Rotations.of(.4),
                    RotationsPerSecond.of(22),
                    Seconds.of(0.8942)),
                new ShooterSetpoint(
                    Meters.of(7.25),
                    Rotations.of(.7),
                    RotationsPerSecond.of(28),
                    Seconds.of(0.9304)),
                new ShooterSetpoint(
                    Meters.of(15), Rotations.of(.8), RotationsPerSecond.of(41), Seconds.of(1.66))
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
      return new ShooterSetpoint(
          Meters.of(0), Rotations.of(0), RotationsPerSecond.of(0), Seconds.of(0));
    else if (firstSetpointOptional.isEmpty()) return secondSetpointOptional.get();
    else if (secondSetpointOptional.isEmpty()) return firstSetpointOptional.get();

    ShooterSetpoint firstSetpoint = firstSetpointOptional.get();
    ShooterSetpoint secondSetpoint = secondSetpointOptional.get();

    double t =
        Interpolation.inverseLerp(
            firstSetpoint.distance.in(Meters),
            secondSetpoint.distance.in(Meters),
            distance.in(Meters));
    double lerpedAngle =
        Interpolation.lerp(
            firstSetpoint.hoodAngle.in(Rotations), secondSetpoint.hoodAngle.in(Rotations), t);
    double lerpedVelocity =
        Interpolation.lerp(
            firstSetpoint.velocity.in(RotationsPerSecond),
            secondSetpoint.velocity.in(RotationsPerSecond),
            t);

    double lerpedTime =
        Interpolation.lerp(firstSetpoint.time.in(Seconds), secondSetpoint.time.in(Seconds), t);

    return new ShooterSetpoint(
        distance,
        Rotations.of(lerpedAngle),
        RotationsPerSecond.of(lerpedVelocity),
        Seconds.of(lerpedTime));
  }

  public static Command shootAtSpeedCommand(Shooter shooter, AngularVelocity velocity) {
    return Commands.startEnd(() -> shooter.setVelocity(velocity), shooter::setIdle, shooter);
  }

  public static Command shootAtSpeedCommand(Shooter shooter, Supplier<AngularVelocity> velocity) {
    return Commands.runEnd(() -> shooter.setVelocity(velocity.get()), shooter::setIdle, shooter);
  }

  public static Command shootAtDistanceCommand(
      Shooter shooter, Supplier<Distance> distance, Supplier<ShooterSetpoint[]> setpoints) {
    return shootAtSpeedCommand(
        shooter,
        () -> {
          Logger.recordOutput("Shooter/Distance to Target", distance.get().in(Feet));
          return interpolateSetpoints(setpoints.get(), distance.get()).velocity;
        });
  }

  public record ShooterSetpoint(
      Distance distance, Angle hoodAngle, AngularVelocity velocity, Time time)
      implements Comparable<ShooterSetpoint> {
    @Override
    public int compareTo(ShooterSetpoint o) {
      return distance.compareTo(o.distance);
    }
  }
}
