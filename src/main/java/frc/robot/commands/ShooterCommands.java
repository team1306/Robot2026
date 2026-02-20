package frc.robot.commands;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

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
  private static final ShooterSetpoint[] SETPOINTS =
      Arrays.stream(
              new ShooterSetpoint[] {
                new ShooterSetpoint(Feet.of(7.5), RotationsPerSecond.of(41)),
                new ShooterSetpoint(Feet.of(9), RotationsPerSecond.of(42)),
                new ShooterSetpoint(Feet.of(12), RotationsPerSecond.of(48)),
                new ShooterSetpoint(Feet.of(15), RotationsPerSecond.of(53.5)),
                new ShooterSetpoint(Feet.of(18), RotationsPerSecond.of(59.75)),
              })
          .sorted()
          .toArray(ShooterSetpoint[]::new);

  public static AngularVelocity interpolateSetpoints(
      ShooterSetpoint[] setpoints, Distance distance) {
    Optional<ShooterSetpoint> firstSetpointOptional =
        Arrays.stream(setpoints)
            .filter(setpoint -> setpoint.distance.lt(distance))
            .reduce((a, b) -> b);

    Optional<ShooterSetpoint> secondSetpointOptional =
        Arrays.stream(setpoints).filter(setpoint -> setpoint.distance.gte(distance)).findFirst();

    if (firstSetpointOptional.isEmpty() && secondSetpointOptional.isEmpty())
      return RotationsPerSecond.of(0);
    else if (firstSetpointOptional.isEmpty()) return secondSetpointOptional.get().velocity;
    else if (secondSetpointOptional.isEmpty()) return firstSetpointOptional.get().velocity;

    ShooterSetpoint firstSetpoint = firstSetpointOptional.get();
    ShooterSetpoint secondSetpoint = secondSetpointOptional.get();

    double t =
        Interpolation.inverseLerp(
            firstSetpoint.distance.in(Meters),
            secondSetpoint.distance.in(Meters),
            distance.in(Meters));
    double lerpedValue =
        Interpolation.lerp(
            firstSetpoint.velocity.in(RotationsPerSecond),
            secondSetpoint.velocity.in(RotationsPerSecond),
            t);

    return RotationsPerSecond.of(lerpedValue);
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
          return interpolateSetpoints(SETPOINTS, distance.get());
        });
  }

  public static Command shootForTimeCommand(
      Shooter shooter, Supplier<Distance> distanceSupplier, Time time) {
    return new ParallelDeadlineGroup(
        new WaitCommand(time), shootAtDistanceCommand(shooter, distanceSupplier));
  }

  public record ShooterSetpoint(Distance distance, AngularVelocity velocity)
      implements Comparable<ShooterSetpoint> {
    @Override
    public int compareTo(ShooterSetpoint o) {
      return distance.compareTo(o.distance);
    }
  }
}
