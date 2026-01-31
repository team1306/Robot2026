package frc.robot.util;

public final class Interpolation {

  private Interpolation() {}

  public static double lerp(double start, double end, double t) {
    return start + t * (end - start);
  }

  public static double inverseLerp(double start, double end, double value) {
    return (value - start) / (end - start);
  }
}
