package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
  // AprilTag layout
  public static final AprilTagFieldLayout APRIL_TAG_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

  // Camera names, must match names configured on coprocessor
  public static final String LEFT_BACK_CAMERA_NAME = "leftBack";
  public static final String LEFT_SIDE_CAMERA_NAME = "leftSide";
  public static final String RIGHT_BACK_CAMERA_NAME = "rightBack";
  public static final String RIGHT_SIDE_CAMERA_NAME = "rightSide";

  // Robot to camera transforms
  public static final Transform3d RIGHT_BACK_CAMERA_POSITION =
      new Transform3d(
          Inches.of(-9.607),
          Inches.of(-13.028588),
          Inches.of(8.125806),
          new Rotation3d(Degrees.of(0), Degrees.of(-23), Degrees.of(-165)));
  public static final Transform3d RIGHT_SIDE_CAMERA_POSITION =
      new Transform3d(
          Inches.of(-7.002289),
          Inches.of(-13.805643),
          Inches.of(10.677571),
          new Rotation3d(Degrees.of(0), Degrees.of(-22), Degrees.of(-90)));
  public static final Transform3d LEFT_BACK_CAMERA_POSITION =
      new Transform3d(
          Inches.of(-9.487822),
          Inches.of(12.996737),
          Inches.of(8.415729),
          new Rotation3d(Degrees.of(0), Degrees.of(-23), Degrees.of(165)));
  public static final Transform3d LEFT_SIDE_CAMERA_POSITION =
      new Transform3d(
          Inches.of(-7.002289),
          Inches.of(13.805643),
          Inches.of(10.677571),
          new Rotation3d(Degrees.of(0), Degrees.of(-22), Degrees.of(90)));

  // Basic filtering thresholds
  public static final double MAX_AMBIGUITY = 0.3;
  public static final double MAX_Z_ERROR = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static final double LINEAR_STDDEV_BASELINE = 0.02; // Meters
  public static final double ANGULAR_STDDEV_BASELINE = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static final double[] CAMERA_STDDEV_FACTORS =
      new double[] {
        1.0, // left
        1.0, // back-left
        1.0, // back-right
        1.0 // right
      };
}
