// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
  // AprilTag layout
  public static final AprilTagFieldLayout APRIL_TAG_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

  // Camera names, must match names configured on coprocessor
  public static final String LEFT_CAMERA_NAME = "";
  public static final String BACK_LEFT_CAMERA_NAME = "";
  public static final String BACK_RIGHT_CAMERA_NAME = "";
  public static final String RIGHT_CAMERA_NAME = "";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static final Transform3d LEFT_CAMERA_POSITION =
      new Transform3d(0.2, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0));
  public static final Transform3d BACK_LEFT_CAMERA_POSITION =
      new Transform3d(0.2, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0));
  public static final Transform3d BACK_RIGHT_CAMERA_POSITION =
      new Transform3d(0.2, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0));
  public static final Transform3d RIGHT_CAMERA_POSITION =
      new Transform3d(0.2, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0));

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
