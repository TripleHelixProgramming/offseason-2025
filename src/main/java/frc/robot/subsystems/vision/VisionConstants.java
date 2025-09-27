// Copyright (c) 2021-2025 Littleton Robotics
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
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;

public class VisionConstants {

  private static String kStemGymAprilTagLayoutPath =
      Filesystem.getDeployDirectory() + "/stemgym.json";

  public static AprilTagFieldLayout getTagLayout() {
    AprilTagFieldLayout tagLayout;
    try {
      tagLayout = new AprilTagFieldLayout(VisionConstants.kStemGymAprilTagLayoutPath);
    } catch (IOException e) {
      System.err.println("Error loading custom AprilTag layout: " + e.getMessage());
      tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    }
    // tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
    return tagLayout;
  }

  // Camera names, must match names configured on coprocessor
  public static String cameraFrontRightName = "OV2311_TH_4";
  public static String cameraFrontLeftName = "OV2311_TH_3";
  public static String cameraBackRightName = "OV2311_TH_2";
  public static String cameraBackLeftName = "OV2311_TH_1";

  // Robot to camera transforms
  public static Transform3d robotToFrontRightCamera =
      new Transform3d(0.286, -0.332, 0.361, new Rotation3d(0, 0, 0.768));
  public static Transform3d robotToFrontLeftCamera =
      new Transform3d(0.292, 0.316, 0.361, new Rotation3d(0, 0, 5.498));
  public static Transform3d robotToBackRightCamera =
      new Transform3d(-0.259, -0.346, 0.628, new Rotation3d(0, 0, 4.311));
  public static Transform3d robotToBackLeftCamera =
      new Transform3d(-0.252, 0.341, 0.628, new Rotation3d(0, 0, 1.972));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Front Right
        1.0, // Front Left
        1.0, // Back Right
        1.0 // Back Left
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
