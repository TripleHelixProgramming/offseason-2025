// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import java.io.IOException;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final Supplier<Pose2d> poseSupplier;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  // LinearFilter ambiguityTestPassRate = LinearFilter.movingAverage(20);
  // LinearFilter flatPoseTestPassRate = LinearFilter.movingAverage(20);
  // LinearFilter withinBoundsTestPassRate = LinearFilter.movingAverage(20);
  // LinearFilter moreThanZeroTagsTestPassRate = LinearFilter.movingAverage(20);

  LinearFilter[] cameraPassRate = {
    LinearFilter.movingAverage(20),
    LinearFilter.movingAverage(20),
    LinearFilter.movingAverage(20),
    LinearFilter.movingAverage(20)
  };

  public Vision(VisionConsumer consumer, Supplier<Pose2d> poseSupplier, VisionIO... io) {
    this.consumer = consumer;
    this.poseSupplier = poseSupplier;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }

    // Initialize logging values
    var allTagPoses = new ArrayList<Pose3d>();
    var allRobotPoses = new ArrayList<Pose3d>();
    var allRobotPosesAccepted = new ArrayList<Pose3d>();
    var allRobotPosesRejected = new ArrayList<Pose3d>();

    // List to store acceptable observations
    var observations = new ArrayList<TestedObservation>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      var tagPoses = new ArrayList<Pose3d>();
      var robotPoses = new ArrayList<Pose3d>();
      var robotPosesAccepted = new ArrayList<Pose3d>();
      var robotPosesRejected = new ArrayList<Pose3d>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = getAprilTagLayout().getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        EnumMap<VisionTest, Double> testResults = new EnumMap<>(VisionTest.class);

        testResults.put(VisionTest.moreThanZeroTags, VisionTest.moreThanZeroTags.test(observation));
        testResults.put(VisionTest.unambiguous, VisionTest.unambiguous.test(observation));
        testResults.put(VisionTest.pitchError, VisionTest.pitchError.test(observation));
        testResults.put(VisionTest.rollError, VisionTest.rollError.test(observation));
        testResults.put(VisionTest.heightError, VisionTest.heightError.test(observation));
        testResults.put(VisionTest.withinBoundaries, VisionTest.withinBoundaries.test(observation));

        Double totalScore =
            testResults.values().stream().reduce(1.0, (subtotal, element) -> subtotal * element);

        observations.add(new TestedObservation(observation, cameraIndex, testResults, totalScore));

        // Add pose to log
        robotPoses.add(observation.pose());
        if (totalScore < minScore) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        cameraPassRate[cameraIndex].calculate(totalScore);
      }

      // Log camera datadata
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
          tagPoses.toArray(new Pose3d[tagPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
          robotPoses.toArray(new Pose3d[robotPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/PassRate",
          cameraPassRate[cameraIndex].lastValue());
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Remove unacceptable observations
    observations.removeIf(o -> o.score < minScore);

    // Sort the list of acceptable observations by timestamp
    observations.sort(
        (lhs, rhs) -> (int) Math.signum(lhs.observation.timestamp() - rhs.observation.timestamp()));

    for (var o : observations) {
      // Calculate standard deviations
      double linearStdDev = linearStdDevBaseline / o.score;
      double angularStdDev = angularStdDevBaseline / o.score;

      // Send acceptable vision observations to the pose estimator with their stddevs
      consumer.accept(
          o.observation.pose().toPose2d(),
          o.observation.timestamp(),
          VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
    }

    // Log summary data
    Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(Pose3d[]::new));
    Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(Pose3d[]::new));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(Pose3d[]::new));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(Pose3d[]::new));

    // Logger.recordOutput("Vision/Summary/AmbiguityTestPassRate",
    // ambiguityTestPassRate.lastValue());
    // Logger.recordOutput("Vision/Summary/FlatPoseTestPassRate", flatPoseTestPassRate.lastValue());
    // Logger.recordOutput(
    //     "Vision/Summary/WithinBoundsTestPassRate", withinBoundsTestPassRate.lastValue());
    // Logger.recordOutput(
    //     "Vision/Summary/MoreThanZeroTagsTestPassRate", moreThanZeroTagsTestPassRate.lastValue());
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  // Associate observations with their camera
  public static record TestedObservation(
      PoseObservation observation,
      int cameraIndex,
      EnumMap<VisionTest, Double> testResults,
      double score) {}

  // Caching for AprilTag layout
  public static AprilTagFieldLayout cachedLayout = null;

  /** Returns the AprilTag layout to use, loading it if necessary. */
  public static AprilTagFieldLayout getAprilTagLayout() {
    if (cachedLayout == null) {
      // Try to load custom layout only if requested and not connected to FMS
      if (useCustomAprilTagLayout && !DriverStation.isFMSAttached()) {
        try {
          cachedLayout = new AprilTagFieldLayout(customAprilTagLayoutPath);
        } catch (IOException e) {
          System.err.println("Error loading custom AprilTag layout: " + e.getMessage());
        }
      }
      // Otherwise load default layout
      if (cachedLayout == null) {
        cachedLayout = AprilTagFieldLayout.loadField(defaultAprilTagFieldLayout);
      }
    }
    return cachedLayout;
  }

  public enum VisionTest {
    unambiguous {
      /**
       * Penalizes ambiguous observations of a single tag, where ambiguity is defined as the ratio
       * of best:alternate pose reprojection errors. This is between 0 and 1 (0 being no ambiguity,
       * and 1 meaning both have the same reprojection error). Numbers above 0.2 are likely to be
       * ambiguous.
       *
       * @param observation The pose observation to check
       * @return Score between 0 and 1
       */
      @Override
      public double test(PoseObservation observation) {
        if (observation.tagCount() == 1) {
          return normalizedSigmoid(1 - observation.ambiguity(), 1 - ambiguityTolerance, 1.0);
        } else {
          return 1.0;
        }
      }
    },
    pitchError {
      /**
       * We assume that the robot is constrained to an orientation that is flat on the field.
       * Penalizes poses with significantly nonzero pitch.
       *
       * @param observation The pose observation to check
       * @return Score between 0 and 1
       */
      @Override
      public double test(PoseObservation observation) {
        return normalizedSigmoid(
            Math.abs(observation.pose().getRotation().getY()), pitchTolerance.in(Radians), 1.0);
      }
    },
    rollError {
      /**
       * We assume that the robot is constrained to an orientation that is flat on the field.
       * Penalizes poses with significantly nonzero roll.
       *
       * @param observation The pose observation to check
       * @return Score between 0 and 1
       */
      @Override
      public double test(PoseObservation observation) {
        return normalizedSigmoid(
            Math.abs(observation.pose().getRotation().getX()), rollTolerance.in(Radians), 1.0);
      }
    },
    heightError {
      /**
       * We assume that the robot is constrained to an orientation that is flat on the field.
       * Penalizes poses with significantly nonzero elevation.
       *
       * @param observation The pose observation to check
       * @return Score between 0 and 1
       */
      @Override
      public double test(PoseObservation observation) {
        return normalizedSigmoid(
            Math.abs(observation.pose().getZ()), elevationTolerance.in(Meters), 1.0);
      }
    },
    withinBoundaries {
      /**
       * Penalizes poses that, when projected to the floor, lie outside of the field boundaries
       *
       * @param observation The pose observation to check
       * @return Score between 0 and 1
       */
      @Override
      public double test(PoseObservation observation) {
        var corner1 = new Translation2d(minRobotWidthMeters / 2.0, minRobotWidthMeters / 2.0);
        var corner2 =
            new Translation2d(
                getAprilTagLayout().getFieldLength() - minRobotWidthMeters / 2.0,
                getAprilTagLayout().getFieldWidth() - minRobotWidthMeters / 2.0);
        var arena = new Rectangle2d(corner1, corner2);
        boolean pass = arena.contains(observation.pose().toPose2d().getTranslation());

        return (pass ? 1.0 : 0.0);
      }
    },
    moreThanZeroTags {
      /**
       * Penalizes observations that see zero tags
       *
       * @param observation The pose observation to check
       * @return Score between 0 and 1
       */
      @Override
      public double test(PoseObservation observation) {
        return Math.min(observation.tagCount(), 1.0);
      }
    },
    distanceToTags {
      /**
       * Rewards observations that see tags closer to the robot
       *
       * @param observation The pose observation to check
       * @return Score between 0 and 1
       */
      @Override
      public double test(PoseObservation observation) {
        double avgDistance = observation.averageTagDistance();
        // Assuming max relevant distance is 5 meters
        return normalizedSigmoid(5.0 - avgDistance, 2.5, 1.0);
      }
    };

    public abstract double test(PoseObservation observation);
  }

  /**
   * Calculates a normalized sigmoid function with a tunable midpoint and steepness. The output is
   * always between 0 and 1.
   *
   * @param x The input value.
   * @param midpoint The x-value where the output should be 0.5.
   * @param steepness The factor controlling the curve's steepness. Higher values result in a
   *     steeper curve, lower values result in a more gradual curve. Must be greater than 0.
   * @return The sigmoid output for the given input, between 0 and 1.
   */
  public static double normalizedSigmoid(double x, double midpoint, double steepness) {
    if (steepness <= 0) {
      throw new IllegalArgumentException("Steepness must be a positive value.");
    }

    double exponent = -steepness * (x - midpoint);
    return 1.0 / (1.0 + Math.exp(exponent));
  }
}
