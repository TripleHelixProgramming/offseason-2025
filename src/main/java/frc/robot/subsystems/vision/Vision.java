// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.io.VisionIO.PoseObservation;
import java.io.IOException;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {

  public static final Distance minRobotWidth = Inches.of(36.875);
  public static final double ambiguityTolerance = 0.15;
  public static final Distance tagDistanceTolerance = Meters.of(4.0);
  public static final Distance elevationTolerance = Meters.of(0.25);
  public static final Angle rollTolerance = Degrees.of(5);
  public static final Angle pitchTolerance = Degrees.of(5);
  public static final String customAprilTagLayoutPath = Filesystem.getDeployDirectory() + "/stemgym-2026.json";
  public static final Boolean useCustomAprilTagLayout = true;
  public static final AprilTagFields defaultAprilTagFieldLayout = AprilTagFields.k2025ReefscapeAndyMark;
  public static final double linearStdDevBaseline = 0.02; // Meters
  public static final double angularStdDevBaseline = 0.06; // Radians
  public static final double maxStdDev = 1.0; // Meters
  public static double minScore = linearStdDevBaseline / maxStdDev;

  private static Vision instance;
  private final List<VisionConsumer> consumers = new ArrayList<>();
  private final Alert[] disconnectedAlerts;
  LinearFilter[] cameraPassRate;

  /** Initializes the Vision subsystem singleton. */
  public static void initialize() {
    if (instance == null) {
      getInstance();
    }
  }

  /** Returns the singleton instance of the Vision subsystem. */
  public static Vision getInstance() {
    if (instance == null) {
      instance = new Vision();
    }
    return instance;
  }

  private Vision() {
    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[Camera.values().length];
    this.cameraPassRate = new LinearFilter[Camera.values().length];
    for (Camera camera : Camera.values()) {
      disconnectedAlerts[camera.ordinal()] =
          new Alert("Vision camera " + camera.name() + " is disconnected.", AlertType.kWarning);
      cameraPassRate[camera.ordinal()] = LinearFilter.movingAverage(20);
    }
  }

  /** A functional interface for consuming vision pose estimations. */
  @FunctionalInterface
  public static interface VisionConsumer {
    /**
     * Accepts a vision measurement.
     *
     * @param visionRobotPoseMeters The estimated robot pose in meters.
     * @param timestampSeconds The timestamp of the measurement in seconds.
     * @param visionMeasurementStdDevs The standard deviations of the measurement.
     */
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  /**
   * Adds a new consumer to the list of vision consumers.
   *
   * @param consumer The consumer to add.
   */
  public void addConsumer(VisionConsumer consumer) {
    consumers.add(consumer);
  }

  /**
   * Removes a consumer from the list of vision consumers. @param consumer The consumer to remove.
   */
  public void removeConsumer(VisionConsumer consumer) {
    consumers.remove(consumer);
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return Camera.values()[cameraIndex].inputs.latestTargetObservation.tx();
  }

  @Override
  public void periodic() {
    for (Camera camera : Camera.values()) {
      camera.updateInputs();
      Logger.processInputs("Vision/Camera." + camera.name(), camera.inputs);
    }

    // Initialize logging values
    var allTagPoses = new ArrayList<Pose3d>();
    var allRobotPoses = new ArrayList<Pose3d>();
    var allRobotPosesAccepted = new ArrayList<Pose3d>();
    var allRobotPosesRejected = new ArrayList<Pose3d>();

    // List to store acceptable observations
    var observations = new ArrayList<TestedObservation>();

    // Loop over cameras
    for (Camera camera : Camera.values()) {
      // Update disconnected alert
      disconnectedAlerts[camera.ordinal()].set(!camera.inputs.connected);

      // Initialize logging values
      var tagPoses = new ArrayList<Pose3d>();
      var robotPoses = new ArrayList<Pose3d>();
      var robotPosesAccepted = new ArrayList<Pose3d>();
      var robotPosesRejected = new ArrayList<Pose3d>();

      // Add tag poses
      for (int tagId : camera.inputs.tagIds) {
        var tagPose = getAprilTagLayout().getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : camera.inputs.poseObservations) {
        EnumMap<VisionTest, Double> testResults = new EnumMap<>(VisionTest.class);

        testResults.put(VisionTest.moreThanZeroTags, VisionTest.moreThanZeroTags.test(observation));
        testResults.put(VisionTest.unambiguous, VisionTest.unambiguous.test(observation));
        testResults.put(VisionTest.pitchError, VisionTest.pitchError.test(observation));
        testResults.put(VisionTest.rollError, VisionTest.rollError.test(observation));
        testResults.put(VisionTest.heightError, VisionTest.heightError.test(observation));
        testResults.put(VisionTest.withinBoundaries, VisionTest.withinBoundaries.test(observation));
        testResults.put(VisionTest.distanceToTags, VisionTest.distanceToTags.test(observation));

        Double totalScore =
            testResults.values().stream().reduce(1.0, (subtotal, element) -> subtotal * element);

        observations.add(new TestedObservation(observation, camera, testResults, totalScore));

        // Add pose to log
        robotPoses.add(observation.pose());
        if (totalScore > minScore) {
          robotPosesAccepted.add(observation.pose());
        } else {
          robotPosesRejected.add(observation.pose());
        }

        cameraPassRate[camera.ordinal()].calculate(totalScore);
      }

      // Log camera datadata
      Logger.recordOutput(
          "Vision/Camera." + camera.name() + "/TagPoses",
          tagPoses.toArray(new Pose3d[tagPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera." + camera.name() + "/RobotPoses",
          robotPoses.toArray(new Pose3d[robotPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera." + camera.name() + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
      Logger.recordOutput(
          "Vision/Camera." + camera.name() + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
      Logger.recordOutput(
          "Vision/Camera." + camera.name() + "/PassRate",
          cameraPassRate[camera.ordinal()].lastValue());
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
      consumers.forEach(c -> c.accept(
        o.observation.pose().toPose2d(),
        o.observation.timestamp(),
        VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)));

      Logger.recordOutput("Vision/Summary/ObservationScore", o.score);
    }

    // Log summary data
    Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(Pose3d[]::new));
    Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(Pose3d[]::new));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(Pose3d[]::new));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
  }

  // Associate observations with their camera
  public static record TestedObservation(
      PoseObservation observation,
      Camera camera,
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
          return 1.0 - normalizedSigmoid(observation.ambiguity(), ambiguityTolerance, 4.0);
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
        return 1.0
            - normalizedSigmoid(
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
        return 1.0
            - normalizedSigmoid(
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
        return 1.0
            - normalizedSigmoid(
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
        var cornerA = new Translation2d(minRobotWidth.div(2.0), minRobotWidth.div(2.0));
        var cornerB =
            new Translation2d(
                    getAprilTagLayout().getFieldLength(), getAprilTagLayout().getFieldWidth())
                .minus(cornerA);
        var arena = new Rectangle2d(cornerA, cornerB);
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
        return 1.0
            - normalizedSigmoid(
                observation.averageTagDistance(), tagDistanceTolerance.in(Meters), 1.0);
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
