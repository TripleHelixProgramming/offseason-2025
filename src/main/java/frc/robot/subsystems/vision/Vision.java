// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD-style
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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.vision.io.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.io.VisionIO.PoseObservationType;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * The Vision subsystem is responsible for processing data from multiple cameras to generate robot
 * pose estimations. It filters observations from AprilTags, calculates standard deviations, and
 * provides valid measurements to registered consumers, such as the {@link Drive} subsystem's pose
 * estimator. This class is implemented as a singleton to ensure a single source of vision data
 * throughout the robot code.
 */
public class Vision extends SubsystemBase {
  // AprilTag layout
  private static final String customAprilTagLayoutPath =
      Filesystem.getDeployDirectory() + "/stemgym.json";
  private static final boolean useCustomAprilTagLayout = true;
  private static final AprilTagFields defaultAprilTagFieldLayout =
      AprilTagFields.k2025ReefscapeAndyMark;

  /**
   * The ratio of best:alternate pose reprojection errors, called ambiguity. This is between 0 and 1
   * (0 being no ambiguity, and 1 meaning both have the same reprojection error). Numbers above 0.2
   * are likely to be ambiguous.
   */
  private static final double maxAmbiguity = 0.3;

  // Pose filtering thresholds
  private static final Distance maxZError = Meters.of(0.75);
  private static final Angle maxRollError = Degrees.of(30);
  private static final Angle maxPitchError = Degrees.of(30);
  private static final Distance maxTravelDistance =
      DriveConstants.maxDriveSpeed.times(Seconds.of(Robot.defaultPeriodSecs));

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  private static final double linearStdDevBaseline = 0.02; // Meters
  private static final double angularStdDevBaseline = 0.06; // Radians

  // Multipliers to apply for MegaTag 2 observations
  private static final double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  private static final double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available

  /** Consumers that accept vision measurements. */
  private final List<VisionConsumer> consumers = new ArrayList<>();

  /** Alerts for disconnected cameras. */
  private final Alert[] disconnectedAlerts;

  /** Filters for tracking the pass rate of each camera's observations. */
  private final LinearFilter[] cameraPassRate;

  /** Filter for tracking the pass rate of the ambiguity check. */
  private final LinearFilter ambiguityTestPassRate = LinearFilter.movingAverage(20);

  /** Filter for tracking the pass rate of the pose flatness check. */
  private final LinearFilter flatPoseTestPassRate = LinearFilter.movingAverage(20);

  /** Filter for tracking the pass rate of the field boundary check. */
  private final LinearFilter withinBoundsTestPassRate = LinearFilter.movingAverage(20);

  /** Filter for tracking the pass rate of the tag count check. */
  private final LinearFilter moreThanZeroTagsTestPassRate = LinearFilter.movingAverage(20);

  /** The singleton instance of the Vision subsystem. */
  private static Vision instance;

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

  /** Constructs a new Vision subsystem. This is private to enforce the singleton pattern. */
  private Vision() {
    // Initialize arrays based on number of cameras
    this.disconnectedAlerts = new Alert[Camera.values().length];
    this.cameraPassRate = new LinearFilter[Camera.values().length];
    for (Camera camera : Camera.values()) {
      disconnectedAlerts[camera.ordinal()] =
          new Alert(
              "Vision camera Camera." + camera.name() + " is disconnected.", AlertType.kWarning);
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
  public final Rotation2d getTargetX(int cameraIndex) {
    return Camera.values()[cameraIndex].inputs.latestTargetObservation.tx();
  }

  /**
   * This method is called once per scheduler run. It updates inputs from all cameras, processes the
   * observations, filters them, and sends valid measurements to all registered consumers. It also
   * handles logging of all vision data to AdvantageKit.
   */
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

    // List to store acceptable observations along with their calculated standard deviations
    var acceptableObservations = new ArrayList<ObservationWithStdDev>();

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
        // Check whether to reject pose
        boolean acceptPose =
            // Must have observed at least one tag
            moreThanZeroTags(observation)

                // Any single-tag observation must have low ambiguity
                && hasLowAmbiguity(observation)

                // Pose must be flat on the floor
                && isPoseFlat(observation)

                // Pose must be within the field boundaries
                && isWithinBoundaries(observation)

            // Pose must be within the max possible travel distance
            // TODO: Disable this filter during initial robot setup
            // || observation
            //         .pose()
            //         .toPose2d()
            //         .getTranslation()
            //         .getDistance(Drive.getInstance().getPose().getTranslation())
            //     > maxTravelDistance.in(Meters);
            ;

        // Add pose to log
        robotPoses.add(observation.pose());
        if (acceptPose) {
          robotPosesAccepted.add(observation.pose());
        } else {
          robotPosesRejected.add(observation.pose());
        }

        cameraPassRate[camera.ordinal()].calculate(acceptPose ? 1.0 : 0.0);

        // Skip if rejected
        if (!acceptPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            (observation.averageTagDistance() * observation.averageTagDistance())
                / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        }
        linearStdDev *= camera.stdDevFactor;
        angularStdDev *= camera.stdDevFactor;

        // Pair the observation with its standard deviations and add it to the list
        acceptableObservations.add(
            new ObservationWithStdDev(
                observation, VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)));
      }

      // Log camera datadata
      Logger.recordOutput(
          "Vision/Camera." + camera.name() + "/TagPoses", tagPoses.toArray(Pose3d[]::new));
      Logger.recordOutput(
          "Vision/Camera." + camera.name() + "/RobotPoses", robotPoses.toArray(Pose3d[]::new));
      Logger.recordOutput(
          "Vision/Camera." + camera.name() + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(Pose3d[]::new));
      Logger.recordOutput(
          "Vision/Camera." + camera.name() + "/RobotPosesRejected",
          robotPosesRejected.toArray(Pose3d[]::new));
      Logger.recordOutput(
          "Vision/Camera." + camera.name() + "/PassRate",
          cameraPassRate[camera.ordinal()].lastValue());
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Sort the list of acceptable observations by timestamp
    acceptableObservations.sort(Comparator.comparingDouble(o -> o.observation.timestamp()));

    // Send sorted vision observations to the pose estimator
    for (ObservationWithStdDev obsWithStdDev : acceptableObservations) {
      for (VisionConsumer consumer : consumers) {
        consumer.accept(
            obsWithStdDev.observation.pose().toPose2d(),
            obsWithStdDev.observation.timestamp(),
            obsWithStdDev.stdDevs);
      }
    }

    // Log summary data
    Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(Pose3d[]::new));
    Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(Pose3d[]::new));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(Pose3d[]::new));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(Pose3d[]::new));

    Logger.recordOutput("Vision/Summary/AmbiguityTestPassRate", ambiguityTestPassRate.lastValue());
    Logger.recordOutput("Vision/Summary/FlatPoseTestPassRate", flatPoseTestPassRate.lastValue());
    Logger.recordOutput(
        "Vision/Summary/WithinBoundsTestPassRate", withinBoundsTestPassRate.lastValue());
    Logger.recordOutput(
        "Vision/Summary/MoreThanZeroTagsTestPassRate", moreThanZeroTagsTestPassRate.lastValue());
  }

  /**
   * A record to associate a {@link PoseObservation} with its calculated standard deviations.
   *
   * @param observation The pose observation from the camera.
   * @param stdDevs The calculated standard deviations for the observation.
   */
  public static record ObservationWithStdDev(PoseObservation observation, Matrix<N3, N1> stdDevs) {}

  /**
   * A cached instance of the AprilTagFieldLayout. This is used to avoid reloading the layout from
   * disk on every call.
   */
  public static AprilTagFieldLayout cachedLayout = null;

  /**
   * Returns the AprilTag layout to use, loading it from the file system if necessary. This method
   * caches the layout to avoid repeated file I/O.
   */
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

  /**
   * Checks if a single-tag observation has a low enough ambiguity.
   *
   * @param observation The pose observation to check.
   * @return True if the observation has low ambiguity, false otherwise.
   */
  public final Boolean hasLowAmbiguity(PoseObservation observation) {
    boolean pass = true;
    if (observation.tagCount() == 1) {
      pass = observation.ambiguity() < maxAmbiguity;
    }

    ambiguityTestPassRate.calculate(pass ? 1.0 : 0.0);
    return pass;
  }

  /**
   * Checks if the observed pose is "flat" on the field floor.
   *
   * @param observation The pose observation to check.
   * @return True if the pose is flat, false otherwise.
   */
  public final Boolean isPoseFlat(PoseObservation observation) {
    boolean pass =
        Math.abs(observation.pose().getZ()) < maxZError.in(Meters)
            && Math.abs(observation.pose().getRotation().getX()) < maxRollError.in(Radians)
            && Math.abs(observation.pose().getRotation().getY()) < maxPitchError.in(Radians);

    flatPoseTestPassRate.calculate(pass ? 1.0 : 0.0);
    return pass;
  }

  /**
   * Checks if the observed pose is within the physical boundaries of the field.
   *
   * @param observation The pose observation to check.
   * @return True if the pose is within the field, false otherwise.
   */
  public final Boolean isWithinBoundaries(PoseObservation observation) {
    boolean pass =
        observation.pose().getX() > 0.0
            && observation.pose().getX() < getAprilTagLayout().getFieldLength()
            && observation.pose().getY() > 0.0
            && observation.pose().getY() < getAprilTagLayout().getFieldWidth();

    withinBoundsTestPassRate.calculate(pass ? 1.0 : 0.0);
    return pass;
  }

  /**
   * Checks if the observation includes at least one AprilTag.
   *
   * @param observation The pose observation to check.
   * @return True if at least one tag was observed, false otherwise.
   */
  public final Boolean moreThanZeroTags(PoseObservation observation) {
    boolean pass = observation.tagCount() > 0;

    moreThanZeroTagsTestPassRate.calculate(pass ? 1.0 : 0.0);
    return pass;
  }
}
