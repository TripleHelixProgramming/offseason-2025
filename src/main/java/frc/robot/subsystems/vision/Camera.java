// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.vision.io.PhotonVisionIO;
import frc.robot.subsystems.vision.io.PhotonVisionSimIO;
import frc.robot.subsystems.vision.io.VisionIO;
import frc.robot.subsystems.vision.io.VisionIOInputsAutoLogged;

/**
 * Represents a single camera on the robot. This enum is responsible for defining the physical
 * properties of each camera and creating the appropriate {@link VisionIO} implementation based on
 * the current robot mode (REAL, SIM, or REPLAY).
 *
 * <p>Each enum constant defines a camera's position, orientation, and other properties using a
 * fluent builder pattern. It also manages the lifecycle of the associated {@link VisionIO} and
 * {@link VisionIOInputs} instances.
 */
public enum Camera {
  FrontRight(
      build()
          .deviceName("OV2311_TH_4")
          .robotToCameraX(0.286)
          .robotToCameraY(-0.332)
          .robotToCameraZ(0.361)
          .robotToCameraYaw(0.768)
          .stdDevFactor(1.0)),

  FrontLeft(
      build()
          .deviceName("OV2311_TH_3")
          .robotToCameraX(0.292)
          .robotToCameraY(0.316)
          .robotToCameraZ(0.361)
          .robotToCameraYaw(5.498)
          .stdDevFactor(1.0)),

  BackRight(
      build()
          .deviceName("OV2311_TH_2")
          .robotToCameraX(-0.259)
          .robotToCameraY(-0.346)
          .robotToCameraZ(0.628)
          .robotToCameraYaw(4.311)
          .stdDevFactor(1.0)),

  BackLeft(
      build()
          .deviceName("OV2311_TH_1")
          .robotToCameraX(-0.252)
          .robotToCameraY(0.341)
          .robotToCameraZ(0.628)
          .robotToCameraYaw(1.972)
          .stdDevFactor(1.0));

  /** The name of the camera device on coprocessor. */
  public final String deviceName;

  /** The transform from the robot's datum to the camera's optical center. */
  public final Transform3d robotToCamera;

  /**
   * Standard deviation multiplier for this camera's measurements. (Adjust to trust some cameras
   * more than others).
   */
  public final double stdDevFactor;

  /**
   * The {@link VisionIO} implementation for this camera, chosen based on the current robot mode.
   */
  private final VisionIO io;

  /** The inputs for this camera, used for logging and data processing. */
  public final VisionIOInputsAutoLogged inputs;

  private static Builder build() {
    return new Builder();
  }

  /**
   * A fluent builder for defining the properties of a {@link Camera} instance. This simplifies the
   * creation and configuration of each enum constant.
   */
  private static class Builder {
    private String deviceName;
    private double x = 0.0;
    private double y = 0.0;
    private double z = 0.0;
    private double roll = 0.0;
    private double pitch = 0.0;
    private double yaw = 0.0;
    private double stdDevFactor = 1.0;

    /** Sets the device name of the camera. */
    public Builder deviceName(String deviceName) {
      this.deviceName = deviceName;
      return this;
    }

    /** Sets the X translation of the camera relative to the robot's center. */
    public Builder robotToCameraX(double x) {
      this.x = x;
      return this;
    }

    /** Sets the Y translation of the camera relative to the robot's center. */
    public Builder robotToCameraY(double y) {
      this.y = y;
      return this;
    }

    /** Sets the Z translation of the camera relative to the robot's center. */
    public Builder robotToCameraZ(double z) {
      this.z = z;
      return this;
    }

    /** Sets the roll rotation of the camera. */
    public Builder robotToCameraRoll(double roll) {
      this.roll = roll;
      return this;
    }

    /** Sets the pitch rotation of the camera. */
    public Builder robotToCameraPitch(double pitch) {
      this.pitch = pitch;
      return this;
    }

    /** Sets the yaw rotation of the camera. */
    public Builder robotToCameraYaw(double yaw) {
      this.yaw = yaw;
      return this;
    }

    /** Sets the standard deviation factor for this camera's measurements. */
    public Builder stdDevFactor(double stdDevFactor) {
      this.stdDevFactor = stdDevFactor;
      return this;
    }
  }

  /**
   * Constructs a new Camera instance using the properties from the provided {@link Builder}.
   *
   * <p>This constructor is private and is only called during the initialization of the enum
   * constants. It sets up the camera's physical properties and instantiates the correct {@link
   * VisionIO} implementation based on the robot's current operational mode.
   *
   * @param builder The builder containing the configuration for this camera.
   */
  private Camera(Builder builder) {
    this.deviceName = builder.deviceName;
    this.robotToCamera =
        new Transform3d(
            builder.x,
            builder.y,
            builder.z,
            new Rotation3d(builder.roll, builder.pitch, builder.yaw));
    this.stdDevFactor = builder.stdDevFactor;
    this.inputs = new VisionIOInputsAutoLogged();

    this.io =
        switch (Constants.currentMode) {
          case REAL -> new PhotonVisionIO(deviceName, robotToCamera);
          case SIM -> new PhotonVisionSimIO(deviceName, robotToCamera, () -> Robot.drive.getPose());
          default -> new VisionIO() {};
        };
  }

  /** Updates the internal {@link #inputs} object with the latest data from this camera. */
  public void updateInputs() {
    io.updateInputs(this.inputs);
  }
}
