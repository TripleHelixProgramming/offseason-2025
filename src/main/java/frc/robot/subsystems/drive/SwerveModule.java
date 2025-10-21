package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.zeroRotationKey;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Preferences;
import frc.robot.Constants;
import frc.robot.subsystems.drive.io.ModuleIO;
import frc.robot.subsystems.drive.io.ModuleIOInputsAutoLogged;
import frc.robot.subsystems.drive.io.ModuleSimIO;
import frc.robot.subsystems.drive.io.ModuleTalonFXIO;
import org.littletonrobotics.junction.Logger;

/**
 * Represents a single swerve module on the robot. This enum defines the configuration for each
 * physical swerve module, including its CAN IDs, physical location, and inversion settings. It also
 * manages the module's I/O and provides methods for controlling the module.
 */
public enum SwerveModule {
  FrontLeft(
      build()
          .driveCanId(28)
          .turnCanId(29)
          .turnAbsoluteEncoderCanId(43)
          .locationX(DriveConstants.wheelBase.div(2.0))
          .locationY(DriveConstants.trackWidth.div(2.0))
          .driveInverted(DriveConstants.kInvertLeftSide)
          .turnInverted(DriveConstants.turnInverted)
          .turnEncoderInverted(DriveConstants.turnEncoderInverted)
          .encoderOffset(Rotations.of(0))),

  FrontRight(
      build()
          .driveCanId(20)
          .turnCanId(21)
          .turnAbsoluteEncoderCanId(33)
          .locationX(DriveConstants.wheelBase.div(2.0))
          .locationY(DriveConstants.trackWidth.div(-2.0))
          .driveInverted(DriveConstants.kInvertRightSide)
          .turnInverted(DriveConstants.turnInverted)
          .turnEncoderInverted(DriveConstants.turnEncoderInverted)
          .encoderOffset(Rotations.of(0))),

  BackLeft(
      build()
          .driveCanId(10)
          .turnCanId(11)
          .turnAbsoluteEncoderCanId(45)
          .locationX(DriveConstants.wheelBase.div(-2.0))
          .locationY(DriveConstants.trackWidth.div(2.0))
          .driveInverted(DriveConstants.kInvertLeftSide)
          .turnInverted(DriveConstants.turnInverted)
          .turnEncoderInverted(DriveConstants.turnEncoderInverted)
          .encoderOffset(Rotations.of(0))),

  BackRight(
      build()
          .driveCanId(18)
          .turnCanId(19)
          .turnAbsoluteEncoderCanId(31)
          .locationX(DriveConstants.wheelBase.div(-2.0))
          .locationY(DriveConstants.trackWidth.div(-2.0))
          .driveInverted(DriveConstants.kInvertRightSide)
          .turnInverted(DriveConstants.turnInverted)
          .turnEncoderInverted(DriveConstants.turnEncoderInverted)
          .encoderOffset(Rotations.of(0)));

  /** The CAN ID for the drive motor of this module. */
  public final int driveCanId;
  /** The CAN ID for the turn motor of this module. */
  public final int turnCanId;
  /** The CAN ID for the absolute turn encoder of this module. */
  public final int turnAbsoluteEncoderCanId;
  /** The offset of the absolute encoder from the module's zero position. */
  public final Angle encoderOffset;
  /** The X-coordinate of the module's location relative to the robot's center. */
  public final Distance locationX;
  /** The Y-coordinate of the module's location relative to the robot's center. */
  public final Distance locationY;
  /** Whether the drive motor is inverted. */
  public final boolean driveInverted;
  /** Whether the turn motor is inverted. */
  public final boolean turnInverted;
  /** Whether the absolute turn encoder is inverted. */
  public final boolean turnEncoderInverted;

  /** The I/O interface for this module, handling hardware or simulation interactions. */
  private final ModuleIO io;
  /** The inputs for this module, updated periodically from the I/O interface. */
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

  /** Alert for when the drive motor is disconnected. */
  private final Alert driveDisconnectedAlert;
  /** Alert for when the turn motor is disconnected. */
  private final Alert turnDisconnectedAlert;

  /**
   * Array of swerve module positions for odometry, updated each periodic cycle. This array holds
   * historical position data for accurate odometry calculations.
   */
  private SwerveModulePosition[] odometryPositions = {};

  /**
   * Constructs a new SwerveModule instance using the properties from the provided {@link Builder}.
   *
   * @param builder The builder containing the configuration for this module.
   */
  private SwerveModule(Builder builder) {
    this.driveCanId = builder.driveCanId;
    this.turnCanId = builder.turnCanId;
    this.turnAbsoluteEncoderCanId = builder.turnAbsoluteEncoderCanId;
    this.encoderOffset = builder.encoderOffset;
    this.locationX = builder.locationX;
    this.locationY = builder.locationY;
    this.driveInverted = builder.driveInverted;
    this.turnInverted = builder.turnInverted;
    this.turnEncoderInverted = builder.turnEncoderInverted;

    var constantCreator =
        new SwerveModuleConstantsFactory<
                TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(DriveConstants.driveMotorReduction)
            .withSteerMotorGearRatio(DriveConstants.turnMotorReduction)
            .withCouplingGearRatio(DriveConstants.kCoupleRatio)
            .withWheelRadius(DriveConstants.wheelRadius)
            .withSteerMotorGains(DriveConstants.steerGains)
            .withDriveMotorGains(DriveConstants.driveGains)
            .withSteerMotorClosedLoopOutput(DriveConstants.kSteerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(DriveConstants.kDriveClosedLoopOutput)
            .withSlipCurrent(DriveConstants.kSlipCurrent)
            .withSpeedAt12Volts(DriveConstants.maxDriveSpeed)
            .withDriveMotorType(DriveConstants.kDriveMotorType)
            .withSteerMotorType(DriveConstants.kSteerMotorType)
            .withFeedbackSource(DriveConstants.kSteerFeedbackType)
            .withDriveMotorInitialConfigs(DriveConstants.driveInitialConfigs)
            .withSteerMotorInitialConfigs(DriveConstants.steerInitialConfigs)
            .withSteerInertia(DriveConstants.kSteerInertia)
            .withDriveInertia(DriveConstants.kDriveInertia)
            .withSteerFrictionVoltage(DriveConstants.kSteerFrictionVoltage)
            .withDriveFrictionVoltage(DriveConstants.kDriveFrictionVoltage);

    // Create the Phoenix SwerveModuleConstants using the builder's parameters
    var constants =
        constantCreator.createModuleConstants(
            turnCanId,
            driveCanId,
            turnAbsoluteEncoderCanId,
            encoderOffset,
            locationX,
            locationY,
            driveInverted,
            turnInverted,
            turnEncoderInverted);

    // Initialize the appropriate ModuleIO based on the current robot mode
    this.io =
        switch (Constants.currentMode) {
          case REAL -> new ModuleTalonFXIO(constants);
          case SIM -> new ModuleSimIO(constants);
          default -> new ModuleIO() {};
        };

    driveDisconnectedAlert =
        new Alert("Disconnected drive motor on module " + name() + ".", AlertType.kError);
    turnDisconnectedAlert =
        new Alert("Disconnected turn motor on module " + name() + ".", AlertType.kError);

    // Initialize Preferences for storing and retrieving the turn zero position.
    // This allows the robot to remember the calibrated zero position across reboots.
    // The key is unique for each module using its ordinal value.
    // Set turn zero from preferences
    Rotation2d turnZeroFromCancoder = inputs.turnZero;
    Preferences.initDouble(zeroRotationKey + ordinal(), turnZeroFromCancoder.getRadians());
    Rotation2d turnZeroFromPreferences =
        new Rotation2d(
            Preferences.getDouble(zeroRotationKey + ordinal(), turnZeroFromCancoder.getRadians()));
    io.setTurnZero(turnZeroFromPreferences);
  }

  /** Periodically updates the module's inputs, processes odometry data, and updates alerts. */
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module." + name(), inputs);

    // Calculate positions for odometry
    int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters =
          inputs.odometryDrivePositionsRad[i] * DriveConstants.wheelRadius.in(Meters);
      Rotation2d angle = inputs.odometryTurnPositions[i];
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }

    // Update alerts
    driveDisconnectedAlert.set(!inputs.driveConnected);
    turnDisconnectedAlert.set(!inputs.turnConnected);
  }

  /**
   * Runs the module with the specified setpoint state. The state is optimized internally to prevent
   * unnecessary module rotation.
   *
   * @param state The desired {@link SwerveModuleState} for the module.
   */
  /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
  public void runSetpoint(SwerveModuleState state) {
    // Optimize velocity setpoint
    state.optimize(getAngle());
    state.cosineScale(inputs.turnPosition);

    // Apply setpoints
    io.setDriveVelocity(state.speedMetersPerSecond / DriveConstants.wheelRadius.in(Meters));
    io.setTurnPosition(state.angle);
  }

  /**
   * Runs the drive motor with a specified open-loop output while keeping the turn motor at zero
   * degrees. This is typically used for characterization.
   *
   * @param output The open-loop output (e.g., voltage or percent output) for the drive motor.
   */
  /** Runs the module with the specified output while controlling to zero degrees. */
  public void runCharacterization(double output) {
    io.setDriveOpenLoop(output);
    io.setTurnPosition(Rotation2d.kZero);
  }

  /** Disables all outputs to motors. */
  /** Stops both the drive and turn motors by setting their open-loop outputs to zero. */
  public void stop() {
    io.setDriveOpenLoop(0.0);
    io.setTurnOpenLoop(0.0);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

  /** Returns the current drive position of the module in meters. */
  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionRad * DriveConstants.wheelRadius.in(Meters);
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * DriveConstants.wheelRadius.in(Meters);
  }

  /** Returns the current {@link SwerveModulePosition} of the module. */
  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /**
   * Returns an array of {@link SwerveModulePosition} objects representing the module's positions
   * recorded during the current periodic cycle for odometry.
   *
   * @return An array of {@link SwerveModulePosition} for odometry.
   */
  /** Returns the module positions received this cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /**
   * Returns an array of timestamps (in seconds) corresponding to the odometry samples received
   * during the current periodic cycle.
   *
   * @return An array of timestamps for odometry samples.
   */
  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  /**
   * Returns the drive motor's position in radians, used for wheel radius characterization.
   *
   * @return The drive motor's position in radians.
   */
  /** Returns the module position in radians. */
  public double getWheelRadiusCharacterizationPosition() {
    return inputs.drivePositionRad;
  }

  /**
   * Returns the drive motor's velocity in radians per second, used for feedforward
   * characterization.
   *
   * @return The drive motor's velocity in radians per second.
   */
  /** Returns the module velocity in rad/sec. */
  public double getFFCharacterizationVelocity() {
    return inputs.driveVelocityRadPerSec;
  }

  /**
   * Sets the zero position of the turn absolute encoder to the current rotation of the module. This
   * value is stored in {@link Preferences} for persistence.
   */
  /** Sets the zero position of the turn axis to the current rotation */
  public void setTurnZero() {
    Rotation2d newTurnZero = inputs.turnZero.minus(inputs.turnPosition);
    io.setTurnZero(newTurnZero);
    Preferences.setDouble(zeroRotationKey + ordinal(), newTurnZero.getRadians());
  }

  /**
   * Creates a new {@link Builder} instance for constructing a {@link SwerveModule}.
   *
   * @return A new {@link Builder}.
   */
  private static Builder build() {
    return new Builder();
  }

  /**
   * A fluent builder class for constructing {@link SwerveModule} instances. This allows for more
   * readable and organized module definitions.
   */
  private static class Builder {
    private int driveCanId;
    private int turnCanId;
    private int turnAbsoluteEncoderCanId;
    private Angle encoderOffset = Rotations.of(0);
    private Distance locationX = Meters.of(0);
    private Distance locationY = Meters.of(0);
    private boolean driveInverted = false;
    private boolean turnInverted = false;
    private boolean turnEncoderInverted = false;

    /** Sets the CAN ID for the drive motor. */
    public Builder driveCanId(int driveCanId) {
      this.driveCanId = driveCanId;
      return this;
    }

    /** Sets the CAN ID for the turn motor. */
    public Builder turnCanId(int turnCanId) {
      this.turnCanId = turnCanId;
      return this;
    }

    /** Sets the CAN ID for the absolute turn encoder. */
    public Builder turnAbsoluteEncoderCanId(int turnAbsoluteEncoderCanId) {
      this.turnAbsoluteEncoderCanId = turnAbsoluteEncoderCanId;
      return this;
    }

    /** Sets the encoder offset. */
    public Builder encoderOffset(Angle encoderOffset) {
      this.encoderOffset = encoderOffset;
      return this;
    }

    /** Sets the X location of the module. */
    public Builder locationX(Distance locationX) {
      this.locationX = locationX;
      return this;
    }

    /** Sets the Y location of the module. */
    public Builder locationY(Distance locationY) {
      this.locationY = locationY;
      return this;
    }

    /** Sets whether the drive motor is inverted. */
    public Builder driveInverted(boolean driveInverted) {
      this.driveInverted = driveInverted;
      return this;
    }

    /** Sets whether the turn motor is inverted. */
    public Builder turnInverted(boolean turnInverted) {
      this.turnInverted = turnInverted;
      return this;
    }

    /** Sets whether the turn encoder is inverted. */
    public Builder turnEncoderInverted(boolean turnEncoderInverted) {
      this.turnEncoderInverted = turnEncoderInverted;
      return this;
    }
  }
}
