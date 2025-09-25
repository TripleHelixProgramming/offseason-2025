package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.AllianceSelector;
import frc.lib.AutoOption;
import frc.lib.AutoSelector;
import frc.lib.CommandZorroController;
import frc.lib.ControllerSelector;
import frc.lib.ControllerSelector.ControllerConfig;
import frc.lib.ControllerSelector.ControllerFunction;
import frc.lib.ControllerSelector.ControllerType;
import frc.robot.Constants.AutoConstants;
import frc.robot.auto.B_Path;
import frc.robot.auto.R_MoveAndRotate;
import frc.robot.auto.R_MoveStraight;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.PathCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOBoron;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.vision.Vision;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private final AllianceSelector allianceSelector =
      new AllianceSelector(AutoConstants.kAllianceColorSelectorPort);
  private final AutoSelector autoSelector =
      new AutoSelector(
          AutoConstants.kAutonomousModeSelectorPorts, allianceSelector::getAllianceColor);

  // Subsystems
  private Drive drive;
  private Vision vision;

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL: // Running on a real robot
        // Log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());

        // Instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOBoron(),
                new ModuleIOTalonFX(DriveConstants.FrontLeft),
                new ModuleIOTalonFX(DriveConstants.FrontRight),
                new ModuleIOTalonFX(DriveConstants.BackLeft),
                new ModuleIOTalonFX(DriveConstants.BackRight));
        vision = new Vision(drive);
        break;

      case SIM: // Running a physics simulator
        // Log to NT
        Logger.addDataReceiver(new NT4Publisher());

        // Instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(DriveConstants.FrontLeft),
                new ModuleIOSim(DriveConstants.FrontRight),
                new ModuleIOSim(DriveConstants.BackLeft),
                new ModuleIOSim(DriveConstants.BackRight));
        break;

      case REPLAY: // Replaying a log
      default:
        // Set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));

        // Disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    // Initialize URCL
    // TODO: Delete if no REV electronics are used
    Logger.registerURCL(URCL.startExternal());

    // Start AdvantageKit logger
    Logger.start();

    configureControlPanelBindings();
    configureAutoOptions();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Optionally switch the thread to high priority to improve loop
    // timing (see the template project documentation for details)
    // Threads.setCurrentThreadPriority(true, 99);

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.putData(CommandScheduler.getInstance());
    SmartDashboard.putData(drive);

    // Return to non-RT thread priority (do not modify the first argument)
    // Threads.setCurrentThreadPriority(false, 10);

    // Logger.recordOutput("Vision/Estimate", vision.getPose().orElse(Pose2d.kZero));
    // Arrays.stream(Camera.values())
    //     .forEach(
    //         cam ->
    //             Logger.recordOutput("Vision/" + cam.getName(),
    // cam.getPose().orElse(Pose2d.kZero)));
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    allianceSelector.disabledPeriodic();
    autoSelector.disabledPeriodic();
    ControllerSelector.getInstance().scan();
  }

  /** This function is called once when autonomous mode is enabled. */
  @Override
  public void autonomousInit() {
    drive.setDefaultCommand(Commands.runOnce(drive::stop, drive).withName("Stop"));
    autoSelector.scheduleAuto();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop mode is enabled. */
  @Override
  public void teleopInit() {
    autoSelector.cancelAuto();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  private void configureControlPanelBindings() {
    ControllerSelector.configure(
        // ZORRO is always preferred as driver in REAL and SIM mode
        new ControllerConfig(
            ControllerFunction.DRIVER,
            ControllerType.ZORRO,
            this::bindZorroDriver,
            Constants.Mode.REAL,
            Constants.Mode.SIM),
        // XBOX is always preferred as operator in REAL and SIM mode
        new ControllerConfig(
            ControllerFunction.OPERATOR,
            ControllerType.XBOX,
            this::bindXboxOperator,
            Constants.Mode.REAL,
            Constants.Mode.SIM),
        // XBOX is permitted as driver in REAL and SIM mode
        new ControllerConfig(
            ControllerFunction.DRIVER,
            ControllerType.XBOX,
            this::bindXboxDriver,
            Constants.Mode.REAL,
            Constants.Mode.SIM));
  }

  public void bindZorroDriver(int port) {
    var zorroDriver = new CommandZorroController(port);

    // Drive in field-relative mode while switch E is up
    // Drive in robot-relative mode while switch E is down
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -zorroDriver.getRightYAxis(),
            () -> -zorroDriver.getRightXAxis(),
            () -> -zorroDriver.getLeftXAxis(),
            () -> zorroDriver.getHID().getEUp()));

    // Reset gyro to 0° when button G is pressed
    zorroDriver
        .GIn()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // Drive 1m forward while button A is held
    zorroDriver.AIn().whileTrue(PathCommands.advanceForward(drive, Meters.of(1)));

    // Switch to X pattern when button D is pressed
    zorroDriver.DIn().onTrue(Commands.runOnce(drive::stopWithX, drive));
  }

  public void bindXboxDriver(int port) {
    var xboxDriver = new CommandXboxController(port);

    // Drive in field-relative mode while left bumper is released
    // Drive in robot-relative mode while left bumper is pressed
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -xboxDriver.getLeftY(),
            () -> -xboxDriver.getLeftX(),
            () -> -xboxDriver.getRightX(),
            () -> !xboxDriver.getHID().getLeftBumperButton()));

    // Reset gyro to 0° when B button is pressed
    xboxDriver
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // Drive 1m forward while A button is held
    xboxDriver.a().whileTrue(PathCommands.advanceForward(drive, Meters.of(1)));

    // Align with pose, approaching in correct orientation from 1 m away
    // xboxDriver
    //     .a()
    //     .whileTrue(
    //         PathCommands.dockToTargetPose(
    //             drive, new Pose2d(8.2296, 4.1148, Rotation2d.kZero), Meters.of(1)));

    // Drive to point, approaching in correct orientation from 2 m away
    // xboxDriver
    //  .a()
    // .whileTrue(
    //     PathCommands.dockToTargetPoint(drive, new Translation2d(8.2296, 4.1148), Meters.of(2)));

    // Switch to X pattern when X button is pressed
    xboxDriver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
  }

  public void bindXboxOperator(int port) {
    var xboxOperator = new CommandXboxController(port);
  }

  public void configureAutoOptions() {
    autoSelector.addAuto(new AutoOption(Alliance.Red, 1, new R_MoveStraight(drive)));
    autoSelector.addAuto(new AutoOption(Alliance.Red, 2, new R_MoveAndRotate(drive)));
    autoSelector.addAuto(new AutoOption(Alliance.Blue, 3, new B_Path(drive)));
  }
}
