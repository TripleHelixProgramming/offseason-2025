package frc.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AutoSelector implements Supplier<Optional<AutoOption>> {

  private final AutoSelectorIO io;
  private final AutoSelectorIOInputsAutoLogged inputs = new AutoSelectorIOInputsAutoLogged();

  private static AutoSelector instance;
  private Optional<AutoOption> currentAutoOption;
  private Supplier<Alliance> allianceColorSupplier;
  private List<AutoOption> autoOptions = new ArrayList<>();
  private EventLoop eventLoop = new EventLoop();
  private BooleanEvent autoSelectionChanged;

  /** Initializes the AutoSelector singleton. */
  public static void initialize() {
    if (instance == null) {
      instance =
          new AutoSelector(
              AutoConstants.kAutonomousModeSelectorPorts,
              AllianceSelector.getInstance()::getAllianceColor);
    }
  }

  /**
   * Returns the singleton instance of the AutoSelector.
   *
   * @throws IllegalStateException if initialize() has not been called yet.
   * @return The singleton instance.
   */
  public static AutoSelector getInstance() {
    if (instance == null) {
      throw new IllegalStateException("AutoSelector not initialized.");
    }
    return instance;
  }

  private AutoSelector(int[] ports, Supplier<Alliance> allianceColorSupplier) {
    this.allianceColorSupplier = allianceColorSupplier;
    io = new AutoSelectorIO(ports);
    autoSelectionChanged = new BooleanEvent(eventLoop, () -> updateAuto());
  }

  public void addAuto(AutoOption newAuto) {
    autoOptions.add(newAuto);
  }

  private Optional<AutoOption> findMatchingOption() {
    return autoOptions.stream()
        .filter(o -> o.getAlliance() == allianceColorSupplier.get())
        .filter(o -> o.getOptionNumber() == inputs.autoSwitchPosition)
        .findFirst();
  }

  private boolean updateAuto() {
    var newAutoOption = findMatchingOption();
    if (newAutoOption.equals(currentAutoOption)) {
      return false;
    }
    currentAutoOption = newAutoOption;
    return true;
  }

  @Override
  public Optional<AutoOption> get() {
    return currentAutoOption;
  }

  /**
   * @return Object for binding a command to a change in autonomous mode selection
   */
  public Trigger getChangedAutoSelection() {
    return autoSelectionChanged.castTo(Trigger::new);
  }

  /** Schedules the command corresponding to the selected autonomous mode */
  public void scheduleAuto() {
    currentAutoOption.ifPresent(ao -> ao.getAutoCommand().ifPresent(Command::schedule));
  }

  /** Deschedules the command corresponding to the selected autonomous mode */
  public void cancelAuto() {
    currentAutoOption.ifPresent(ao -> ao.getAutoCommand().ifPresent(Command::cancel));
  }

  public void disabledPeriodic() {
    eventLoop.poll();
    io.updateInputs(inputs);
    Logger.processInputs("AutoSelector", inputs);

    currentAutoOption.ifPresentOrElse(
        ao -> {
          Logger.recordOutput("AutoSelector/SelectedAutoMode", ao.getName());
          ao.getInitialTrajectory()
              .ifPresent(
                  traj -> Logger.recordOutput("AutoSelector/AutonomousInitialTrajectory", traj));
          ao.getInitialPose()
              .ifPresent(pose -> Logger.recordOutput("AutoSelector/AutonomousInitialPose", pose));
        },
        () -> {
          Logger.recordOutput("AutoSelector/SelectedAutoMode", "No auto mode assigned");
          Logger.recordOutput("AutoSelector/AutonomousInitialPose", Pose2d.kZero);
        });
  }

  public Optional<Pose2d> getInitialPose() {
    return currentAutoOption.isPresent()
        ? currentAutoOption.get().getInitialPose()
        : Optional.empty();
  }
}
