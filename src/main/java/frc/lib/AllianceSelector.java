package frc.lib;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import org.littletonrobotics.junction.Logger;

public class AllianceSelector {

  private final AllianceSelectorIO io;
  private final AllianceSelectorIOInputsAutoLogged inputs =
      new AllianceSelectorIOInputsAutoLogged();
  private static AllianceSelector instance;

  private EventLoop eventLoop = new EventLoop();
  private BooleanEvent changedAlliance;
  private BooleanEvent agreementInAllianceInputs;

  /** Initializes the AllianceSelector singleton. */
  public static void initialize() {
    if (instance == null) {
      instance = new AllianceSelector(AutoConstants.kAllianceColorSelectorPort);
    }
  }

  /**
   * Returns the singleton instance of the AllianceSelector.
   *
   * @throws IllegalStateException if initialize() has not been called yet.
   * @return The singleton instance.
   */
  public static AllianceSelector getInstance() {
    if (instance == null) {
      throw new IllegalStateException("AllianceSelector not initialized.");
    }
    return instance;
  }

  private AllianceSelector(int port) {
    io = new AllianceSelectorIO(port);
    changedAlliance = new BooleanEvent(eventLoop, () -> inputs.allianceChanged);
    agreementInAllianceInputs = new BooleanEvent(eventLoop, () -> inputs.agreementInAllianceInputs);
  }

  /**
   * @return Whether the field is rotated from the driver's perspective
   */
  public boolean fieldRotated() {
    return inputs.allianceFromSwitch.equals(Alliance.Red);
  }

  /**
   * @return The current alliance
   */
  public Alliance getAllianceColor() {
    return inputs.allianceFromSwitch;
  }

  /**
   * @return Object for binding a command to a change in alliance color
   */
  public Trigger getAllianceColorChange() {
    return changedAlliance.castTo(Trigger::new);
  }

  /**
   * @return Object for binding a command to agreement between the sources of information for
   *     alliance color
   */
  public Trigger getAgreementInAllianceColor() {
    return agreementInAllianceInputs.castTo(Trigger::new);
  }

  public void disabledPeriodic() {
    eventLoop.poll();
    io.updateInputs(inputs);
    Logger.processInputs("AllianceSelector", inputs);
  }
}
