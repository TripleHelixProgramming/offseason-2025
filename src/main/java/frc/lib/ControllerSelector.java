package frc.lib;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.Constants.Mode;
import java.util.ArrayList;
import java.util.List;

public class ControllerSelector {

  private Mode mode;
  private List<ControlPanelBinding> controlPanelBindings = new ArrayList<>();
  private List<GenericHID> controllers = new ArrayList<>();
  private List<String> controllerNames = new ArrayList<>();

  public ControllerSelector(Mode mode) {
    this.mode = mode;

    // Create a joystick at each port so we can check their names later
    // Most of these objects will go unused
    for (int i = 0; i < 6; i++) {
      controllers.add(new GenericHID(i));
      controllerNames.add(controllers.get(i).getName());
    }
  }

  /**
   * Detect if the list of controllers connected to USB has changed. If so, update the list of
   * controllers and return TRUE.
   *
   * @return has the list of controllers changed?
   */
  private boolean controllersChanged() {

    boolean changed = false;
    for (int i = 0; i < 6; i++) {
      String controllerName = controllers.get(i).getName();
      if (!controllerNames.get(i).equals(controllerName)) {
        changed = true;
        controllerNames.set(i, controllerName);
      }
    }
    return changed;
  }

  public void add(Mode[] allowedModes, ControllerBinding driverController) {
    controlPanelBindings.add(new ControlPanelBinding(allowedModes, driverController));
  }

  public void add(
      Mode[] allowedModes,
      ControllerBinding driverController,
      ControllerBinding operatorController) {
    controlPanelBindings.add(
        new ControlPanelBinding(allowedModes, driverController, operatorController));
  }

  public void rebindControlPanel() {
    if (!controllersChanged()) return;

    // Filter the stream of control panel bindings for the first exact match where:
    //  * The given number of controllers are connected
    //  * Connected controllers are of the given type
    //  * Connected controllers of the correct type are at the given port
    //  * We are in the allowed mode (real/sim) for the configuration
    //
    // If no exact matches are found, then assume they are in the wrong ports.
    // Filter the stream of control panel bindings for the first match where:
    //  * The given number of controllers are connected
    //  * Connected controllers are of the given type
    //  * We are in the allowed mode (real/sim) for the configuration
    //
    // In the case where the driver and operator controller are the same type,
    // and exactly 2 of these devices are plugged into the given ports, then
    // assume they are plugged in correctly and assign their functionality according
    // to the given port #s.
    //
    // In the case where the driver and operator controller are the same type,
    // but there are NOT exactly 2 of these devices connected, or they are NOT plugged
    // into the given ports, then do not bind any commands.
    //
    // Then, for each valid controller, bind its commands.
  }

  public int getDriverPort() {
    return 0;
  }

  public int getOperatorPort() {
    return 1;
  }
}
