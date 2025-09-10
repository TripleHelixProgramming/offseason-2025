package frc.lib;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.Mode;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.IntConsumer;
import org.littletonrobotics.junction.Logger;

/**
 * This class manages the selection and binding of controllers for driver and operator roles. It
 * supports different controller types and configurations based on the robot's mode (REAL or SIM).
 * The class scans for connected controllers, prioritizes them based on a predefined configuration,
 * and binds the appropriate commands to the selected controllers.
 *
 * <p>**Important:** The selection of controllers is prioritized in the order that configurations
 * are added. The first matching configuration found during the scan will be used. Note also that
 * the driver controller is always found first, and the operator controller will not consider the
 * port used by the driver controller.
 */
public class ControllerSelector {

  /** Defines the possible functions for a controller: DRIVER or OPERATOR. */
  public enum ControllerFunction {
    DRIVER,
    OPERATOR
  }

  /** Defines the choices of controller hardware. */
  public enum ControllerType {
    ZORRO("Zorro"),
    XBOX("XBOX"),
    RADIOMASTER("TX16S"),
    PS4("P");

    String deviceName;

    ControllerType(String deviceName) {
      this.deviceName = deviceName;
    }

    String getDeviceName() {
      return deviceName;
    }
  }

  /**
   * A data class that encapsulates the configuration for a controller binding. It includes the
   * modes in which the configuration is valid, the controller function (DRIVER or OPERATOR), the
   * controller type, and a callback function to bind the controller's commands.
   */
  public static class ControllerConfig {
    public final Set<Mode> modes;
    public final ControllerFunction controllerFunction;
    public final ControllerType controllerType;
    public final IntConsumer bindingCallback;

    /**
     * Constructs a new ControllerConfig object.
     *
     * @param controllerFunction The function of the controller (DRIVER or OPERATOR).
     * @param controllerType The type of the controller (e.g., XBOX, ZORRO).
     * @param bindingCallback The callback function that binds the controller's commands. This
     *     function takes the port number of the controller as an argument.
     * @param modes The modes in which this configuration is valid (REAL, SIM, or both).
     */
    public ControllerConfig(
        ControllerFunction controllerFunction,
        ControllerType controllerType,
        IntConsumer bindingCallback,
        Mode... modes) {
      this.modes = new HashSet<>(Arrays.asList(modes));
      this.controllerFunction = controllerFunction;
      this.controllerType = controllerType;
      this.bindingCallback = bindingCallback;
    }
  }

  private final Mode mode;
  private final List<ControllerConfig> controllerConfigs = new ArrayList<>();
  private final List<GenericHID> controllers = new ArrayList<>();
  private final List<String> controllerNames = new ArrayList<>();
  private int driverPort = -1;
  private int operatorPort = -1;
  private ControllerConfig driverConfig = null;
  private ControllerConfig operatorConfig = null;

  /**
   * Constructs a new ControllerSelector object.
   *
   * @param mode The current mode of the robot (REAL or SIM).
   */
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
   * Detects if the list of controllers connected to USB has changed. If so, update the list of
   * controllers and return TRUE.
   *
   * @return TRUE if the list of controllers has changed, FALSE otherwise.
   */
  private boolean controllersChanged() {
    var changed = false;
    for (int i = 0; i < 6; i++) {
      var controllerName = controllers.get(i).getName();
      if (!controllerNames.get(i).equals(controllerName)) {
        changed = true;
        controllerNames.set(i, controllerName);
      }
    }
    return changed;
  }

  /**
   * Adds a new controller configuration to the list of available configurations.
   *
   * @param config The controller configuration to add.
   */
  public void addConfig(ControllerConfig config) {
    controllerConfigs.add(config);
  }

  public void rebindControlPanel() {
    if (!controllersChanged()) {
      return;
    }
    bindControlPanel();
  }

  /**
   * Rebinds the control panel based on the current mode and connected controllers. This method
   * performs the following steps:
   *
   * <p>1. Checks if the connected controllers have changed. If not, it returns immediately.
   *
   * <p>2. Resets the driver and operator ports and configurations.
   *
   * <p>3. Scans for a driver controller.
   *
   * <p>4. Scans for an operator controller, excluding the port used by the driver.
   *
   * <p>5. If a driver controller is found, it binds the controller's commands and updates the
   * SmartDashboard and Logger with the driver's information.
   *
   * <p>6. If an operator controller is found, it binds the controller's commands and updates the
   * SmartDashboard and Logger with the operator's information.
   *
   * <p>7. If no driver or operator controllers are found, it displays a warning on the
   * SmartDashboard and Logger.
   *
   * <p>8. If no controllers are connected, it sets the RobotController to brownout voltage to
   * signal that no controllers are connected.
   */
  public void bindControlPanel() {
    // Clear any active buttons
    CommandScheduler.getInstance().getDefaultButtonLoop().clear();

    driverPort = -1;
    operatorPort = -1;
    driverConfig = null;
    operatorConfig = null;

    scanForController(ControllerFunction.DRIVER, -1);
    if (driverConfig != null) {
      scanForController(ControllerFunction.OPERATOR, driverPort);
    }

    // Bind Driver Controller
    if (driverPort != -1 && driverConfig != null) {
      driverConfig.bindingCallback.accept(driverPort);
      Logger.recordOutput(
          "ControllerSelector/DriverController",
          driverConfig.controllerType + " on port " + driverPort);
    } else {
      Logger.recordOutput(
          "ControllerSelector/DriverController", "WARNING: No Driver Controller Found!");
    }

    // Bind Operator Controller
    if (operatorPort != -1 && operatorConfig != null) {
      operatorConfig.bindingCallback.accept(operatorPort);
      Logger.recordOutput(
          "ControllerSelector/OperatorController",
          operatorConfig.controllerType + " on port " + operatorPort);
    } else {
      Logger.recordOutput(
          "ControllerSelector/OperatorController", "WARNING: No Operator Controller Found!");
    }

    if (driverPort == -1 && operatorPort == -1) {
      // display a blinking alert on the rio that no controllers are connected.
      RobotController.setBrownoutVoltage(6);
    }
  }

  /**
   * Scans for a controller of the specified function (DRIVER or OPERATOR).
   *
   * <p>This method iterates through the available controller configurations and attempts to find a
   * matching controller connected to the robot. The search is performed based on the controller's
   * mode, function, and type.
   *
   * @param controllerFunction The function of the controller to scan for (DRIVER or OPERATOR).
   * @param excludedPort The port number to exclude from the scan (used to prevent assigning the
   *     same controller to both driver and operator).
   */
  private void scanForController(ControllerFunction controllerFunction, int excludedPort) {
    for (int port = 0; port < 6; port++) {
      if (port == excludedPort) continue;

      int currentPort = port;
      Optional<ControllerConfig> matchingConfig =
          controllerConfigs.stream()
              .filter(
                  config ->
                      config.modes.contains(mode)
                          && config.controllerFunction == controllerFunction
                          && checkControllerType(currentPort, config.controllerType))
              .findFirst();

      if (matchingConfig.isPresent()) {
        if (controllerFunction == ControllerFunction.DRIVER) {
          driverPort = currentPort;
          driverConfig = matchingConfig.get();
        } else {
          operatorPort = currentPort;
          operatorConfig = matchingConfig.get();
        }
        return; // Exit after finding the first suitable controller
      }
    }
  }

  /**
   * Checks if the controller connected to the specified port matches the given controller type.
   *
   * @param port The port number of the controller to check.
   * @param controllerType The type of controller to match (e.g., XBOX, ZORRO).
   * @return TRUE if the controller type contains the controller name (case-insensitive), FALSE
   *     otherwise.
   */
  private boolean checkControllerType(int port, ControllerType controllerType) {
    var controllerName = controllers.get(port).getName();
    if (controllerName == null || controllerName.isEmpty()) {
      return false;
    }

    return controllerName.toLowerCase().contains(controllerType.getDeviceName().toLowerCase());
  }

  public int getDriverPort() {
    return driverPort;
  }

  public int getOperatorPort() {
    return operatorPort;
  }
}
