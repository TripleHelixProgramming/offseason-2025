package frc.lib;

import frc.robot.Constants.Mode;
import java.util.Optional;

public class ControlPanelBinding {
  private Optional<ControllerBinding> driver;
  private Optional<ControllerBinding> operator;
  private Mode[] allowedModes;

  public ControlPanelBinding(Mode[] allowedModes, ControllerBinding driverController) {
    this.allowedModes = allowedModes;
    this.driver = Optional.of(driverController);
    this.operator = Optional.empty();
  }

  public ControlPanelBinding(
      Mode[] allowedModes,
      ControllerBinding driverController,
      ControllerBinding operatorController) {
    this.allowedModes = allowedModes;
    this.driver = Optional.of(driverController);
    this.operator = Optional.of(operatorController);
  }

  public Optional<ControllerBinding> getDriver() {
    return this.driver;
  }

  public Optional<ControllerBinding> getOperator() {
    return this.operator;
  }

  public Mode[] getAllowedModes() {
    return this.allowedModes;
  }
}
