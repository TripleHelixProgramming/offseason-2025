package frc.lib;

import java.util.Optional;

public class ControlPanelBinding {
  private Optional<ControllerBinding> driver;
  private Optional<ControllerBinding> operator;

  public ControlPanelBinding(ControllerBinding driverController) {
    this.driver = Optional.of(driverController);
    this.operator = Optional.empty();
  }

  public ControlPanelBinding(
      ControllerBinding driverController, ControllerBinding operatorController) {
    this.driver = Optional.of(driverController);
    this.operator = Optional.of(operatorController);
  }

  public Optional<ControllerBinding> getDriver() {
    return this.driver;
  }

  public Optional<ControllerBinding> getOperator() {
    return this.operator;
  }
}
