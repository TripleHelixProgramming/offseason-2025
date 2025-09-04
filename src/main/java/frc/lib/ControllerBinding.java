package frc.lib;

public class ControllerBinding {
  private int defaultPort;
  private ControllerType controllerType;
  private Runnable bindAction;

  public ControllerBinding(ControllerType controllerType, int port, Runnable bindAction) {
    this.controllerType = controllerType;
    this.defaultPort = port;
    this.bindAction = bindAction;
  }

  public ControllerType getControllerType() {
    return controllerType;
  }

  public int getPort() {
    return defaultPort;
  }

  public void bind() {
    bindAction.run();
  }

  public enum ControllerType {
    ZORRO,
    XBOX,
    RADIOMASTER,
    PS4;
  }
}
