package frc.lib;

public class ControllerBinding<T> {
  private int defaultPort;
  private T controller;
  private Runnable bindAction;

  public ControllerBinding(T controller, int port, Runnable bindAction) {
    this.controller = controller;
    this.defaultPort = port;
    this.bindAction = bindAction;
  }

  public T getController() {
    return controller;
  }

  public int getPort() {
    return defaultPort;
  }

  public void bind() {
    bindAction.run();
  }
}
