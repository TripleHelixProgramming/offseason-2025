package frc.lib;

public class Controller<T> {
    private int defaultPort;
    private T controller;

    public Controller(T controller, int port) {
        this.controller = controller;
        this.defaultPort = port;
    }

    public T getController() {
        return controller;
    }

    public int getPort() {
        return defaultPort;
    }
}
