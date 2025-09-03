package frc.lib;

import java.util.Optional;
import java.util.OptionalInt;

public class ControllerOption {
    private OptionalInt defaultDriverPort;
    private OptionalInt defaultOperatorPort;
    private Optional<?> driverController;
    private Optional<?> operatorController;

    // public enum Controller {
    //     Zorro(new String[] {"Zorro"}),
    //     XBox(new String[] {"XBOX"}),
    //     PS4(new String[] {"P"}),
    //     RadioMaster(new String[] {"TX16S"});

    //     private final String[] id;

    //     private Controller(String[] id) {
    //         this.id = id;
    //     }

    //     public String[] getID() {
    //         return this.id;
    //     }
    // }

    public ControllerOption(Controller<?> driverController) {
        this.driverController = Optional.of(driverController.getController());
        this.defaultDriverPort = OptionalInt.of(driverController.getPort());

        this.operatorController = Optional.empty();
        this.defaultOperatorPort = OptionalInt.empty();
    }

    public ControllerOption(Controller<?> driverController, Controller<?> operatorController) {
        this.driverController = Optional.of(driverController.getController());
        this.defaultDriverPort = OptionalInt.of(driverController.getPort());

        this.operatorController = Optional.of(operatorController.getController());
        this.defaultOperatorPort = OptionalInt.of(operatorController.getPort());
    }

    public Optional<Controller<?>> getDriverController() {
        return this.driverController;
    }

    public Optional<Controller<?>> getOperatorController() {
        return this.operatorController;
    }

    public OptionalInt getDefaultDriverPort() {
        return this.defaultDriverPort;
    }

    public OptionalInt getDefaultOperatorPort() {
        return this.defaultOperatorPort;
    }
}
