package frc.lib;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;

public class ControllerSelector implements Supplier<Optional<ControllerOption>> {

    private Optional<ControllerOption> currentControllerOption;
    private List<ControllerOption> controllerOptions = new ArrayList<>();
    private EventLoop eventLoop = new EventLoop();
    private BooleanEvent controllerSelectionChanged;

    public ControllerSelector() {
        controllerSelectionChanged = new BooleanEvent(eventLoop, () -> updateControllers());
    }

    public void add(Controller<?> driverController) {
        controllerOptions.add(new ControllerOption(driverController));
    }

    public void add(Controller<?> driverController, Controller<?> operatorController) {
        controllerOptions.add(new ControllerOption(driverController, operatorController));
    }

    private Optional<ControllerOption> findMatchingOption() {
        return controllerOptions.stream()
            .findFirst();
      }

    private boolean updateControllers() {
        var newControllerOption = findMatchingOption();
        if (newControllerOption.equals(currentControllerOption)) {
          return false;
        }
        currentControllerOption = newControllerOption;
        return true;
      }

    @Override
    public Optional<ControllerOption> get() {
      return currentControllerOption;
    }
    
}
