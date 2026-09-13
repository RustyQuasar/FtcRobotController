package Commands;

import Subsystems.PIDFController;
import Utilities.ConfigVariables;

public class Flywheel {
    private PIDFController shooterController;
    public Flywheel() {
        shooterController = new PIDFController(ConfigVariables.shooterP, ConfigVariables.shooterI, ConfigVariables.shooterD, ConfigVariables.shooterF, 0);
    }
    public double calculateVelocity(double distance, double currentVel){
        return shooterController.calculate(distance * 4.55 + 600, currentVel);
    }
}