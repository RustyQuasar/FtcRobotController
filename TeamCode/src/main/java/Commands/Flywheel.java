package Commands;

import Subsystems.PIDFController;
import Utilities.ConfigVariables;
import Utilities.Constants;

public class Flywheel {
    final double maxVel = 2800;
    private double currentVel = 0, targetVel;
    private PIDFController flywheelController;
    public Flywheel() {
        flywheelController = new PIDFController(ConfigVariables.shooterP, ConfigVariables.shooterI, ConfigVariables.shooterD, 0, 0);
    }
    public double getFlywheelPower(double distance, double currentVel, Constants.FlywheelConstants.FlywheelState state){
        targetVel = 500;
        if (state == Constants.FlywheelConstants.FlywheelState.SCORE) targetVel = distance * 4.55 + 600;
        else if (state == Constants.FlywheelConstants.FlywheelState.PASS) targetVel = distance * 2 + 600;
        //return flywheelController.calculate(targetVel, currentVel);
        return targetVel / maxVel;
    }

    public double getHeadPositions(Constants.FlywheelConstants.FlywheelState state, double distance){
        if (state == Constants.FlywheelConstants.FlywheelState.SCORE) return distance / 288;
        else if (state == Constants.FlywheelConstants.FlywheelState.PASS) return .6;
        else return 0;
    }

    public boolean atTargetVel(){
        return (Math.abs(currentVel - targetVel) <= 15);
    }

}