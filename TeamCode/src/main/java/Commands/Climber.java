package Commands;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Utilities.Constants;

public class Climber {
    private final CRServo elevator;
    private boolean raised = false;
    public Climber(HardwareMap hardwareMap) {
        elevator = hardwareMap.get(CRServo.class, Constants.ClimberConstants.elevator);
        elevator.setPower(0);
    }
    public void raiseControl(boolean raise, boolean lower ) {
        if (raise && !lower) elevator.setPower(1);
        else if (lower && !raise) elevator.setPower(-1);
        else elevator.setPower(0);
    }
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Elevator power: ", elevator.getPower());
    }
}
