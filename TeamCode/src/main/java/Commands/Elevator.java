package Commands;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import Utilities.Constants;

public class Elevator {
    private final CRServo elevator;
    private boolean raised = false;
    public Elevator(HardwareMap hardwareMap) {
        elevator = hardwareMap.get(CRServo.class, Constants.ElevatorConstants.elevator);
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
