package Commands;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Elevator {
    private final CRServo elevator;
    private boolean raised = false;
    private final DistanceSensor distanceSensor;
    public Elevator(HardwareMap hardwareMap) {
        elevator = hardwareMap.get(CRServo.class, "elevator");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "elevatorDistanceSensor");
        elevator.setPower(0);
    }
    public void elevate(boolean leftBumper, boolean rightBumper) {
        if (leftBumper && !rightBumper) elevator.setPower(1);
        else if (rightBumper && !leftBumper) elevator.setPower(-1);
        else elevator.setPower(0);
    }

    public void swapElevation(boolean swap){
        if (swap) raised = !raised;
        if (raised && distanceSensor.getDistance(DistanceUnit.INCH) < 10) elevator.setPower(1);
        else if (!raised && distanceSensor.getDistance(DistanceUnit.INCH) > 4) elevator.setPower(-1);
        else elevator.setPower(0);
    }
}
