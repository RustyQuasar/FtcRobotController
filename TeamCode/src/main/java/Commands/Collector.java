package Commands;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import Utilities.Constants;

public class Collector {
    private final Servo claw, arm;
    private final Servo elevator;
    private final DistanceSensor collectorHeight;
    private boolean collecting = false;
    public Collector(HardwareMap hardwareMap) {
        elevator = hardwareMap.get(Servo.class, Constants.CollectorConstants.elevator);
        arm = hardwareMap.get(Servo.class, Constants.CollectorConstants.arm);
        claw = hardwareMap.get(Servo.class, Constants.CollectorConstants.claw);
        collectorHeight = hardwareMap.get(DistanceSensor.class, Constants.CollectorConstants.collectorHeight);
        arm.setPosition(0);
        claw.setPosition(0);
    }
    public void collect(boolean swap){
        if (swap) collecting = !collecting;
        if (collecting) claw.setPosition(1);
        else claw.setPosition(0);
    }
    public void armControl(double x, double y) {
        if (Math.sqrt(x*x + y*y) < 0.5) return;
        arm.setPosition(Math.atan2(y, x) / (Math.PI * Constants.CollectorConstants.armRatio));
    }
    public void raiseControl(boolean raise, boolean lower ) {
        if (raise && !lower && collectorHeight.getDistance(DistanceUnit.INCH) < 10) elevator.setPosition(elevator.getPosition() + 0.01);
        else if (lower && !raise && collectorHeight.getDistance(DistanceUnit.INCH) > 4) elevator.setPosition(elevator.getPosition() - 0.01);
    }
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Arm position: ", arm.getPosition());
        telemetry.addData("Collector height: ", collectorHeight.getDistance(DistanceUnit.INCH));
        telemetry.addData("Claw position: ", claw.getPosition());
        telemetry.addData("Elevator position: ", elevator.getPosition());
    }
}
