package Commands;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Collector {
    private final Servo claw, extender;
    private final CRServo elevator;
    private final DistanceSensor distanceSensor;
    public boolean collecting, transitioning = false;
    public Collector(HardwareMap hardwareMap) {
        elevator = hardwareMap.get(CRServo.class, "elevator");
        claw = hardwareMap.get(Servo.class, "claw");
        extender = hardwareMap.get(Servo.class, "extender");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "collectorElevation");
        extender.setPosition(0);
        claw.setPosition(0);
    }
    public void collect(boolean swap){
        if (swap) {
            collecting = !collecting;
            transitioning = true;
        }
        if (!transitioning) return;
        if (collecting){
            claw.setPosition(0.8);
            if (claw.getPosition() >= 0.6) {
                if (distanceSensor.getDistance(DistanceUnit.INCH) < 5) {
                    elevator.setPower(0);
                    extender.setPosition(0);
                    collecting = !(extender.getPosition() <= 0.1);
                } else {
                    elevator.setPower(1);
                }
            }
        } else {
            claw.setPosition(0);
            if (claw.getPosition() < 0.2){
                extender.setPosition(0.6);
                transitioning = false;
            }
        }

    }
    public void raiseControl(boolean raise, boolean lower ) {
        if (raise && !lower) extender.setPosition(extender.getPosition() + 0.05);
        else if (lower && !raise) extender.setPosition(extender.getPosition() - 0.05);
    }
}
