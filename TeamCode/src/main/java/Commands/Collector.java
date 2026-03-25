package Commands;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Collector {
    private final Servo claw, rotator, extender;
    public boolean collecting, transitioning = false;
    public Collector(HardwareMap hardwareMap) {
        rotator = hardwareMap.get(Servo.class, "rotator");
        claw = hardwareMap.get(Servo.class, "claw");
        extender = hardwareMap.get(Servo.class, "extender");
        extender.setPosition(0);
        rotator.setPosition(0);
        claw.setPosition(0);
    }
    public void clawSwitchState(boolean swap){
        if (swap) {
            collecting = !collecting;
            transitioning = true;
        }
        if (!transitioning) return;
        if (collecting){
            claw.setPosition(1);
            if (claw.getPosition() >= 0.8) {
                extender.setPosition(1);
                if (extender.getPosition() >= 0.8){
                    rotator.setPosition(1);
                    if (rotator.getPosition() >= 0.8) {
                        claw.setPosition(1);
                        if (claw.getPosition() >= 0.8) {
                            transitioning = false;
                            collecting = false;
                        }
                    }
                }
            }
        } else {
            claw.setPosition(1);
            if (rotator.getPosition() >= 0.8) {
                rotator.setPosition(0);
                if (rotator.getPosition() <= 0.2) {
                    extender.setPosition(0);
                    if (extender.getPosition() <= 0.2) {
                        claw.setPosition(0);
                        if (claw.getPosition() <= 0.2) {
                            transitioning = false;
                            collecting = false;
                        }
                    }
                }
            }
        }
    }
    public void raiseControl(boolean raise, boolean lower ) {
        if (raise && !lower) extender.setPosition(extender.getPosition() + 0.05);
        else if (lower && !raise) extender.setPosition(extender.getPosition() - 0.05);
    }
}
