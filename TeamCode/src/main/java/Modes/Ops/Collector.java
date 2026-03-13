package Modes.Ops;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Collector {
    private final Servo claw;
    private final CRServo extender;
    private boolean closedClaw = false;
    public Collector(HardwareMap hardwareMap) {
        claw = hardwareMap.get(Servo.class, "claw");
        extender = hardwareMap.get(CRServo.class, "extender");
    }
    public void clawSwitchState(){
        closedClaw = !closedClaw;
        if (closedClaw) claw.setPosition(0);
        else claw.setPosition(1);
    }
    public void extendControl (double extendX, double extendY, double heading) {
        double joystickHeading = Math.atan2(extendY, extendX);
        boolean sameDirection = Math.abs(joystickHeading - heading) < Math.PI/2;
        double power = Math.sqrt(Math.pow(extendY, 2) + Math.pow(extendX, 2));
        if (sameDirection) extender.setPower(power);
        else extender.setPower(-power);
    }
}
