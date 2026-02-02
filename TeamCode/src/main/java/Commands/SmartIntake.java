package Commands;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Utilities.Constants;

public class SmartIntake {
    private final DcMotor motorIntake;

    public SmartIntake(HardwareMap hardwareMap) {
        motorIntake = hardwareMap.get(DcMotor.class, Constants.IntakeConstants.intake);
        motorIntake.setDirection(DcMotorSimple.Direction.REVERSE);}
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Intaking: ", motorIntake.getPower() != 0);
    }

    public void intake(boolean trigger, boolean a, boolean shooting) {
        double motorPower = 0.4;
        if (!trigger && a) motorPower *= -1;
        if (shooting) motorPower = 1;
        boolean buttonPressed = trigger || a;
        if (buttonPressed) {
            motorIntake.setPower(motorPower);
        } else {
            motorIntake.setPower(0);
        }
    }
}

