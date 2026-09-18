package Commands;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Utilities.Constants;

public class Intake {
    private final DcMotor motorIntake;
    private double lastMotorPower = 0;

    public Intake(HardwareMap hardwareMap) {
        motorIntake = hardwareMap.get(DcMotor.class, Constants.IntakeConstants.intake);
        motorIntake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void intake(boolean trigger, boolean a) {
        double motorPower = 0.8;
        if (!trigger && a) motorPower *= -1;
        else if (!trigger) motorPower = 0;
        if (motorPower == lastMotorPower) return;
        lastMotorPower = motorPower;
        motorIntake.setPower(motorPower);
    }

    public void chill() {
        motorIntake.setPower(0);
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Intaking: ", motorIntake.getPower() != 0);
    }

}

