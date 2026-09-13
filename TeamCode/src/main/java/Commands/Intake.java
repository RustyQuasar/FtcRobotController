package Commands;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Utilities.Constants;

public class Intake {
    private final DcMotor motorIntake;
    private final CRServo transferServo, transferServo2;

    public Intake(HardwareMap hardwareMap) {
        motorIntake = hardwareMap.get(DcMotor.class, Constants.IntakeConstants.intake);
        motorIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        transferServo = hardwareMap.get(CRServo.class, Constants.IntakeConstants.transferServo);
        transferServo2 = hardwareMap.get(CRServo.class, Constants.IntakeConstants.transferServo2);
    }

    public void intake(boolean trigger, boolean a) {
        double motorPower = 0.8;
        if (!trigger && a) motorPower *= -1;
        boolean buttonPressed = trigger || a;
        if (buttonPressed) {
            motorIntake.setPower(motorPower);
        } else {
            motorIntake.setPower(0);
        }
    }

    public void transfer(boolean buttonPressed) {
        if (!buttonPressed) transferServo.setPower(-0.4);
        else transferServo.setPower(1);

        transferServo2.setPower(-transferServo.getPower());
    }

    public void chill() {
        transferServo.setPower(0);
        transferServo2.setPower(0);
        motorIntake.setPower(0);
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Intaking: ", motorIntake.getPower() != 0);
        telemetry.addData("Transferring: ", transferServo.getPower() != 0);
    }

}

