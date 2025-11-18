package Commands;

import android.widget.Button;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

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
}

