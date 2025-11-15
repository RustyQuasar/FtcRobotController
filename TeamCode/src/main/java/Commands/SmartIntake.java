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
    private final CRServo transferServo, transferServo2;
    private final ColorSensor colorSen;
    private boolean motifState = false;
    private int ballCount = 0;
    public String[] artifactOrder = {"N", "N", "N"};
    boolean scanningBall = false;
    boolean ballPresent = false;
    int scans = 0;

    public SmartIntake(HardwareMap hardwareMap) {
        motorIntake = hardwareMap.get(DcMotor.class, Constants.IntakeConstants.intake);
        motorIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        colorSen = hardwareMap.get(ColorSensor.class, Constants.IntakeConstants.colourSensor);
        transferServo = hardwareMap.get(CRServo.class, Constants.IntakeConstants.transferServo);
        transferServo2 = hardwareMap.get(CRServo.class, Constants.IntakeConstants.transferServo2);
    }

    public boolean isBall() {
        return colorSen.blue() < 50;
    }

    public void colorRegister() {
        double colorRedValue = colorSen.red();
        double colourBlueValue = colorSen.blue();
        double colourGreenValue = colorSen.green();
        double maxColour = Math.max(colourGreenValue, Math.max(colourBlueValue, colorRedValue));
        ballPresent = !(colorRedValue == maxColour);
        if (ballPresent) {
            if (!scanningBall) {
                if (maxColour < 300) return;
                if (colourBlueValue == maxColour) {
                    scans++;
                    scanningBall = true;
                    artifactOrder[2] = artifactOrder[1];
                    artifactOrder[1] = artifactOrder[0];
                    artifactOrder[0] = "P";
                }
                if (colourGreenValue == maxColour) {
                    scans++;
                    scanningBall = true;
                    artifactOrder[2] = artifactOrder[1];
                    artifactOrder[1] = artifactOrder[0];
                    artifactOrder[0] = "G";
                }
            }
        } else {
            scanningBall = false;
        }
    }

    public void colorWipe() {
        artifactOrder = new String[]{"N", "N", "N"};
        ballCount = 0;
    }

    public void intakeStateSwitch() {
        motifState = !motifState;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addLine("Intake");
        telemetry.addData("Ball count: ", ballCount);
        telemetry.addData("Slots: ", artifactOrder[0] + artifactOrder[1] + artifactOrder[2]);
        telemetry.addData("Motif state: ", motifState);
        telemetry.addData("RGB: ", colorSen.red() + " " + colorSen.green() + " " + colorSen.blue());
        telemetry.addData("Scanning ball: ", scanningBall);
        telemetry.addData("Scans: ", scans);
    }

    public void intake(boolean buttonPressed) {
        double motorPower = 0.8;
        colorRegister();
        if (buttonPressed) {
            if (motifState) {
                if (!isBall()) {
                    motorIntake.setPower(motorPower);
                    transferServo.setPower(1);
                    return;
                }
                ballCount++;
                if (!(artifactOrder[ballCount - 1].equals(Constants.VisionConstants.colours[3 - ballCount]))) {
                    ballCount--;
                    motorIntake.setPower(-motorPower);
                    transferServo.setPower(-1);
                    try {
                        Thread.sleep(500);
                    } catch (Exception ignored) {

                    }
                }
            } else {
                transferServo.setPower(-1);
                motorIntake.setPower(motorPower);
            }
        } else {
            transferServo.setPower(0);
            motorIntake.setPower(0);
        }
        transferServo2.setPower(-transferServo.getPower());
    }
}

