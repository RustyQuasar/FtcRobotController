package Commands;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Utilities.Constants;

public class SmartIntake {
    private final DcMotor motorIntake;

    private final ColorSensor colorSen;

    private boolean motifState = false;
    private int ballCount = 0;
    public String[] artifactOrder = {"N", "N", "N"};

    public SmartIntake(HardwareMap hardwareMap) {
        motorIntake = hardwareMap.get(DcMotor.class, Constants.IntakeConstants.intake);
        colorSen = hardwareMap.get(ColorSensor.class, Constants.IntakeConstants.colourSensor);
    }

    public boolean isBall() {
        return colorSen.blue() < 50;
    }

    public void colorRegister() {
        double colorRedValue = colorSen.red();
        artifactOrder[2] = artifactOrder[1];
        artifactOrder[1] = artifactOrder[0];
        if (colorRedValue >= 200) {
            artifactOrder[0] = "P";
        } else {
            artifactOrder[0] = "G";
        }
    }

    public void colorWipe() {
        artifactOrder = new String[]{"N","N","N"};

        ballCount = 0;
    }

    public void intakeStateSwitch() {
        motifState = !motifState;
    }

    public void periodic(Telemetry telemetry) {
        telemetry.addLine("Intake");
        telemetry.addData("Ball count: ", ballCount);
        telemetry.addData("Slots: ", artifactOrder[0] + artifactOrder[1] + artifactOrder[2]);
        telemetry.addData("Motif state: ", motifState);
    }

    public void intake(boolean buttonPressed) {


        if (buttonPressed) {
            if (motifState) {
                while (!isBall()) {
                    motorIntake.setPower(0.8);
                }
                ballCount++;
                colorRegister();
                if (!(artifactOrder[ballCount - 1].equals(Constants.VisionConstants.colours[3 - ballCount]))) {
                    ballCount--;
                    motorIntake.setPower(-1);
                    try {
                        Thread.sleep(500);
                    } catch (Exception ignored) {

                    }
                }

            } else {
                while (buttonPressed) {
                    motorIntake.setPower(0.8);
                }
                ballCount++;
                colorRegister();
            }
        }else{
        motorIntake.setPower(0);
}


    }

}


