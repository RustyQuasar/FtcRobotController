package Commands;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Utilities.Constants;

public class SmartIntake {
    private final CRServo motorIntake;

    private final ColorSensor colorSen;

    private boolean motifState = false;
    private int ballCount = 0;
  public String[] artifactOrder = {"N", "N", "N"};

    public SmartIntake(HardwareMap hardwareMap) {
        motorIntake = hardwareMap.get(CRServo.class, Constants.IntakeConstants.intake);
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
        artifactOrder[0] = "N";
        artifactOrder[1] = "N";
        artifactOrder[2] = "N";
        ballCount = 0;
    }

    public void intakeStateSwitch() {
        motifState = !motifState;
    }

    public void intake(boolean buttonPressed) {
        if (buttonPressed && ballCount != 3) {
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
                    } catch (Exception e) {

                    }
                }

            } else {
                while (!isBall()) {
                    motorIntake.setPower(0.8);
                }
                ballCount++;
                colorRegister();
            }
        }
        motorIntake.setPower(0);


    }

}


