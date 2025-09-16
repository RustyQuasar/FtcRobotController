package Commands;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SmartIntake {
    private final DcMotor leftIntake;
    private final DcMotor rightIntake;

    private boolean motifState = false;
    private String[] artifactOrder = {"N","N","N"};
    public SmartIntake(HardwareMap hardwareMap) {
        leftIntake = hardwareMap.get(DcMotor.class, "leftIntake");
        rightIntake = hardwareMap.get(DcMotor.class, "rightIntake");
    }

    public void colorRegister(int colorRedValue) {
        artifactOrder[3]=  artifactOrder[2];
        artifactOrder[2]=  artifactOrder[1];
if(colorRedValue>=200){
    artifactOrder[1]=  "P";
} else{
    artifactOrder[1]=  "G";
}
    }
    public void intakeStateSwitch() {
 motifState=!motifState;
        }
    public void intake(boolean swap_sort) {

    }

}


