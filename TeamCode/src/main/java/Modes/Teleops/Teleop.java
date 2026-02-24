package Modes.Teleops;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;


import Commands.MechanumDrive;
import Utilities.Constants;

public class Teleop {
    Gamepad activeGamepad1;
    MechanumDrive Mechanum;
    boolean autoNeck = true;
    public void init(HardwareMap hardwareMap, String team){
        Constants.TEAM = team;
        activeGamepad1 = new Gamepad();
        Mechanum = new MechanumDrive(hardwareMap);
    }
    public void run(Gamepad gamepad1) {
            activeGamepad1.copy(gamepad1);
            Mechanum.drive(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            );
    }
}
