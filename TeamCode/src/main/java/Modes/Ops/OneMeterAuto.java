package Modes.Ops;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Commands.Collector;
import Commands.Elevator;
import Commands.MechanumDrive;
import Utilities.Constants;

public class OneMeterAuto {
    Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
    MechanumDrive Mechanum;
    int phase = 1;
    long startTime = 0;
    public void init(HardwareMap hardwareMap, String team){
        Constants.TEAM = team;
        Mechanum = new MechanumDrive(hardwareMap);
        Mechanum.resetIMU();
    }
    public void run() {
        switch(phase) {
            case 1:
                if (Mechanum.setDistance(50)) phase++;
                break;
            case 2:
                if (Mechanum.setHeading(Math.toRadians(-90))) phase++;
                break;
            case 3:
                if (Mechanum.setDistance(100)) phase++;
                break;
                case 4:
                    if (startTime == 0) startTime = System.currentTimeMillis();
                    if (System.currentTimeMillis() - startTime > 5000) phase++;
                    break;
            case 5:
                if (Mechanum.setDistance(-95)) phase++;
                break;
            case 6:
                if (Mechanum.setHeading(Math.toRadians(90))) phase++;
                break;
            case 7:
                if (Mechanum.setDistance(-50)) phase++;
                break;
                default:
                    Mechanum.drive(0, 0, 0);
                    break;
        }
        Mechanum.telemetry(telemetry);
        telemetry.addData("Current phase: ", phase);
        telemetry.update();

    }
}
