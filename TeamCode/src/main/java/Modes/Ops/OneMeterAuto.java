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
    public void init(HardwareMap hardwareMap, String team){
        Constants.TEAM = team;
        Mechanum = new MechanumDrive(hardwareMap);
        Mechanum.resetIMU();
    }
    public void run() {
        if (!Mechanum.at1Meter()) {
            Mechanum.drive(
                    -0.1,
                    0,
                    0
            );
        } else {
            Mechanum.drive(
                    -0,
                    0,
                    0
            );
        }
        Mechanum.telemetry(telemetry);
        telemetry.update();
    }
}
