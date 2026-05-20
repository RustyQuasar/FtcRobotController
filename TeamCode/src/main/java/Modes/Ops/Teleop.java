package Modes.Ops;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.Telemetry;

import Commands.Collector;
import Commands.Climber;
import Commands.MecanumDrive;
import Utilities.Constants;

public class Teleop {
    Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
    MecanumDrive Mecanum;
    Climber Climber;
    Collector Collector;
    boolean aLastState, bLastState = true;
    public void init(HardwareMap hardwareMap, String team){
        Constants.TEAM = team;
        Mecanum = new MecanumDrive(hardwareMap);
        Climber = new Climber(hardwareMap);
        Collector = new Collector(hardwareMap);
    }

    public void run(Gamepad driver, Gamepad operator) {
            Mecanum.drive(
                    -driver.left_stick_y,
                    driver.left_stick_x,
                    driver.right_stick_x
            );
            //Collector.raiseControl(operator.left_bumper, operator.right_bumper);
            //Climber.raiseControl(operator.dpad_up, operator.dpad_down);
            //Collector.collect(operator.b && !bLastState);
            //Collector.armControl(operator.left_stick_x, -operator.left_stick_y);
            if (driver.dpad_down){
                Mecanum.resetIMU();
            }
            aLastState = operator.a;
            bLastState = operator.b;
        //Mecanum.telemetry(telemetry);
        Collector.telemetry(telemetry);
        Climber.telemetry(telemetry);
        telemetry.addData("Driver inputs: ", driver);
        telemetry.addData("Operator inputs: ", operator);
        telemetry.update();
    }
}
