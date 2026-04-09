package Modes.Ops;


import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.Drivetrain;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.Telemetry;

import Commands.Collector;
import Commands.Elevator;
import Commands.MechanumDrive;
import Utilities.Constants;

public class Teleop {
    Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
    Gamepad driver, operator;
    MechanumDrive Mechanum;
    Elevator Elevator;
    Collector Collector;
    boolean aLastState, bLastState = true;
    public void init(HardwareMap hardwareMap, String team){
        Constants.TEAM = team;
        driver = new Gamepad();
        operator = new Gamepad();
        Mechanum = new MechanumDrive(hardwareMap);
        Elevator = new Elevator(hardwareMap);
        Collector = new Collector(hardwareMap);
        Mechanum.resetIMU();
    }
    public void run(Gamepad gamepad1, Gamepad gamepad2) {
        driver.copy(gamepad1);
        operator.copy(gamepad2);
            Mechanum.drive(
                    -driver.left_stick_y,
                    driver.left_stick_x,
                    driver.right_stick_x
            );
            Collector.raiseControl(operator.left_bumper, operator.right_bumper);
            Elevator.swapElevation(operator.a && !aLastState);
            Collector.collect(operator.b && !bLastState);
            if (driver.dpad_down){
                Mechanum.resetIMU();
            }
            aLastState = operator.a;
            bLastState = operator.b;
        Mechanum.telemetry(telemetry);
        telemetry.addData("Joystick values: ", driver.left_stick_x + " " + driver.left_stick_y);
        telemetry.update();
    }
}
