package Modes.Ops;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;

import Commands.Collector;
import Commands.MecanumDrive;
import Utilities.Constants;

import com.qualcomm.hardware.lynx.LynxModule;

import java.util.List;

public class Teleop {
    //Sample teleop, shows how this all should be structured
    Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
    MecanumDrive Mecanum;
    Collector collector;
    boolean clawOpen = false;
    ElapsedTime controlLoopTimer;
    //List<LynxModule> allHubs;
    Gamepad lastDriver = new Gamepad(), lastOperator = new Gamepad();

    public void init(HardwareMap hardwareMap, String team) {
        Constants.TEAM = team;
        Mecanum = new MecanumDrive(hardwareMap);
        controlLoopTimer = new ElapsedTime();
        collector = new Collector(hardwareMap);
        /*
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
         */
    }

    public void run(Gamepad driver, Gamepad operator) {
            Mecanum.drive(
                    -driver.left_stick_y,
                    driver.left_stick_x,
                    driver.right_stick_x
            );

        if (driver.dpadDownWasReleased()) {
            Mecanum.resetIMU();
        }


        if (operator.b && !lastOperator.b) {
            if (clawOpen) collector.closeClaw();
            else collector.openClaw();
            clawOpen = !clawOpen;
        }

        if (operator.a && !lastOperator.a) collector.setFloorPosition();
        if (operator.x && !lastOperator.x) collector.setDiamondPositions();
        if (operator.y && !lastOperator.y) collector.setBinPositions();

        collector.armControl(operator.left_stick_y);
        collector.wristControl(operator.right_stick_y);
        collector.updateHardware();

        lastOperator.copy(operator);
        lastDriver.copy(driver);
        telemetry();
        //Commands that run each loop should have their own built-in check, such as a shooter only re-doing math when the position changes

    }

    public void telemetry() {
        //telemetry.addData("Driver inputs: ", driver);
        //telemetry.addData("Operator inputs: ", operator);
        //Mecanum.telemetry(telemetry);
        telemetry.addData("Control loop time (ms): ", controlLoopTimer.time() * 1e3);
        telemetry.addData("Driver inputs: ", lastDriver);
        //Mecanum.telemetry(telemetry);
        controlLoopTimer.reset();
        telemetry.update();
    }

}
