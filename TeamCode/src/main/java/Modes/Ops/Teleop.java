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
    boolean bLastState = false, aLastState = false, xLastState = false, yLastState = false, clawOpen = false;
    ElapsedTime controlLoopTimer;
    List<LynxModule> allHubs;
    public void init(HardwareMap hardwareMap, String team) {
        Constants.TEAM = team;
        Mecanum = new MecanumDrive(hardwareMap);
        controlLoopTimer = new ElapsedTime();
        collector = new Collector(hardwareMap);
        allHubs = hardwareMap.getAll(LynxModule.class);
    }

    public void run(Gamepad driver, Gamepad operator) {
        //XboxController driver = new XboxController(gamepad1);
        //XboxController operator = new XboxController(gamepad2);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        Mecanum.drive(
                -driver.left_stick_y,
                driver.left_stick_x,
                driver.right_stick_x
        );

        if (driver.dpad_down) {
            Mecanum.resetIMU();
        }


        if (operator.b && !bLastState) {
            if (clawOpen) collector.closeClaw();
            else collector.openClaw();
            clawOpen = !clawOpen;
        }

        if (operator.a && !aLastState) collector.setFloorPosition();
        if (operator.x && !xLastState) collector.setDiamondPositions();
        if (operator.y && !yLastState) collector.setBinPositions();

        collector.armControl(operator.left_stick_y);
        collector.wristControl(operator.right_stick_y);
        collector.updateHardware();

        bLastState = operator.b;

        collector.telemetry(telemetry);
        //telemetry.addData("Driver inputs: ", driver);
        //telemetry.addData("Operator inputs: ", operator);
        //Mecanum.telemetry(telemetry);
        telemetry.addData("Control loop time (ms): ", controlLoopTimer.time()*1e3);
        controlLoopTimer.reset();
        telemetry.update();
    }
}
