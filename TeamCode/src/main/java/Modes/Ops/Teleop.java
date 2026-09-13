package Modes.Ops;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;

import Commands.MecanumDrive;
import Utilities.Constants;

public class Teleop {
    //Sample teleop, shows how this all should be structured
    Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
    MecanumDrive Mecanum;
    boolean clawOpen = false;
    ElapsedTime controlLoopTimer;
    //List<LynxModule> allHubs;
    Gamepad lastDriver = new Gamepad(), lastOperator = new Gamepad();

    public void init(HardwareMap hardwareMap, boolean onRed) {
        Constants.onRed = onRed;
        Mecanum = new MecanumDrive(hardwareMap);
        controlLoopTimer = new ElapsedTime();
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
