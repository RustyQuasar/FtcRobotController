package Modes.AllianceModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Utilities.Constants;
public class AllianceTeleop {
    @TeleOp(name = "Red TeleOp", group = "Teleop")
    static class RedTeleop extends LinearOpMode {
        Modes.Teleop teleop = new Modes.Teleop();

        @Override
        public void runOpMode() {
            teleop.init(hardwareMap, "RED");
            waitForStart();
            teleop.run(opModeIsActive(), gamepad1);
        }
    }

    @TeleOp(name = "Blue TeleOp", group = "Teleop")
    class BlueTeleop extends LinearOpMode {
        Modes.Teleop teleop = new Modes.Teleop();

        @Override
        public void runOpMode() {
            teleop.init(hardwareMap, "RED");
            waitForStart();
            teleop.run(opModeIsActive(), gamepad1);
        }
    }
}