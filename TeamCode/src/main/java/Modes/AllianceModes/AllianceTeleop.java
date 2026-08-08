package Modes.AllianceModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Modes.Ops.Teleop;

public class AllianceTeleop {
    @TeleOp(name = "Red TeleOp", group = "Teleop")
    public static class RedTeleop extends OpMode {
        Teleop teleop = new Teleop();
        @Override
        public void init(){
            teleop.init(hardwareMap, "RED");
        }
        @Override
        public void loop() {
            teleop.run(gamepad1, gamepad2);
        }
    }

    @TeleOp(name = "Blue TeleOp", group = "Teleop")
    public static class BlueTeleop extends OpMode {
        Teleop teleop = new Teleop();
        @Override
        public void init(){
            teleop.init(hardwareMap, "BLUE");
        }
        @Override
        public void loop() {
            teleop.run(gamepad1, gamepad2);
        }
    }
}