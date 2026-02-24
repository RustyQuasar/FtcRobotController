package Modes.AllianceModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Modes.Teleops.Teleop;

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
            teleop.run(gamepad1);
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
            teleop.run(gamepad1);
        }
    }
}