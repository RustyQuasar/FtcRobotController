package Modes.AllianceModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

public class AllianceTeleop {
    static FtcDashboard dashboard = FtcDashboard.getInstance();
    @TeleOp(name = "Red TeleOp", group = "Teleop")
    public static class RedTeleop extends OpMode {
        Modes.Teleop teleop = new Modes.Teleop();
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
        Modes.Teleop teleop = new Modes.Teleop();
        @Override
        public void init(){
            teleop.init(hardwareMap, "BLUE");
        }
        @Override
        public void loop() {
            teleop.run(gamepad1);
        }
    }

    @TeleOp(name = "Red Tuning", group = "Tuning")
    public static class RedTuning extends OpMode {
        Modes.ManualControl teleop = new Modes.ManualControl();
        @Override
        public void init(){
            teleop.init(hardwareMap, "RED");
        }
        @Override
        public void loop() {
            teleop.run(gamepad1);
        }
    }

    @TeleOp(name = "Blue Tuning", group = "Tuning")
    public static class BlueTuning extends OpMode {
        Modes.ManualControl teleop = new Modes.ManualControl();
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