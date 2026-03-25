package Modes.AllianceModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Modes.Ops.OneMeterAuto;
import Modes.Ops.Teleop;

public class Auto1m {
    @Autonomous(name = "1 meter Auto", group = "Teleop")
    public static class OneMAuto extends OpMode {
        OneMeterAuto auto = new OneMeterAuto();
        @Override
        public void init(){
            auto.init(hardwareMap, "RED");
        }
        @Override
        public void loop() {
            auto.run();
        }
    }

}