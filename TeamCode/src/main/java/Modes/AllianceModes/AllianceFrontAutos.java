package Modes.AllianceModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import Modes.Autos.FrontGate12;
import Modes.Autos.SisterFrontAuto;

public class AllianceFrontAutos {
    @Autonomous(name = "Red Close 1 Gate", group = "Close Auto")
    public static class RedFrontGate12 extends OpMode {
        FrontGate12 frontGate12 = new FrontGate12();
        @Override
        public void init(){
            frontGate12.init(hardwareMap, "RED");
        }
        @Override
        public void loop(){
            frontGate12.loop();
        }
        @Override
        public void start(){
            frontGate12.start();
        }
    }
    @Autonomous(name = "Blue Close 1 Gate", group = "Close Auto")
    public static class BlueFrontGate12 extends OpMode {
        FrontGate12 frontGate12 = new FrontGate12();
        @Override
        public void init(){
            frontGate12.init(hardwareMap, "BLUE");
        }
        @Override
        public void loop(){
            frontGate12.loop();
        }
        @Override
        public void start(){
            frontGate12.start();
        }
    }
    @Autonomous(name = "Red Close 2 Gate", group = "Close Auto")
    public static class RedSister12 extends OpMode {
        SisterFrontAuto frontGate12 = new SisterFrontAuto();
        @Override
        public void init(){
            frontGate12.init(hardwareMap, "RED");
        }
        @Override
        public void loop(){
            frontGate12.loop();
        }
        @Override
        public void start(){
            frontGate12.start();
        }
    }
    @Autonomous(name = "Blue Close 2 Gate", group = "Close Auto")
    public static class BlueSister12 extends OpMode {
        SisterFrontAuto frontGate12 = new SisterFrontAuto();
        @Override
        public void init(){
            frontGate12.init(hardwareMap, "BLUE");
        }
        @Override
        public void loop(){
            frontGate12.loop();
        }
        @Override
        public void start(){
            frontGate12.start();
        }
    }
}
