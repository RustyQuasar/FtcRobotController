package Modes.AllianceModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import Modes.Autos.Back3;
import Modes.Autos.Back6;
import Modes.Autos.Back9;

public class AllianceBackAutos {
    @Autonomous(name = "Red Far 3", group = "Far Auto")
    public static class RedBack3 extends OpMode {
        Back3 back3 = new Back3();
        @Override
        public void init(){
            back3.init(hardwareMap, "RED");
        }
        @Override
        public void loop(){
            back3.loop();
        }
        @Override
        public void init_loop(){
            back3.init_loop();
        }
    }
    @Autonomous(name = "Blue Far 3", group = "Far Auto")
    public static class BlueBack3 extends OpMode {
        Back3 back3 = new Back3();
        @Override
        public void init(){
            back3.init(hardwareMap, "BLUE");
        }
        @Override
        public void loop(){
            back3.loop();
        }
        @Override
        public void init_loop(){
            back3.init_loop();
        }
    }
    @Autonomous(name = "Red Far 6", group = "Far Auto")
    public static class RedBack6 extends OpMode {
        Back6 back6 = new Back6();
        @Override
        public void init(){
            back6.init(hardwareMap, "RED");
        }
        @Override
        public void loop(){
            back6.loop();
        }
        @Override
        public void init_loop(){
            back6.init_loop();
        }
    }
    @Autonomous(name = "Blue Far 6", group = "Far Auto")
    public static class BlueBack6 extends OpMode {
        Back6 back6 = new Back6();
        @Override
        public void init(){
            back6.init(hardwareMap, "BLUE");
        }
        @Override
        public void loop(){
            back6.loop();
        }
        @Override
        public void init_loop(){
            back6.init_loop();
        }
    }
    @Autonomous(name = "Red Far 9", group = "Far Auto")
    public static class RedBack9 extends OpMode {
        Back9 back9 = new Back9();
        @Override
        public void init(){
            back9.init(hardwareMap, "RED");
        }
        @Override
        public void loop(){
            back9.loop();
        }
        @Override
        public void init_loop(){
            back9.init_loop();
        }
    }
    @Autonomous(name = "Blue Far 9", group = "Far Auto")
    public static class BlueBack9 extends OpMode {
        Back9 back9 = new Back9();
        @Override
        public void init(){
            back9.init(hardwareMap, "BLUE");
        }
        @Override
        public void loop(){
            back9.loop();
        }
        @Override
        public void init_loop(){
            back9.init_loop();
        }
    }
}
