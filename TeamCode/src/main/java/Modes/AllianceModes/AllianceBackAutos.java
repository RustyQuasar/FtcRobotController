package Modes.AllianceModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class AllianceBackAutos {
    @Autonomous(name = "Red Back 3", group = "Auto")
    public static class RedBack3 extends OpMode {
        Modes.Back3 back3 = new Modes.Back3();
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
    @Autonomous(name = "Blue Back 3", group = "Auto")
    public static class BlueBack3 extends OpMode {
        Modes.Back3 back3 = new Modes.Back3();
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
    @Autonomous(name = "Red Back 6", group = "Auto")
    public static class RedBack6 extends OpMode {
        Modes.Back6 back6 = new Modes.Back6();
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
    @Autonomous(name = "Blue Back 6", group = "Auto")
    public static class BlueBack6 extends OpMode {
        Modes.Back6 back6 = new Modes.Back6();
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
}
