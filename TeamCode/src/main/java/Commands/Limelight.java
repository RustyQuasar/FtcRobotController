package Commands;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Subsystems.Vision;

public class Limelight {
    Vision vision;
       Telemetry telemetry;

    public Limelight(HardwareMap hardwareMap, Telemetry telemetry){
        Vision vision= new Vision(hardwareMap,telemetry);
        telemetry = this.telemetry;
    }
    public void setPipeline(String pipe){
        switch (pipe)
        {
            case("blue"):
              vision.setCurrentPipeline(1);
            case("red"):
              vision.setCurrentPipeline(2);
            case("aprilTag"):
                vision.setCurrentPipeline(0);

        }
    }

    public void runVision(boolean opState){
         while(opState){
             vision.updateAprilTags();
             vision.telemetry(telemetry);
             System.out.println("I have no mouth but I must scream");
         }

    }

}
