package Commands;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;

import Subsystems.Vision;
import Utilities.Constants;

public class Limelight {
    Vision vision;
       Telemetry telemetry;

    public Limelight(HardwareMap hardwareMap, Telemetry telemetry){
        Vision vision= new Vision(hardwareMap,telemetry);
        telemetry = this.telemetry;
    }
    public void setPipeline(int pipe){
        switch (pipe)
        {
            case(1):
              vision.setCurrentPipeline(1);
            case(2):
              vision.setCurrentPipeline(2);
            case(0):
                vision.setCurrentPipeline(0);

        }
    }

    public void runVision(boolean opState){
         while(opState){
             if(Constants.VisionConstants.colours[1]==null){
                 Constants.VisionConstants.colours= new String[]{Arrays.toString(vision.setColours())};
             }
             vision.updateAprilTags();
             vision.telemetry(telemetry);
         }

    }

}
