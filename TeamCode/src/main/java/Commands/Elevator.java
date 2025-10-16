package Commands;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Utilities.Constants;

public class Elevator {
    double curentElevatorheight=0;
    DcMotor elevator;
    public Elevator(HardwareMap hardwareMap){
        elevator = hardwareMap.get(DcMotor.class, Constants.ElevatorConstants.elevator);
    }
    public void setHeight(double wantedHeight){
        double error = wantedHeight-curentElevatorheight;
        while(Math.abs(error)>0.1){
            error = wantedHeight-curentElevatorheight;
            double speed=Math.max(0.1,Math.min(error/12,1));
            elevator.setPower(speed);
        }
        curentElevatorheight=wantedHeight;
        elevator.setPower(0);

    }

}
