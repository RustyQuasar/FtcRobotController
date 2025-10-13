package Commands;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Utilities.Constants;

public class Elevator {
    double curentElevatorhight=0;
    DcMotor rightElevator,leftElevator;
    public Elevator(HardwareMap hardwareMap){
        leftElevator = hardwareMap.get(DcMotor.class, Constants.ElevatorConstants.elevatorLeft);
        rightElevator = hardwareMap.get(DcMotor.class, Constants.ElevatorConstants.elevatorRight);
    }
    public void setHeight(double wantedHeight){
        double error = wantedHeight-curentElevatorhight;
        while(Math.abs(error)>0.1){
            error = wantedHeight-curentElevatorhight;
            double speed=Math.max(0.1,Math.min(error/12,1));
            leftElevator.setPower(speed);
            rightElevator.setPower(speed);
        }
        curentElevatorhight=wantedHeight;
        leftElevator.setPower(0);
        rightElevator.setPower(0);

    }

}
