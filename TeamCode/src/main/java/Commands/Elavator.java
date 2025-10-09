package Commands;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Utilities.Constants;

public class Elavator {
    double curentElevatorhight=0;
    DcMotor rightElevator,leftElevator;
    public void Elevator(HardwareMap hardwareMap){
        leftElevator = hardwareMap.get(DcMotor.class, Constants.ElevatorConstants.elevatorLeft);
        rightElevator = hardwareMap.get(DcMotor.class, Constants.ElevatorConstants.elevatorRight);
    }
    public void setHight(double wantedHight){
        double error = wantedHight-curentElevatorhight;
        while(Math.abs(error)>0.1){
            error = wantedHight-curentElevatorhight;
            double speed=Math.max(0.1,Math.min(error/12,1));
            leftElevator.setPower(speed);
            rightElevator.setPower(speed);
        }
        curentElevatorhight=wantedHight;
        leftElevator.setPower(0);
        rightElevator.setPower(0);

    }

}
