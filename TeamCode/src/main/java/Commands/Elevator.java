package Commands;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import Utilities.Constants;

public class Elevator {
    Servo elevator;
    boolean raised = false;
    public Elevator(HardwareMap hardwareMap){
        elevator = hardwareMap.get(Servo.class, Constants.ElevatorConstants.elevator);
    }
    public void switchState(){
        if (!raised){
            elevator.setPosition(1);
        } else {
            elevator.setPosition(0);
        }
    }

}
