package Commands;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Elevator {
    private final Servo elevator;
    private boolean raised = false;
    public Elevator(HardwareMap hardwareMap) {
        elevator = hardwareMap.get(Servo.class, "elevator");
    }
    public void control(boolean leftBumper, boolean rightBumper) {
        double currentPos = elevator.getPosition();
        if (leftBumper) currentPos += 0.2;
        if (rightBumper) currentPos -= 0.2;
        elevator.setPosition(currentPos);
    }

    public void swapState(){
        raised = !raised;
        if (raised) elevator.setPosition(1);
        else elevator.setPosition(0);
    }
}
