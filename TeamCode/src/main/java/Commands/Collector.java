package Commands;

import com.pedropathing.ivy.Command;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import Utilities.Constants;

public class Collector {
    private final Servo claw, arm;
    private final Servo elevator;
    private boolean collecting = false, autoCollecting = false;
    private long lastLoop = 0, lastCommand = 0;
    private double coneArmPos = 0.5, coneElevatorPos = 0.1, gemArmPos = 0.3, gemElevatorPos = 0.1, diamondArmPos = 0.9, diamondElevatorPos = 1;
    private double lastPos = 0;
    private ColorSensor colourSensor;
    public Collector(HardwareMap hardwareMap) {
        elevator = hardwareMap.get(Servo.class, Constants.CollectorConstants.elevator);
        arm = hardwareMap.get(Servo.class, Constants.CollectorConstants.arm);
        claw = hardwareMap.get(Servo.class, Constants.CollectorConstants.claw);
        colourSensor = hardwareMap.get(RevColorSensorV3.class, Constants.CollectorConstants.colourSensor);
        elevator.setPosition(1);
        //arm.setPosition(0);
        //claw.setPosition(0);
        lastLoop = System.currentTimeMillis();
        lastPos = claw.getPosition();
        lastCommand = System.currentTimeMillis();
    }
    public void collect(boolean swap){
        if (swap) {
            collecting = !collecting;
            if (collecting) claw.setPosition(1);
            else claw.setPosition(0);
        }
        if (atRest()) claw.setPosition(claw.getPosition());
    }
    public void armControl(double x, double y) {
        if (Math.sqrt(x*x + y*y) < 0.5) return;
        double armQuadrantPos = Math.toDegrees(Math.atan2(y, x));
        if (x < 0 && y < 0) armQuadrantPos = 270 - armQuadrantPos;
        else if (y < 0) armQuadrantPos += 180;
        arm.setPosition(Math.max(Math.min(armQuadrantPos / Constants.CollectorConstants.armRatio, 0), 1));
    }
    public void raiseControl(boolean raise, boolean lower ) {
        if (raise && !lower) elevator.setPosition(elevator.getPosition() + 0.01);
        if (lower && !raise) elevator.setPosition(elevator.getPosition() - 0.01);
    }

    public boolean atRest() {
        return Math.abs(claw.getPosition() - lastPos) / (System.currentTimeMillis() - lastLoop) < 0.1 && lastCommand - System.currentTimeMillis() > 500;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Arm position: ", arm.getPosition());
        telemetry.addData("Claw position: ", claw.getPosition());
        telemetry.addData("Elevator position: ", elevator.getPosition());
        telemetry.addData("Collecting: ", collecting);
        telemetry.addData("Colour sensor: ", colourSensor.red() + " " +colourSensor.green() + " " + colourSensor.blue());
    }

    public Command pickupCone(){
        return Command.build()
                .setStart(() -> {
                    claw.setPosition(0);
                    autoCollecting = false;
                })
                .setStart(() -> claw.setPosition(0))
                .setExecute(() -> {
                    arm.setPosition(coneArmPos);
                    elevator.setPosition(coneElevatorPos);
                })
                .setDone(() -> {
                        if (Math.abs(elevator.getPosition() - coneElevatorPos) < 0.1 && Math.abs(arm.getPosition() - coneArmPos) < 0.1) {
                            collect(!autoCollecting);
                            autoCollecting = true;
                            atRest();
                        }
                    return Math.abs(elevator.getPosition() - coneElevatorPos) < 0.1 && Math.abs(arm.getPosition() - coneArmPos) < 0.1 && atRest();
                });
    }
    
    public Command pickupGem() {
        return Command.build()
                .setStart(() -> {
                    claw.setPosition(0);
                    autoCollecting = false;
                })
                .setExecute(() -> {
                    arm.setPosition(gemArmPos);
                    elevator.setPosition(gemElevatorPos);
                })
                .setDone(() -> {
                    if (Math.abs(elevator.getPosition() - gemElevatorPos) < 0.1 && Math.abs(arm.getPosition() - gemArmPos) < 0.1) {
                        collect(!autoCollecting);
                        autoCollecting = true;
                    }
                    return Math.abs(elevator.getPosition() - gemElevatorPos) < 0.1 && Math.abs(arm.getPosition() - gemArmPos) < 0.1 && atRest();
                });
    }
    
    public Command pickupDiamond() {
        return Command.build()
                .setStart(() -> {
                    claw.setPosition(0);
                    autoCollecting = false;
                })
                .setExecute(() -> {
                    arm.setPosition(diamondArmPos);
                    elevator.setPosition(diamondElevatorPos);
                })
                .setDone(() -> {
                    if (Math.abs(elevator.getPosition() - diamondElevatorPos) < 0.1 && Math.abs(arm.getPosition() - diamondArmPos) < 0.1) {
                        collect(!autoCollecting);
                        autoCollecting = true;
                    }
                    return Math.abs(elevator.getPosition() - diamondElevatorPos) < 0.1 && Math.abs(arm.getPosition() - diamondArmPos) < 0.1 && atRest();
                });
    }
    public Command setNeutral(){
        return Command.build()
                .setExecute(() -> {
                    arm.setPosition(0.5);
                    elevator.setPosition(0.5);
                });
    }
    public boolean seesPiece(){
        return colourSensor.red() > 1000 || colourSensor.green() > 1000;
    }
}
