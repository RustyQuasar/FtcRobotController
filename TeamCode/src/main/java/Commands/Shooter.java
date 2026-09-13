package Commands;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import Subsystems.Vision;
import Utilities.Constants;

public class Shooter {
    private final DcMotorEx turretNeckMotor;
    private final DcMotorEx leftShooter, rightShooter;
    private final Servo turretHead;
    private double flywheelVel, flywheelPower;
    private double neckCurrentPos, neckTargetPos, neckPower;
    private Vision vision;
    Flywheel flywheel;
    Turret turret;
    private Pose lastUpdate = Constants.OdometryConstants.fieldPos;
    private int offset, lastOffset;
    double shooterToBotCenter = 1.541;
    private final Pose targetPose;
    private final Pose redTarget = new Pose(0, 0, Constants.heading(0)), blueTarget = new Pose(0, 0, Constants.heading(0));

    public Shooter(HardwareMap hardwareMap) {
        vision = new Vision(hardwareMap);
        turretNeckMotor = hardwareMap.get(DcMotorEx.class, Constants.TurretConstants.turretNeckMotor);
        turretNeckMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftShooter = hardwareMap.get(DcMotorEx.class, Constants.ShooterConstants.leftShooter);
        rightShooter = hardwareMap.get(DcMotorEx.class, Constants.ShooterConstants.rightShooter);
        turretHead = hardwareMap.get(Servo.class, Constants.ShooterConstants.turretHeadServo);
        leftShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretNeckMotor.setTargetPosition(0);
        turretHead.setPosition(1);
        turretNeckMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        turretHead.setDirection(Servo.Direction.FORWARD);
        if (Constants.onRed) targetPose = redTarget;
        else targetPose = blueTarget;

        flywheel = new Flywheel();
        turret = new Turret(vision);

        aim(true);

    }

    public void aim(boolean autoAim) {
        if (Constants.OdometryConstants.fieldPos.roughlyEquals(lastUpdate, 1)) return;
        double xChange = Constants.OdometryConstants.fieldPos.getX() - targetPose.getX();
        double yChange = Constants.OdometryConstants.fieldPos.getY() - targetPose.getY();
        double distance = Math.sqrt(Math.pow(xChange, 2) + Math.pow(yChange, 2));
        double time = Math.sqrt(Math.pow((distance - 20) / 39.37, 2) / 9.8) * 2; //TODO: FIND A BETTER WAY TO GET TIME
        xChange += Constants.OdometryConstants.fieldVels[0] * time;
        yChange += Constants.OdometryConstants.fieldVels[1] * time;
        distance = Math.sqrt(Math.pow(xChange, 2) + Math.pow(yChange, 2));

        flywheelPower = flywheel.calculateVelocity(distance, flywheelVel);
        if (autoAim) neckTargetPos = turret.aim(xChange, yChange, neckCurrentPos, offset);
        else neckTargetPos = 0;

        neckPower = turret.calculateNeckPower(neckTargetPos, neckCurrentPos);
    }

    public void turretHeadTester(boolean pressed){
        if (pressed) turretHead.setPosition(0.01);
        else turretHead.setPosition(1);
    }

    public void chill(){
        leftShooter.setPower(0);
        rightShooter.setPower(0);
        turretNeckMotor.setPower(0);
    }

    public void updateVariables(){
        neckCurrentPos = turretNeckMotor.getCurrentPosition();
        flywheelVel = leftShooter.getVelocity();
    }

    public void manualOffset(boolean leftTrigger, boolean rightTrigger){
        if (leftTrigger) offset -= 4;
        if (rightTrigger) offset += 4;
    }

}