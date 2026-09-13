package Commands;

import com.arcrobotics.ftclib.geometry.Vector2d;
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
    private double neckCurrentPos, neckTargetPos, neckPower, headPos;
    Flywheel flywheel;
    Turret turret;
    Vision vision;
    private Pose lastPos = Constants.OdometryConstants.fieldPos;
    private int offset, lastOffset, lastTarget = 1;
    double shooterToBotCenter = 1.541;
    private Vector2d targetPose;
    private final Vector2d redTarget1 = new Vector2d(0, 0), redTarget2 = new Vector2d(0, 0);
    private final Vector2d blueTarget1 = new Vector2d(0, 0), blueTarget2 = new Vector2d(0, 0);


    public Shooter(HardwareMap hardwareMap) {

        vision = new Vision(hardwareMap);

        turretNeckMotor = hardwareMap.get(DcMotorEx.class, Constants.TurretConstants.turretNeckMotor);
        turretNeckMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftShooter = hardwareMap.get(DcMotorEx.class, Constants.FlywheelConstants.leftShooter);
        rightShooter = hardwareMap.get(DcMotorEx.class, Constants.FlywheelConstants.rightShooter);
        turretHead = hardwareMap.get(Servo.class, Constants.FlywheelConstants.turretHeadServo);

        leftShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        turretNeckMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretHead.setDirection(Servo.Direction.FORWARD);

        if (Constants.onRed) targetPose = redTarget1;
        else targetPose = blueTarget1;

        flywheel = new Flywheel();
        turret = new Turret(hardwareMap);

        aim(Constants.FlywheelConstants.FlywheelState.SCORE, Constants.TurretConstants.TurretState.AUTO, 0, 0);
    }

    public boolean aim(Constants.FlywheelConstants.FlywheelState flywheelState, Constants.TurretConstants.TurretState turretState, double manualX, double manualY) {
        vision.updateAprilTags();
        updateVariables();
        updateTarget();

        boolean redidMath = false;
        if (Constants.OdometryConstants.fieldPos.roughlyEquals(lastPos, 1) || offset != lastOffset) {
            double xChange = Constants.OdometryConstants.fieldPos.getX() - targetPose.getX();
            double yChange = Constants.OdometryConstants.fieldPos.getY() - targetPose.getY();
            double distance = Math.sqrt(Math.pow(xChange, 2) + Math.pow(yChange, 2));
            double time = Math.sqrt(Math.pow((distance - 20) / 39.37, 2) / 9.8) * 2; //TODO: FIND A BETTER WAY TO GET TIME
            xChange += Constants.OdometryConstants.fieldVels[0] * time;
            yChange += Constants.OdometryConstants.fieldVels[1] * time;
            distance = Math.sqrt(Math.pow(xChange, 2) + Math.pow(yChange, 2));

            flywheelPower = flywheel.getFlywheelPower(distance, flywheelVel, flywheelState);
            headPos = flywheel.getHeadPositions(flywheelState, distance);

            if (turretState == Constants.TurretConstants.TurretState.AUTO)
                neckTargetPos = turret.aim(xChange, yChange, neckCurrentPos, offset);
            else if (turretState == Constants.TurretConstants.TurretState.MANUAL)
                neckTargetPos = turret.aim(manualX, manualY, neckCurrentPos, offset);
            else neckTargetPos = 0;

            lastOffset = offset;
            redidMath = true;
            lastPos = Constants.OdometryConstants.fieldPos;
        }
        neckPower = turret.calculateNeckPower(neckTargetPos, neckCurrentPos);
        return redidMath;
    }
    private void updateVariables() {
        neckCurrentPos = turretNeckMotor.getCurrentPosition();
        flywheelVel = leftShooter.getVelocity();
    }

    public void updateFlywheelHardware() {
        leftShooter.setPower(flywheelPower);
        rightShooter.setPower(flywheelPower);
        turretHead.setPosition(headPos);
    }

    public void updateTurretHardware() {
        turretNeckMotor.setPower(neckPower);
    }

    public Pose getVisionPos(){
        return vision.getPose(turret.getNeckHeading());
    }

    public void manualOffset(boolean leftTrigger, boolean rightTrigger) {
        if (leftTrigger) offset -= 4;
        if (rightTrigger) offset += 4;
    }

    public void updateTarget(){
        if (vision.tiltedSide() == lastTarget) return;
        if (lastTarget == 1) {
            if (Constants.onRed) targetPose = redTarget2;
            else targetPose = blueTarget2;
            lastTarget = 2;
        } else {
            if (Constants.onRed) targetPose = redTarget1;
            else targetPose = blueTarget1;
            lastTarget = 1;
        }
    }

    public void target2() {
        if (Constants.onRed) targetPose = redTarget2;
        else targetPose = blueTarget2;
    }

    public void chill() {
        leftShooter.setPower(0);
        rightShooter.setPower(0);
        turretNeckMotor.setPower(0);
    }

}