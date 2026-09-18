package Commands;

import com.arcrobotics.ftclib.geometry.Vector2d;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import Subsystems.Vision;
import Utilities.Constants;

public class StraightShooter {
    private final DcMotorEx flywheelMotor1, flywheelMotor2;
    //private final DcMotor transfer;
    private final Servo turretHood;
    private double flywheelVel, flywheelPower, lastFlywheelPower, transferPower, lastTransferPower, hoodPos, distance;
    Flywheel flywheel;
    Vision vision;
    private Constants.FlywheelConstants.FlywheelState lastFlywheelState = Constants.FlywheelConstants.FlywheelState.SCORE;
    private Pose lastPos = Constants.OdometryConstants.fieldPos;
    private int offset, lastOffset, lastTarget = 1;
    private Vector2d targetPose;
    private final Vector2d redTarget1 = new Vector2d(58, 53), redTarget2 = new Vector2d(58, 91);
    private final Vector2d blueTarget1 = new Vector2d(84, 53), blueTarget2 = new Vector2d(84, 91);


    public StraightShooter(HardwareMap hardwareMap) {

        vision = new Vision(hardwareMap);


        flywheelMotor1 = hardwareMap.get(DcMotorEx.class, Constants.FlywheelConstants.flywheel1);
        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, Constants.FlywheelConstants.flywheel2);
        turretHood = hardwareMap.get(Servo.class, Constants.FlywheelConstants.turretHeadServo);
        //transfer = hardwareMap.get(DcMotor.class, Constants.FlywheelConstants.transfer);

        flywheelMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        flywheelMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        if (Constants.onRed) targetPose = redTarget1;
        else targetPose = blueTarget1;

        flywheel = new Flywheel();

        aim(Constants.FlywheelConstants.FlywheelState.SCORE);
    }

    public boolean aim(Constants.FlywheelConstants.FlywheelState flywheelState) {
        vision.updateAprilTags();
        updateVariables();
        updateTarget();

        boolean redidMath = flywheelState != lastFlywheelState;

        if (Constants.OdometryConstants.fieldPos.distance(lastPos) < 1 || offset != lastOffset || redidMath) {
            lastFlywheelState = flywheelState;
            double xChange = Constants.OdometryConstants.fieldPos.x() - targetPose.getX();
            double yChange = Constants.OdometryConstants.fieldPos.y() - targetPose.getY();
            distance = Math.sqrt(Math.pow(xChange, 2) + Math.pow(yChange, 2));
            /*
            double time = Math.sqrt(Math.pow((distance - 20) / 39.37, 2) / 9.8) * 2; //TODO: FIND A BETTER WAY TO GET TIME
            xChange += Constants.OdometryConstants.fieldVels[0] * time;
            yChange += Constants.OdometryConstants.fieldVels[1] * time;
            distance = Math.sqrt(Math.pow(xChange, 2) + Math.pow(yChange, 2));
             */

            hoodPos = flywheel.getHeadPositions(flywheelState, distance);
            lastOffset = offset;
            redidMath = true;
            lastPos = Constants.OdometryConstants.fieldPos;
        }

        if (transferPower != lastTransferPower) {
            //updateTransfer();
            lastTransferPower = transferPower;
        }

        flywheelPower = flywheel.getFlywheelPower(distance, flywheelVel, flywheelState);
        if (flywheelPower != lastFlywheelPower) {
            updateFlywheelHardware();
            lastFlywheelPower = flywheelPower;
        }

        return redidMath;
    }

    public void transfer(boolean transferring){
        if (transferring && flywheel.atTargetVel()) transferPower = 1;
        else transferPower = 0;
    }

    public void updateTarget() {
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

    private void updateVariables() {
        flywheelVel = flywheelMotor1.getVelocity();
    }

    public void updateFlywheelHardware() {
        flywheelMotor1.setPower(flywheelPower);
        flywheelMotor2.setPower(flywheelPower);
        turretHood.setPosition(hoodPos);
    }

    public void updateTransfer() {
        //transfer.setPower(transferPower);
    }

    public void chill() {
        flywheelMotor1.setPower(0);
        flywheelMotor2.setPower(0);
    }

}