package Commands;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.List;

import Subsystems.RTPAxon;
import Subsystems.Vision;
import Utilities.ConfigVariables;
import Utilities.Constants;

public class SmartShooter3 {
    private final DcMotorEx leftShooter, rightShooter;
    private final Servo flipServo;
    private final DcMotorEx turretNeckMotor;
    private final RTPAxon turretHead;
    private final CRServo transferServo, transferServo2;
    private final int aimedTagID;
    Vision Vision;
    double distance = 0;
    double xChange;
    double yChange;
    boolean aimed = false;
    boolean seeTarget = true;
    double shooterVel = 0;
    double t;
    double z;
    double fv;
    Pose2d cameraPos;
    int targetNeckPos;
    public SmartShooter3(HardwareMap hardwareMap, Vision vision) {
        leftShooter = hardwareMap.get(DcMotorEx.class, Constants.ShooterConstants.leftShooter);
        rightShooter = hardwareMap.get(DcMotorEx.class, Constants.ShooterConstants.rightShooter);
        turretNeckMotor = hardwareMap.get(DcMotorEx.class, Constants.ShooterConstants.turretNeckMotor);
        AnalogInput turretHeadEncoder = hardwareMap.get(AnalogInput.class, Constants.ShooterConstants.turretHeadEncoder);
        CRServo turretHeadServo = hardwareMap.get(CRServo.class, Constants.ShooterConstants.turretHeadServo);
        turretHeadServo.setDirection(DcMotorSimple.Direction.FORWARD);
        turretHead = new RTPAxon(turretHeadServo, turretHeadEncoder);
        flipServo = hardwareMap.get(Servo.class, Constants.ShooterConstants.flipServo);
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretNeckMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        turretNeckMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretNeckMotor.setTargetPosition(0);
        turretNeckMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        if (Constants.TEAM.equals("RED")) {
            aimedTagID = 24;
        } else {
            aimedTagID = 20;
        }
        Vision = vision;

        transferServo = hardwareMap.get(CRServo.class, Constants.IntakeConstants.transferServo);
        transferServo2 = hardwareMap.get(CRServo.class, Constants.IntakeConstants.transferServo2);
        turretHead.setTargetRotation(0);

    }

    public void shoot(double vel) {
        leftShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(ConfigVariables.p, ConfigVariables.i, ConfigVariables.d, 0));
        leftShooter.setVelocity(vel);
        rightShooter.setPower(leftShooter.getPower());
    }

    public void aim() {
        if (turretNeckMotor.getCurrentPosition() != turretNeckMotor.getTargetPosition()) {
            turretNeckMotor.setPower(0.5);
        } else {
            turretNeckMotor.setPower(0);
        }
        turretHead.update();
        Vector2d targetPose;
        if (Arrays.equals(Constants.VisionConstants.colours, new String[] {"N", "N", "N"})){
            targetPose = Constants.OdometryConstants.targetPosMotif;
        } else if (Constants.TEAM.equals("RED")){
            targetPose = Constants.OdometryConstants.targetPosRed;
            Constants.VisionConstants.pipeline = 1;
        } else {
            targetPose = Constants.OdometryConstants.targetPosBlue;
            Constants.VisionConstants.pipeline = 2;
        }
        double detectedX;
        double botHeading = Constants.OdometryConstants.fieldPos.heading.toDouble();
        double max = 2 * Math.PI;
        if (botHeading < 0) {
            botHeading += max;
        }
        double neckHeading= botHeading + ((double) turretNeckMotor.getCurrentPosition() / Constants.StudickaMotorMax) * max / Constants.ShooterConstants.turretNeckGearRatio;
        neckHeading -= (Math.floor(neckHeading / max) * max);
        fv = Constants.OdometryConstants.fieldVels.linearVel.x * Math.cos(neckHeading);
        if (Double.isNaN(fv)) fv = 0;
        double sv = Constants.OdometryConstants.fieldVels.linearVel.y * Math.sin(neckHeading);
        if (Double.isNaN(sv)) sv = 0;
        seeTarget = false;
        // Step through the list of detections and display info for each one.
        LLResult result = Vision.getDetections();
        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fr : fiducialResults) {
            if (fr.getFiducialId() == aimedTagID) {
                seeTarget = true;
                detectedX = fr.getTargetXPixels();
                distance = (30 - Constants.Sizes.robotHeight) / Math.tan(Math.toRadians(result.getTy()) + Constants.VisionConstants.cameraAngle);
                double angleOffset = Math.toDegrees(Math.acos(Constants.VisionConstants.inOffset / distance));
                double xOffset = detectedX - Constants.VisionConstants.resX / 2;
                double angleToTurn = ((double) Constants.VisionConstants.FOV / Constants.VisionConstants.resX) * xOffset + angleOffset; // degrees;
                if (Math.abs(angleToTurn + turretNeckMotor.getCurrentPosition() / Constants.StudickaMotorMax * 360 / Constants.ShooterConstants.turretNeckGearRatio) > Constants.ShooterConstants.maxNeckAngle) angleToTurn = 0;
                aiming(distance, fv, sv, angleToTurn);
                //x and y gotta be swapped cuz roadrunner swaps em
                double botX = targetPose.x - Math.abs(distance * Math.sin(neckHeading));
                double botY = targetPose.y - Math.signum(targetPose.y) * Math.abs(distance * Math.cos(neckHeading));
                //Cam values update pos
                Constants.OdometryConstants.fieldPos = new Pose2d(new Vector2d(botX, botY), Constants.OdometryConstants.fieldPos.heading);
            }
        }
        if (!seeTarget){
            //+72 because negatives suck
            xChange = (targetPose.x+72) - (Constants.OdometryConstants.fieldPos.position.x+72);
            yChange = (targetPose.y+72) - (Constants.OdometryConstants.fieldPos.position.y+72);
            //THEOREM OF PYTHAGORAS
            distance = Math.sqrt(Math.pow(xChange, 2) + Math.pow(yChange, 2));
            //TOA in the indian princess' name
            double angleToTurn = Math.tan(Math.toRadians(yChange / xChange));
            if (Math.abs(angleToTurn + (double) turretNeckMotor.getCurrentPosition() / Constants.StudickaMotorMax * 360 / Constants.ShooterConstants.turretNeckGearRatio) > Constants.ShooterConstants.maxNeckAngle) angleToTurn = 0;
            aiming(distance, fv, sv, angleToTurn);
        }
        aimed = true;
    }

    public void transfer(boolean buttonPressed) {
        //if (!buttonPressed || !(Math.abs(getShooterVelocity() - shooterVel - fv) < 204)) {
        if (!buttonPressed) {
            flipServo.setPosition(0);
            transferServo.setPower(0);
        } else {
            flipServo.setPosition(1);
            transferServo.setPower(-1);
        }
        transferServo2.setPower(-transferServo.getPower());
    }
    public void aiming(double distance, double frontV, double sideV, double angleToTurn) {
        //SO MUCH METH MATH THE CRACKHEADS ARE JEALOUS
        distance = Math.max(distance - 24, 0);
        double h = (38 - Constants.Sizes.robotHeight + Constants.Sizes.artifactRadius * 2 + 2) / 39.37; //2 is some buffer :P
        double g = -9.8; //9.8m/s in inches
        double distanceMeters = distance / 39.37;
        z = Math.abs(h / (g * distanceMeters) - distanceMeters);
        double AOS = z / 2;
        //This is the root form of the parabola dw, y = -9.8(AOS-0)(AOS-z)
        double H = Math.abs(g * AOS * AOS);
        t = Math.abs(Math.sqrt(2 * H / -g));
        distance -= t * frontV;
        double angle = distance * 0.3;
        if (angle < 0){
            angle = 0;
            //Idk why this would be needed but, eh
        } else if (angle > Constants.ShooterConstants.maxHeadAngle){
            angle = Constants.ShooterConstants.maxHeadAngle;
        }
        shooterVel = distance * ((double) 450 / (120 - 24)) + 1000;
        turretHead.setTargetRotation(angle / Constants.ShooterConstants.turretHeadGearRatio);
        targetNeckPos = (int) (turretNeckMotor.getCurrentPosition() + xTurn(angleToTurn, sideV, distance, t));
        turretNeckMotor.setTargetPosition(targetNeckPos);
        shoot(shooterVel);
    }



    public void telemetry(Telemetry telemetry) {
        telemetry.addLine("Shooter");
        telemetry.addData("Left Shooter Power: ", leftShooter.getPower());
        telemetry.addData("Right Shooter Power: ", rightShooter.getPower());
        telemetry.addData("Turret neck pos: ", turretNeckMotor.getCurrentPosition());
        telemetry.addData("Turret head angle: ", turretHead.getCurrentAngle());
        telemetry.addData("Target head angle: ", turretHead.getTargetRotation());
        telemetry.addData("Turret neck target pos:", targetNeckPos);
        telemetry.addData("Distance: ", distance);
        //telemetry.addData("aimed: ", aimed);
        //telemetry.addData("sees april tag:", seeTarget);
        //telemetry.addData("Current pos:", Constants.OdometryConstants.fieldPos.position);
        //telemetry.addData("Current heading:", Constants.OdometryConstants.fieldPos.heading);
        telemetry.addData("Target vel: ", shooterVel);
        //telemetry.addData("Time: ", t);
        telemetry.addData("Shooter vel", leftShooter.getVelocity());
        telemetry.update();
    }

    private double xTurn(double angleToTurnDeg, double velocity, double distance, double time) {
        double leadAngleDeg = Math.toDegrees(Math.atan2(velocity, distance / time));
        double angleDiff = angleToTurnDeg - leadAngleDeg;
        if (angleDiff < 0){
            angleDiff += 360;
        }
        return angleDiff * Constants.StudickaMotorMax * Constants.ShooterConstants.turretNeckGearRatio / 360;
        //return (angleDiff /Constants.StudickaMotorMax) * ((double) Constants.StudickaMotorMax / 360);

    }

}