package Commands;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
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

import Subsystems.Vision;
import Utilities.ConfigVariables;
import Utilities.Constants;

public class SmartShooter3 {
    private final DcMotorEx leftShooter, rightShooter;
    private final Servo flipServo, turretHead;
    private final DcMotorEx turretNeckMotor;
    private final CRServo transferServo, transferServo2;
    private final int aimedTagID;
    Vision Vision;
    double distance = 0;
    double xChange;
    double yChange;
    double shooterVel = 0;
    int targetNeckPos;
    double neckHeading;
    double actualTargetNeckPos;
    double angle;
    double headingTarget;
    double targetPos;
    public double offset;
    double limelightToShooterCenter = 5.72;
    double shooterToBotCenter = 1.39;
    public SmartShooter3(HardwareMap hardwareMap, Vision vision) {
        leftShooter = hardwareMap.get(DcMotorEx.class, Constants.ShooterConstants.leftShooter);
        rightShooter = hardwareMap.get(DcMotorEx.class, Constants.ShooterConstants.rightShooter);
        turretNeckMotor = hardwareMap.get(DcMotorEx.class, Constants.ShooterConstants.turretNeckMotor);
        turretHead = hardwareMap.get(Servo.class, Constants.ShooterConstants.turretHeadServo);
        flipServo = hardwareMap.get(Servo.class, Constants.ShooterConstants.flipServo);
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretNeckMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretNeckMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretNeckMotor.setTargetPosition(0);
        turretHead.setPosition(1);
        turretNeckMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        turretHead.setDirection(Servo.Direction.FORWARD);
        if (Constants.TEAM.equals("RED")) {
            aimedTagID = 24;
        } else {
            aimedTagID = 20;
        }
        Vision = vision;
        transferServo = hardwareMap.get(CRServo.class, Constants.IntakeConstants.transferServo);
        transferServo2 = hardwareMap.get(CRServo.class, Constants.IntakeConstants.transferServo2);
    }

    public void shoot(double vel) {
        leftShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(ConfigVariables.p, ConfigVariables.i, ConfigVariables.d, 0));
        leftShooter.setVelocity(vel);
        rightShooter.setPower(leftShooter.getPower());
    }
    public void turretHeadTester(boolean pressed){
        if (pressed) {turretHead.setPosition(0.2);} else {
            turretHead.setPosition(1);
        }
    }

    public void aim(boolean auto) {
        //if (Math.abs(Math.abs(turretNeckMotor.getCurrentPosition()) - Math.abs(turretNeckMotor.getTargetPosition())) > 5) {
            turretNeckMotor.setPower(0.3);
        //} else {
        //    turretNeckMotor.setPower(0);
        //}
        Vector2d targetPose;
        if (Arrays.equals(Constants.VisionConstants.colours, new String[] {"N", "N", "N"}) && false ){
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
        neckHeading = botHeading - ((double) turretNeckMotor.getCurrentPosition() / (Constants.GoBildaMotorMax * Constants.ShooterConstants.turretNeckGearRatio) * max);
        neckHeading -= (Math.floor(neckHeading / max) * max);
        double fv =
                Constants.OdometryConstants.fieldVels.linearVel.x * Math.cos(neckHeading);
                if (!Constants.OdometryConstants.directions[0]) fv *= -1;
                //0;
        if (Double.isNaN(fv)) fv = 0;
        double sv =
                //Constants.OdometryConstants.fieldVels.linearVel.y * Math.sin(neckHeading);
                //if (!Constants.OdometryConstants.directions[1]) sv *= -1;
                0;
        if (Double.isNaN(sv)) sv = 0;
        // Step through the list of detections and display info for each one.
        LLResult result = Vision.getDetections();
        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fr : fiducialResults) {
            if (fr.getFiducialId() == aimedTagID) {
                double neckDeltaHeading = neckHeading - botHeading + Math.PI/2;
                double[] limelightPos = new double[]{limelightToShooterCenter * Math.sin(neckDeltaHeading), limelightToShooterCenter * Math.cos(neckDeltaHeading)};
                double[] shooterPos = new double[]{shooterToBotCenter * Math.sin(botHeading), shooterToBotCenter * Math.cos(botHeading)};
                double[] totalOffsets = new double[]{shooterPos[0] + limelightPos[0], shooterPos[1] + limelightPos[1]};
                Constants.OdometryConstants.fieldPos = new Pose2d(Constants.OdometryConstants.fieldPos.position.x - totalOffsets[0], Constants.OdometryConstants.fieldPos.position.y - totalOffsets[1], Constants.OdometryConstants.fieldPos.heading.toDouble());
            }
        }
        //+72 because negatives suck
            xChange = (targetPose.x+72) - (Constants.OdometryConstants.fieldPos.position.x+72);
            yChange = (targetPose.y+72) - (Constants.OdometryConstants.fieldPos.position.y+72);
            //THEOREM OF PYTHAGORAS
            distance = Math.sqrt(Math.pow(xChange, 2) + Math.pow(yChange, 2));
            //TOA in the indian princess' name
            headingTarget = Math.atan2(xChange, yChange);
            double angleToTurn = Math.toDegrees(neckHeading + headingTarget);
           // if (Constants.TEAM.equals("BLUE")) {angleToTurn -= 5; }
            //else {angleToTurn -= 2; }
            aiming(distance, fv, sv, angleToTurn, auto);
    }
    public void manualOffset(boolean leftTrigger, boolean rightTrigger){
        if (leftTrigger) offset -= 3;
        if (rightTrigger) offset += 3;
    }
    public void transfer(boolean buttonPressed) {
        if (!buttonPressed || !(Math.abs(leftShooter.getVelocity() - shooterVel) < 80)) {
        //if (!buttonPressed) {
            flipServo.setPosition(0.13);
            transferServo.setPower(0);
        } else {
            flipServo.setPosition(0.25);
            transferServo.setPower(0.2);
        }
        transferServo2.setPower(-transferServo.getPower());
    }
    public void aiming(double distance, double frontV, double sideV, double angleToTurn, boolean auto) {
        //SO MUCH METH MATH THE CRACKHEADS ARE JEALOUS
        distance = Math.max(distance - 28, 0);
        double h = (38 - Constants.Sizes.robotHeight + Constants.Sizes.artifactRadius * 2 + 2) / 39.37; //2 is some buffer :P
        double g = -9.8;
        double distanceMeters = distance / 39.37;
        double z = Math.abs(h / (g * distanceMeters) - distanceMeters);
        double AOS = z / 2;
        //This is the root form of the parabola dw, y = -9.8(AOS-0)(AOS-z)
        double H = Math.abs(g * Math.pow(AOS, 2));
        double t = Math.sqrt(H / -g);
        distance -= Math.max(t * frontV, 0);
        if (Double.isNaN(distance)) distance = 0;
        angle = distance * 0.9;

        shooterVel = distance * 3.80334 + 1030.9556;
        if (distance > 75) shooterVel += 40;
        double totalTicks = Constants.ShooterConstants.turretNeckGearRatio * Constants.GoBildaMotorMax;
        targetNeckPos = (int) (turretNeckMotor.getCurrentPosition() + xTurn(angleToTurn, sideV, distance, t) + offset);
        targetNeckPos -= (int) (Math.floor(Math.abs(targetNeckPos / totalTicks)) * totalTicks * Math.signum(targetNeckPos));
        if (targetNeckPos > totalTicks / 2) targetNeckPos -= (int) totalTicks;
        if (targetNeckPos < -totalTicks / 2) targetNeckPos += (int) totalTicks;
        targetNeckPos -= 15;
        actualTargetNeckPos = targetNeckPos;
        if (Math.abs(targetNeckPos) > 1000) targetNeckPos = (int) (1000 * Math.signum(targetNeckPos));
        targetPos = (1 - Math.max(0, Math.min(0.75, angle / Constants.ShooterConstants.maxHeadAngle)));
        turretHead.setPosition(targetPos);
        if (auto) {
            turretNeckMotor.setTargetPosition(targetNeckPos);
        } else {
            turretNeckMotor.setTargetPosition(0);
        }
        shoot(shooterVel);
    }



    public void telemetry(Telemetry telemetry) {
        telemetry.addLine("Shooter");
        //telemetry.addData("Left Shooter Power: ", leftShooter.getPower());
        //telemetry.addData("Right Shooter Power: ", rightShooter.getPower());
        //telemetry.addData("Turret neck pos: ", turretNeckMotor.getCurrentPosition());
        //telemetry.addData("Turret heading: ", neckHeading);
        telemetry.addData("Turret head pos: ", targetPos);
        telemetry.addData("Turret neck target pos:", actualTargetNeckPos);
        telemetry.addData("Target neck heading: ", headingTarget);
        telemetry.addData("Distance: ", distance);
        //telemetry.addData("Target vel: ", shooterVel);
        //telemetry.addData("Shooter vel", leftShooter.getVelocity());
        telemetry.update();
    }
    public void manualNeckMotor(boolean leftBumper, boolean rightBumper){
        if (leftBumper && rightBumper) {turretNeckMotor.setTargetPosition(0); return;}
        if (leftBumper && Constants.TEAM.equals("RED")) {turretNeckMotor.setTargetPosition(950); return;}
        if (leftBumper && Constants.TEAM.equals("BLUE")) {turretNeckMotor.setTargetPosition(-950); return;}
        if (rightBumper) turretNeckMotor.setTargetPosition(0);
        //target pos +- 950
        //left bumper is offset pos
        //right bumper is straight forward (0)
    }
    private double xTurn(double angleToTurnDeg, double velocity, double distance, double time) {
        double leadAngleDeg = Math.toDegrees(Math.acos(velocity / (distance / time)));
        if (leadAngleDeg < 0) leadAngleDeg += 360;

        double angleDiff = angleToTurnDeg - leadAngleDeg;
        if (angleDiff < 0){
            angleDiff += 360;
        }
        return angleDiff * Constants.GoBildaMotorMax * Constants.ShooterConstants.turretNeckGearRatio / 360;
    }

}