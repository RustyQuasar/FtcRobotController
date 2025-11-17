package Commands;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Arrays;

import Subsystems.RTPAxon;
import Subsystems.Vision;
import Utilities.Constants;

public class SmartShooter {
    private final DcMotorEx leftShooter, rightShooter;
    private final Servo flipServo;
    private final DcMotorEx turretNeckMotor;
    private final CRServo transferServo, transferServo2;
    private final RTPAxon turretHead;
    private final int aimedTagID;
    Vision Vision;

    public SmartShooter(HardwareMap hardwareMap, Vision vision) {
        leftShooter = hardwareMap.get(DcMotorEx.class, Constants.ShooterConstants.leftShooter);
        rightShooter = hardwareMap.get(DcMotorEx.class, Constants.ShooterConstants.rightShooter);
        turretNeckMotor = hardwareMap.get(DcMotorEx.class, Constants.ShooterConstants.turretNeckMotor);
        AnalogInput turretHeadEncoder = hardwareMap.get(AnalogInput.class, Constants.ShooterConstants.turretHeadEncoder);
        CRServo turretHeadServo = hardwareMap.get(CRServo.class, Constants.ShooterConstants.turretHeadServo);

        turretHead = new RTPAxon(turretHeadServo, turretHeadEncoder);
        flipServo = hardwareMap.get(Servo.class, Constants.ShooterConstants.flipServo);
        leftShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        transferServo = hardwareMap.get(CRServo.class, Constants.IntakeConstants.transferServo);
        transferServo2 = hardwareMap.get(CRServo.class, Constants.IntakeConstants.transferServo2);

        if (Constants.TEAM.equals("RED")) {
            aimedTagID = 24;
        } else {
            aimedTagID = 20;
        }
        Vision = vision;
    }

    public void shoot(double power) {
        leftShooter.setVelocity(power);
        rightShooter.setPower(leftShooter.getPower());
    }

    public void aim() {
        turretHead.update();
        Vector2d targetPose;
        if (Arrays.equals(Constants.VisionConstants.colours, new String[] {"N", "N", "N"})){
            targetPose = Constants.OdometryConstants.targetPosMotif;
        } else if (Constants.TEAM.equals("RED")){
            targetPose = Constants.OdometryConstants.targetPosRed;
        } else {
            targetPose = Constants.OdometryConstants.targetPosBlue;
        }
        double detectedX;
        double distanceMeters;
        double botHeading = Math.toDegrees(Constants.OdometryConstants.fieldPos.heading.toDouble());
        if (botHeading < 0) {
            botHeading = 360 + botHeading;
        }
        double neckHeading= botHeading + ((double) turretNeckMotor.getCurrentPosition() / Constants.StudickaMotorMax) * 360 / Constants.ShooterConstants.turretNeckGearRatio;
        neckHeading -= (Math.round((neckHeading / 360) - 0.5)) * 360;
        neckHeading += Constants.DriveTrainConstants.controlHubOffset;
        double fv = Constants.OdometryConstants.fieldVels.linearVel.x * Math.cos(Math.toRadians(neckHeading));
        double sv = Constants.OdometryConstants.fieldVels.linearVel.y * Math.sin(Math.toRadians(neckHeading));
        boolean seeTarget = false;
        // Step through the list of detections and display info for each one.
        if (!Arrays.equals(Constants.VisionConstants.colours, new String[] {"N", "N", "N"})) {
            for (AprilTagDetection detection : Vision.getDetections()) {
                if (detection.id == aimedTagID) {
                    seeTarget = true;
                    // convert servo position to degrees (your existing mapping)
                    detectedX = detection.ftcPose.x;
                    // Convert range (inch) to meters consistently, and add centerOffset (inches) then convert:
                    distanceMeters = (detection.ftcPose.range + Constants.ShooterConstants.centerOffset);
                    double xOffset = detectedX - Constants.VisionConstants.resX / 2;
                    double angleToTurn = ((double) Constants.VisionConstants.FOV / Constants.VisionConstants.resX) * xOffset; // degrees;
                    if (Math.abs(angleToTurn + (double) turretNeckMotor.getCurrentPosition() / Constants.StudickaMotorMax * 360 * Constants.ShooterConstants.turretNeckGearRatio) > Constants.ShooterConstants.maxNeckAngle)
                        angleToTurn = 0;
                    // Update turret positions with corrected unit handling and corrected math
                    turretNeckMotor.setTargetPosition((int) (turretNeckMotor.getTargetPosition() + xTurn(angleToTurn, sv, distanceMeters)));
                    turretHead.setTargetRotation(turretHead.getTargetRotation() + yTurn(distanceMeters, fv));
                    double botX = targetPose.y - Math.abs(distanceMeters * Math.sin(Math.toRadians(neckHeading)));
                    double botY = targetPose.x - Math.signum(targetPose.x) * Math.abs(distanceMeters * Math.cos(Math.toRadians(neckHeading)));
                    Constants.OdometryConstants.fieldPos = new Pose2d(new Vector2d(botX, botY), Constants.OdometryConstants.fieldPos.heading);
                }
            }
        }
        if (!seeTarget){
            double xChange = (targetPose.x + 72) - (Constants.OdometryConstants.fieldPos.position.x + 72);
            double yChange = (targetPose.y + 72) - (Constants.OdometryConstants.fieldPos.position.y + 72);
            double distance = Math.sqrt(Math.pow(xChange, 2) + Math.pow(yChange, 2));
            double angleToTurn = Math.tan(Math.toRadians(yChange / xChange));
            if (Math.abs(angleToTurn + turretNeckMotor.getCurrentPosition() / Constants.StudickaMotorMax * 360 * Constants.ShooterConstants.turretNeckGearRatio) > Constants.ShooterConstants.maxNeckAngle) angleToTurn = 0;
            turretHead.setTargetRotation(turretHead.getTargetRotation() + yTurn(distance, fv));
            turretNeckMotor.setTargetPosition((int) (turretNeckMotor.getTargetPosition() + xTurn(angleToTurn - ((double) Constants.VisionConstants.resX / 2), sv, distance)));
        }
    }

    public double getShooterVelocity() {
        if (leftShooter.getPower() != 0) {
            return (leftShooter.getVelocity() * Constants.ShooterConstants.shooterGearRatio * Math.PI * Constants.ShooterConstants.flyWheelDiameter / Constants.GoBildaMotorMax);
        } else {
            return 0;
        }
    }

    public void transfer(boolean buttonPressed) {
        if (buttonPressed) {
            flipServo.setPosition(1);
            transferServo.setPower(0);
        } else {
            flipServo.setPosition(0);
            transferServo.setPower(1);
        }
        transferServo2.setPower(-transferServo.getPower());
    }

    private void setShooterVelocity(double R /* horizontal distance in meters */, double h /* height diff in meters */, double currentVelocity) {
        double g = 386.08858267717; //9.8m/s in inches
        double t = 1.0; // desired travel time (s), adjust if you want shorter/longer arcs

        // Required launch velocity components
        double vX = R / t;
        double vY = (h + 0.5 * g * t * t) / t;

        // Magnitude of required velocity
        double vRequired = Math.sqrt(vX * vX + vY * vY);

        // Extra velocity needed (beyond current)
        double neededLinearVelocity = vRequired - currentVelocity;
        if (neededLinearVelocity <= 0) {
            return; // already fast enough
        }

        // Convert to wheel/motor RPS
        double wheelCircum = Math.PI * Constants.ShooterConstants.flyWheelDiameter;
        double wheelRevsPerSec = neededLinearVelocity / wheelCircum;
        double motorRevsPerSec = wheelRevsPerSec * Constants.ShooterConstants.shooterGearRatio;

        // Normalize by default motor RPS
        double power = motorRevsPerSec / Constants.defaultDCRPS;

        shoot(power);
    }


    public void telemetry(Telemetry telemetry) {
        telemetry.addLine("Shooter");
        telemetry.addData("Left Shooter Power: ", leftShooter.getPower());
        telemetry.addData("Right Shooter Power: ", rightShooter.getPower());
        telemetry.addLine("Turret neck angle: " + turretNeckMotor.getCurrentPosition());
        telemetry.addLine("Turret head position: " + turretHead.getCurrentAngle());
        telemetry.update();
    }

    private double xTurn(double angleToTurnDeg, double velocity, double distance) {
        double leadAngleDeg = Math.toDegrees(Math.atan2(velocity, distance));
        double angleDiff =  (angleToTurnDeg - leadAngleDeg) / Constants.ShooterConstants.turretNeckGearRatio;
        if (angleDiff < 0){
            angleDiff = 360 + angleDiff;
        }
        return angleDiff * Constants.StudickaMotorMax / Constants.ShooterConstants.turretNeckGearRatio;
    }

    private double yTurn(double distance, double velocity) {
        // distance (m): horizontal distance to target
        // velocity (in/s): current shooter wheel linear velocity

        double heightDiffM = (48 - Constants.Sizes.robotHeight + Constants.Sizes.artifactRadius + 2);

        if (distance <= 0) {
            return 0.5;
        }

        // Ensure shooter is spun up for ~1s trajectory
        setShooterVelocity(distance, heightDiffM, velocity);

        double g = 9.8;
        double t = 1.0; // same travel time assumption

        // Velocity components required
        double vX = distance / t;
        double vY = (heightDiffM + 0.5 * g * Math.pow(t, 2)) / t;

        // Launch angle (radians → degrees)
        double angleRad = Math.atan2(vY, vX);
        double angleDeg = Math.toDegrees(angleRad);
        if (angleDeg < 0){
            angleDeg = 0;
        } else if (angleDeg > Constants.ShooterConstants.maxHeadAngle){
            angleDeg = Constants.ShooterConstants.maxHeadAngle;
        }

        return angleDeg / Constants.ShooterConstants.turretHeadGearRatio;
    }
}