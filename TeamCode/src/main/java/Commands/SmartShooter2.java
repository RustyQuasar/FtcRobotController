package Commands;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import Subsystems.RTPAxon;
import Utilities.Constants;

public class SmartShooter2 {
    private final DcMotorEx leftShooter, rightShooter;
    private final CRServo transferServo;
    private final DcMotorEx turretNeckMotor;
    private final RTPAxon turretHead;
    private final int aimedTagID;
    Vision Vision;

    public SmartShooter2(HardwareMap hardwareMap, Vision vision) {
        leftShooter = hardwareMap.get(DcMotorEx.class, Constants.ShooterConstants.leftShooter);
        rightShooter = hardwareMap.get(DcMotorEx.class, Constants.ShooterConstants.rightShooter);
        turretNeckMotor = hardwareMap.get(DcMotorEx.class, Constants.ShooterConstants.turretNeckMotor);
        AnalogInput turretHeadEncoder = hardwareMap.get(AnalogInput.class, Constants.ShooterConstants.turretHeadEncoder);
        CRServo turretHeadServo = hardwareMap.get(CRServo.class, Constants.ShooterConstants.turretHeadServo);

        turretHead = new RTPAxon(turretHeadServo, turretHeadEncoder);
        transferServo = hardwareMap.get(CRServo.class, Constants.ShooterConstants.transferServo);
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        if (Constants.TEAM.equals("RED")){
            targetPose = Constants.OdometryConstants.targetPosRed;
        } else {
            targetPose = Constants.OdometryConstants.targetPosBlue;
        }
        double detectedX;
        double distance;
        double botHeading = Constants.OdometryConstants.fieldPos.heading.toDouble();
        double max = 2 * Math.PI;
        if (botHeading < 0) {
            botHeading += max;
        }
        double neckHeading= botHeading + ((double) turretNeckMotor.getCurrentPosition() / Constants.StudickaMotorMax) * max / Constants.ShooterConstants.turretNeckGearRatio;
        neckHeading -= (Math.floor(neckHeading / max) * max);
        double fv = Constants.OdometryConstants.fieldVels.linearVel.x * Math.cos(neckHeading);
        double sv = Constants.OdometryConstants.fieldVels.linearVel.y * Math.sin(neckHeading);
        boolean seeTarget = false;
        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : Vision.getDetections()) {
            if (detection.id == aimedTagID) {
                seeTarget = true;
                detectedX = detection.ftcPose.x;
                distance = (detection.ftcPose.range + Constants.ShooterConstants.centerOffset);
                double angleOffset = Math.toDegrees(Math.acos(Constants.VisionConstants.inOffset / distance));
                double xOffset = detectedX - Constants.VisionConstants.resX / 2;
                double angleToTurn = ((double) Constants.VisionConstants.FOV / Constants.VisionConstants.resX) * xOffset + angleOffset; // degrees;
                if (Math.abs(angleToTurn + turretNeckMotor.getCurrentPosition() / Constants.StudickaMotorMax * 360 * Constants.ShooterConstants.turretNeckGearRatio) > Constants.ShooterConstants.maxNeckAngle) angleToTurn = 0;
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
            double xChange = (targetPose.x+72) - (Constants.OdometryConstants.fieldPos.position.x+72);
            double yChange = (targetPose.y+72) - (Constants.OdometryConstants.fieldPos.position.y+72);
            //THEOREM OF PYTHAGORAS
            distance = Math.sqrt(Math.pow(xChange, 2) + Math.pow(yChange, 2));
            //TOA in the indian princess' name
            double angleToTurn = Math.tan(Math.toRadians(yChange / xChange));
            if (Math.abs(angleToTurn + (double) turretNeckMotor.getCurrentPosition() / Constants.StudickaMotorMax * 360 * Constants.ShooterConstants.turretNeckGearRatio) > Constants.ShooterConstants.maxNeckAngle) angleToTurn = 0;
            aiming(distance, fv, sv, angleToTurn);
        }
    }

    public double getShooterVelocity() {
        //learn to read names, they mean stuff y'know
        if (leftShooter.getPower() != 0 ) {
            return (leftShooter.getVelocity() * Constants.ShooterConstants.shooterGearRatio * Math.PI * Constants.ShooterConstants.flyWheelDiameter / Constants.GoBildaMotorMax);
        } else {
            return 0;
        }
    }

    public void transfer(boolean buttonPressed) {
        //Fuckin transfers the balls what do you expect
        if (buttonPressed) {
            transferServo.setPower(1);
        } else {
            transferServo.setPower(0);
        }
    }
    public void aiming(double distance, double frontV, double sideV, double angleToTurn) {
        //SO MUCH METH MATH THE CRACKHEADS ARE JEALOUS
        double h = (48 - Constants.Sizes.robotHeight + Constants.Sizes.artifactRadius * 2 + 2); //2 is some buffer :P
        double g = -386.08858267717; //9.8m/s in inches
        double z = Math.abs(h / (g * distance) - distance);
        double AOS = z / 2;
        //This is the root form of the parabola dw, y = -9.8(AOS-0)(AOS-z)
        double H = Math.abs(g * Math.pow(AOS, 2));
        double t = Math.abs(Math.sqrt(2 * H / g));
        double vH = 2 * z / t;
        double vV = t * g;
        double angle = Math.toDegrees(Math.atan2(vV, vH));
        if (angle < 0){
            angle = 0;
            //Idk why this would be needed but, eh
        } else if (angle > Constants.ShooterConstants.maxHeadAngle){
            angle = Constants.ShooterConstants.maxHeadAngle;
            vV /= Math.sin(Math.toRadians(angle));
        }
        double shooterVel = Math.sqrt(Math.pow(vH, 2) + Math.pow(vV, 2)) / Constants.ShooterConstants.shooterGearRatio;
        turretHead.setTargetRotation(angle / Constants.ShooterConstants.turretHeadGearRatio);
        turretNeckMotor.setTargetPosition((int) (turretNeckMotor.getTargetPosition() + xTurn(angleToTurn, sideV, distance, t)));
        shoot(shooterVel - frontV);
    }



    public void telemetry(Telemetry telemetry) {
        telemetry.addLine("Shooter");
        telemetry.addData("Left Shooter Power: ", leftShooter.getPower());
        telemetry.addData("Right Shooter Power: ", rightShooter.getPower());
        telemetry.addLine("Turret neck angle: " + turretNeckMotor.getCurrentPosition());
        telemetry.addLine("Turret head position: " + turretHead.getCurrentAngle());
        telemetry.update();
    }

    private double xTurn(double angleToTurnDeg, double velocity, double distance, double time) {
        double leadAngleDeg = Math.toDegrees(Math.atan2(velocity, distance / time));
        double angleDiff =  (angleToTurnDeg - leadAngleDeg) / Constants.ShooterConstants.turretNeckGearRatio;
        if (angleDiff < 0){
            angleDiff += 360;
        }
        return angleDiff * Constants.StudickaMotorMax / Constants.ShooterConstants.turretNeckGearRatio / 360;
    }

}