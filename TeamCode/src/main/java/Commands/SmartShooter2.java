package Commands;


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
        leftShooter = hardwareMap.get(DcMotorEx.class, Constants.ShooterConstants.leftShooter0);
        rightShooter = hardwareMap.get(DcMotorEx.class, Constants.ShooterConstants.rightShooter1);
        turretNeckMotor = hardwareMap.get(DcMotorEx.class, Constants.ShooterConstants.turretNeckMotor);
        AnalogInput turretHeadEncoder = hardwareMap.get(AnalogInput.class, Constants.ShooterConstants.turretHeadEncoder);
        CRServo turretHeadServo = hardwareMap.get(CRServo.class, Constants.ShooterConstants.turretHeadServo);

        turretHead = new RTPAxon(turretHeadServo, turretHeadEncoder);
        transferServo = hardwareMap.get(CRServo.class, Constants.ShooterConstants.transferServo);
        leftShooter.setDirection(DcMotor.Direction.REVERSE);
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (Constants.TEAM.equals("RED")) {
            aimedTagID = 24;
        } else {
            aimedTagID = 20;
        }
        Vision = vision;
    }

    public void shoot(double power) {
        leftShooter.setPower(power);
        rightShooter.setPower(power);
    }

    public void aim() {
        turretHead.update();
        double detectedX;
        double distanceMeters;
        double botHeading = Math.toDegrees(Constants.OdometryConstants.fieldPos.heading.toDouble());
        if (botHeading < 0) {
            botHeading = 360 + botHeading;
        }
        double neckHeading= botHeading + ((double) turretNeckMotor.getCurrentPosition() / Constants.StudickaMotorMax) * 360 / Constants.ShooterConstants.turretNeckGearRatio;
        neckHeading -= (Math.round((neckHeading / 360) - 0.5)) * 360;
        neckHeading += Constants.DriveTrainConstants.controlHubOffset;
        double fv = Constants.OdometryConstants.fieldVels.linearVel.x * Math.cos(Math.toDegrees(neckHeading));
        double sv = Constants.OdometryConstants.fieldVels.linearVel.y * Math.sin(Math.toDegrees(neckHeading));
        boolean seeTarget = false;
        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : Vision.getDetections()) {
            if (detection.id == aimedTagID) {
                seeTarget = true;
                detectedX = detection.ftcPose.x;
                distanceMeters = (detection.ftcPose.range + Constants.ShooterConstants.centerOffset);
                double xOffset = detectedX - Constants.VisionConstants.resX / 2;
                double angleToTurn = ((double) Constants.VisionConstants.FOV / Constants.VisionConstants.resX) * xOffset; // degrees;
                turretNeckMotor.setTargetPosition((int) (turretNeckMotor.getTargetPosition() + xTurn(angleToTurn, sv, distanceMeters)));
                aiming(distanceMeters, fv);
            }
        }
        if (!seeTarget){
            Vector2d targetPose;
            if (Constants.TEAM.equals("RED")){
                targetPose = Constants.OdometryConstants.targetPosRed;
            } else {
                targetPose = Constants.OdometryConstants.targetPosBlue;
            }
            double xChange = targetPose.x - Constants.OdometryConstants.fieldPos.position.x;
            double yChange = targetPose.y - Constants.OdometryConstants.fieldPos.position.y;
            double distance = Math.sqrt(Math.pow(xChange, 2) + Math.pow(yChange, 2));
            double angleToTurn = Math.tan(Math.toRadians(yChange / xChange));
            turretNeckMotor.setTargetPosition((int) (turretNeckMotor.getTargetPosition() + xTurn(angleToTurn, sv, distance)));
            aiming(distance, fv);
        }
    }

    public double getShooterVelocity() {
        if (leftShooter.getPower() != 0 && rightShooter.getPower() != 0) {
            return (leftShooter.getVelocity() * Constants.ShooterConstants.shooterGearRatio * Math.PI * Constants.ShooterConstants.flyWheelDiameter / Constants.GoBildaMotorMax + (rightShooter.getVelocity() * Constants.ShooterConstants.shooterGearRatio * Math.PI * Constants.ShooterConstants.flyWheelDiameter / Constants.GoBildaMotorMax) / 2);
        } else {
            return 0;
        }
    }

    public void transfer(boolean buttonPressed) {
        if (buttonPressed) {
            transferServo.setPower(1);
        } else {
            transferServo.setPower(0);
        }
    }
    public void aiming(double distance, double frontV) {
        double h = (48 - Constants.Sizes.robotHeight + Constants.Sizes.artifactRadius + 2); //2 is some buffer :P
        double g = -386.08858267717; //9.8m/s in inches
        double z = Math.abs(h / (g * distance) - distance);
        double AOS = z / 2;
        double H = Math.abs(g * Math.pow(AOS, 2));
        double t = Math.sqrt(2 * H / g);
        double vH = z / t;
        double vV = 0.5 * g * Math.pow(t, 2);
        double shooterVel = Math.sqrt(Math.pow(vH, 2) + Math.pow(vV, 2)) / Constants.ShooterConstants.shooterGearRatio / (Math.PI * Constants.ShooterConstants.flyWheelDiameter);
        double angle = Math.toDegrees(Math.atan2(vH, vV));
        turretHead.setTargetRotation(angle);
        shoot(shooterVel - frontV);
    }



    public void periodic(Telemetry telemetry) {
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
        return (360 / angleDiff) * Constants.StudickaMotorMax;

    }

}
