package Commands;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import Subsystems.RTPAxon;
import Utilities.Constants;

public class SmartShooter {
    private final DcMotorEx leftShooter, rightShooter;
    private final CRServo turretNeckServo;
    private final Servo turretHead;
    private final CRServo transferServo;
    private final RTPAxon turretNeck;
    private final int aimedTagID;
    private static boolean canMake = false;
    private final Vision Vision;
    double detectedX = 10;
    double distanceMeters = 10;
    boolean poseStatus = false;
    public SmartShooter(HardwareMap hardwareMap, String TEAM, Vision vision) {
        leftShooter = hardwareMap.get(DcMotorEx.class, Constants.ShooterConstants.leftShooter0);
        rightShooter = hardwareMap.get(DcMotorEx.class, Constants.ShooterConstants.rightShooter1);
        turretNeckServo = hardwareMap.get(CRServo.class, Constants.ShooterConstants.turretNeckServo);
        AnalogInput turretNeckEncoder = hardwareMap.get(AnalogInput.class, Constants.ShooterConstants.turretNeckEncoder);
        turretNeck = new RTPAxon(turretNeckServo, turretNeckEncoder);
        turretHead = hardwareMap.get(Servo.class, Constants.ShooterConstants.turretHead);
        transferServo = hardwareMap.get(CRServo.class, Constants.ShooterConstants.transferServo);
        rightShooter.setDirection(DcMotor.Direction.REVERSE);
        leftShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (TEAM.equals("RED")) {
            aimedTagID = 24;
        } else {
            aimedTagID = 20;
        }
        Vision = vision;
    }

    public void shoot(double targetVelocity) {
        rightShooter.setVelocity(targetVelocity);
        leftShooter.setVelocity(targetVelocity);
    }

    public void aim(double[] v) {
        turretNeck.update();
        double fv = v[0];
        double sv = v[1];
        // Step through the list of detections and display info for each one.
        boolean foundTag = false;
        for (AprilTagDetection detection : Vision.getDetections()) {
            int angle1, angle2;
            double frontV, sideV;
            if (detection.id == aimedTagID) {
                if (detection.ftcPose != null) {
                    poseStatus = true;
                    foundTag = true;
                    canMake = true;
                    // convert servo position to degrees (your existing mapping)
                    double neckHeading = (turretNeck.getCurrentAngle() * Constants.ShooterConstants.turretNeckGearRatio);
                    neckHeading -= (Math.round((neckHeading / 360) - 0.5)) * 360;
                    neckHeading += Constants.DriveTrainConstants.controlHubOffset;
                    if (fv > 0) {
                        angle1 = 0;
                    } else {
                        angle1 = 180;
                    }
                    if (sv > 0) {
                        angle2 = 90;
                    } else {
                        angle2 = 270;
                    }
                    double[] totals = {neckHeading - angle1, neckHeading - angle2};
                    for (int i = 0; i < totals.length; i++) {
                        if (totals[i] > 360) {
                            totals[i] -= 360;
                        }
                        if (totals[i] < 0) {
                            totals[i] *= -1;
                        }
                    }
                    double hypotenuse = Math.sqrt(Math.pow(fv, 2) + Math.pow(sv, 2));
                    if (totals[0] <= totals[1]) {
                        frontV = hypotenuse * Math.sin(Math.toRadians(totals[0]));
                        sideV = hypotenuse * Math.cos(Math.toRadians(totals[0]));
                    } else {
                        frontV = hypotenuse * Math.cos(Math.toRadians(totals[1]));
                        sideV = hypotenuse * Math.sin(Math.toRadians(totals[1]));
                    }
                    detectedX = detection.ftcPose.x;
                    // Convert range (inch) to meters consistently, and add centerOffset (inches) then convert:
                    distanceMeters = (detection.ftcPose.range + Constants.ShooterConstants.centerOffset);
                    double heightMeters = (48 - 16 + 5 + 2) * Constants.inToM;
                    // Update turret positions with corrected unit handling and corrected math
                    turretNeck.setTargetRotation(turretNeck.getTargetRotation() + xTurn(detectedX - ((double) Constants.VisionConstants.resX / 2), sideV, distanceMeters, getShooterVelocity(), heightMeters));
                    setShooterVelocity(distanceMeters, heightMeters);
                } else {
                    poseStatus = false;
                }
            }
        }
        if (!foundTag){
            turretNeck.setTargetRotation(turretNeck.getTargetRotation());
            shoot((leftShooter.getVelocity() + rightShooter.getVelocity()) / 2);
            canMake = false;
        }
    }

    public double getShooterVelocity() {
        if (rightShooter.getPower() != 0 && leftShooter.getPower() != 0) {
            return ((rightShooter.getVelocity() * Constants.ShooterConstants.shooterGearRatio * Math.PI * Constants.ShooterConstants.flyWheelDiameter / Constants.GoBildaMotorMax + leftShooter.getVelocity() * Constants.ShooterConstants.shooterGearRatio * Math.PI * Constants.ShooterConstants.flyWheelDiameter / Constants.GoBildaMotorMax) / 2);
        } else {
            return 0;
        }
    }

    public void transfer() {
        transferServo.setPower(1);
    }

    private void setShooterVelocity(double d, double h) {
            final double g = 9.8;
            if (d <= 0) return;

            double theta = Math.toRadians(Constants.ShooterConstants.shooterAngle);
            double cosTh = Math.cos(theta);
            double tanTh = Math.tan(theta);

            // avoid near-vertical numeric trouble
            if (Math.abs(cosTh) < 1e-6) {
                canMake = false;
                return;
            }

            double denom = 2.0 * cosTh * cosTh * ((d-12 * Constants.inToM) * tanTh - h);
            if (denom <= 0.0 || Double.isNaN(denom)) {
                // impossible geometry for this angle: best effort by spinning to max
                canMake = false;
                return;
            }

            double vRequired = Math.sqrt((g * Math.pow(d, 2)) / denom);

            // Convert linear speed -> wheel revs -> motor revs -> normalized power
            double wheelCircum = Math.PI * Constants.ShooterConstants.flyWheelDiameter; // meters
            double wheelRPS = vRequired / wheelCircum; // wheel rev/s required
            double motorRPS = wheelRPS * Constants.ShooterConstants.shooterGearRatio;

            shoot(motorRPS); // always command the shooter (don't early return)
        }

    public void periodic(Telemetry telemetry, TelemetryPacket packet) {
        telemetry.addLine("Shooter");
        telemetry.addData("Left Shooter Power: " , leftShooter.getPower());
        telemetry.addData("Right Shooter Power: ", rightShooter.getPower());
        telemetry.addData("Turret neck angle: " , turretNeck.getCurrentAngle());
        telemetry.addData("Turret head position: " , turretHead.getPosition());
        telemetry.addData("Can make shot: " , canMake);
        telemetry.update();

        packet.addLine("Shooter");
        packet.put("Left Shooter Power: ", leftShooter.getPower());
        packet.put("Right Shooter Power: ", rightShooter.getPower());
        packet.put("Turret neck angle: ", turretNeck.getCurrentAngle());
        packet.put("Turret head position: ", turretHead.getPosition());
        packet.put("Detected X: ", detectedX);
        packet.put("Detected Distance: ", distanceMeters);
        packet.put("ftcPose likes us? ", poseStatus);
        packet.put("Can make shot: ", canMake);
    }

    private double xTurn(double xOffset, double targetLateralVel, double distance, double launchVel, double height) {
        // xOffset: pixel offset of target from center
        // targetLateralVel: target's sideways velocity (m/s)
        // distance: horizontal distance to target (m)
        // launchVel: projectile launch velocity (m/s) from yTurn

        // camera offset angle (degrees per pixel * offset)
        double angleToTurnDeg = ((double) Constants.VisionConstants.FOV / Constants.VisionConstants.resX) * xOffset;

        // --- estimate projectile time of flight ---
        double angleRad = Math.atan2(height, distance);
        double vX = launchVel * Math.cos(angleRad);
        double t = distance / vX;

        // --- lead angle (based on how far target moves in that time) ---
        double leadAngleRad = Math.atan2(targetLateralVel * t, distance);
        double leadAngleDeg = Math.toDegrees(leadAngleRad);

        // --- final correction ---
        double angleDiff = (angleToTurnDeg - leadAngleDeg) / Constants.ShooterConstants.turretNeckGearRatio;
        if (angleDiff < 0) {
            angleDiff += 360;
        }
        if (angleDiff > 360) {
            angleDiff -= 360;
        }
        return angleDiff;
    }
}
