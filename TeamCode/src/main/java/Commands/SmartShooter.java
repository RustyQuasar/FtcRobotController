package Commands;


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
    private boolean canMake = false;
    private final Vision Vision;

    public SmartShooter(HardwareMap hardwareMap, String TEAM, Vision vision) {
        leftShooter = hardwareMap.get(DcMotorEx.class, Constants.ShooterConstants.leftShooter0);
        rightShooter = hardwareMap.get(DcMotorEx.class, Constants.ShooterConstants.rightShooter1);
        turretNeckServo = hardwareMap.get(CRServo.class, Constants.ShooterConstants.turretNeckServo);
        AnalogInput turretNeckEncoder = hardwareMap.get(AnalogInput.class, Constants.ShooterConstants.turretNeckEncoder);
        turretNeck = new RTPAxon(turretNeckServo, turretNeckEncoder);
        turretHead = hardwareMap.get(Servo.class, Constants.ShooterConstants.turretHead);
        transferServo = hardwareMap.get(CRServo.class, Constants.ShooterConstants.transferServo);
        leftShooter.setDirection(DcMotor.Direction.REVERSE);
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (TEAM.equals("RED")) {
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

    public void aim(double[] v) {
        turretNeck.update();
        double detectedX;
        double distanceMeters;
        double fv = v[0];
        double sv = v[1];
        // Step through the list of detections and display info for each one.
        boolean foundTag = false;
        for (AprilTagDetection detection : Vision.getDetections()) {
            int angle1, angle2;
            double frontV, sideV;
            if (detection.id == aimedTagID) {
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
                distanceMeters = (detection.ftcPose.range + Constants.ShooterConstants.centerOffset) * Constants.inToM;
                double heightMeters =  (48 - 16 + 5 + 2) * Constants.inToM;
                // Update turret positions with corrected unit handling and corrected math
                turretNeck.setTargetRotation(turretNeck.getTargetRotation() + xTurn(detectedX - ((double) Constants.VisionConstants.resX / 2), sideV, distanceMeters, getShooterVelocity(), heightMeters));
                setShooterVelocity(distanceMeters, heightMeters, getShooterVelocity());
            }
        }
        if (!foundTag){
            turretNeck.setTargetRotation(turretNeck.getTargetRotation());
            shoot((leftShooter.getPower() + rightShooter.getPower()) / 2);
            canMake = false;
        }
    }

    public double getShooterVelocity() {
        if (leftShooter.getPower() != 0 && rightShooter.getPower() != 0) {
            return (leftShooter.getVelocity() * Constants.ShooterConstants.shooterGearRatio * Math.PI * Constants.ShooterConstants.flyWheelDiameter / Constants.DCMotorMax + (rightShooter.getVelocity() * Constants.ShooterConstants.shooterGearRatio * Math.PI * Constants.ShooterConstants.flyWheelDiameter / Constants.DCMotorMax) / 2);
        } else {
            return 0;
        }
    }

    public void transfer() {
        transferServo.setPower(1);
    }

    private void setShooterVelocity(double d, double h, double currentVelocity) {
        final double g = 9.8;

        if (d <= 0) return;

        // feasibility check: denominator must be positive
        double denom = d - h;
        if (denom <= 0) {
            // impossible to hit (R,h) with a 45° launch.
            // Option: spin to max power to get as close as possible.
            canMake = false;
            shoot(1);
            return;
        }

        //This formula kinda sketchy ngl
        // required linear speed (m/s) for a 45° launch to pass through (R,h)
        double vRequired = Math.sqrt((g * Math.pow(d, 2)) / denom);

        // how much extra linear speed we need vs current wheel linear speed
        double neededLinearVelocity = vRequired - currentVelocity;
        if (neededLinearVelocity <= 0) {
            shoot(0.1);
            return;
        }

        // wheel circumference (meters)
        double wheelCircum = Math.PI * Constants.ShooterConstants.flyWheelDiameter;

        // wheel revs/sec required for the delta speed
        double wheelRevsPerSec = neededLinearVelocity / wheelCircum;

        // motor revs/sec required (motorRev = wheelRev * motorPerWheelRev)
        double motorRevsPerSec = wheelRevsPerSec * Constants.ShooterConstants.shooterGearRatio;

        // normalize to motor max RPS -> desired power (clamped 0..1)
        double power = motorRevsPerSec / Constants.defaultDCRPS;
        if (power > 1.0) {
            canMake = false;
            power = 1.0;
        }
        if (power < 0.0) power = 0.0;

        // finally set shooter power
        shoot(power);
    }



    public void periodic(Telemetry telemetry) {
        telemetry.addLine("Shooter");
        telemetry.addData("Left Shooter Power: ", leftShooter.getPower());
        telemetry.addData("Right Shooter Power: ", rightShooter.getPower());
        telemetry.addLine("Turret neck angle: " + turretNeck.getCurrentAngle());
        telemetry.addLine("Turret head position: " + turretHead.getPosition());
        telemetry.addData("Can make shot: ", canMake);
        telemetry.update();
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
