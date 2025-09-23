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
    Vision Vision;

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
        for (AprilTagDetection detection : Vision.getDetections()) {
            int angle1, angle2;
            double frontV, sideV;
            if (detection.id == aimedTagID) {
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

                // Update turret positions with corrected unit handling and corrected math
                yTurn(distanceMeters, getShooterVelocity());
                turretNeck.setTargetRotation(turretNeck.getTargetRotation() + xTurn(detectedX - ((double) Constants.VisionConstants.resX / 2), sideV, distanceMeters, getShooterVelocity()));
                /*
                Telemetry scanned info
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                 */
            }
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

    private void setShooterVelocity(double R /* horizontal distance in meters */, double h /* height diff in meters */, double currentVelocity) {
        //ChatGPT helped write this thanks to me not knowing the formulas
        double g = 9.8;
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
        if (power > 1){
            power = 1;
        }
        shoot(power);
    }


    public void periodic(Telemetry telemetry) {
        telemetry.addLine("Shooter");
        telemetry.addData("Left Shooter Power: ", leftShooter.getPower());
        telemetry.addData("Right Shooter Power: ", rightShooter.getPower());
        telemetry.addLine("Turret neck angle: " + turretNeck.getCurrentAngle());
        telemetry.addLine("Turret head position: " + turretHead.getPosition());
        telemetry.update();
    }

    private double xTurn(double xOffset, double targetLateralVel, double distance, double launchVel) {
        // xOffset: pixel offset of target from center
        // targetLateralVel: target's sideways velocity (m/s)
        // distance: horizontal distance to target (m)
        // launchVel: projectile launch velocity (m/s) from yTurn

        // camera offset angle (degrees per pixel * offset)
        double angleToTurnDeg = ((double) Constants.VisionConstants.FOV / Constants.VisionConstants.resX) * xOffset;

        // --- estimate projectile time of flight ---
        double heightDiffM = (48 - 16 + 5 + 2) * Constants.inToM; // same as yTurn
        double angleRad = Math.atan2(heightDiffM, distance);
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

    private double yTurn(double distance, double velocityCurrent) {
        //ChatGPT helped write this thanks to me not knowing the formulas
        // distance (m): horizontal distance to target
        // velocityCurrent (m/s): current shooter wheel linear velocity (m/s)

        double heightDiffM = (48 - 16 + 5 + 2) * Constants.inToM; // target - shooter height + buffer

        if (distance <= 0) {
            return 0;
        }

        // --- Step 1: compute angle based on target triangle ---
        double angleRad = Math.atan2(heightDiffM, distance);
        double angleDeg = Math.toDegrees(angleRad);

        // --- Step 2: approximate arc length velocity ---
        double straightLine = Math.sqrt(distance * distance + heightDiffM * heightDiffM);
        double velocityTarget = 0.75 * straightLine; // 1.5*L / 2 = 0.75*L

        // update shooter velocity to match new target
        setShooterVelocity(distance, heightDiffM, velocityTarget);

        // --- Step 3: convert to servo position delta ---
        double positionDelta = (angleDeg / 360.0) * Constants.ShooterConstants.turretHeadGearRatio;
        if (positionDelta > Constants.ServoMax) {
            positionDelta = Constants.ServoMax;
        }

        return positionDelta;
    }


}
