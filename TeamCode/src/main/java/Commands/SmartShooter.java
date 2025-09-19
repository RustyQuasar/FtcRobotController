package Commands;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import Utilities.Constants;

public class SmartShooter {
    private final DcMotorEx leftShooter, rightShooter;
    private final Servo turretNeck, turretHead;
    private final CRServo transferServo;
    private int aimedTagID;
    Vision Vision;

    public SmartShooter(HardwareMap hardwareMap, String TEAM, Vision vision) {
        leftShooter = hardwareMap.get(DcMotorEx.class, Constants.ShooterConstants.leftShooter0);
        rightShooter = hardwareMap.get(DcMotorEx.class, Constants.ShooterConstants.rightShooter1);
        turretNeck = hardwareMap.get(Servo.class, Constants.ShooterConstants.turretNeck);
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

    public void aim(double[] v, String[] colours) {
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
                double neckHeading = (turretNeck.getPosition() * Constants.ShooterConstants.turretNeckGearRatio);
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
                turretNeck.setPosition(turretNeck.getPosition() + xTurn(detectedX - (Constants.VisionConstants.resX / 2), sideV, distanceMeters));
                turretHead.setPosition(turretHead.getPosition() + yTurn(distanceMeters, frontV));
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

    public double getShooterVelocity(int DCMotorMax, int gearRatio) {
        if (leftShooter.getPower() != 0 && rightShooter.getPower() != 0) {
            return (leftShooter.getVelocity() * gearRatio * Math.PI * Constants.ShooterConstants.flyWheelDiameter / Constants.DCMotorMax + (rightShooter.getVelocity() * gearRatio * Math.PI * Constants.ShooterConstants.flyWheelDiameter / Constants.DCMotorMax) / 2);
        } else {
            return 0;
        }
    }

    public void transfer() {
        transferServo.setPower(1);
    }

    private void setShooterVelocity(double R /* horizontal distance in meters */, double h /* height diff in meters */, double currentVelocity) {
        // Corrected physics:
        // Compute minimum initial projectile speed v_min (m/s) required to reach horizontal distance R and vertical offset h:
        // v_min = sqrt( g * R^2 / ( sqrt(R^2 + h^2) - h ) )
        // (derived from discriminant of projectile equation)
        double g = 9.8;
        if (R <= 0) {
            return; // can't compute
        }
        double denom = Math.sqrt(R * R + h * h) - h;
        if (denom <= 0) {
            return; // invalid geometry
        }
        double vMin = Math.sqrt((g * R * R) / denom);

        // compute required additional linear velocity (m/s) to reach vMin (accounting for current wheel/projectile linear speed)
        double neededLinearVelocity = vMin - currentVelocity;
        if (neededLinearVelocity <= 0) {
            // already at or above required velocity
            return;
        }

        // Convert required linear velocity of projectile to shooter wheel / motor units.
        // wheelCircum = circumference of 4" wheel in meters
        double wheelCircum = Math.PI * Constants.ShooterConstants.flyWheelDiameter;
        // wheel revs per second needed:
        double wheelRevsPerSec = neededLinearVelocity / wheelCircum;
        // motor revs per second needed (if shooterGearRatio is motorRev / wheelRev)
        double motorRevsPerSec = wheelRevsPerSec * Constants.ShooterConstants.shooterGearRatio;

        // Without knowing a motor-max-RPS, we keep your original simple scaling:
        // power ~ motorRevsPerSec (you may want to divide by maxMotorRevsPerSec if you have it)
        double power = motorRevsPerSec / Constants.defaultDCRPS; // optional normalization: convert RPS to "per minute" scale (example).
        // NOTE: The correct normalization depends on your expected mapping of 'power' to motor revs.
        shoot(power);
    }

    public void periodic(Telemetry telemetry) {
        telemetry.addLine("Shooter");
        telemetry.addData("Left Shooter Power: ", leftShooter.getPower());
        telemetry.addData("Right Shooter Power: ", rightShooter.getPower());
        telemetry.addLine("Turret neck position: " + turretNeck.getPosition());
        telemetry.addLine("Turret head position: " + turretHead.getPosition());
        telemetry.update();
    }

    private double xTurn(double xOffset, double velocity, double distance) {
        // xOffset: pixel offset from center
        // velocity: lateral velocity (m/s)
        // distance: target distance (m)
        // compute degrees to turn from pixel offset
        double angleToTurnDeg = (Constants.VisionConstants.FOV / (double) Constants.VisionConstants.resX) * xOffset; // degrees

        // compute lead angle (radians) then convert to degrees:
        double leadAngleDeg = Math.toDegrees(Math.atan2(velocity, distance)); // small-angle approximation: atan(velocity/distance)
        // convert degrees to servo position deltas:
        double neckDeltaFromPixels = angleToTurnDeg / Constants.ShooterConstants.turretNeckGearRatio;
        double neckDeltaFromLead = leadAngleDeg / Constants.ShooterConstants.turretNeckGearRatio;

        // final position delta (servo units)
        return neckDeltaFromPixels - neckDeltaFromLead;
    }

    private double yTurn(double distance, double velocity) {
        // distance (m) horizontal distance to target
        // velocity (m/s) current shooter wheel linear velocity (m/s)

        // compute height difference (target height - shooter height) in meters (use your constants)
        double heightDiffM = (48 - 16 + 5 + 2) * Constants.inToM; // kept your original numbers and converted to meters, the extra 2in is buffer

        double R = distance;
        double h = heightDiffM;

        if (R <= 0) {
            return 0;
        }

        // compute minimal required initial speed (and set shooter speed)
        setShooterVelocity(R, h, velocity);

        // compute the pitch angle (radians) for the minimal-speed trajectory:
        // using t = tan(theta) = v^2 / (g * R) where v is the v_min computed earlier.
        double g = 9.8;
        double denom = Math.sqrt(R * R + h * h) - h;
        if (denom <= 0) {
            return 0;
        }
        double vMin = Math.sqrt((g * R * R) / denom);
        double t = (vMin * vMin) / (g * R); // tan(theta)
        double angleRad = Math.atan(t);
        double angleDeg = Math.toDegrees(angleRad);

        // convert degrees to turret-head servo position delta (using same mapping as you used elsewhere)
        double positionDelta = angleDeg / Constants.ShooterConstants.turretHeadGearRatio;
        return positionDelta;
        //#TODO: This is assuming equal levelling and no air res, fix based on the height difference
    }
}
