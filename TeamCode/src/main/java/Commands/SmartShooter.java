package Commands;


import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import Subsystems.RTPAxon;
import Utilities.Constants;
import Utilities.ConfigVariables;
import Utilities.Odometry;

public class SmartShooter {
    private static final Logger log = LoggerFactory.getLogger(SmartShooter.class);
    private final DcMotorEx leftShooter, rightShooter;
    private final CRServo turretNeckServo, transferServo;
    private final RTPAxon turretNeck;
    private final int aimedTagID;
    private static boolean canMake = false;
    private final Vision Vision;
    double offsetX = 0;
    double distanceMeters = 0;
    double tVel;
    Vector2d targetPos;

    public SmartShooter(HardwareMap hardwareMap, String TEAM, Vision vision) {
        leftShooter = hardwareMap.get(DcMotorEx.class, Constants.ShooterConstants.leftShooter0);
        rightShooter = hardwareMap.get(DcMotorEx.class, Constants.ShooterConstants.rightShooter1);
        turretNeckServo = hardwareMap.get(CRServo.class, Constants.ShooterConstants.turretNeckServo);
        AnalogInput turretNeckEncoder = hardwareMap.get(AnalogInput.class, Constants.ShooterConstants.turretNeckEncoder);
        turretNeck = new RTPAxon(turretNeckServo, turretNeckEncoder);
        transferServo = hardwareMap.get(CRServo.class, Constants.ShooterConstants.transferServo);
        leftShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        if (TEAM.equals("RED")) {
            aimedTagID = 24;
            targetPos = Constants.OdometryConstrants.targetPosRed;
        } else {
            aimedTagID = 20;
            targetPos = Constants.OdometryConstrants.targetPosBlue;
        }
        Vision = vision;
    }

    public void shoot(double targetVelocity) {
        PIDFCoefficients pidCoefficients = new PIDFCoefficients();
        pidCoefficients.p = ConfigVariables.P;
        pidCoefficients.i = ConfigVariables.I;
        pidCoefficients.d = ConfigVariables.D;
        leftShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidCoefficients);
        rightShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidCoefficients);
        rightShooter.setVelocity(targetVelocity);
        leftShooter.setVelocity(targetVelocity);
        tVel = targetVelocity;
    }

    public void aim(double[] v) {
        turretNeck.update();
        double fv = v[0];
        double sv = v[1];
        double heightMeters = (48 - 16 + 5 + 2) * Constants.inToM;
        boolean foundTag = false;
        double neckHeading = (turretNeck.getCurrentAngle() * Constants.ShooterConstants.turretNeckGearRatio) + Constants.heading;
        neckHeading -= (Math.round((neckHeading / 360) - 0.5)) * 360;
        neckHeading += Constants.DriveTrainConstants.controlHubOffset;
        int angle1, angle2;
        double frontV, sideV;
        canMake = true;
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

        for (AprilTagDetection detection : Vision.getDetections()) {
            if (detection.id == aimedTagID) {
                if (detection.ftcPose != null) {
                    foundTag = true;
                    // convert servo position to degrees (your existing mapping)
                    offsetX = detection.center.x - (double) Constants.VisionConstants.resX / 2;
                    // Convert range (inch) to meters consistently, and add centerOffset (inches) then convert:
                    distanceMeters = (detection.ftcPose.range + Constants.ShooterConstants.centerOffset);
                    // Update turret positions with corrected unit handling and corrected math
                    double angleToTurnDeg = ((double) Constants.VisionConstants.FOV / Constants.VisionConstants.resX) * offsetX;
                    turretNeck.setTargetRotation(turretNeck.getTargetRotation() + xTurn(angleToTurnDeg, sideV, distanceMeters, getShooterVelocity()));
                    setShooterVelocity(distanceMeters, heightMeters, frontV);
                }
            }
        }

        if (!foundTag) {
            turretNeck.setTargetRotation(turretNeck.getTargetRotation());
            double x = Constants.OdometryConstrants.fieldPos.x - targetPos.x;
            double y = Constants.OdometryConstrants.fieldPos.y - targetPos.y;
            distanceMeters = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)) + Constants.ShooterConstants.centerOffset;
            xTurn(Math.toDegrees(Math.atan2(y, x)), sideV, distanceMeters, getShooterVelocity());
            setShooterVelocity(distanceMeters, heightMeters, frontV);
        }
    }

    public double getShooterVelocity() {
        // if either motor has zero power, you might decide to return 0 — keep your logic if desired
        if (rightShooter.getPower() == 0 || leftShooter.getPower() == 0) return 0;
        double wheelCircum = Math.PI * Constants.ShooterConstants.flyWheelDiameter;

        double leftTPS = leftShooter.getVelocity();   // ticks/sec
        double rightTPS = rightShooter.getVelocity();

        double leftMotorRPS = leftTPS / Constants.GoBildaMotorMax;
        double rightMotorRPS = rightTPS / Constants.GoBildaMotorMax;

        double leftWheelRPS = leftMotorRPS / Constants.ShooterConstants.shooterGearRatio;
        double rightWheelRPS = rightMotorRPS / Constants.ShooterConstants.shooterGearRatio;

        double leftLinear = leftWheelRPS * wheelCircum;   // m/s
        double rightLinear = rightWheelRPS * wheelCircum; // m/s

        return (leftLinear + rightLinear) / 2.0;
    }


    public void transfer() {
        transferServo.setPower(1);
    }
    private void setShooterVelocity(double d, double h /* wallHeight (m) */, double v) {
        final double g = 9.8;
        final double inToM = Constants.inToM; // 0.0254
        if (d <= 1.0) {
            canMake = false;
            stall();
            return;
        }

        double theta = Math.toRadians(Constants.ShooterConstants.shooterAngle);
        double cosTh = Math.cos(theta);
        double tanTh = Math.tan(theta);

        // avoid near-vertical numeric trouble
        if (Math.abs(cosTh) < 1e-6) {
            canMake = false;
            return;
        }

        // --- compute vHit (velocity that intersects (d, targetHeight)) ---
        // targetHeight: keep your existing definition if different; here we're using same as before
        double targetHeight = (48 - 16 + 5 + 2) * Constants.inToM; // meters (adjust if needed)

        // epsilon to avoid denominator collapse (very small to preserve physics)
        final double EPS_TERM = 1e-4;
// Check for invalid geometry: target is too close or too high for this shooter angle
        double critDist = h / tanTh; // distance where d*tanTh == h (apex at target)
        if (d <= critDist) {
            // No descending solution possible — we’d have to fire upward to hit it.
            // Use capped velocity and mark as unreachable
            canMake = false;
            double safeVel = 0.8 * Constants.GoBildaMotorMax; // or whatever your nominal high speed is
            double wheelCircum = Math.PI * Constants.ShooterConstants.flyWheelDiameter;
            double wheelRPS = safeVel / wheelCircum;
            double motorRPS = wheelRPS * Constants.ShooterConstants.shooterGearRatio;
            double motorTPS = motorRPS * Constants.GoBildaMotorMax;
            shoot(motorTPS);
            return;
        }

        double termHit = d * tanTh - targetHeight;
        if (termHit < EPS_TERM) termHit = EPS_TERM;
        double denomHit = 2.0 * cosTh * cosTh * termHit;
        if (denomHit <= 0.0 || Double.isNaN(denomHit)) {
            canMake = false;
            return;
        }
        double vHit = Math.sqrt((g * d * d) / denomHit);

        // --- compute vWall only when wall is meaningfully in front of shooter and before target ---
        double xWall = d - (12.0 * inToM);
        double vWall = Double.NaN;
        boolean wallApplicable = (xWall > 0.15) && (xWall < d - 0.05); // require wall at least 15cm away and before target
        if (wallApplicable) {
            double termWall = xWall * tanTh - h; // h is wall top height in meters (as you clarified)
            if (termWall < EPS_TERM) termWall = EPS_TERM;
            double denomWall = 2.0 * cosTh * cosTh * termWall;
            if (!(denomWall <= 0.0 || Double.isNaN(denomWall))) {
                vWall = Math.sqrt((g * xWall * xWall) / denomWall);
            } else {
                // wall impossible to clear at this angle -> mark as not applicable
                wallApplicable = false;
            }
        }

        // --- pick final velocity, smoothing the transition if both vHit and vWall are valid ---
        double vFinal;
        if (!wallApplicable || Double.isNaN(vWall)) {
            vFinal = vHit;
        } else {
            // Instead of hard max, blend over a small band (prevents discontinuities)
            // If vWall is much bigger than vHit, we still tend toward vWall but smoothly.
            double blendBand = 0.5; // meters over which we blend (tweakable)
            double distToWall = d - xWall; // >= 0
            // compute a blend factor based on how close the provided distance d is to the wall-target geometry
            // when distToWall is small (wall close to target) bias toward vWall; when large bias to vHit
            double t = Math.min(Math.max((blendBand - Math.abs(distToWall - (blendBand/2))) / blendBand, 0.0), 1.0);
            // simple smooth blend (linear); you can swap to smoother curve if desired
            vFinal = (1.0 - t) * vHit + t * Math.max(vHit, vWall);
            // also guarantee monotonicity: never choose a vFinal smaller than vHit (we must still hit target)
            if (vFinal < vHit) vFinal = vHit;
        }

        // --- ensure descending intersection: nudge vFinal upwards if apex is after target ---
        // but cap loop to avoid infinite increase
        final int MAX_ITER = 200;
        int iter = 0;
        double xApex = (vFinal * vFinal * Math.sin(2 * theta)) / (2 * g);
        while ((xApex > d - 0.05) && iter < MAX_ITER && vFinal < Constants.GoBildaMotorMax) {
            vFinal += 0.02; // small increment to move apex earlier
            xApex = (vFinal * vFinal * Math.sin(2 * theta)) / (2 * g);
            iter++;
        }
        if (iter >= MAX_ITER) {
            // couldn't force descending branch — bail out safely
            canMake = false;
            return;
        }

        if (vFinal > Constants.GoBildaMotorMax) {
            vFinal = Constants.GoBildaMotorMax;
        }

        // --- conversion to motor units (preserve your existing pipeline) ---
        double wheelCircum = Math.PI * Constants.ShooterConstants.flyWheelDiameter; // meters
        double wheelRPS = vFinal / wheelCircum; // wheel rev/s
        double motorRPS = wheelRPS * Constants.ShooterConstants.shooterGearRatio;
        double motorTicksPerSecond = motorRPS * Constants.GoBildaMotorMax;

        // issue command (you use setVelocity on DcMotorEx elsewhere)
        shoot(motorTicksPerSecond);

        canMake = true;
    }




    public void stall() {
        double avgVelocity = (leftShooter.getVelocity() + rightShooter.getVelocity()) / 2;
        leftShooter.setVelocity(avgVelocity);
        rightShooter.setVelocity(avgVelocity);
    }

    public void periodic(Telemetry telemetry) {
        telemetry.addLine("Shooter");
        telemetry.addData("Left Shooter Velocity: ", leftShooter.getVelocity());
        telemetry.addData("Right Shooter Velocity: ", rightShooter.getVelocity());
        telemetry.addData("Distance: ", distanceMeters);
        telemetry.addData("Target Velocity: ", tVel);
        //telemetry.addData("Left PID: ", leftShooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        telemetry.addData("Right PID: ", rightShooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        //telemetry.addData("Turret neck angle: ", turretNeck.getCurrentAngle());
        //telemetry.addData("Turret neck target: ", turretNeck.getTargetRotation());
        //telemetry.addData("X pos: ", offsetX + Constants.VisionConstants.resX / 2);
        //telemetry.addData("Offset X: ", offsetX);
        telemetry.addData("Can make shot: ", canMake);
        telemetry.update();
    }
    public double xTurn(double xOffset, double targetLateralVel, double distance, double wallHeight) {
        if (xOffset == 0 || distance <= 0.1) return 0;

        double g = 9.8;
        double theta = Math.toRadians(Constants.ShooterConstants.shooterAngle);
        double cosTh = Math.cos(theta);
        double tanTh = Math.tan(theta);
        double inToM = 0.0254;
        double d = distance;
        double h = wallHeight;

        double denom = 2.0 * cosTh * cosTh * (d * tanTh - h);
        if (denom <= 0.0 || Double.isNaN(denom)) return 0;

        double vRequired = Math.sqrt((g * d * d) / denom);

        double xApex = (vRequired * vRequired * Math.sin(2 * theta)) / (2 * g);
        int safetyCount = 0;
        while (xApex > d - 0.05 && safetyCount < 100) {
            vRequired += 0.05;
            xApex = (vRequired * vRequired * Math.sin(2 * theta)) / (2 * g);
            safetyCount++;
        }

        double vX = vRequired * Math.cos(theta);
        double t = d / vX;

        double leadAngleRad = Math.atan2(targetLateralVel * t, d);
        double leadAngleDeg = Math.toDegrees(leadAngleRad);

        double angleToTurnDeg =
                ((double) Constants.VisionConstants.FOV / Constants.VisionConstants.resX) * xOffset;

        double angleDiff = (angleToTurnDeg - leadAngleDeg)
                / Constants.ShooterConstants.turretNeckGearRatio;

        return angleDiff;
    }
}
