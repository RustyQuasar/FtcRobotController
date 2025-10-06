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
            double x = Constants.OdometryConstrants.fieldPos.position.x - targetPos.x;
            double y = Constants.OdometryConstrants.fieldPos.position.y - targetPos.y;
            double distance = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)) + Constants.ShooterConstants.centerOffset;
            xTurn(Math.toDegrees(Math.atan2(y, x)), sideV, distance, getShooterVelocity());
            setShooterVelocity(distance, heightMeters, frontV);
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
        if (d <= 0) {
            stall();
            canMake = false;
            return;
        }

        double theta = Math.toRadians(Constants.ShooterConstants.shooterAngle);
        double cosTh = Math.cos(theta);
        double tanTh = Math.tan(theta);

        // avoid near-vertical numeric trouble
        if (Math.abs(cosTh) < 1e-6) {
            stall();
            canMake = false;
            return;
        }

        // --- target height at x = d (use the same expression you used in aim())
        // If you have a different target height, replace this line.
        double targetHeight = (48 - 16 + 5 + 2) * Constants.inToM; // meters

        // 1) required speed to hit (d, targetHeight)
        double denomHit = 2.0 * cosTh * cosTh * (d * tanTh - targetHeight);
        if (denomHit <= 0.0 || Double.isNaN(denomHit)) {
            stall();
            canMake = false;
            return;
        }
        double vHit = Math.sqrt((g * d * d) / denomHit);

        // 2) wall clearance check (wall is 12in behind the provided distance)
        double xWall = d - Constants.ShooterConstants.centerOffset;    // wall x-position (meters)
        double vFinal = vHit;

        if (xWall > 1e-6) { // only check if wall is between shooter and target
            double denomWall = 2.0 * cosTh * cosTh * (xWall * tanTh - h); // h is wallHeight
            if (denomWall <= 0.0 || Double.isNaN(denomWall)) {
                // impossible to clear the wall with this fixed angle
                stall();
                canMake = false;
                return;
            }
            double vWall = Math.sqrt((g * xWall * xWall) / denomWall);
            // to both hit target and clear wall we need at least the larger speed
            vFinal = Math.max(vHit, vWall) - v;
        }

        // 3) convert linear speed -> wheel rev/s -> motor rev/s -> motor ticks/sec
        double wheelCircum = Math.PI * Constants.ShooterConstants.flyWheelDiameter; // meters
        double wheelRPS = vFinal / wheelCircum; // wheel revs per second
        double motorRPS = wheelRPS * Constants.ShooterConstants.shooterGearRatio;
        double motorTicksPerSecond = motorRPS * Constants.GoBildaMotorMax;

        // 4) command the shooter
        canMake = true;
        shoot(motorTicksPerSecond);
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
        telemetry.addData("Left PID: ", leftShooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        telemetry.addData("Right PID: ", rightShooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        telemetry.addData("Turret neck angle: ", turretNeck.getCurrentAngle());
        telemetry.addData("Turret neck target: ", turretNeck.getTargetRotation());
        telemetry.addData("X pos: ", offsetX + Constants.VisionConstants.resX / 2);
        telemetry.addData("Offset X: ", offsetX);
        telemetry.addData("Can make shot: ", canMake);
        telemetry.update();
    }


    private double xTurn(double angleToTurnDeg, double lateralVel, double distance, double launchVel) {
        // Guard: no valid solution if inputs are degenerate
        if (launchVel <= 0 || distance <= 0) {
            return 0;
        }

        // --- projectile time of flight ---
        double vX = launchVel * Math.cos(Math.toRadians(Constants.ShooterConstants.shooterAngle));   // horizontal component of launch velocity
        if (vX <= 1e-6) {
            return angleToTurnDeg / Constants.ShooterConstants.turretNeckGearRatio; // avoid div/0
        }
        double t = distance / vX; // time of flight

        // --- lead angle (lateral offset due to target motion) ---
        double lateralOffset = lateralVel * t;
        double leadAngleRad = Math.atan2(lateralOffset, distance);
        double leadAngleDeg = Math.toDegrees(leadAngleRad);

        // --- final correction ---
        double angleDiff = (angleToTurnDeg + leadAngleDeg) / Constants.ShooterConstants.turretNeckGearRatio;
        return angleDiff;
    }


}
