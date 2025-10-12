package Commands;


import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import Subsystems.RTPAxon;
import Utilities.Constants;
import Utilities.ConfigVariables;

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
        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        if (TEAM.equals("RED")) {
            aimedTagID = 24;
            targetPos = Constants.OdometryConstants.targetPosRed;
        } else {
            aimedTagID = 20;
            targetPos = Constants.OdometryConstants.targetPosBlue;
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
        double neckHeading = (turretNeck.getCurrentAngle() * Constants.ShooterConstants.turretNeckGearRatio) + Constants.OdometryConstants.fieldPos.heading.toDouble();
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
            double x = targetPos.x - Constants.OdometryConstants.fieldPos.position.x;
            double y = targetPos.y - Constants.OdometryConstants.fieldPos.position.y;
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

    private void setShooterVelocity(double d, double h /* wallHeight (m) */, double botV) {
        final double g = 9.8; // m/s^2
        final double wheelCircumference = Constants.OdometryConstants.deadwheelDiameter * Math.PI; // example wheel
        double angle = Math.toRadians(Constants.ShooterConstants.shooterAngle);
        // sanity checks
        if (d <= 0 || h < 0) {
            stall();
            return;
        }

        // compute required launch speed for falling target:
        // v = sqrt( g * d^2 / (2 * cos^2(angle) * (d*tan(angle) + hDrop)) )
        double denom = 2 * Math.pow(Math.cos(angle), 2) * (d * Math.tan(angle) + h);
        if (denom <= 0) {
            stall();
            return;
        }
        double v = Math.sqrt((g * d * d) / denom);
        v += botV;

        // compute x of the "wall" (12in before the target)
        double xWall = d - Constants.ShooterConstants.centerOffset;
        if (xWall <= 0) {
            stall();
            return;
        }

        // define vertical coordinates explicitly
        double launchY = 0.0;              // choose launch reference (0 = launch height)
        double targetY = launchY - h;  // target is below launch

        // projectile height formula (relative to launchY)
        double yWall = launchY
                + xWall * Math.tan(angle)
                - (g * xWall * xWall) / (2 * v * v * Math.pow(Math.cos(angle), 2));

        // numeric tolerance to avoid floating-point flakiness
        double eps = 1e-9;
        if (yWall + eps < targetY) {
            stall();
            return;
        }

        // ticks-per-second (robot wheel) if you still want it
        double tps = v / wheelCircumference + 1;
        tps *= 1.5; //TODO: This is added since the tps seems too low, but this will need adjusting at best
        shoot(tps);
    }


    public void stall() {
        canMake = false;
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
