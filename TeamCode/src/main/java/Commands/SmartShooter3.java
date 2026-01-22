package Commands;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Subsystems.PIDController;
import Subsystems.Vision;
import Utilities.ConfigVariables;
import Utilities.Constants;

public class SmartShooter3 {
    private final DcMotorEx leftShooter, rightShooter;
    private final Servo turretHead, finger;
    private final DcMotorEx turretNeckMotor;
    private final CRServo transferServo, transferServo2;
    Vision Vision;
    double distance = 0;
    double xChange;
    double yChange;
    double shooterVel = 0;
    int targetNeckPos;
    double neckHeading;
    double offsetAngle = 0;
    double angle;
    double headingTarget;
    double targetPos;
    public double offset = 0;
    double limelightToShooterCenter = 5.72;
    double shooterToBotCenter = 1.680;
    double[] totalOffsets;
    PIDController neckController;
    public SmartShooter3(HardwareMap hardwareMap, Vision vision) {
        leftShooter = hardwareMap.get(DcMotorEx.class, Constants.ShooterConstants.leftShooter);
        rightShooter = hardwareMap.get(DcMotorEx.class, Constants.ShooterConstants.rightShooter);
        turretNeckMotor = hardwareMap.get(DcMotorEx.class, Constants.ShooterConstants.turretNeckMotor);
        turretHead = hardwareMap.get(Servo.class, Constants.ShooterConstants.turretHeadServo);
        finger = hardwareMap.get(Servo.class, Constants.ShooterConstants.fingerServo);
        finger.setPosition(0);
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretNeckMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretNeckMotor.setTargetPosition(0);
        turretHead.setPosition(1);
        turretNeckMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        turretHead.setDirection(Servo.Direction.FORWARD);
        Vision = vision;
        transferServo = hardwareMap.get(CRServo.class, Constants.IntakeConstants.transferServo);
        transferServo2 = hardwareMap.get(CRServo.class, Constants.IntakeConstants.transferServo2);
        neckController = new PIDController(ConfigVariables.neckp, ConfigVariables.necki, ConfigVariables.neckd);

    }

    public void shoot(double vel) {
        leftShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(ConfigVariables.p, ConfigVariables.i, ConfigVariables.d, 0));
        leftShooter.setVelocity(vel);
        rightShooter.setPower(leftShooter.getPower());
    }
    public void turretHeadTester(boolean pressed){
        if (pressed) {turretHead.setPosition(0.2);}
        else {turretHead.setPosition(1);}
    }

    public void aim(boolean autoAim) {
        double[] shooterCenterPos = new double[2];
        Vector2d targetPose;
        if (Constants.TEAM.equals("RED")){
            targetPose = Constants.OdometryConstants.targetPosRed;
        } else {
            targetPose = Constants.OdometryConstants.targetPosBlue;
        }
        double botHeading = Constants.OdometryConstants.fieldPos.heading.toDouble() + Constants.OdometryConstants.startHeading;
        double max = 2 * Math.PI;
        if (botHeading < 0) {
            botHeading += max;
        }
        neckHeading = botHeading - (turretNeckMotor.getCurrentPosition() / (Constants.GoBildaMotorMax * Constants.ShooterConstants.turretNeckGearRatio) * max);
        neckHeading -= (Math.floor(neckHeading / max) * max);
        offsetAngle = (offset / (Constants.GoBildaMotorMax * Constants.ShooterConstants.turretNeckGearRatio) * max);
        double xv =
                //Constants.OdometryConstants.fieldVels.linearVel.x;
                0;
                if (!Constants.OdometryConstants.directions[0]) xv *= -1;
        if (Double.isNaN(xv)) xv = 0;
        double yv =
                //Constants.OdometryConstants.fieldVels.linearVel.y;
                0;
                if (!Constants.OdometryConstants.directions[1]) yv *= -1;
        if (Double.isNaN(yv)) yv = 0;
        double fv  =  xv * Math.cos(neckHeading) + xv * Math.sin(neckHeading);
        double sv = -yv * Math.sin(neckHeading) + yv * Math.cos(neckHeading);
        // Step through the list of detections and display info for each one.
        double[] shooterPos = new double[]{shooterToBotCenter * Math.sin(botHeading), shooterToBotCenter * Math.cos(botHeading)};
        if (Vision.hasTarget()) {
                double neckDeltaHeading = neckHeading - botHeading + Math.PI/2;
                double[] limelightPos = new double[]{limelightToShooterCenter * Math.sin(neckDeltaHeading), limelightToShooterCenter * Math.cos(neckDeltaHeading)};
                totalOffsets = new double[]{shooterPos[0] + limelightPos[0], shooterPos[1] + limelightPos[1]};
                Vector2d llPos = Vision.getPose(neckHeading + offsetAngle);
                if (Constants.TEAM.equals("BLUE")) {
                    Constants.OdometryConstants.fieldPos = new Pose2d(llPos.x + totalOffsets[0], llPos.y + totalOffsets[1], Constants.OdometryConstants.fieldPos.heading.toDouble());
                } else {
                    Constants.OdometryConstants.fieldPos = new Pose2d(llPos.x - totalOffsets[0], llPos.y - totalOffsets[1], Constants.OdometryConstants.fieldPos.heading.toDouble());
                }
                if (Constants.TEAM.equals("BLUE")) {
                    shooterCenterPos[0] = llPos.x + limelightPos[0];
                    shooterCenterPos[1] = llPos.y + limelightPos[1];
                } else {
                    shooterCenterPos[0] = llPos.x - limelightPos[0];
                    shooterCenterPos[1] = llPos.y - limelightPos[1];
                }
                xChange = targetPose.x - shooterCenterPos[0];
                yChange = targetPose.y - shooterCenterPos[1];
            } else {
                xChange = (targetPose.x) - (Constants.OdometryConstants.fieldPos.position.x - shooterPos[0]);
                yChange = (targetPose.y) - (Constants.OdometryConstants.fieldPos.position.y - shooterPos[1]);
            }
            //THEOREM OF PYTHAGORAS
            distance = Math.sqrt(Math.pow(xChange, 2) + Math.pow(yChange, 2));
            //TOA in the indian princess' name
            headingTarget = Math.atan2(xChange, yChange);
            double angleToTurn;
            angleToTurn = Math.toDegrees(headingTarget + neckHeading);
            aiming(distance, fv, sv, angleToTurn, autoAim);
    }
    public void manualOffset(boolean leftTrigger, boolean rightTrigger){
        if (leftTrigger) offset -= 4;
        if (rightTrigger) offset += 4;
    }
    public void transfer(boolean buttonPressed) {
        if (!buttonPressed || !(Math.abs(leftShooter.getVelocity() - shooterVel) < 20)) {
            transferServo.setPower(-0.2);
        } else {
            transferServo.setPower(1);
        }
        transferServo2.setPower(-transferServo.getPower());

        if (!buttonPressed || !(Math.abs(leftShooter.getVelocity() - shooterVel) < -1)) {
            finger.setPosition(0);
        } else {
            finger.setPosition(0.25);
        }
    }

    public void overrideTransfer(boolean buttonPressed) {
        //if (!buttonPressed || !(Math.abs(leftShooter.getVelocity() - shooterVel) < 80)) {
        if (!buttonPressed) {
                transferServo.setPower(-0.2);
            } else {

                transferServo.setPower(0.2);
        }
        transferServo2.setPower(-transferServo.getPower());
    }

    public void aiming(double distance, double frontV, double sideV, double angleToTurn, boolean autoAim) {
        //SO MUCH METH MATH THE CRACKHEADS ARE JEALOUS
        distance = Math.min(Math.max(distance, 55), 148);
        double h = (38 - Constants.Sizes.robotHeight + Constants.Sizes.artifactRadius * 2 + 2) / 39.37; //2 is some buffer :P
        double g = -9.8;
        double distanceMeters = distance / 39.37;
        double z = Math.abs(h / (g * distanceMeters) - distanceMeters);
        double AOS = z / 2;
        //This is the root form of the parabola dw, y = -9.8(AOS-0)(AOS-z)
        double H = Math.abs(g * Math.pow(AOS, 2));
        double t = Math.sqrt(H / -g);
        angle = distance * 0.3;
        shooterVel = (distance - frontV * t) * 4.84622 + 600.90476;
        double totalTicks = Constants.ShooterConstants.turretNeckGearRatio * Constants.GoBildaMotorMax;
        targetNeckPos = (int) (turretNeckMotor.getCurrentPosition() + xTurn(angleToTurn, sideV, distance, t));
        targetNeckPos -= (int) (Math.floor(Math.abs(targetNeckPos / totalTicks)) * totalTicks * Math.signum(targetNeckPos));
        if (targetNeckPos > totalTicks / 2) targetNeckPos -= (int) totalTicks;
        if (targetNeckPos < -totalTicks / 2) targetNeckPos += (int) totalTicks;
        if (Math.abs(targetNeckPos - offset) > 1000) targetNeckPos = (int) (1000 * Math.signum(targetNeckPos));
        targetPos = (1 - Math.max(0, Math.min(0.8, angle / Constants.ShooterConstants.maxHeadAngle)));
        turretHead.setPosition(targetPos);
        if (autoAim) {
            turretNeckMotor.setPower(neckController.calculate(targetNeckPos + offset, turretNeckMotor.getCurrentPosition()));
        } else {
            turretNeckMotor.setPower(neckController.calculate(offset, turretNeckMotor.getCurrentPosition()));
        }
        shoot(shooterVel);
    }



    public void telemetry(Telemetry telemetry) {
        telemetry.addLine("Shooter");
        //telemetry.addData("Left Shooter Power: ", leftShooter.getPower());
        //telemetry.addData("Right Shooter Power: ", rightShooter.getPower());
        //telemetry.addData("Turret neck pos: ", turretNeckMotor.getCurrentPosition());
        telemetry.addData("Turret heading: ", neckHeading);
        //telemetry.addData("Turret head pos: ", targetPos);
        //telemetry.addData("Target neck heading: ", headingTarget);
        telemetry.addData("Camera pos: ", Vision.getPose(neckHeading + offsetAngle));
        telemetry.addData("Offset: ", offset);
        telemetry.addData("Distance: ", distance);
        telemetry.addData("Target vel: ", shooterVel);
        telemetry.addData("Current vel", leftShooter.getVelocity());
        telemetry.update();
    }
    public void manualNeckMotor(boolean leftBumper, boolean rightBumper){
        if (leftBumper && rightBumper) {turretNeckMotor.setTargetPosition(0); return;}
        if (leftBumper && Constants.TEAM.equals("RED")) {turretNeckMotor.setTargetPosition(950); return;}
        if (leftBumper && Constants.TEAM.equals("BLUE")) {turretNeckMotor.setTargetPosition(-950); return;}
        if (rightBumper) turretNeckMotor.setTargetPosition(0);
        //target pos +- 950
        //left bumper is offset pos
        //right bumper is straight forward (0)
    }
    private double xTurn(double angleToTurnDeg, double velocity, double distance, double time) {
        double leadAngleDeg = Math.toDegrees(Math.acos(velocity / (distance / time)));
        if (leadAngleDeg < 0) leadAngleDeg += 360;

        double angleDiff = angleToTurnDeg - leadAngleDeg;
        if (angleDiff < 0){
            angleDiff += 360;
        }
        return angleDiff * Constants.GoBildaMotorMax * Constants.ShooterConstants.turretNeckGearRatio / 360;
    }
    public void chill(){
        leftShooter.setPower(0);
        rightShooter.setPower(0);
        turretNeckMotor.setPower(0);
    }
}