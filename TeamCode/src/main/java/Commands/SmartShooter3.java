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
    double limelightToShooterCenter = -5.446;
    double shooterToBotCenter = 1.541;
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
        if (pressed) {finger.setPosition(0.01);}
        else {finger.setPosition(0.07407407407);}
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
        double cos = Math.cos(botHeading);
        double sin = Math.sin(botHeading);

        double xv =  Constants.OdometryConstants.fieldVels.linearVel.x * cos + Constants.OdometryConstants.fieldVels.linearVel.y * sin;
        double yv = Constants.OdometryConstants.fieldVels.linearVel.x * sin + Constants.OdometryConstants.fieldVels.linearVel.y * cos;

        double[] shooterOffsets = new double[]{shooterToBotCenter * Math.sin(botHeading), shooterToBotCenter * Math.cos(botHeading)};
        if (Vision.hasTarget()) {
            double[] limelightOffsets = {limelightToShooterCenter * Math.sin(neckHeading), limelightToShooterCenter * Math.cos(neckHeading)};
            Vector2d llPos = Vision.getPose(neckHeading + offsetAngle);

            // Camera --> shooter
            double botX = llPos.x + limelightOffsets[1] + shooterOffsets[0];
            double botY = llPos.y + limelightOffsets[0] + limelightOffsets[1];

            Constants.OdometryConstants.fieldPos =
                    new Pose2d(botX, botY, Constants.OdometryConstants.fieldPos.heading.toDouble());
            }
            xChange = (targetPose.x) - (Constants.OdometryConstants.fieldPos.position.x - shooterOffsets[0]);
            yChange = (targetPose.y) - (Constants.OdometryConstants.fieldPos.position.y - shooterOffsets[1]);
            //THEOREM OF PYTHAGORAS
            distance = Math.sqrt(Math.pow(xChange, 2) + Math.pow(yChange, 2));
            double time = Math.sqrt(Math.pow((distance - 20) / 39.37, 2) / 9.8) * 2;
            xChange += xv * time;
            yChange += -yv * time;
            distance = Math.sqrt(Math.pow(xChange, 2) + Math.pow(yChange, 2));
            if (Double.isNaN(distance)) return;
            //TOA in the indian princess' name
            headingTarget = Math.atan2(xChange, yChange);
            double angleToTurn;
            angleToTurn = Math.toDegrees(headingTarget + neckHeading);
            aiming(distance, angleToTurn, autoAim);
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

        if (!buttonPressed || !(Math.abs(leftShooter.getVelocity() - shooterVel) < 60)) {
            finger.setPosition(0.01);
        } else {
            finger.setPosition(0.09);
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

    public void aiming(double distance, double angleToTurn, boolean autoAim) {
        //SO MUCH METH MATH THE CRACKHEADS ARE JEALOUS
        distance = Math.min(Math.max(distance, 55), 148);
        angle = Math.max((distance - 30), 0) * 0.4;
        shooterVel = (distance) * 4.77143 + 604.85714;
        double totalTicks = Constants.ShooterConstants.turretNeckGearRatio * Constants.GoBildaMotorMax;
        targetNeckPos = (int) (turretNeckMotor.getCurrentPosition() + xTurn(angleToTurn, 0, distance, 0));
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
        //telemetry.addLine("Shooter");
        //telemetry.addData("Left Shooter Power: ", leftShooter.getPower());
        //telemetry.addData("Right Shooter Power: ", rightShooter.getPower());
        //telemetry.addData("Turret neck pos: ", turretNeckMotor.getCurrentPosition());
        //telemetry.addData("Turret heading: ", neckHeading);
        //telemetry.addData("Turret head pos: ", targetPos);
        //telemetry.addData("Target neck heading: ", headingTarget);
        telemetry.addData("Camera pos: ", Vision.getPose(neckHeading + offsetAngle));
        //telemetry.addData("Offset: ", offset);
        telemetry.addData("Distance: ", distance);
        //telemetry.addData("Target vel: ", shooterVel);
        //telemetry.addData("Current vel", leftShooter.getVelocity());
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
    public void chill(){
        leftShooter.setPower(0);
        rightShooter.setPower(0);
        turretNeckMotor.setPower(0);
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
}