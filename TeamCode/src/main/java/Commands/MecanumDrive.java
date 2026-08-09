package Commands;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Subsystems.PIDFController;
import Subsystems.WrappingPIDFController;
import Utilities.ConfigVariables;
import Utilities.Constants;

public class MecanumDrive {

    private final DcMotor frontLeft, frontRight, backLeft, backRight;
    private double lastFL, lastFR, lastBL, lastBR;
    private final BNO055IMU imu;
    private double headingOffset = 0;
    private double headingTarget = 0;
    private WrappingPIDFController autoAlignment;
    private ElapsedTime rotationTimer = new ElapsedTime();
    private boolean driving = true;
    private final double powerRatio = 0.6 / 0.4;

    public MecanumDrive(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotor.class, Constants.DriveTrainConstants.frontLeftMotor);
        frontRight = hardwareMap.get(DcMotor.class, Constants.DriveTrainConstants.frontRightMotor);
        backLeft = hardwareMap.get(DcMotor.class, Constants.DriveTrainConstants.backLeftMotor);
        backRight = hardwareMap.get(DcMotor.class, Constants.DriveTrainConstants.backRightMotor);

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(BNO055IMU.class, Constants.DriveTrainConstants.imu);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        autoAlignment = new WrappingPIDFController(Constants.DriveTrainConstants.autoAlignmentP, Constants.DriveTrainConstants.autoAlignmentI, Constants.DriveTrainConstants.autoAlignmentD, Constants.DriveTrainConstants.autoAlignmentF, Constants.DriveTrainConstants.autoAlignmentTolerance,2 * Math.PI);
    }

    public double getRawHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    public void resetIMU() {
        headingOffset = getRawHeading();
        headingTarget = getHeading();
    }

    public double getHeading() {
        return getRawHeading() - headingOffset;
    }

    public void drive(double driveY, double driveX, double rotation) {
        double botHeading = getHeading();
        if (Math.abs(rotation) <0.05) {
            if (rotationTimer.time() > 0.2) {
                if (driving) {
                    headingTarget = botHeading;
                    driving = false;
                }
                if (Math.abs(headingTarget - botHeading) > Constants.DriveTrainConstants.autoAlignmentTolerance) {
                    rotation = -autoAlignment.calculate(headingTarget, botHeading);
                } else {
                    rotation = 0;
                }
            }
        } else {
            rotationTimer.reset();
            driving = true;
        }
            double sin = Math.sin(-botHeading);
            double cos = Math.cos(-botHeading);

            double fieldX = driveX * cos - driveY * sin;
            double fieldY = driveX * sin + driveY * cos;
            double denominator = Math.max(Math.abs(fieldY) + Math.abs(fieldX) + Math.abs(rotation), 1);
            double frontLeftPower = (fieldY + fieldX + rotation) / denominator;
            double backLeftPower = (fieldY - fieldX + rotation) / denominator;
            double frontRightPower = (fieldY - fieldX - rotation) / denominator;
            double backRightPower = (fieldY + fieldX - rotation) / denominator;

        if (lastFL != frontLeftPower) {
            frontLeft.setPower(frontLeftPower);
            lastFL = frontLeftPower;
        }
        if (lastFR != frontRightPower) {
            frontRight.setPower(frontRightPower);
            lastFR = frontRightPower;
        }
        if (lastBL != backLeftPower) {
            backLeft.setPower(backLeftPower / powerRatio);
            lastBL = backLeftPower;
        }
        if (lastBR != backRightPower) {
            backRight.setPower(backRightPower / powerRatio);
            lastBR = backRightPower;
        }
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Front Left Power: ", frontRight.getPower());
//        telemetry.addData("Front Right Power: ", frontRight.getPower());
        telemetry.addData("Back Left Power: ", backLeft.getPower());
//        telemetry.addData("Back Right Power: ", backRight.getPower());
//        telemetry.addData("Current heading: ", getHeading());
//        telemetry.addData("Target heading: ", headingTarget);
//        telemetry.addData("IMU velocities: ", imu.getVelocity().xVeloc);
    }
}