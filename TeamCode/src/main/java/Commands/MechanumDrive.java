package Commands;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Utilities.Constants;

public class MechanumDrive {

    private final DcMotor frontLeft0, frontRight1, backLeft2, backRight3;
    private final BNO055IMU imu;
    private double headingOffset = 0;
    private double startTicks, startHeading, currentTarget = 0;
    private boolean inMovement, turning = false;
    private final double inPerTick = -39.37/230;
    public MechanumDrive(HardwareMap hardwareMap) {
        frontLeft0 = hardwareMap.get(DcMotor.class, Constants.DriveTrainConstants.frontLeftMotor);
        frontRight1 = hardwareMap.get(DcMotor.class, Constants.DriveTrainConstants.frontRightMotor);
        backLeft2 = hardwareMap.get(DcMotor.class, Constants.DriveTrainConstants.backLeftMotor);
        backRight3 = hardwareMap.get(DcMotor.class, Constants.DriveTrainConstants.backRightMotor);
        frontLeft0.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight1.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft2.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight3.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(BNO055IMU.class, Constants.DriveTrainConstants.imu);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        startTicks = backRight3.getCurrentPosition();
        
    }
    public double getRawHeading() {
        return imu.getAngularOrientation().firstAngle;
    }
    public void resetIMU() {
        headingOffset = getRawHeading();
    }
    public double getHeading() {
        return getRawHeading() - headingOffset + Constants.heading(Math.PI/2);
    }
    public void drive(double driveY, double driveX, double rotation) {
        double botHeading = 0;
                //getHeading() - Constants.heading(Math.PI/2);

        // Rotate the movement direction counter to the bot's rotation
        double sin = Math.sin(-botHeading);
        double cos = Math.cos(-botHeading);

        double fieldOrientedX = driveX * cos - driveY * sin;
        double fieldOrientedY = driveX * sin + driveY * cos;

        double denominator = Math.max(Math.abs(fieldOrientedY) + Math.abs(fieldOrientedX) + Math.abs(rotation), 1);

        double frontLeftPower = (fieldOrientedY + fieldOrientedX + rotation) / denominator;
        double backLeftPower = (fieldOrientedY - fieldOrientedX + rotation) / denominator;
        double frontRightPower = (fieldOrientedY - fieldOrientedX - rotation) / denominator;
        double backRightPower = (fieldOrientedY + fieldOrientedX - rotation) / denominator;

        frontLeft0.setPower(frontLeftPower);
        frontRight1.setPower(frontRightPower);
        backLeft2.setPower(backLeftPower);
        backRight3.setPower(backRightPower);
    }
    public boolean setDistance(double currentTarget){
        if (Double.isNaN(backRight3.getCurrentPosition())) return false;
        currentTarget /= inPerTick;
        if (!inMovement){
            this.currentTarget = currentTarget;
            startTicks = backRight3.getCurrentPosition();
            inMovement = true;
        }
            if (startTicks + currentTarget >= backRight3.getCurrentPosition()){
                drive(-0.15 , 0, 0);
                return false;
            } else if (startTicks + currentTarget <= backRight3.getCurrentPosition() - 15){
                drive(0.15, 0, 0);
                return false;
            } else {
                drive(0, 0, 0);
                inMovement = false;
                return true;
            }
    }
    public boolean setHeading(double targetHeading){
        if (!turning){
            turning = true;
            startHeading = getHeading();
        }
        targetHeading += startHeading;
        if (Math.abs(getHeading() - targetHeading) < 0.1){
            turning = false;
            return true;
        } else {
            drive(0, 0, 0.13);
            return false;
        }
    }


    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Front Left Power: ", frontLeft0.getPower());
        telemetry.addData("Front Right Power: ", frontRight1.getPower());
        telemetry.addData("Back Left Power: ", backLeft2.getPower());
        telemetry.addData("Back Right Power: ", backRight3.getPower());
        telemetry.addData("Wheel pos: ", backRight3.getCurrentPosition());
        telemetry.addData("Start ticks: ", startTicks);
        telemetry.addData("Current target: ", currentTarget);
    }
}