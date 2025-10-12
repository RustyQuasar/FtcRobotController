package Commands;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Utilities.Constants;
import Subsystems.Odometry;

public class MechanumDrive {

    private final DcMotor frontLeft0, frontRight1, backLeft2, backRight3;
    private Odometry odometry;
    public MechanumDrive(HardwareMap hardwareMap, Odometry odometry) {
        frontLeft0 = hardwareMap.get(DcMotor.class, Constants.DriveTrainConstants.frontLeftMotor0);
        frontRight1 = hardwareMap.get(DcMotor.class, Constants.DriveTrainConstants.frontRightMotor1);
        backLeft2 = hardwareMap.get(DcMotor.class, Constants.DriveTrainConstants.backLeftMotor2);
        backRight3 = hardwareMap.get(DcMotor.class, Constants.DriveTrainConstants.backRightMotor3);
        frontLeft0.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight1.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft2.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight3.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.odometry = odometry;
    }
    public void goToPos(Pose2d targetPos){
        boolean inRange = false;
        while (!inRange) {
            odometry.update();
            double x = targetPos.position.x - Constants.OdometryConstants.fieldPos.position.x;
            double y = targetPos.position.y - Constants.OdometryConstants.fieldPos.position.y;
            double heading = targetPos.heading.toDouble() - Constants.OdometryConstants.fieldPos.heading.toDouble();
            double max = Math.max(Math.max(Math.abs(x), Math.abs(y)), Math.abs(heading));
            /*
            //This is able to get things done, but it feels better to have PID instead
            double threshold = 0.2;
            double multiplier = 1/threshold;
            if (max <= threshold){
                drive(x * multiplier, y/max * multiplier, heading/max * multiplier);
            } else {
                double xPower = x/max;
                double yPower = y/max;
                double headingPower = heading/max;
                drive(xPower, yPower, headingPower);
            }
             */
        }
    }
    public void goToRoughPos(Pose2d targetPos){
        boolean inRange = false;
        while (!inRange) {
            odometry.update();
            double x = targetPos.position.x - Constants.OdometryConstants.fieldPos.position.x;
            double y = targetPos.position.y - Constants.OdometryConstants.fieldPos.position.y;
            double heading = targetPos.heading.toDouble() - Constants.OdometryConstants.fieldPos.heading.toDouble();
            double max = Math.max(Math.max(Math.abs(x), Math.abs(y)), Math.abs(heading));
            drive((y / max), (x / max), (heading / max));
            if (range(targetPos.position.x, Constants.OdometryConstants.fieldPos.position.x, 0.1) && range(targetPos.position.y, Constants.OdometryConstants.fieldPos.position.y, 0.1) && range(targetPos.heading.toDouble(), Constants.OdometryConstants.fieldPos.heading.toDouble(), 0.1)) {
                inRange = true;
            }
            odometry.update();
        }
    }
    public boolean range(double target, double current, double range){
        return Math.abs(current - target) <= range;
    }
    public void drive(double driveY, double driveX, double rotation) {

        double botHeading = Constants.OdometryConstants.fieldPos.heading.toDouble();
        double headingRadians = Math.toRadians(botHeading);

        // Rotate the movement direction counter to the bot's rotation

        double sin = Math.sin(-headingRadians);
        double cos = Math.cos(-headingRadians);

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

    /**
     * Raw heading of the robot before yaw offset is applied
     *
     * @return heading of the robot
     */

    /**
     * Updates the yaw offset to the current heading
     */


    public void periodic(Telemetry telemetry) {
        telemetry.addLine("Drive train");
        telemetry.addData("Heading: ", Constants.OdometryConstants.fieldPos.heading.toDouble());
        telemetry.addData("Front Left Power: ", frontLeft0.getPower());
        telemetry.addData("Front Right Power: ", frontRight1.getPower());
        telemetry.addData("Back Left Power: ", backLeft2.getPower());
        telemetry.addData("Back Right Power: ", backRight3.getPower());
    }
}
