package Commands;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import Utilities.Constants;

public class MechanumDrive {

    private final DcMotorEx frontLeft0;
    private final DcMotorEx frontRight1;
    private final DcMotorEx backLeft2;
    private final DcMotorEx backRight3;

    private final BNO055IMU imu;

    private double yawOffset;

    public MechanumDrive(HardwareMap hardwareMap) {
        frontLeft0 = hardwareMap.get(DcMotorEx.class, Constants.DriveTrainConstants.frontLeftMotor0);
        frontRight1 = hardwareMap.get(DcMotorEx.class, Constants.DriveTrainConstants.frontRightMotor1);
        backLeft2 = hardwareMap.get(DcMotorEx.class, Constants.DriveTrainConstants.backLeftMotor2);
        backRight3 = hardwareMap.get(DcMotorEx.class, Constants.DriveTrainConstants.backRightMotor3);

        frontLeft0.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight1.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft2.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight3.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(BNO055IMU.class, Constants.DriveTrainConstants.imu);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        yawOffset = imu.getAngularOrientation().firstAngle - Constants.DriveTrainConstants.controlHubOffset;
    }

    public void drive(double driveY, double driveX, double rotation) {

        double botHeading = getHeading();
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
    public double getRawHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    /**
     * Updates the yaw offset to the current heading
     */
    public void resetYaw() {
        yawOffset = getRawHeading() - Constants.DriveTrainConstants.controlHubOffset;
    }

    /**
     * Returns the updated yaw after the yaw offset is applied
     *
     * @return adjusted heading of the robot
     */
    public double getHeading() {
        double heading = getRawHeading() - yawOffset;

        if (heading > 180) {
            heading -= 360;
        }
        if (heading < -180) {
            heading += 360;
        }

        return heading;
    }

    /**
     * Returns {forwardVelocity, strafeVelocity} in distance units per second.
     *
     * @param wheelDiameter Diameter of wheel (same unit as desired velocity output)
     * @param gearRatio     Gear ratio (wheel revs per motor rev)
     * @return double[] with {forward, strafe} velocities
     */

    public double[] getDrivetrainVelocities(double wheelDiameter, double gearRatio) {
        double[] TicksPerSecond = {frontLeft0.getVelocity(), frontRight1.getVelocity(), backLeft2.getVelocity(), backRight3.getVelocity()};
        double wheelCircumference = Math.PI * wheelDiameter; // distance per rev
        double[] vWheel = new double[4];
        for (int i = 0; i < 4; i++) {
            double distance = wheelCircumference * gearRatio;
            vWheel[i] = distance * TicksPerSecond[i] / Constants.DCMotorMax;
        }
        double forward = (vWheel[0] + vWheel[1] + vWheel[2] + vWheel[3]) / 4.0; //Forward positive, backward negative
        double strafe = (vWheel[0] - vWheel[1] - vWheel[2] + vWheel[3]) / 4.0; //Right positive, left negative
        strafe *= Math.sqrt(2);
        return new double[]{forward, strafe};

    }

    public void periodic(Telemetry telemetry) {
        telemetry.addLine("Drive train");
        telemetry.addData("Heading: ", getHeading());
        telemetry.addData("Front Left Power: ", frontLeft0.getPower());
        telemetry.addData("Front Right Power: ", frontRight1.getPower());
        telemetry.addData("Back Left Power: ", backLeft2.getPower());
        telemetry.addData("Back Right Power: ", backRight3.getPower());
    }
}
