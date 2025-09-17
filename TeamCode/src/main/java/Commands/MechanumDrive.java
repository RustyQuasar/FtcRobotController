package Commands;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class MechanumDrive {

    private final DcMotor frontLeft0;
    private final DcMotor frontRight1;
    private final DcMotor backLeft2;
    private final DcMotor backRight3;

    private final BNO055IMU imu;

    private double yawOffset;

    public MechanumDrive(HardwareMap hardwareMap) {
        frontLeft0 = hardwareMap.get(DcMotor.class, Constants.DriveTrainConstants.frontLeftMotor0);
        frontRight1 = hardwareMap.get(DcMotor.class, Constants.DriveTrainConstants.frontRightMotor1);
        backLeft2 = hardwareMap.get(DcMotor.class, Constants.DriveTrainConstants.backLeftMotor2);
        backRight3 = hardwareMap.get(DcMotor.class, Constants.DriveTrainConstants.backRightMotor3);

        frontLeft0.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight1.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft2.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight3.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
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
        try {
            // ---- 1. Record initial encoder positions ----
            int[] initReading = {
                    frontLeft0.getCurrentPosition(),
                    frontRight1.getCurrentPosition(),
                    backLeft2.getCurrentPosition(),
                    backRight3.getCurrentPosition()
            };

            // ---- 2. Measure time between samples ----
            long startTime = System.currentTimeMillis();
            Thread.sleep(5); // short delay (5ms)
            long endTime = System.currentTimeMillis();
            double deltaTime = (endTime - startTime) / 1000.0; // convert ms → seconds

            // ---- 3. Record final encoder positions ----
            int[] finalReading = {
                    frontLeft0.getCurrentPosition(),
                    frontRight1.getCurrentPosition(),
                    backLeft2.getCurrentPosition(),
                    backRight3.getCurrentPosition()
            };

            // ---- 4. Compute tick differences for each motor ----
            double[] diffs = {
                    finalReading[0] - initReading[0],
                    finalReading[1] - initReading[1],
                    finalReading[2] - initReading[2],
                    finalReading[3] - initReading[3]
            };

            // ---- 5. Convert ticks → linear wheel velocities ----
            double ticksPerRev = 537.7;  // adjust for your motor type (ex: goBILDA 5202/3/4)
            double wheelCircumference = Math.PI * wheelDiameter; // distance per rev
            double[] vWheel = new double[4];

            for (int i = 0; i < 4; i++) {
                double revs = diffs[i] / ticksPerRev; // motor revolutions in deltaTime
                double distance = revs * wheelCircumference * gearRatio;
                vWheel[i] = distance / deltaTime; // velocity of each wheel
            }

            // ---- 6. Apply mecanum math ----
            double forward = (vWheel[0] + vWheel[1] + vWheel[2] + vWheel[3]) / 4.0; //Forward positive, backward negative
            double strafe = (vWheel[0] - vWheel[1] - vWheel[2] + vWheel[3]) / 4.0; //Right positive, left negative

            // ---- 7. Correct for √2 loss in strafe ----
            strafe *= Math.sqrt(2);

            // ---- 8. Return {forward, strafe} ----
            return new double[]{forward, strafe};

        } catch (Exception e) {
            return new double[]{0, 0}; // safe fallback
        }
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
