package Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LazyHardwareMapImu;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import Utilities.Constants;
import messages.ThreeDeadWheelInputsMessage;

@Config
public final class ThreeDeadWheelLocalizer {
    public static class Params {
        public double par0YTicks = 0.0; // y position of the first parallel encoder (in tick units)
        public double par1YTicks = 0.0; // y position of the second parallel encoder (in tick units)
        public double perpXTicks = 0.0; // x position of the perpendicular encoder (in tick units)

        public double inPerTick = Constants.OdometryConstants.deadwheelDiameter / Constants.OdometryConstants.externalMax;
    }

    public static Params PARAMS = new Params();

    public final Encoder par0, par1, perp;
    private int lastPar0Pos, lastPar1Pos, lastPerpPos;
    private boolean initialized;
    BNO055IMU imu;
    private double yawOffset;

    public ThreeDeadWheelLocalizer(HardwareMap hardwareMap, Pose2d initialPose) {

        imu = hardwareMap.get(BNO055IMU.class, Constants.DriveTrainConstants.imu);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        // TODO: make sure your config has **motors** with these names (or change them)
        //   the encoders should be plugged into the slot matching the named motor
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        /*
        //Competition encoder ports
        par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, Constants.ShooterConstants.leftShooter1)));
        par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, Constants.ShooterConstants.rightShooter2)));
        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, Constants.ShooterConstants.turretNeckMotor3)));
        */
        //Tuning encoder ports
        par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, Constants.DriveTrainConstants.frontRightMotor0)));
        par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, Constants.DriveTrainConstants.backLeftMotor1)));
        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, Constants.DriveTrainConstants.frontLeftMotor2)));

        // TODO: reverse encoder directions if needed
        //   par0.setDirection(DcMotorSimple.Direction.REVERSE);

        FlightRecorder.write("THREE_DEAD_WHEEL_PARAMS", PARAMS);

        Constants.OdometryConstants.fieldPos = initialPose;
        resetYaw();
    }

    private double getRawHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public void resetYaw() {
        yawOffset = getRawHeading();
    }

    public void update() {
        PositionVelocityPair par0PosVel = par0.getPositionAndVelocity();
        PositionVelocityPair par1PosVel = par1.getPositionAndVelocity();
        PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();

        FlightRecorder.write("THREE_DEAD_WHEEL_INPUTS", new ThreeDeadWheelInputsMessage(par0PosVel, par1PosVel, perpPosVel));

        if (!initialized) {
            initialized = true;

            lastPar0Pos = par0PosVel.position;
            lastPar1Pos = par1PosVel.position;
            lastPerpPos = perpPosVel.position;

            Constants.OdometryConstants.fieldVels =  new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0);
        }

        int par0PosDelta = par0PosVel.position - lastPar0Pos;
        int par1PosDelta = par1PosVel.position - lastPar1Pos;
        int perpPosDelta = perpPosVel.position - lastPerpPos;

        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {
                                (PARAMS.par0YTicks * par1PosDelta - PARAMS.par1YTicks * par0PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                                (PARAMS.par0YTicks * par1PosVel.velocity - PARAMS.par1YTicks * par0PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                        }).times(PARAMS.inPerTick),
                        new DualNum<Time>(new double[] {
                                (PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosDelta - par0PosDelta) + perpPosDelta),
                                (PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosVel.velocity - par0PosVel.velocity) + perpPosVel.velocity),
                        }).times(PARAMS.inPerTick)
                ),
                new DualNum<>(new double[] {
                        (par0PosDelta - par1PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                        (par0PosVel.velocity - par1PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                })
        );

        lastPar0Pos = par0PosVel.position;
        lastPar1Pos = par1PosVel.position;
        lastPerpPos = perpPosVel.position;

        Constants.OdometryConstants.fieldPos = Constants.OdometryConstants.fieldPos.plus(twist.value());
        Constants.OdometryConstants.fieldPos = new Pose2d(Constants.OdometryConstants.fieldPos.position.x, Constants.OdometryConstants.fieldPos.position.y, getRawHeading() - yawOffset);
        Constants.OdometryConstants.fieldVels = twist.velocity().value();
    }
}
