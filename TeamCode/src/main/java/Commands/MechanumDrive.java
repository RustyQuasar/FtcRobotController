package Commands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LazyHardwareMapImu;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

import Subsystems.Localizer;
import Utilities.Constants;
import Utilities.Odometry;

public class MechanumDrive {

    private final DcMotor frontLeft0, frontRight1, backLeft2, backRight3;
    private final BNO055IMU imu;
    private double yawOffset;
    Odometry odometry;
    
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

        imu = hardwareMap.get(BNO055IMU.class, Constants.DriveTrainConstants.imu);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
        this.odometry = odometry;
        yawOffset = imu.getAngularOrientation().firstAngle - Constants.DriveTrainConstants.controlHubOffset;
    }

    public void drive(double driveY, double driveX, double rotation) {

        double botHeading = Constants.heading;
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

    public void updateHeading() {
        double heading = getRawHeading() - yawOffset;
        heading += 180;
        Constants.heading = heading;
    }

    public void periodic(Telemetry telemetry) {
        telemetry.addLine("Drive train");
        telemetry.addData("Heading: ", Constants.heading);
        telemetry.addData("Front Left Power: ", frontLeft0.getPower());
        telemetry.addData("Front Right Power: ", frontRight1.getPower());
        telemetry.addData("Back Left Power: ", backLeft2.getPower());
        telemetry.addData("Back Right Power: ", backRight3.getPower());
    }
            public static class Params {
                // IMU orientation
                // TODO: fill in these values based on
                //   see https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html?highlight=imu#physical-hub-mounting
                public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                        RevHubOrientationOnRobot.LogoFacingDirection.UP;
                public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

                // drive model parameters
                public double inPerTick = 1;
                public double lateralInPerTick = inPerTick;
                public double trackWidthTicks = 0;

                // feedforward parameters (in tick units)
                public double kS = 0;
                public double kV = 0;
                public double kA = 0;

                // path profile parameters (in inches)
                public double maxWheelVel = 50;
                public double minProfileAccel = -30;
                public double maxProfileAccel = 50;

                // turn profile parameters (in radians)
                public double maxAngVel = Math.PI; // shared with path
                public double maxAngAccel = Math.PI;

                // path controller gains
                public double axialGain = 0.0;
                public double lateralGain = 0.0;
                public double headingGain = 0.0; // shared with turn

                public double axialVelGain = 0.0;
                public double lateralVelGain = 0.0;
                public double headingVelGain = 0.0; // shared with turn
            }

            public static Params PARAMS = new Params();

            public final MecanumKinematics kinematics = new MecanumKinematics(
                    PARAMS.inPerTick * PARAMS.trackWidthTicks, PARAMS.inPerTick / PARAMS.lateralInPerTick);

            public final TurnConstraints defaultTurnConstraints = new TurnConstraints(
                    PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);
            public final VelConstraint defaultVelConstraint =
                    new MinVelConstraint(Arrays.asList(
                            kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
                            new AngularVelConstraint(PARAMS.maxAngVel)
                    ));
            public final AccelConstraint defaultAccelConstraint =
                    new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);
            private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

            private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
            private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
            private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
            private final DownsampledWriter mecanumCommandWriter = new DownsampledWriter("MECANUM_COMMAND", 50_000_000);

            public void setDrivePowers(PoseVelocity2d powers) {
                MecanumKinematics.WheelVelocities<Time> wheelVels = new MecanumKinematics(1).inverse(
                        PoseVelocity2dDual.constant(powers, 1));

                double maxPowerMag = 1;
                for (DualNum<Time> power : wheelVels.all()) {
                    maxPowerMag = Math.max(maxPowerMag, power.value());
                }

                frontLeft0.setPower(wheelVels.leftFront.get(0) / maxPowerMag);
                backLeft2.setPower(wheelVels.leftBack.get(0) / maxPowerMag);
                backRight3.setPower(wheelVels.rightBack.get(0) / maxPowerMag);
                frontRight1.setPower(wheelVels.rightFront.get(0) / maxPowerMag);
            }
/*
            public final class FollowTrajectoryAction implements Action {
                public final TimeTrajectory timeTrajectory;
                private double beginTs = -1;

                private final double[] xPoints, yPoints;

                public FollowTrajectoryAction(TimeTrajectory t) {
                    timeTrajectory = t;

                    List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                            0, t.path.length(),
                            Math.max(2, (int) Math.ceil(t.path.length() / 2)));
                    xPoints = new double[disps.size()];
                    yPoints = new double[disps.size()];
                    for (int i = 0; i < disps.size(); i++) {
                        Pose2d p = t.path.get(disps.get(i), 1).value();
                        xPoints[i] = p.position.x;
                        yPoints[i] = p.position.y;
                    }
                }

                @Override
                public boolean run(@NonNull TelemetryPacket p) {
                    double t;
                    if (beginTs < 0) {
                        beginTs = Actions.now();
                        t = 0;
                    } else {
                        t = Actions.now() - beginTs;
                    }

                    if (t >= timeTrajectory.duration) {
                        frontLeft0.setPower(0);
                        backLeft2.setPower(0);
                        backRight3.setPower(0);
                        frontRight1.setPower(0);

                        return false;
                    }

                    Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);
                    targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

                    PoseVelocity2d robotVelRobot = updatePoseEstimate();

                    PoseVelocity2dDual<Time> command = new HolonomicController(
                            PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                            PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
                    )
                            .compute(txWorldTarget, localizer.getPose(), robotVelRobot);
                    driveCommandWriter.write(new DriveCommandMessage(command));

                    MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
                    double voltage = voltageSensor.getVoltage();

                    final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                            PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
                    double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
                    double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
                    double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
                    double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
                    mecanumCommandWriter.write(new MecanumCommandMessage(
                            voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
                    ));

                    frontLeft0.setPower(leftFrontPower);
                    backLeft2.setPower(leftBackPower);
                    backRight3.setPower(rightBackPower);
                    frontRight1.setPower(rightFrontPower);

                    p.put("x", localizer.getPose().position.x);
                    p.put("y", localizer.getPose().position.y);
                    p.put("heading (deg)", Math.toDegrees(localizer.getPose().heading.toDouble()));

                    Pose2d error = txWorldTarget.value().minusExp(localizer.getPose());
                    p.put("xError", error.position.x);
                    p.put("yError", error.position.y);
                    p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

                    // only draw when active; only one drive action should be active at a time
                    Canvas c = p.fieldOverlay();
                    drawPoseHistory(c);

                    c.setStroke("#4CAF50");
                    Drawing.drawRobot(c, txWorldTarget.value());

                    c.setStroke("#3F51B5");
                    Drawing.drawRobot(c, localizer.getPose());

                    c.setStroke("#4CAF50FF");
                    c.setStrokeWidth(1);
                    c.strokePolyline(xPoints, yPoints);

                    return true;
                }

                @Override
                public void preview(Canvas c) {
                    c.setStroke("#4CAF507A");
                    c.setStrokeWidth(1);
                    c.strokePolyline(xPoints, yPoints);
                }
            }


            public final class TurnAction implements Action {
                private final TimeTurn turn;

                private double beginTs = -1;

                public TurnAction(TimeTurn turn) {
                    this.turn = turn;
                }

                @Override
                public boolean run(@NonNull TelemetryPacket p) {
                    double t;
                    if (beginTs < 0) {
                        beginTs = Actions.now();
                        t = 0;
                    } else {
                        t = Actions.now() - beginTs;
                    }

                    if (t >= turn.duration) {
                        frontLeft0.setPower(0);
                        backLeft2.setPower(0);
                        backRight3.setPower(0);
                        frontRight1.setPower(0);

                        return false;
                    }

                    Pose2dDual<Time> txWorldTarget = turn.get(t);
                    targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

                    PoseVelocity2d robotVelRobot = updatePoseEstimate();

                    PoseVelocity2dDual<Time> command = new HolonomicController(
                            PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                            PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
                    )
                            .compute(txWorldTarget, localizer.getPose(), robotVelRobot);
                    driveCommandWriter.write(new DriveCommandMessage(command));

                    MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
                    double voltage = voltageSensor.getVoltage();
                    final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                            PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
                    double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
                    double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
                    double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
                    double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
                    mecanumCommandWriter.write(new MecanumCommandMessage(
                            voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
                    ));

                    frontLeft0.setPower(feedforward.compute(wheelVels.leftFront) / voltage);
                    backLeft2.setPower(feedforward.compute(wheelVels.leftBack) / voltage);
                    backRight3.setPower(feedforward.compute(wheelVels.rightBack) / voltage);
                    frontRight1.setPower(feedforward.compute(wheelVels.rightFront) / voltage);

                    Canvas c = p.fieldOverlay();
                    drawPoseHistory(c);

                    c.setStroke("#4CAF50");
                    Drawing.drawRobot(c, txWorldTarget.value());

                    c.setStroke("#3F51B5");
                    Drawing.drawRobot(c, localizer.getPose());

                    c.setStroke("#7C4DFFFF");
                    c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

                    return true;
                }

                @Override
                public void preview(Canvas c) {
                    c.setStroke("#7C4DFF7A");
                    c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
                }
            }

            public PoseVelocity2d updatePoseEstimate() {
                PoseVelocity2d vel = localizer.update();
                poseHistory.add(localizer.getPose());

                while (poseHistory.size() > 100) {
                    poseHistory.removeFirst();
                }

                estimatedPoseWriter.write(new PoseMessage(localizer.getPose()));


                return vel;
            }

            private void drawPoseHistory(Canvas c) {
                double[] xPoints = new double[poseHistory.size()];
                double[] yPoints = new double[poseHistory.size()];

                int i = 0;
                for (Pose2d t : poseHistory) {
                    xPoints[i] = t.position.x;
                    yPoints[i] = t.position.y;

                    i++;
                }

                c.setStrokeWidth(1);
                c.setStroke("#3F51B5");
                c.strokePolyline(xPoints, yPoints);
            }

            public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
                return new TrajectoryActionBuilder(
                        TurnAction::new,
                        FollowTrajectoryAction::new,
                        new TrajectoryBuilderParams(
                                1e-6,
                                new ProfileParams(
                                        0.25, 0.1, 1e-2
                                )
                        ),
                        beginPose, 0.0,
                        defaultTurnConstraints,
                        defaultVelConstraint, defaultAccelConstraint
                );
            }
        }
        }
        */


    }

