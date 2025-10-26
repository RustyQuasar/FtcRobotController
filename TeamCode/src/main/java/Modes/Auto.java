package Modes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Arrays;

import Commands.MecanumDrive;
import Commands.SmartIntake;
import Commands.SmartShooter;
import Commands.Vision;
import Subsystems.Odometry;
import Utilities.Constants;

@Autonomous
public class Auto extends LinearOpMode {
    FtcDashboard dashboard;
    Odometry odometry;
    SmartIntake intake;
    SmartShooter shooter;
    MecanumDrive drive;
    Vision vision;
    @Override
    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();
        vision = new Vision(hardwareMap, dashboard);
        drive = new MecanumDrive(hardwareMap);
        shooter = new SmartShooter(hardwareMap, Constants.TEAM, vision);
        intake = new SmartIntake(hardwareMap);
        Action sequence;

        waitForStart();
        if (isStopRequested()) return;
        int path = 1;
        if (path == 1) {

        } else if (path == 2) {
                sequence = drive.actionBuilder(new Pose2d(-11.02, 0.39, Math.toRadians(90.00)))
                    .splineTo(new Vector2d(29.51, y(-2.16)), heading(-3.61))
                    .splineToSplineHeading(new Pose2d(-0.20, y(-19.48), heading(210.23)), heading(210.23))
                    .splineToLinearHeading(new Pose2d(-36.00, y(-25.18), heading(189.05)), heading(189.05))
                    .splineToConstantHeading(new Vector2d(-58.23, y(6.49)), heading(125.06))
                    .build();
        } else {
            stop();
            sequence = drive.actionBuilder(new Pose2d(0, 0, 0))
                    .build();
        }

        TelemetryPacket packet = new TelemetryPacket();

        // --- Run the action with continuous odometry updates ---
        while (opModeIsActive()) {
            // Update odometry from your Constants class (if implemented there)
            odometry.update();
            vision.updateAprilTags();
            intake.intake(true);
            shooter.aim(new double[] {Constants.OdometryConstants.fieldVels.linearVel.x, Constants.OdometryConstants.fieldVels.linearVel.y});
            if (odometry.isInTriangle()){
                shooter.transfer();
            }
            // Run the Road Runner action
            boolean running = sequence.run(packet);
            if (!running) break; // trajectory finished

            // You can add other side logic here (arm control, sensor reads, etc.)
            telemetry.addData("x", Constants.OdometryConstants.fieldPos.position.x);
            telemetry.addData("y", Constants.OdometryConstants.fieldPos.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(Constants.OdometryConstants.fieldPos.heading.toDouble()));
            telemetry.update();
        }

        telemetry.addLine("Sequence complete!");
        telemetry.update();
    }
    private double y(double offset){
        if (Constants.TEAM.equals("BLUE")){
            offset *= -1;
        }
        return offset;
    }
    private double heading(double angle) {
        double angleR = Math.toRadians(angle);
        if (Constants.TEAM.equals("BLUE")) angleR *= -1;
        return angleR;
    }
    private double targetColumn(){
        if (Arrays.equals(Constants.VisionConstants.colours, new String[] {"U", "U", "U"})){
            return Constants.Sizes.field/2;
        } else if (Arrays.equals(Constants.VisionConstants.colours, new String[] {"G", "P", "P"})){
            return 35.066;
        } else if (Arrays.equals(Constants.VisionConstants.colours, new String[] {"P", "G", "P"})){
            return 58.604;
        } else if (Arrays.equals(Constants.VisionConstants.colours, new String[] {"P", "P", "G"})){
            return 82.323;
        } else {
            return 0;
        }
    }
}
