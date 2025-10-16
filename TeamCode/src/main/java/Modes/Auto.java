package Modes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
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
            //A bit off dead center x, against bottom of y
            Pose2d startPose = new Pose2d(x(Constants.Sizes.robotOffset), Constants.Sizes.robotOffset, 0);
            //Quick sample
            Pose2d target1 = new Pose2d(x(1), 1, heading(90));
            Pose2d target2 = new Pose2d(x(60), 60, heading(270));
            odometry = new Odometry(hardwareMap, startPose);
            sequence = drive.actionBuilder(startPose)
                    .splineToLinearHeading(target1, target1.heading)
                    .splineToLinearHeading(target2, target2.heading)
                    .build();
        } else if (path == 2) {
                Pose2d startPose = new Pose2d(0, 0, 0);
                sequence = drive.actionBuilder(startPose)
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
    private double x(double offset){
        if (Constants.TEAM.equals("BLUE")){
            offset *= -1;
        }
        return (offset + Constants.Sizes.field/2);
    }

    private double heading(double angle) {
        //TODO This kinda sketchy ngl
        if (Constants.TEAM.equals("RED")) {
            return Math.toRadians(angle);
        } else {
            return Math.toRadians(angle - 2 * (180 - angle));
        }
    }
    private double targetRow(){
        if (Arrays.equals(Constants.VisionConstants.colours, new String[] {"U", "U", "U"})){
            return Constants.Sizes.field/2;
        } else if (Arrays.equals(Constants.VisionConstants.colours, new String[] {"G", "P", "P"})){
            return 35.066;
        } else if (Arrays.equals(Constants.VisionConstants.colours, new String[] {"P", "G", "P"})){
            return 58.604;
        } else if (Arrays.equals(Constants.VisionConstants.colours, new String[] {"P", "P", "G"})){
            return 82.323;
        }
    }
}
