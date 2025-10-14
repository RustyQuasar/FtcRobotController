package Modes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import Commands.MecanumDrive;
import Subsystems.Odometry;

@Autonomous(name = "RoadRunner Auto Sample", group = "RoadRunner")
public class Auto extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d startPose = new Pose2d(0, 0, 0);
        Pose2d targetPose = new Pose2d(30, 20, Math.toRadians(90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Odometry(hardwareMap, startPose));

        waitForStart();
        if (isStopRequested()) return;

        // Build an Action (trajectory)
        Action action = drive.actionBuilder(startPose)
                .splineToLinearHeading(targetPose, targetPose.heading)
                .build();

        // Run the Action until complete
        TelemetryPacket packet = new TelemetryPacket();
        while (opModeIsActive()) {
            boolean stillRunning = action.run(packet);
            if (!stillRunning) break;  // stop when the trajectory finishes
            telemetry.addLine("Running trajectory...");
            telemetry.update();
        }

        telemetry.addLine("Trajectory complete!");
        telemetry.update();
    }
}
