package Modes;

import androidx.annotation.NonNull;

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
        // --- initialize subsystems ---
        dashboard = FtcDashboard.getInstance();
        vision = new Vision(hardwareMap, dashboard);
        shooter = new SmartShooter(hardwareMap, vision);
        intake = new SmartIntake(hardwareMap);
        drive = new MecanumDrive(hardwareMap);
        double artifactY = y(40.133805);
        Action driveSequence;

        int path = 1; // keep your path selection logic here
        if (path == 1) {
            Pose2d startPose = new Pose2d(28.92, y(2.16), heading(90.00));
            odometry = new Odometry(hardwareMap, startPose);

            // NOTE: Do NOT add intake/shooter one-shot toggles as the first actions here.
            //       Toggle intake before the loop and call shooter.aim() in the loop.
            driveSequence = drive.actionBuilder(startPose)
                    // if you want to toggle transfer/shoot at specific times, include TransferCommand below
                    .stopAndAdd(new TransferCommand(shooter, true)) // spin transfer on
                    .waitSeconds(3)
                    .stopAndAdd(new TransferCommand(shooter, false)) // stop transfer
                    .splineTo(new Vector2d(-4.72, y(23.21)), heading(147.96))
                    .splineTo(new Vector2d(-7.87, y(-18.69)), heading(265.70))
                    .splineTo(new Vector2d(30.69, y(-20.26)), heading(-2.34))
                    .splineTo(new Vector2d(64.72, y(24.79)), heading(52.93))
                    .splineTo(new Vector2d(15.15, y(52.52)), heading(150.77))
                    .splineTo(new Vector2d(-41.51, y(53.11)), heading(179.40))
                    .splineTo(new Vector2d(-48.00, y(17.70)), heading(259.61))
                    .splineTo(new Vector2d(-37.18, y(-23.41)), heading(-75.26))
                    .splineTo(new Vector2d(-21.64, y(-59.02)), heading(-66.42))
                    .splineTo(new Vector2d(23.21, y(-52.33)), heading(8.48))
                    .splineTo(new Vector2d(60.39, y(-45.25)), heading(10.78))
                    .build();

        } else {
            odometry = new Odometry(hardwareMap, new Pose2d(0, 0, 0));
            driveSequence = drive.actionBuilder(new Pose2d(0, 0, 0)).build();
        }

        // Wait for start - keep init logic above so dashboard/vision/odometry are ready
        waitForStart();
        if (isStopRequested()) return;

        // HYBRID C1: enable intake once (one-shot toggle) so it runs for the whole auto
        try {
            intake.intake(true); // one-shot: turns intake ON
        } catch (Exception e) {
            telemetry.addData("Intake init err", e.getMessage());
            telemetry.update();
        }

        // Main loop: update odometry, continuously call shooter.aim(), step the driveSequence
        boolean sequenceRunning = true;
        while (opModeIsActive() && sequenceRunning) {
            // 1) Update odometry (must be done every loop so follower has live pose)
            try {
                odometry.update();
            } catch (Exception e) {
                telemetry.addData("Odometry update error", e.getMessage());
            }

            // 2) Update vision (non-blocking)
            try {
                vision.updateAprilTags();
            } catch (Exception e) {
                telemetry.addData("Vision err", e.getMessage());
            }

            // 3) Keep shooter aiming/revving every loop (C1)
            try {
                shooter.aim(); // you said this is the no-arg continuous call
            } catch (Exception e) {
                telemetry.addData("Shooter aim err", e.getMessage());
            }

            // 4) Step the Road Runner drive sequence once per loop; this writes motor powers
            TelemetryPacket packet = new TelemetryPacket();
            try {
                // driveSequence.run returns true while it's active; false when finished
                sequenceRunning = driveSequence.run(packet);
            } catch (Exception e) {
                telemetry.addData("RR Exception", e.getMessage());
                sequenceRunning = false;
            }

            // 5) Send packet to dashboard
            try {
                dashboard.sendTelemetryPacket(packet);
            } catch (Exception e) {
                telemetry.addData("Dashboard send err", e.getMessage());
            }

            // 6) Motor telemetry so you can see if RR actually wrote powers
            try {
                telemetry.addData("lf power", drive.leftFront.getPower());
                telemetry.addData("lb power", drive.leftBack.getPower());
                telemetry.addData("rb power", drive.rightBack.getPower());
                telemetry.addData("rf power", drive.rightFront.getPower());
            } catch (Exception e) {
                telemetry.addData("motor read err", e.getMessage());
            }

            // 7) Pose telemetry
            if (Constants.OdometryConstants.fieldPos != null) {
                telemetry.addData("x", Constants.OdometryConstants.fieldPos.position.x);
                telemetry.addData("y", Constants.OdometryConstants.fieldPos.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(Constants.OdometryConstants.fieldPos.heading.toDouble()));
            } else {
                telemetry.addLine("fieldPos is null");
            }

            telemetry.addData("RR running", sequenceRunning);
            telemetry.update();
        }

        // Sequence finished or opmode ending: stop drive motors & subsystems
        try {
            drive.leftFront.setPower(0.0);
            drive.leftBack.setPower(0.0);
            drive.rightBack.setPower(0.0);
            drive.rightFront.setPower(0.0);
        } catch (Exception e) {
            telemetry.addData("Stop motors err", e.getMessage());
        }

        // Disable intake and shooter transfer to be safe
        try { intake.intake(false); } catch (Exception ignored){}
        try { shooter.transfer(false); } catch (Exception ignored){}

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
    private double artifactX(){
        //These are all minimum distances, gotten from the onshape: https://cad.onshape.com/documents/c7b090d255194e764d0c133c/w/cc69d1642e39bd3ea8a1bb0c/e/29119d697678d4622a7fda34
        if (Arrays.equals(Constants.VisionConstants.colours, new String[] {"N", "N", "N"})){
            return 0;
        } else if (Arrays.equals(Constants.VisionConstants.colours, new String[] {"P", "P", "G"})){
            return -11.884447;
        } else if (Arrays.equals(Constants.VisionConstants.colours, new String[] {"P", "G", "P"})){
            return 11.617313;
        } else if (Arrays.equals(Constants.VisionConstants.colours, new String[] {"G", "P", "P"})){
            return 35.579189;
        } else {
            return 0;
        }
    }
}

/**
 * One-shot transfer action: toggles shooter.transfer(...) when the action runs, then completes immediately.
 * Including these in the drive.builder via stopAndAdd(...) is fine — they are executed sequentially at that point.
 */
class TransferCommand implements Action {
    private final SmartShooter shooter;
    private final boolean transferOn;
    private boolean ran = false;

    public TransferCommand(SmartShooter shooter, boolean transferOn) {
        this.shooter = shooter;
        this.transferOn = transferOn;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (!ran) {
            shooter.transfer(transferOn);
            ran = true;
        }
        // returning false here signals this action is finished and the drive sequence continues
        return false;
    }
}