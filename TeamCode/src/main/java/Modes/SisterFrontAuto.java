package Modes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import Commands.SmartIntake;
import Commands.SmartShooter3;
import Subsystems.Vision;
import Utilities.AutoConstants;
import Utilities.Constants;

@Autonomous(name = "Front 2 Gate", group = "Auto")
public class SisterFrontAuto extends OpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    public static Follower follower;
    SmartIntake intake;
    SmartShooter3 shooter;
    Vision vision;
    PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, Path11, Path12;
    int currentPath = 1;
    double lastTime = 10;
    double pathStartTime = 0;
    boolean lastTimeSet = false;
    boolean running = true;
    double pathCooldown = 1000;
    long lastVisionScan = 0;
    Telemetry telemetry = dashboard.getTelemetry();
    @Override
    public void loop() {
        follower.update();
        Pose2D followerPose = PoseConverter.poseToPose2D(follower.getPose(), FTCCoordinates.INSTANCE);
        Constants.OdometryConstants.fieldPos = new Pose2d(Constants.OdometryConstants.fieldPos.position, followerPose.getHeading(AngleUnit.RADIANS) - Math.PI);
        vision.updateAprilTags();
        shooter.aim(true);
        if (running) {
            telemetry.addData("Pedro Pos: ", follower.getPose());
            telemetry.addData("Bot pos: ", Constants.OdometryConstants.fieldPos.position);
            telemetry.addData("Bot heading: ", Constants.OdometryConstants.fieldPos.heading.toDouble());
            telemetry.addData("Stage: ", currentPath);
            shooter.telemetry(telemetry);
            telemetry.update();
            //also mentions of follower.atParametricEnd() but idk how much to trust that
            if ((!follower.isBusy()) && System.currentTimeMillis() - pathStartTime > pathCooldown) {
                switch(currentPath){
                    case 1:
                        intake.intake(true, false);
                        if (!lastTimeSet) {
                            lastTime = System.currentTimeMillis();
                            lastTimeSet = true;
                        }
                        if (System.currentTimeMillis() - lastTime > AutoConstants.closeShootTime) {
                            lastTimeSet = false;
                            follower.followPath(Path2);
                            pathStartTime = System.currentTimeMillis();
                            currentPath = 2;
                            shooter.transfer(false);
                        } else {
                            shooter.transfer(true);
                        }
                        break;
                    case 2:
                        follower.followPath(Path3);
                        pathStartTime = System.currentTimeMillis();
                        currentPath = 3;
                        break;
                    case 3:
                        follower.followPath(Path4);
                        pathStartTime = System.currentTimeMillis();
                        currentPath = 4;
                        break;
                    case 4:
                        if (!lastTimeSet){
                            lastTime = System.currentTimeMillis();
                            lastTimeSet = true;
                        }
                        if (System.currentTimeMillis() - lastTime > AutoConstants.gateHoldTime) {
                            follower.followPath(Path5);
                            pathStartTime = System.currentTimeMillis();
                            currentPath = 5;
                            lastTimeSet = false;
                        }
                        break;
                    case 5:
                        if (!lastTimeSet) {
                            lastTime = System.currentTimeMillis();
                            lastTimeSet = true;
                        }
                        if (System.currentTimeMillis() - lastTime > AutoConstants.closeShootTime) {
                            lastTimeSet = false;
                            follower.followPath(Path6);
                            pathStartTime = System.currentTimeMillis();
                            currentPath = 6;
                            shooter.transfer(false);
                        } else {
                            shooter.transfer(true);
                        }
                        break;
                    case 6:
                        follower.followPath(Path7);
                        pathStartTime = System.currentTimeMillis();
                        currentPath = 7;
                        break;
                    case 7:
                            follower.followPath(Path8);
                            pathStartTime = System.currentTimeMillis();
                            currentPath = 8;
                        break;
                    case 8:
                        if (!lastTimeSet){
                            lastTime = System.currentTimeMillis();
                            lastTimeSet = true;
                        }
                        if (System.currentTimeMillis() - lastTime > AutoConstants.gateHoldTime) {
                            follower.followPath(Path9);
                            pathStartTime = System.currentTimeMillis();
                            currentPath = 9;
                            lastTimeSet = false;
                        }
                        break;
                    case 9:
                        if (!lastTimeSet) {
                            lastTime = System.currentTimeMillis();
                            lastTimeSet = true;
                        }
                        if (System.currentTimeMillis() - lastTime > AutoConstants.closeShootTime) {
                            lastTimeSet = false;
                            follower.followPath(Path10);
                            pathStartTime = System.currentTimeMillis();
                            currentPath = 10;
                            shooter.transfer(false);
                        } else {
                            shooter.transfer(true);
                        }
                        break;
                    case 10:
                        pathStartTime = System.currentTimeMillis();
                        follower.followPath(Path11);
                        currentPath = 11;
                        break;
                    case 11:
                        pathStartTime = System.currentTimeMillis();
                        follower.followPath(Path12);
                        currentPath = 12;
                        break;
                    case 12:
                        if (!lastTimeSet) {
                            lastTime = System.currentTimeMillis();
                            lastTimeSet = true;
                        }
                        if (System.currentTimeMillis() - lastTime > AutoConstants.closeShootTime) {
                            lastTimeSet = false;
                            pathStartTime = System.currentTimeMillis();
                            currentPath = 13;
                            shooter.transfer(false);
                        } else {
                            shooter.transfer(true);
                        }
                    default: running = false;
                }
            }
        } else {
            intake.intake(true, false);
            shooter.transfer(true);
        }
    }
    @Override
    public void start(){
        follower.setPose(new Pose(x(29), 135, heading(180)));
        pathStartTime = System.currentTimeMillis();
        follower.followPath(Path1);
        shooter.aim(false);
    }
    @Override
    public void init() {
        vision = new Vision(hardwareMap);
        shooter = new SmartShooter3(hardwareMap, vision);
        intake = new SmartIntake(hardwareMap);
        follower = AutoConstants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(x(29), 135, heading(180)));
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(x(29.000), 135.000),

                                new Pose(x(50.000), 86.000)
                        )
                ).setLinearHeadingInterpolation(heading(180), heading(45))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(x(50.000), 86.000),

                                new Pose(x(41.000), 84.000)
                        )
                ).setLinearHeadingInterpolation(heading(45), heading(0))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(x(41.000), 84.000),

                                new Pose(x(15.000), 84.000)
                        )
                ).setConstantHeadingInterpolation(heading(0))
                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(x(15.00), 84.000),
                                new Pose(x(27.875), 79.121),
                                new Pose(x(10.5), 77.000)
                        )
                ).setLinearHeadingInterpolation(heading(0), heading(0))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(x(10.5), 77.000),

                                new Pose(x(50.000), 86.000)
                        )
                ).setLinearHeadingInterpolation(heading(0), heading(45))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(x(50.000), 86.000),

                                new Pose(x(41.000), 62)
                        )
                ).setLinearHeadingInterpolation(heading(45), heading(0))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(x(41.000), 62),

                                new Pose(x(9.500), 60)
                        )
                ).setLinearHeadingInterpolation(heading(0), heading(0))

                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(x(9.000), 60),
                                new Pose(x(31.040), 57.332),
                                new Pose(x(10.5), 70)
                        )
                ).setLinearHeadingInterpolation(heading(0), heading(0))

                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(x(10.5), 70.000),

                                new Pose(x(50.000), 86.000)
                        )
                ).setLinearHeadingInterpolation(heading(0), heading(45))

                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(x(50.000), 86.000),

                                new Pose(x(41.000), 40)
                        )
                ).setLinearHeadingInterpolation(heading(45), heading(0))

                .build();

        Path11 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(x(41.000), 40),

                                new Pose(x(8.000), 38)
                        )
                ).setConstantHeadingInterpolation(heading(0))
                .build();

        Path12 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(x(8.000), 38.000),
                                new Pose(x(50.306), 103.466)
                        )
                ).setLinearHeadingInterpolation(heading(0), heading(180))

                .build();
    }
    private static double x(double offset){
        if (Constants.TEAM.equals("RED")) offset = 144 - offset;
        return offset;
    }
    private static double heading(double angle) {
        if (Constants.TEAM.equals("RED")) angle += (90-angle) * 2;
        return Math.toRadians(angle);
    }
}