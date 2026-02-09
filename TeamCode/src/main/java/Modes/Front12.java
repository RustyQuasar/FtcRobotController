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

@Autonomous(name = "Red No Gate Autonomous", group = "Red Auto")
public class Front12 extends OpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    public static Follower follower;
    SmartIntake intake;
    SmartShooter3 shooter;
    Vision vision;
    PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10;
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
        Constants.OdometryConstants.fieldPos = new Pose2d(Constants.OdometryConstants.fieldPos.position, followerPose.getHeading(AngleUnit.RADIANS));
        vision.updateAprilTags();
        shooter.aim(false);
        double heading = (follower.getHeading()) + Math.PI * 2;
        heading -= (Math.floor(heading / (Math.PI * 2)) * Math.PI / 2 - Math.PI/2);
        Constants.OdometryConstants.fieldPos = new Pose2d(Constants.OdometryConstants.fieldPos.position, follower.getHeading() + 0.5 * Math.PI);
        if (running) {
            telemetry.addData("Pedro Pos: ", follower.getPose());
            telemetry.addData("Bot pos: ", Constants.OdometryConstants.fieldPos);
            telemetry.addData("Stage: ", currentPath);
            shooter.telemetry(telemetry);
            telemetry.update();
            //also mentions of follower.atParametricEnd() but idk how much to trust that
            if ((!follower.isBusy()) && System.currentTimeMillis() - pathStartTime > pathCooldown) {
                switch(currentPath){
                    case 1:
                        if (!lastTimeSet) {
                            lastTime = System.currentTimeMillis();
                            lastTimeSet = true;
                        }
                        if (System.currentTimeMillis() - lastTime > AutoConstants.closeShootTime + 700) {
                            lastTimeSet = false;
                            follower.followPath(Path2);
                            pathStartTime = System.currentTimeMillis();
                            currentPath = 2;
                            shooter.transfer(false);
                        } else {
                            shooter.transfer(true);
                            intake.intake(true, false);
                        }
                        break;

                        case 2:
                            intake.intake(true, false);
                        follower.followPath(Path3);
                        pathStartTime = System.currentTimeMillis();
                        currentPath = 3;
                        break;
                        case 3:
                            intake.intake(false, false);
                        follower.followPath(Path4);
                            pathStartTime = System.currentTimeMillis();
                        currentPath = 4;
                        break;
                        case 4:
                            if (!lastTimeSet) {
                                lastTime = System.currentTimeMillis();
                                lastTimeSet = true;
                            }
                            if (System.currentTimeMillis() - lastTime > AutoConstants.closeShootTime) {
                                intake.intake(false, false);
                                lastTimeSet = false;
                                follower.followPath(Path5);
                                pathStartTime = System.currentTimeMillis();
                                currentPath = 5;
                                shooter.transfer(false);
                            } else {
                                shooter.transfer(true);
                                intake.intake(true, true);
                            }
                        break;
                        case 5:
                            intake.intake(true, false);
                        follower.followPath(Path6);
                            pathStartTime = System.currentTimeMillis();
                        currentPath = 6;
                        break;
                        case 6:
                            intake.intake(false, false);
                        follower.followPath(Path7);
                            pathStartTime = System.currentTimeMillis();
                        currentPath = 7;
                        break;
                        case 7:
                            if (!lastTimeSet) {
                                lastTime = System.currentTimeMillis();
                                lastTimeSet = true;
                            }
                            if (System.currentTimeMillis() - lastTime > AutoConstants.closeShootTime) {
                                intake.intake(false, false);
                                lastTimeSet = false;
                                follower.followPath(Path8);
                                pathStartTime = System.currentTimeMillis();
                                currentPath = 8;
                                shooter.transfer(false);
                            } else {
                                shooter.transfer(true);
                                intake.intake(true, true);
                            }
                        break;
                        case 8:
                            intake.intake(true, false);
                        follower.followPath(Path9);
                            pathStartTime = System.currentTimeMillis();
                        currentPath = 9;
                        break;
                        case 9:
                            intake.intake(false, false);
                        follower.followPath(Path10);
                            pathStartTime = System.currentTimeMillis();
                        currentPath = 10;
                        break;
                        case 10:
                            pathStartTime = System.currentTimeMillis();
                            if (!lastTimeSet) {
                                lastTime = System.currentTimeMillis();
                                lastTimeSet = true;
                            }if (System.currentTimeMillis() - lastTime > AutoConstants.closeShootTime) {
                                lastTimeSet = false;
                                currentPath = 11;
                            } else {
                            shooter.transfer(true);
                            intake.intake(true, true);
                            }
                        break;
                    default: running = false;
                }
            }
            if (vision.hasTarget()) {
                if (!follower.isBusy() && System.currentTimeMillis() - pathStartTime > pathCooldown + 100 && System.currentTimeMillis() - lastVisionScan > 3000) {

                    lastVisionScan = System.currentTimeMillis();
                    //follower.setPose(PoseConverter.pose2DToPose(new Pose2D(DistanceUnit.INCH, Constants.OdometryConstants.fieldPos.position.x, Constants.OdometryConstants.fieldPos.position.y, AngleUnit.RADIANS, Constants.OdometryConstants.fieldPos.heading.toDouble()), PedroCoordinates.INSTANCE));
                } else {
                    Constants.OdometryConstants.fieldPos = new Pose2d(followerPose.getX(DistanceUnit.INCH), followerPose.getY(DistanceUnit.INCH), followerPose.getHeading(AngleUnit.RADIANS));
                }
            }
        } else {
            shooter.chill();
            intake.intake(false, false);
            shooter.transfer(false);
        }
    }
    @Override
    public void start(){
        follower.setPose(new Pose(x(110), 134, heading(0)));
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
        follower.setStartingPose(new Pose(x(110), 134, heading(0)));
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(111.000, 135.000),

                                new Pose(90.000, 90.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(50))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(90.000, 90.000),

                                new Pose(100.000, 83.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(180))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(100.000, 83.000),

                                new Pose(127.000, 83.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(127.000, 83.000),

                                new Pose(90.000, 90.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(45))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(90.000, 90.000),

                                new Pose(100.000, 60.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(180))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(100.000, 60.000),

                                new Pose(134.000, 60.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(134.000, 60.000),
                                new Pose(109.688, 77.507),
                                new Pose(90.000, 90.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(45))

                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(90.000, 90.000),

                                new Pose(100.000, 36.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(180))

                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(100.000, 36.000),

                                new Pose(134.000, 36.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(134.000, 36.000),

                                new Pose(90.000, 130.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))

                .build();
    }
    private static double x(double offset){
        if (Constants.TEAM.equals("BLUE")) offset += (72-offset) * 2;
        return offset;
    }
    private static double heading(double angle) {
        if (Constants.TEAM.equals("BLUE")) angle += (90-angle) * 2;
        return Math.toRadians(angle);
    }
}