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

@Autonomous(name = "Back 6", group = "Auto")
public class Back6 extends OpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    public static Follower follower;
    SmartIntake intake;
    SmartShooter3 shooter;
    Vision vision;
    PathChain Path1, Path2, Path3, Path4;
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
        Constants.OdometryConstants.fieldPos = new Pose2d(Constants.OdometryConstants.fieldPos.position, followerPose.getHeading(AngleUnit.RADIANS) + Constants.OdometryConstants.startHeading * 2);
        vision.updateAprilTags();
        if (running) {
            shooter.aim(true);
            telemetry.addData("Pedro Pos: ", follower.getPose());
            telemetry.addData("Bot pos: ", Constants.OdometryConstants.fieldPos);
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
                        if (System.currentTimeMillis() - lastTime > AutoConstants.farShootTime) {
                            lastTimeSet = false;
                            follower.followPath(Path1);
                            pathStartTime = System.currentTimeMillis();
                            currentPath = 2;
                            shooter.transfer(false);
                        } else {
                            shooter.transfer(true);
                        }
                        break;
                    case 2:
                        follower.followPath(Path2);
                        pathStartTime = System.currentTimeMillis();
                        currentPath = 3;
                        break;
                    case 3:
                        follower.followPath(Path3);
                        pathStartTime = System.currentTimeMillis();
                        currentPath = 4;
                        break;
                    case 4:
                        pathStartTime = System.currentTimeMillis();
                        follower.followPath(Path4);
                        currentPath = 5;
                        break;
                    case 5:
                        if (!lastTimeSet){
                            lastTime = System.currentTimeMillis();
                            lastTimeSet = true;
                        }
                        if (System.currentTimeMillis() - lastTime > AutoConstants.farShootTime) {
                            pathStartTime = System.currentTimeMillis();
                            currentPath = 6;
                            lastTimeSet = false;
                        } else {
                            shooter.transfer(true);
                            intake.intake(true, false);
                        }
                        break;
                    default: running = false;
                }
            }
        } else {
            shooter.chill();
            intake.intake(false, false);
        }
    }
    @Override
    public void start(){
        follower.setPose(new Pose(x(64), 9, heading(180)));
        pathStartTime = System.currentTimeMillis();
        shooter.aim(false);
    }
    @Override
    public void init() {
        vision = new Vision(hardwareMap);
        shooter = new SmartShooter3(hardwareMap, vision);
        intake = new SmartIntake(hardwareMap);
        follower = AutoConstants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(x(64), 9, heading(180)));
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(x(64.000), 9.000),

                                new Pose(x(50.000), 20.000)
                        )
                ).setLinearHeadingInterpolation(heading(180), heading(0))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(x(50.000), 20.000),

                                new Pose(x(25.000), 22.000)
                        )
                ).setLinearHeadingInterpolation(heading(0), heading(30))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(x(25.000), 22.000),

                                new Pose(x(10.000), 9.000)
                        )
                ).setLinearHeadingInterpolation(heading(30), heading(90))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(x(10.000), 9.000),

                                new Pose(x(58.000), 15.000)
                        )
                ).setLinearHeadingInterpolation(heading(90), heading(180))

                .build();
    }
    private static double x(double offset){
        if (Constants.TEAM.equals("RED")) offset = 145 - offset;
        return offset;
    }
    private static double heading(double angle) {
        if (Constants.TEAM.equals("RED")) angle += (90-angle) * 2;
        return Math.toRadians(angle);
    }
}