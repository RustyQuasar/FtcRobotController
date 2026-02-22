package Modes.Autos;

import com.acmerobotics.roadrunner.Pose2d;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.*;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import Commands.SmartIntake;
import Commands.SmartShooter3;
import Subsystems.Vision;
import Utilities.AutoConstants;
import Utilities.Constants;

public class Back9 {
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
    public void loop() {
        follower.update();
        Pose2D followerPose = PoseConverter.poseToPose2D(follower.getPose(), FTCCoordinates.INSTANCE);
        Constants.OdometryConstants.fieldPos = new Pose2d(Constants.OdometryConstants.fieldPos.position, followerPose.getHeading(AngleUnit.RADIANS) - Math.PI);
        vision.updateAprilTags();
        if (running) {
            shooter.aim(true, false);
            //also mentions of follower.atParametricEnd() but idk how much to trust that
            if ((!follower.isBusy()) && System.currentTimeMillis() - pathStartTime > pathCooldown || (System.currentTimeMillis() - pathStartTime> 3000)) {
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
                        if (!lastTimeSet){
                            lastTime = System.currentTimeMillis();
                            lastTimeSet = true;
                        }
                        if (System.currentTimeMillis() - lastTime > AutoConstants.farShootTime) {
                            shooter.transfer(false);
                            pathStartTime = System.currentTimeMillis();
                            currentPath = 5;
                            lastTimeSet = false;
                            follower.followPath(Path4);
                        } else {
                            shooter.transfer(true);
                            intake.intake(true, false);
                        }
                        break;
                    case 5:
                        follower.followPath(Path5);
                        pathStartTime = System.currentTimeMillis();
                        currentPath = 6;
                        break;
                    case 6:
                        pathStartTime = System.currentTimeMillis();
                        follower.followPath(Path6);
                        currentPath = 7;
                        break;
                    case 7:
                        if (!lastTimeSet){
                            lastTime = System.currentTimeMillis();
                            lastTimeSet = true;
                        }
                        if (System.currentTimeMillis() - lastTime > AutoConstants.farShootTime) {
                            shooter.transfer(false);
                            pathStartTime = System.currentTimeMillis();
                            currentPath = 8;
                            lastTimeSet = false;
                            follower.followPath(Path7);
                        } else {
                            shooter.transfer(true);
                            intake.intake(true, false);
                        }
                        break;
                    case 8:
                        pathStartTime = System.currentTimeMillis();
                        follower.followPath(Path8);
                        currentPath = 9;
                        break;
                    case 9:
                        pathStartTime = System.currentTimeMillis();
                        follower.followPath(Path9);
                        currentPath = 10;
                        break;
                    case 10:
                        if (!lastTimeSet){
                            lastTime = System.currentTimeMillis();
                            lastTimeSet = true;
                        }
                        if (System.currentTimeMillis() - lastTime > AutoConstants.farShootTime) {
                            shooter.transfer(false);
                            pathStartTime = System.currentTimeMillis();
                            currentPath = 11;
                            lastTimeSet = false;
                            follower.followPath(Path10);
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

    public void init_loop(){
        pathStartTime = System.currentTimeMillis();
        vision.updateAprilTags();
    }

    public void init(HardwareMap hardwareMap, String team) {
        Constants.TEAM = team;
        vision = new Vision(hardwareMap);
        shooter = new SmartShooter3(hardwareMap, vision);
        intake = new SmartIntake(hardwareMap);
        follower = AutoConstants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose( x(64), 9, heading(180)));
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose( x(64.000), 9.000),

                                new Pose( x(50.000), 35.000)
                        )
                ).setLinearHeadingInterpolation(heading(180), heading(0))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose( x(50.000), 35.000),

                                new Pose( x(14.000), 36.000)
                        )
                ).setConstantHeadingInterpolation(heading(0))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose( x(14.000), 36.000),

                                new Pose( x(62.000), 20.000)
                        )
                ).setLinearHeadingInterpolation(heading(0), heading(10))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose( x(62.000), 20.000),

                                new Pose( x(35.000), 28.000)
                        )
                ).setLinearHeadingInterpolation(heading(10), heading(-20))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose( x(35.000), 28.000),
                                new Pose( x(10.000), 30.000),
                                new Pose( x(10.000), 10.000)
                        )
                ).setLinearHeadingInterpolation(heading(-20), heading(90))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose( x(10.000), 10.000),

                                new Pose( x(60.000), 15.000)
                        )
                ).setLinearHeadingInterpolation(heading(90), heading(45))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose( x(60.000), 15.000),

                                new Pose( x(42.000), 15.000)
                        )
                ).setLinearHeadingInterpolation(heading(45), heading(0))

                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose( x(42.000), 15.000),

                                new Pose( x(12.000), 10.000)
                        )
                ).setConstantHeadingInterpolation(heading(0))

                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose( x(12.000), 10.000),

                                new Pose( x(60.000), 18.000)
                        )
                ).setLinearHeadingInterpolation(heading(0), heading(180))

                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose( x(60.000), 18.000),

                                new Pose( x(40.000), 18.000)
                        )
                ).setConstantHeadingInterpolation(180)

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