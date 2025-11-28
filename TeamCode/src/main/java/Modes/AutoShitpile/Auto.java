package Modes.AutoShitpile;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import Commands.SmartIntake;
import Commands.SmartShooter3;
import Utilities.Constants;

public final class Auto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Object TuningOpModes = null;
        int side =1;
        Pose2d beginPose = new Pose2d(300, side*10, 0);
        if(Constants.TEAM.equals("blue")){side=-1;}

        {
            Mechanum drive = new Mechanum(hardwareMap, beginPose);
            SmartIntake intake = new SmartIntake(hardwareMap);
            SmartShooter3 shooter = new SmartShooter3(hardwareMap);
            waitForStart();
                shooter.aim();

                    drive.actionBuilder(beginPose)


                            .stopAndAdd(shooter.transfer(true))
                            .waitSeconds(0.5)
                            .stopAndAdd(shooter.transfer(true))
                            .waitSeconds(0.5)
                            .stopAndAdd(shooter.transfer(true))
                            .splineTo(new Vector2d(300, side*300), Math.toRadians(side*270))
                            .waitSeconds(0.5)
                            .splineTo(new Vector2d(300, side*10), Math.toRadians(side*270))
                            .stopAndAdd(shooter.transfer(true))
                            .waitSeconds(0.5)
                            .stopAndAdd(shooter.transfer(true))
                            .waitSeconds(0.5)
                            .stopAndAdd(shooter.transfer(true))
                            .build();


        }}
}
