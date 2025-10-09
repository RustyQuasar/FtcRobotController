package Modes;

import Commands.MechanumDrive;
import Subsystems.Odometry;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;


import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Unified Road Runner Autonomous Mode
 * Works both in MeepMeep simulation and on the physical FTC robot.
 */
@Autonomous(name = "RoadRunner Auto", group = "Competition")
public class RoadRunnerAuto extends LinearOpMode {

    // --- REAL ROBOT ENTRYPOINT ---
    @Override
    public void runOpMode() throws InterruptedException {
        Odometry odometry = new Odometry(hardwareMap);
        MechanumDrive drive = new MechanumDrive(hardwareMap, odometry);

        Action autoSequence = buildAutoSequence(drive, 1);

        waitForStart();
        if (isStopRequested()) return;

        // Updated for new Road Runner API
        com.acmerobotics.roadrunner.ftc.ActionRunner runner =
                new com.acmerobotics.roadrunner.ftc.ActionRunner();

        runner.runBlocking(autoSequence);
    }


    // --- MEEPMEEP SIMULATION ENTRYPOINT ---
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60,
                        Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        // We create a mock MechanumDrive (no hardware) just to reuse your builder logic
        MechanumDrive fakeDrive = new MechanumDrive(null, null);

        // Pick auto routine (matches robot’s switch)
        Action autoSequence = new RoadRunnerAuto().buildAutoSequence(fakeDrive, 1);

        myBot.runAction(autoSequence);

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_BLACK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    // --- SHARED AUTO LOGIC ---
    public Action buildAutoSequence(MechanumDrive drive, int wantedAuto) {
        switch (wantedAuto) {
            case 1:
                return drive.actionBuilder(new Pose2d(-34, -34, 0))
                        .turn(Math.toRadians(-30))
                        .lineToX(-8)
                        .lineToX(-38)
                        .turn(Math.toRadians(15))
                        .lineToX(10)
                        .lineToX(-38)
                        .build();

            case 2:
                return drive.actionBuilder(new Pose2d(52, -52, 0))
                        .lineToX(32)
                        .lineToX(52)
                        .lineToX(10)
                        .lineToX(52)
                        .build();

            default:
                telemetry.addLine("Invalid auto selected!");
                telemetry.update();
                return drive.actionBuilder(new Pose2d(0, 0, 0)).build();
        }
    }
}
