package Modes;


import Commands.MechanumDrive;
import Subsystems.Odometry;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous
public class MeepMeepAuto extends LinearOpMode {
    @Override
    public void runOpMode(){
        // ---------------- MeepMeep Setup ----------------
        MeepMeep meepMeep = new MeepMeep(800);

        // Create a sample MeepMeep RoadRunner bot using default constraints
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(
                        60, 60,                    // maxVel, maxAccel (in/s)
                        Math.toRadians(180),       // maxAngVel (rad/s)
                        Math.toRadians(180),       // maxAngAccel (rad/s^2)
                        15                         // track width
                )
                .build();

        // ---------------- MechanumDrive Setup ----------------
        // This is normally your FTC robot object; for MeepMeep simulation,
        // we only demonstrate the path usage via actionBuilder.
        Odometry odometry = new Odometry(hardwareMap); // Stub; replace with real odometry on robot
        MechanumDrive drive = new MechanumDrive(hardwareMap, odometry); // HardwareMap for simulation

        // ---------------- Define Auto Paths ----------------
        int wantedAuto = 2;

        switch (wantedAuto) {
            case 1:
                drive.actionBuilder(new Pose2d(-34, -34, 0))
                        .turn(Math.toRadians(-30))
                        .lineToX(-8)
                        .lineToX(-38)
                        .turn(Math.toRadians(15))
                        .lineToX(10)
                        .lineToX(-38)
                        .build(); // For simulation, build just prepares the trajectory
                break;

            case 2:
                drive.actionBuilder(new Pose2d(52, -52, 0))
                        .lineToX(32)
                        .lineToX(52)
                        .lineToX(10)
                        .lineToX(52)
                        .build();
                break;
        }

        // ---------------- Start MeepMeep Simulation ----------------
        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_BLACK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
