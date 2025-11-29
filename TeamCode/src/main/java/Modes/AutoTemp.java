package Modes;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import Commands.MechanumDrive;
import Commands.SmartIntake;
import Commands.SmartShooter3;
import Subsystems.ThreeDeadWheelLocalizer;
import Utilities.Constants;

@Autonomous
public class AutoTemp extends LinearOpMode {
    ThreeDeadWheelLocalizer odometry;
    SmartIntake intake;
    SmartShooter3 shooter;
    MechanumDrive drive;

    @Override
    public void runOpMode() {
        // --- initialize subsystems ---
        shooter = new SmartShooter3(hardwareMap);
        intake = new SmartIntake(hardwareMap);
        drive = new MechanumDrive(hardwareMap);
        odometry = new ThreeDeadWheelLocalizer(hardwareMap, Constants.OdometryConstants.startPos);
        waitForStart();
        double startTime = System.currentTimeMillis();
        if (isStopRequested()) return;
        while(opModeIsActive()){
            odometry.update();
            shooter.aim();
            //3000, 4500, 8500, 10000, 13000
            if (System.currentTimeMillis() - startTime < 3000) {
                //Initial shoot
                shooter.transfer(true);
                intake.intake(true, false);
                drive.drive(0, 0, 0);
            } else if (System.currentTimeMillis() - startTime < 3500) {
                //Driving backward to pick up balls
                shooter.transfer(false);
                drive.drive(-0.7, 0, 0);
            } else if (System.currentTimeMillis() - startTime < 7500) {
                //Picking up balls
                shooter.transfer(false);
                drive.drive(0, 0, 0);
            } else if (System.currentTimeMillis() - startTime < 8000){
                //Driving forward after picking up balls
                shooter.transfer(false);
                drive.drive(0.7, 0, 0);
            } else if (System.currentTimeMillis() - startTime < 11000) {
                //Final shooting
                shooter.transfer(true);
                drive.drive(0, 0, 0);
            } else {
                //Stop
                shooter.transfer(false);
                intake.intake(false, false);
                drive.drive(0, 0, 0);
            }
        }
    }
}
