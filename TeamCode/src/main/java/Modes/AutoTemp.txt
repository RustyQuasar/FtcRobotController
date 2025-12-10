package Modes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import Commands.MechanumDrive;
import Commands.SmartIntake;
import Commands.SmartShooter3;
import Subsystems.ThreeDeadWheelLocalizer;
import Subsystems.Vision;
import Utilities.Constants;

@Autonomous
public class AutoTemp extends LinearOpMode {
    ThreeDeadWheelLocalizer odometry;
    SmartIntake intake;
    SmartShooter3 shooter;
    MechanumDrive drive;
    Vision vision;

    @Override
    public void runOpMode() {
        // --- initialize subsystems ---
        vision = new Vision(hardwareMap, telemetry);
        shooter = new SmartShooter3(hardwareMap, vision);
        intake = new SmartIntake(hardwareMap);
        drive = new MechanumDrive(hardwareMap);
        odometry = new ThreeDeadWheelLocalizer(hardwareMap, Constants.OdometryConstants.startPos);
        waitForStart();
        double startTime = System.currentTimeMillis();
        if (isStopRequested()) return;
        while(opModeIsActive()){
            odometry.update();
            shooter.aim(true);
            //3000, 4500, 8500, 10000, 13000
            if (System.currentTimeMillis() - startTime < 7000) {
                //Initial shoot
                shooter.transfer(true);
                intake.intake(true, false);
                drive.drive(0, 0, 0);
            } else if (Math.abs(Constants.OdometryConstants.startPos.position.y) + 13 + Constants.Sizes.robotOffset > Math.abs(Constants.OdometryConstants.fieldPos.position.y)) {
                //Now driving forward
                shooter.transfer(false);
                drive.drive(0.2, 0, 0);
            }
            else {
                //Stop
                shooter.transfer(false);
                intake.intake(false, false);
                drive.drive(0, 0, 0);
            }

        }
    }
}
