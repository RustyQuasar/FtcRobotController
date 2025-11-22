package Modes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Commands.MechanumDrive;
import Commands.SmartIntake;
import Commands.SmartShooter3;
import Subsystems.Vision;
import Subsystems.ThreeDeadWheelLocalizerOriginal;
import Utilities.Constants;

@TeleOp
public class Teleop extends LinearOpMode {

    Gamepad activeGamepad1;
    MechanumDrive Mechanum;
    SmartIntake Intake;
    SmartShooter3 Shooter;
    Vision Vision;
    ThreeDeadWheelLocalizerOriginal odometry;
    //Elevator Elevator;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry telemetry = dashboard.getTelemetry(); //Comment this out before comps
    @Override
    public void runOpMode() {
        odometry = new ThreeDeadWheelLocalizerOriginal(hardwareMap, Constants.OdometryConstants.fieldPos);
        activeGamepad1 = new Gamepad();
        Mechanum = new MechanumDrive(hardwareMap);
        Vision = new Vision(hardwareMap, telemetry);
        Intake = new SmartIntake(hardwareMap);
        Shooter = new SmartShooter3(hardwareMap, Vision);
        //Elevator = new Elevator(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {
            odometry.update();
            Vision.updateAprilTags();
            Shooter.aim();
            activeGamepad1.copy(gamepad1);
            Intake.intake(activeGamepad1.right_trigger > 0.5, activeGamepad1.a);
            if (activeGamepad1.left_trigger > 0.3) {
                //Shooter.shoot(activeGamepad1.left_trigger * 2040);
                Shooter.transfer(true);
            } else {
                Shooter.transfer(false);
                //Shooter.shoot(0);
            }

            Mechanum.drive(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );

            if (activeGamepad1.dpad_down) {
                odometry.resetYaw();
            }
            //Mechanum.telemetry(telemetry);
            Shooter.telemetry(telemetry);
            //Intake.telemetry(telemetry);
            //Vision.telemetry(telemetry);
            odometry.telemetry(telemetry);
            telemetry.update();
            dashboard.updateConfig();
        }
    }
}
