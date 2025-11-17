package Modes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Commands.Elevator;
import Commands.MechanumDrive;
import Commands.SmartIntake;
import Commands.SmartShooter2;
import Subsystems.Vision;
import Subsystems.ThreeDeadWheelLocalizer;
import Utilities.Constants;

@TeleOp
public class Teleop extends LinearOpMode {

    Gamepad activeGamepad1;
    MechanumDrive Mechanum;
    SmartIntake Intake;
    SmartShooter2 Shooter;
    Vision Vision;
    ThreeDeadWheelLocalizer odometry;
    Elevator Elevator;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry telemetry = dashboard.getTelemetry(); //Comment this out before comps
    @Override
    public void runOpMode() {
        odometry = new ThreeDeadWheelLocalizer(hardwareMap, Constants.OdometryConstants.fieldPos);
        activeGamepad1 = new Gamepad();
        Mechanum = new MechanumDrive(hardwareMap);
        Vision = new Vision(hardwareMap, dashboard);
        Intake = new SmartIntake(hardwareMap);
        Shooter = new SmartShooter2(hardwareMap, Vision);
        Elevator = new Elevator(hardwareMap);
        boolean lastYInput = false;
        waitForStart();

        while (opModeIsActive()) {
            odometry.update();
            Vision.updateAprilTags();
            //Shooter.aim();
            activeGamepad1.copy(gamepad1);
            Intake.intake(activeGamepad1.right_trigger > 0.5, activeGamepad1.a);
            if (activeGamepad1.left_trigger > 0.3) {
                Shooter.shoot(activeGamepad1.left_trigger * 2040);
                Shooter.transfer(true);
                Intake.colorWipe();
            } else {
                Shooter.transfer(false);
                Shooter.shoot(0);
            }

            Mechanum.drive(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );

            if (activeGamepad1.dpad_down) {
                odometry.resetYaw();
            }

            if (activeGamepad1.y && !lastYInput) {
                Elevator.switchState();
            }
            lastYInput = activeGamepad1.y;
            //Mechanum.telemetry(telemetry);
            //Shooter.telemetry(telemetry);
            //Intake.telemetry(telemetry);
            odometry.telemetry(telemetry);
            telemetry.update();
            dashboard.updateConfig();
        }
    }
}
