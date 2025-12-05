package Modes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Commands.MechanumDrive;
import Commands.SmartIntake;
import Commands.SmartShooter3;
import Subsystems.ThreeDeadWheelLocalizer;
import Subsystems.Vision;
import Utilities.Constants;

@TeleOp
public class Teleop extends LinearOpMode {

    Gamepad activeGamepad1;
    MechanumDrive Mechanum;
    SmartIntake Intake;
    SmartShooter3 Shooter;
    Vision Vision;
    ThreeDeadWheelLocalizer odometry;
    //Elevator Elevator;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry telemetry = dashboard.getTelemetry(); //Comment this out before comps
    @Override
    public void runOpMode() {
        Constants.OdometryConstants.endPos = new Pose2d(Constants.OdometryConstants.resetPosBlue.x, Constants.OdometryConstants.resetPosBlue.y, Math.PI);
        Constants.OdometryConstants.startPos = new Pose2d(Constants.OdometryConstants.resetPosBlue.x, Constants.OdometryConstants.resetPosBlue.y, Math.PI);
        odometry = new ThreeDeadWheelLocalizer(hardwareMap, Constants.OdometryConstants.endPos);
        activeGamepad1 = new Gamepad();
        Mechanum = new MechanumDrive(hardwareMap);
        Vision = new Vision(hardwareMap, telemetry);
        Intake = new SmartIntake(hardwareMap);
        Shooter = new SmartShooter3(hardwareMap, Vision);
        //Constants.OdometryConstants.fieldPos = new Pose2d(Constants.OdometryConstants.fieldPos.position.x, Constants.OdometryConstants.fieldPos.position.y + 12 * Math.signum(Constants.OdometryConstants.fieldPos.position.y), Constants.OdometryConstants.fieldPos.heading.toDouble());
        //Elevator = new Elevator(hardwareMap);
        boolean upLastState = false;
        boolean autoNeck = true;

        waitForStart();

        while (opModeIsActive()) {
            //lime.runVision(test);
            odometry.update();
            Vision.updateAprilTags();
            if (gamepad1.dpad_up != upLastState && gamepad1.dpad_up) {
                autoNeck = !autoNeck;
            }
            upLastState = gamepad1.dpad_up;
            Shooter.aim(autoNeck);
            activeGamepad1.copy(gamepad1);
            Intake.intake(activeGamepad1.right_trigger > 0.5, activeGamepad1.a);
            Shooter.transfer(activeGamepad1.left_trigger > 0.3);
            //Shooter.transfer(true);
            Shooter.manualOffset(activeGamepad1.left_bumper, activeGamepad1.right_bumper);
            //Shooter.manualNeckMotor(activeGamepad1.left_bumper, activeGamepad1.right_bumper);
            //Shooter.turretHeadTester(activeGamepad1.b);
            //Shooter.shoot(activeGamepad1.left_trigger * 1040 + 1000);
            Mechanum.drive(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            );
            if (activeGamepad1.dpad_down) {
                odometry.resetYaw();
            }
            if (activeGamepad1.dpad_left) {
                Constants.OdometryConstants.fieldPos = new Pose2d(Constants.OdometryConstants.resetPosBlue, Constants.OdometryConstants.fieldPos.heading);
            }
            if (activeGamepad1.dpad_right){
                Constants.OdometryConstants.fieldPos = new Pose2d(Constants.OdometryConstants.resetPosRed, Constants.OdometryConstants.fieldPos.heading);
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
