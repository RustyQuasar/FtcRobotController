package Modes;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;


import Commands.MechanumDrive;
import Commands.SmartIntake;
import Commands.SmartShooter3;
import Subsystems.ThreeDeadWheelLocalizer;
import Subsystems.Vision;
import Utilities.Constants;

@TeleOp
public class Teleop {
    Gamepad activeGamepad1;
    MechanumDrive Mechanum;
    SmartIntake Intake;
    SmartShooter3 Shooter;
    Vision Vision;
    ThreeDeadWheelLocalizer odometry;
    boolean upLastState = false;
    boolean autoNeck = true;
    public void init(HardwareMap hardwareMap, String team){
        Constants.TEAM = team;
        odometry = new ThreeDeadWheelLocalizer(hardwareMap, Constants.OdometryConstants.endPos);
        odometry.update();
        activeGamepad1 = new Gamepad();
        Mechanum = new MechanumDrive(hardwareMap);
        Vision = new Vision(hardwareMap);
        Intake = new SmartIntake(hardwareMap);
        Shooter = new SmartShooter3(hardwareMap, Vision);
    }
    public void run(Gamepad gamepad1) {
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
            Shooter.manualOffset(activeGamepad1.left_bumper, activeGamepad1.right_bumper);
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
    }
}
