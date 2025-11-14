package Modes;

import static Utilities.Constants.DriveTrainConstants.imu;

//import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

//import Commands.Elevator;
import Commands.MechanumDrive;

import Commands.SmartIntake;
import Commands.SmartShooter2;
//import Commands.Vision;
//import Subsystems.ThreeDeadWheelLocalizer;
import Utilities.Constants;

@TeleOp
public class Teleop extends LinearOpMode {

    Gamepad activeGamepad1;
    MechanumDrive Mechanum;
    SmartIntake Intake;
    SmartShooter2 Shooter;
   // Vision Vision;
   // ThreeDeadWheelLocalizer odometry;
   // Elevator Elevator;
 //   FtcDashboard dashboard = FtcDashboard.getInstance();

   // Telemetry telemetry = dashboard.getTelemetry(); //Comment this out before comps
    @Override
    public void runOpMode() {


      //  odometry = new ThreeDeadWheelLocalizer(hardwareMap, Constants.OdometryConstants.fieldPos);
        activeGamepad1 = new Gamepad();
        Mechanum = new MechanumDrive(hardwareMap);
        Intake = new SmartIntake(hardwareMap);
        Shooter = new SmartShooter2(hardwareMap);
        // Elevator = new Elevator(hardwareMap);
        boolean lastYInput = false;
        waitForStart();

        while (opModeIsActive()) {

        activeGamepad1.copy(gamepad1);
        if(activeGamepad1.right_trigger>0.1){Shooter.shoot(0.8);}
        else{Shooter.shoot(0);}
        if ((activeGamepad1.right_trigger > 0.1)) {
            Intake.intake((true), 0.8);
        } else if (activeGamepad1.right_bumper) {
            Intake.intake((true), -0.8);
        } else if (activeGamepad1.y){


        }else {
            Intake.intake((false), -0.8);
        }

//        if(activeGamepad1.a){
//            Shooter.manualTurn(0.1);
//        }else if (activeGamepad1.b){  Shooter.manualTurn(-0.1);}
//        else{ Shooter.manualTurn(0);}

        Mechanum.drive(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );


    }
    
    }

}
//            if (activeGamepad1.dpad_down) {
//                odometry.resetYaw();
//            }
//            /*
//            if (activeGamepad1.left_trigger > 0.1) {
//                Shooter.shoot(activeGamepad1.left_trigger * 2040);
//            }
//             */
//
////            if (activeGamepad1.y && !lastYInput) {
////                Elevator.switchState();
////            }
//            lastYInput = activeGamepad1.y;
//
//           // Shooter.aim();
//            //Mechanum.periodic(telemetry);
//            //Shooter.periodic(telemetry);
//            Intake.telemetry(telemetry);
//            telemetry.update();
//            dashboard.updateConfig();
//        }


