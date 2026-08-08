package Commands;

import com.pedropathing.ivy.Command;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Utilities.Constants;

public class Collector {
    //Sample Collection system from HEIST, used to demonstrate command usage beyond the drivetrain

    //Creates servos
    private final Servo claw, arm1, arm2, wrist;
    //Keeps track of pre-set positions that could be helpful later, Cones are 24in away from center of bot, Diamonds are 16in away from front of bot
    //"floorArmPos" and "floorWristPos" are used for both floor gems and cones, since the positions are both effective. However, they can be separated at a more advanced level
    private final double floorArmPos = 1, floorWristPos = .18, diamondArmPos = .69, diamondWristPos = 0.31, binArmPos = 0.4733333333333333, binWristPos = 1;
    //These variables are unique: because servos cannot provide live positions, you have to either guess where they are with time or used a fourth wire to track voltage and math out the current position
    //I used time based:
    private long lastArmTime, lastClawTime, lastWristTime; //Tracks the last time the servos were called on
    private final double armFullTime = 1.3e3; //How long it takes for a full arm rotation
    private final double wristFullTime = 1e3; //How long it takes for a full wrist rotation
    boolean clawSwapped = false; //Tracks if the claw was opened or closed during auto, usually the last part in commands
    private double armTargetTime = 0, wristTargetTime = 0; //Target times are calculated during commands, will explain when they are defined properly
    //Creates a colour sensor
    private final ColorSensor colourSensor;
    //Intended positions for the servos to update to, both being at 0.5 by default allows easy transitions
    private double armTargetPos = 0.5, wristTargetPose = 0.5;
    private ElapsedTime armTime, wristTime;
    private final double armVelocity = 1 / armFullTime, wristVelocity = 1 / wristFullTime;

    public Collector(HardwareMap hardwareMap) {
        //Initializes everything to the hardwaremap
        arm1 = hardwareMap.get(Servo.class, Constants.CollectorConstants.arm1);
        arm2 = hardwareMap.get(Servo.class, Constants.CollectorConstants.arm2);
        claw = hardwareMap.get(Servo.class, Constants.CollectorConstants.claw);
        wrist = hardwareMap.get(Servo.class, Constants.CollectorConstants.wrist);
        colourSensor = hardwareMap.get(RevColorSensorV3.class, Constants.CollectorConstants.colourSensor);
        //Regulation times
        armTime = new ElapsedTime();
        wristTime = new ElapsedTime();

        //Sets everything to placing in the bin, a nice default that avoid hitting other bots and possibly allows a quick score if the claw has a piece from auto
        setBinPositions();
    }

    //Basic wrist control, adds or subtracts from the target value depending on if up or down is true
    //Attached to the right  joystick in teleop
    public void wristControl(double joystick) {
        if (Math.abs(joystick) > 0.05) wristTargetPose += wristVelocity * wristTime.milliseconds() * joystick;
        wristTime.reset();
        wristTargetPose = Math.max(Math.min(wristTargetPose, 0.63), 0);
    }

    //Opens the claw
    public void openClaw() {
        claw.setPosition(1);
    }

    //Closes the claw
    public void closeClaw() {
        claw.setPosition(2.0 / 3);
    }

    //Same premise as the wrist control method, just attached to the left joystick in teleop
    public void armControl(double joystick) {
        if (Math.abs(joystick) > 0.05) armTargetPos += armVelocity * armTime.milliseconds() * joystick;
        armTime.reset();
        armTargetPos = Math.max(Math.min(armTargetPos, 1), 0);
    }

    //Telemetry values to read positions, timings, sensors, etc
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Arm1 position: ", arm1.getPosition());
//        telemetry.addData("Arm2 position: ", arm2.getPosition());
//        telemetry.addData("Claw position: ", claw.getPosition());
        telemetry.addData("Wrist position: ", wrist.getPosition());
        telemetry.addData("Colour sensor: ", colourSensor.red() + " " + colourSensor.green() + " " + colourSensor.blue());
//        telemetry.addData("Claw done: ", atClawPos());
//        telemetry.addData("Claw last time: ", lastClawTime);
    }

    //These are the 4 auto commands

    //Command to pickup the cone
    public Command pickupCone() {
        return Command.build().setStart(() -> {
                    //Resets the timings
                    resetTimes();
                    //Target times are estimated by taking the full rotation time and multiplying it by the absolute difference in position
                    //Because servo positions are 0-1, no external factors are needed
                    wristTargetTime = (Math.abs(wrist.getPosition() - floorWristPos) * wristFullTime);
                    armTargetTime = (Math.abs(arm1.getPosition() - floorArmPos) * armFullTime);
                }).setExecute(this::setFloorPosition) //Sets the positions to the floor positions during the loop, Android Studio auto-made the "this::" part
                .setDone(() -> atWristPos() && atArmPos()); //Sets the finished criteria, when both the wrist and arm are predicted to be at the same position
    }

    //Command to pickup gems from the floor
    //Due to the positions being the same, both commands are functionally the same right now
    //For higher-level usage, however, I would specify different positions for the cone and floor gem
    public Command pickupGem() {
        return Command.build().setStart(() -> {
                    //Resets timings
                    resetTimes();
                    //Guesses time needed to get to position
                    wristTargetTime = (Math.abs(wrist.getPosition() - floorWristPos) * wristFullTime);
                    armTargetTime = (Math.abs(arm1.getPosition() - floorArmPos) * armFullTime);
                }).setExecute(this::setFloorPosition) //Executes the command
                .setDone(() -> atWristPos() && atArmPos()); //Finish statement
    }

    //This is used to pickup diamonds, same format with different coordinates
    public Command pickupDiamond() {
        return Command.build().setStart(() -> {
                    //Resets times
                    resetTimes();
                    //Estimates time to target position (the target position is different here)
                    wristTargetTime = (Math.abs(wrist.getPosition() - diamondWristPos) * wristFullTime);
                    armTargetTime = (Math.abs(arm1.getPosition() - diamondArmPos) * armFullTime);
                }).setExecute(this::setDiamondPositions) //Executes the command
                .setDone(() -> atWristPos() && atArmPos()); //Finish statement
    }

    //This is used to drop something in a bin, this command is special because I often chain it as a prefix for others
    public Command bin() {
        return Command.build().setStart(() -> {
                    //This uses the claw, so immediately I state the command has not used it yet
                    clawSwapped = false;
                    //Reset times
                    resetTimes();
                    //Estimates time to target position
                    wristTargetTime = (Math.abs(wrist.getPosition() - binWristPos) * wristFullTime);
                    armTargetTime = (Math.abs(arm1.getPosition() - binArmPos) * armFullTime);
                }).setExecute(this::setBinPositions) //Moves to bin positions
                .setDone(() -> {
                    //This is why clawSwapped is helpful, it makes sure there's only 1 swap for 1 timing.
                    //Without it, the command would indefinitely loop because the times would keep resetting
                    if (!clawSwapped) {
                        //Checks if the wrist and arm are at the correct position
                        if (atWristPos() && atArmPos()) {
                            //Opens the claw to drop it in the bin
                            openClaw();
                            //Updates clawSwapped to true, to prevent the indefinite loop
                            clawSwapped = true;
                            //Sets the claw's start time
                            setClawTime();
                        }
                    }
                    return clawSwapped && atClawPos(); //Only returns true (that the statement is done) when the claw has both been told to move (open for the bin) AND has finished moving
                });
    }

    public void resetTimes() {
        //Method to reset arm and wrist times
        lastArmTime = System.currentTimeMillis();
        lastWristTime = System.currentTimeMillis();
    }

    public boolean atArmPos() {
        //Returns true if the time for the arm to reach it's target position has passed
        return System.currentTimeMillis() - lastArmTime >= armTargetTime;
    }

    public boolean atWristPos() {
        //Returns true if the time for the wrist to reach it's target position has passed
        return System.currentTimeMillis() - lastWristTime >= wristTargetTime;
    }

    public boolean atClawPos() {
        //returns if the claw has reached it's target position
        //This is the only use case for clawTargetTime (which is always the same since it loops between 2 points), so I made it a local variable instead
        double clawTargetTime = 0.5e3;
        return System.currentTimeMillis() - lastClawTime >= clawTargetTime;
    }

    public void setClawTime() {
        //Resets the claw time, made it a method for possible expansion and easier calls
        lastClawTime = System.currentTimeMillis();
    }

    public boolean detectsGem() {
        //Checks if the colour sensor sees a certain % of red or blue, the colours of the two gems
        return (colourSensor.red() > colourSensor.green() && colourSensor.red() > colourSensor.blue()) || (colourSensor.blue() > colourSensor.green() && colourSensor.blue() > colourSensor.red());
    }

    public boolean detectsCone() {
        //Checks if the colour sensor sees a certain amount of green, apparently the dominant rgb colour of cones
        return (colourSensor.green() > colourSensor.red() && colourSensor.green() > colourSensor.blue() * 2);
    }

    //The next 3 methods set the target positions for the arm and wrist
    public void setFloorPosition() {
        //Target: floor pieces (cones and floor gems)
        wristTargetPose = floorWristPos;
        armTargetPos = floorArmPos;
    }

    public void setDiamondPositions() {
        //Target: diamonds (significantly higher than floor pieces)
        wristTargetPose = diamondWristPos;
        armTargetPos = diamondArmPos;
    }

    public void setBinPositions() {
        //Target: Dropping gems in the bin (assumed to be latched in the bot)
        wristTargetPose = binWristPos;
        armTargetPos = binArmPos;
    }

    //The prior methods only set a number as the target servo positions
    //Should two commands accidentally run at once, this method ensures only the last command in the loop updates the servos
    //This is mostly to prevent accidentally burning out the servos by repeatedly updating them to two different positions in one loop
    //Can also be used in multithreading instances, not needed in this game however
    public void updateHardware() {
        //.setPosition is the command that sends the target position to the servos
        arm1.setPosition(armTargetPos);
        arm2.setPosition(1 - armTargetPos); //Because arm2 is the opposite of arm1, it needs to be flipped (i.e arm1 at position 1 is arm2 at positon 0, arm1 at position 0.7 is arm2 at position 0.3, etc)
        wrist.setPosition(wristTargetPose);
    }


}
