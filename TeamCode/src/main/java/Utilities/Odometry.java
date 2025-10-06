package Utilities;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public final class Odometry{
        public final Encoder par, perp;
        public final double metersPerTick;
        private int lastParPos, lastPerpPos;
    public Odometry(HardwareMap hardwareMap) {
        par = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, Constants.DriveTrainConstants.frontLeftMotor0)));
        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, Constants.DriveTrainConstants.frontRightMotor1)));
        metersPerTick = Constants.DriveTrainConstants.deadwheelDiameter * Math.PI/ Constants.DriveTrainConstants.externalMax;
        lastParPos = par.getPositionAndVelocity().position;
        lastPerpPos = perp.getPositionAndVelocity().position;
    }
    public double[] getVelocity(){
        double x = perp.getPositionAndVelocity().velocity * metersPerTick;
        double y = par.getPositionAndVelocity().velocity * metersPerTick;
        return new double[] {x, y};
    }

    public void updatePose() {
        Constants.fieldPos[0] = (perp.getPositionAndVelocity().position - lastPerpPos) * sin(Math.toRadians(Constants.heading));
        Constants.fieldPos[1] = (par.getPositionAndVelocity().position - lastParPos) * cos(Math.toRadians(Constants.heading));
    }

}