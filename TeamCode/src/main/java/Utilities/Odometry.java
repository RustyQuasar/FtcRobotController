package Utilities;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
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
    /*
     *    ^
     *    |
     *    | ( x direction)
     *    |
     *    v
     *    <----( y direction )---->

     *        (forward)
     *    /--------------\
     *    |     ____     |
     *    |     ----     |    <- Perpendicular Wheel
     *    |           || |
     *    |           || |    <- Parallel Wheel
     *    |              |
     *    |              |
     *    \--------------/
     *
     */
    public Odometry(HardwareMap hardwareMap) {
        par = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, Constants.DriveTrainConstants.frontLeftMotor0)));
        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, Constants.DriveTrainConstants.frontRightMotor1)));
        metersPerTick = Constants.OdometryConstrants.deadwheelDiameter * Math.PI/ Constants.OdometryConstrants.externalMax;
        lastParPos = par.getPositionAndVelocity().position;
        lastPerpPos = perp.getPositionAndVelocity().position;
    }
    public double[] getVelocity(){
        double x = perp.getPositionAndVelocity().velocity * metersPerTick;
        double y = par.getPositionAndVelocity().velocity * metersPerTick;
        return new double[] {x, y};
    }

    public void updatePose() {
        double x  = (perp.getPositionAndVelocity().position - lastPerpPos) * cos(Math.toRadians(Constants.heading));
        double y = (par.getPositionAndVelocity().position - lastParPos) * sin(Math.toRadians(Constants.heading));
        Constants.OdometryConstrants.fieldPos = new Vector2d(x, y);

    }

    //IDK where to put it so it here now
    public boolean isInTriangle() {
        double[] pose =  {Constants.OdometryConstrants.fieldPos.x,Constants.OdometryConstrants.fieldPos.y} ;
        boolean isInBigTriangle = pose[1] >= pose[0]&&pose[1]>=-pose[0]+12;
        boolean isInSmallTriangle = pose[1] >= pose[0]-(2*0.3048)&&pose[1] >= -pose[0]+(4*0.3048);
        boolean isIn = isInBigTriangle||isInSmallTriangle;
return isIn;
    }

}