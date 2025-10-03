package Commands;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.tan;
import Utilities.Constants;
public class Odometry{
    static MechanumDrive drivetrain;
    public static double[] pos2d;

    public Odometry(MechanumDrive drive) {
    drivetrain = drive;
    }

public void updatePose(double[] vs){
       double x = vs[0];
       double y =vs[1];
       double t = vs[2];
       Constants.DriveTrainConstants.fieldPos[0] += x * t * Constants.nanoToSeconds;
       Constants.DriveTrainConstants.fieldPos[1] += y * t * Constants.nanoToSeconds;
    }
public static double[] getFieldVelocities() {
    double[] velocities = drivetrain.getDrivetrainVelocities();
    double heading = drivetrain.getHeading();
       double radians = radians(heading);
       double x = velocities[1] * cos(radians);
        double y = velocities[0] * sin(radians);
        return new double[]{x, y};
    }

    private static double radians(double angleDegrees) {
        return angleDegrees/360;
    }


}