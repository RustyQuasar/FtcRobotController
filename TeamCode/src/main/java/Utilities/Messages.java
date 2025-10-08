package Utilities;

import com.acmerobotics.roadrunner.Pose2d;

public class Messages {
    public static class MecanumCommandMessage {
        public double voltage;
        public double leftFront;
        public double leftBack;
        public double rightBack;
        public double rightFront;

        public MecanumCommandMessage(double voltage, double lf, double lb, double rb, double rf) {
            this.voltage = voltage;
            this.leftFront = lf;
            this.leftBack = lb;
            this.rightBack = rb;
            this.rightFront = rf;
        }
    }
    public static class PoseMessage {
        public double x, y, heading;

        public PoseMessage(Pose2d pose) {
            x = pose.position.x;
            y = pose.position.y;
            heading = pose.heading.toDouble();
        }
    }
}
