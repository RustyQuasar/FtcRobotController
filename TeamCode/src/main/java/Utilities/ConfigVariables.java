package Utilities;

import com.acmerobotics.dashboard.config.Config;
@Config
public final class ConfigVariables {
    public static double maxWheelVel = 50;
    public static double minProfileAccel = -30; //Has to be negative
    public static double maxProfileAccel = 45;
    public static double kV = 0.0013;
    public static double kA = 0.00030;
    public static double kS = 0.045;
    public static double headingGain = 2;
    public static double headingVelGain = headingGain * 0.2;
    public static int path = 1;
}