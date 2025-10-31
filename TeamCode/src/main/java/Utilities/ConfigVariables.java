package Utilities;

import com.acmerobotics.dashboard.config.Config;
@Config
public final class ConfigVariables {
    public static double maxWheelVel = 50;
    public static double minProfileAccel = -30; //Has to be negative
    public static double maxProfileAccel = 50;
    public static double kV = 1; //Between 0.5 and 1
}
