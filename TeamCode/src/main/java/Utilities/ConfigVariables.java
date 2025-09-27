package Utilities;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class ConfigVariables {
    //default: (p=10.000000 i=3.000000 d=0.000000 f=0.000000 alg=LegacyPID)
    public static double P = 10;
    public static double I = 3;
    public static double D = 1;
}
