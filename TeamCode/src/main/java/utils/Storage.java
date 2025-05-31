package utils;

import com.pedropathing.localization.Pose;

public class Storage {
    public static Pose CurrentPose = new Pose(0, 0, Math.toRadians(180));
    public static boolean isRed = true;
    public static double liftPos = 0;
    public static double extendoPos = 0;
}
