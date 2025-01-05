package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class SubsystemUtil {
    private static double distanceFromSpeaker = 0;
    private static boolean isNoteIndexed = false;



    // public static PIDController getPIDShuffleboard(String name) {
    //     return   Shuffleboard.getTab("PIDControllers").
    // }

    public static void setDistanceFromSpeaker(double distance) {
        distanceFromSpeaker = distance;
    }

    public static void setNoteIndexed(boolean isIndexed) {
        isNoteIndexed = isIndexed;
    }

    public static double getDistanceFromSpeaker() {
        return distanceFromSpeaker;
    }

    public static boolean getIsNoteIndexed() {
        return isNoteIndexed;
    }

    public static double lerp(double input, double minX, double minY, double maxX, double maxY) {
        double slope = (maxY - minY)/(maxX - minX);
        return minY + (slope * (input - minX));
    }

}
