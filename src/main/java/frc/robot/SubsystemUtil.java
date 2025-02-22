
package frc.robot;

public class SubsystemUtil {

    private static double swerveSpeedFactor = 0.6;

    public void setSwerveMaxSpeed(double speedMod) {
        swerveSpeedFactor = speedMod;
    }

    public double getSwerveMaxSpeed() {
        return swerveSpeedFactor;
    }

}
