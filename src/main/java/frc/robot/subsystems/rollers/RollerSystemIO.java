package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface RollerSystemIO {
    @AutoLog
    public class RollerSystemIOInputs {
        public RollerSystemIOData data = new RollerSystemIOData(0, 0, 0, 0, 0, 0, false, false);
    }

    record RollerSystemIOData(
        double positionRads,
        double velocityRadsPerSec,
        double appliedVoltage,
        double supplyCurrentAmps,
        double torqueCurrentAmps,
        double tempCelsius,
        boolean tempFault,
        boolean connected) {}

    default void updateInputs(RollerSystemIOInputs inputs) {}

    /* Run rollers at volts */
    default void runVolts(double volts) {}

    /* Run rollers at amps */
    default void runTorqueCurrent(double amps) {}

    /* Stop rollers */
    default void stop() {}

    default void setCurrentLimit(double setCurrentLimit) {}

    default void setBrakeMode(boolean enabled) {}
}
