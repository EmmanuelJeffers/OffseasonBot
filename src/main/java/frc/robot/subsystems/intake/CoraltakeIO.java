package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface CoraltakeIO {
  @AutoLog
  class CoraltakeIOInputs {
    public CoraltakeIOData data = new CoraltakeIOData(false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  }

  record CoraltakeIOData(
      boolean motorConnected,
      double positionRad,
      double velocityRadPerSec,
      double appliedVolts,
      double torqueCurrentAmps,
      double supplyCurrentAmps,
      double tempCelsius) {}

  default void updateInputs(CoraltakeIOInputs inputs) {}

  default void runOpenLoop(double output) {}

  default void runVolts(double volts) {}

  default void stop() {}

  default void runPosition(double positionRad, double feedforward) {}

  default void setPID(double kP, double kI, double kD) {}

  default void setBrakeMode(boolean enabled) {}
}