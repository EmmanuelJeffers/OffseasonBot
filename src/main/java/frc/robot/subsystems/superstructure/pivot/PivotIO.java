package frc.robot.subsystems.superstructure.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
  @AutoLog
  class PivotIOInputs {
    public PivotIOData data =
        new PivotIOData(false, false, Rotation2d.kZero, Rotation2d.kZero, 0, 0, 0, 0, 0, 0);
  }

  record PivotIOData(
      boolean motorConnected,
      boolean encoderConnected,
      Rotation2d internalPosition,
      Rotation2d encoderAbsolutePosition,
      double encoderRelativePosition,
      double velocityRadPerSec,
      double appliedVolts,
      double supplyCurrentAmps,
      double torqueCurrentAmps,
      double tempCelsius) {}

  default void updateInputs(PivotIOInputs inputs) {}

  default void runOpenLoop(double output) {}

  default void runVolts(double volts) {}

  default void stop() {}

  default void runPosition(Rotation2d position, double feedforward) {}

  default void setPID(double kP, double kI, double kD) {}

  default void setBrakeMode(boolean enabled) {}
}
