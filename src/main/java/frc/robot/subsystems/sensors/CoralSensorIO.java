package frc.robot.subsystems.sensors;

import org.littletonrobotics.junction.AutoLog;

public interface CoralSensorIO {
  @AutoLog
  class CoralSensorIOInputs {
    public CoralSensorIOData data = new CoralSensorIOData(0.0, false);
  }

  record CoralSensorIOData(double distanceMeters, boolean valid) {}

  default void updateInputs(CoralSensorIOInputs inputs) {}
}