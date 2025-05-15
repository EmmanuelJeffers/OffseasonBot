package frc.robot.subsystems.sensors;

import com.ctre.phoenix6.hardware.CANrange;

public class CoralSensorCanRange implements CoralSensorIO{
    private final CANrange canRange;

    public CoralSensorCanRange(int id, String canbus) {
        canRange = new CANrange(id, canbus);
    }

    @Override
    public void updateInputs(CoralSensorIOInputs inputs) {
        var measurement = canRange.getDistance().getValueAsDouble();
        boolean valid = canRange.isConnected();
        boolean tripped = canRange.getIsDetected().getValue();
        inputs.data = new CoralSensorIOData(valid ? measurement : 0.0, valid, tripped);
    }
}
