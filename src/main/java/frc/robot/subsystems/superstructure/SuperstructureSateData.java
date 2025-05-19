package frc.robot.subsystems.superstructure;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.superstructure.endeffecter.EndEffecter;
import lombok.AccessLevel;
import lombok.Builder;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

@Builder(toBuilder = true, access = AccessLevel.PACKAGE)
@Getter
public class SuperstructureSateData {
    @Builder.Default
    private final SuperstructurePose pose = new SuperstructurePose(() -> 0.0, () -> Rotation2d.kZero);

    @Builder.Default private final EndEffecter.GripperGoal gripperGoal = EndEffecter.GripperGoal.IDLE;
    @Builder.Default private final DoubleSupplier intakeVolts = () -> 0.0;

    /** What height is the carriage above? */
    @RequiredArgsConstructor
    @Getter
    public enum Height {
        BOTTOM(0),
        FIRST_STAGE(SuperstructureConstants.stage1ToStage2Height),
        SECOND_STAGE(SuperstructureConstants.stage2ToStage3Height);

        private final double position;

        public boolean lowerThan(Height other) {
            return position <= other.position;
        }

        public boolean upperTHan(Height other) {
            return position > other.position;
        }
    }
}
