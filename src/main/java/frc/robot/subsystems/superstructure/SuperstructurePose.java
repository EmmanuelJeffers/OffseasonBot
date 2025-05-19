package frc.robot.subsystems.superstructure;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.FieldConstants.ReefLevel;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

/** Add your docs here. */
public record SuperstructurePose(DoubleSupplier elevatorHeight, Supplier<Rotation2d> pivotAngle) {
    private static final LoggedTunableNumber intakeHeightBaseline =
      new LoggedTunableNumber("Superstructure/Intake/ElevatorBaseline", 0.025);
    private static final LoggedTunableNumber intakeHeightRange =
        new LoggedTunableNumber("Superstructure/Intake/ElevatorRange", 0.0);
    private static final LoggedTunableNumber intakeHeightTimeFactor =
        new LoggedTunableNumber("Superstructure/Intake/ElevatorTimeFactor", 25.0);
    private static final LoggedTunableNumber intakePivot =
        new LoggedTunableNumber("Superstructure/Intake/IntakePivot", 12.0);
    private static final LoggedTunableNumber l1Height =
        new LoggedTunableNumber("Superstructure/L1/Elevator", 0.48);
    private static final LoggedTunableNumber l1Pivot =
        new LoggedTunableNumber("Superstructure/L1/Pivot", 0.0);
    private static final LoggedTunableNumber l1LaunchAdjustment =
        new LoggedTunableNumber("Superstructure/L1/LaunchAdjustment", 0.0);
    public static final LoggedTunableNumber algaeSuperPositionDeg =
        new LoggedTunableNumber("Superstructure/ReefScore/AlgaeSuperPositionDegrees", -10.0);
    private static final LoggedTunableNumber l2ReefIntakeHeight =
        new LoggedTunableNumber("Superstructure/AlgaeIntake/L2/Elevator", 0.65);
    private static final LoggedTunableNumber l3ReefIntakeHeight =
        new LoggedTunableNumber("Superstructure/AlgaeIntake/L3/Elevator", 1.0);
    private static final LoggedTunableNumber reefIntakeHeightAdjustment =
        new LoggedTunableNumber("Superstructure/AlgaeIntake/HeightAdjustment", 0.05);
    private static final LoggedTunableNumber reefIntakeMinEndEffecterDistance =
        new LoggedTunableNumber("Superstructure/AlgaeIntake/MinEndEffecterDistance", 0.18);
    private static final LoggedTunableNumber reefIntakeMaxEndEffecterDistance =
        new LoggedTunableNumber("Superstructure/AlgaeIntake/MaxEndEffecterDistance", 0.4);

    private static final Map<ReefLevel, Pair<LoggedTunableNumber, LoggedTunableNumber>>
      ejectDistance = new HashMap<>();
    private static final Map<ReefLevel, Pair<LoggedTunableNumber, LoggedTunableNumber>> ejectAngles =
        new HashMap<>();
    private static final Map<ReefLevel, Pair<LoggedTunableNumber, LoggedTunableNumber>> heightFudges =
        new HashMap<>();

    private static void addInitialValue(
        Map<ReefLevel, Pair<LoggedTunableNumber, LoggedTunableNumber>> map,
        ReefLevel reefLevel,
        double initialValue,
        double initialValueAlgae,
        String key) {
        map.put(
            reefLevel,
            Pair.of(
                new LoggedTunableNumber(
                    "Superstructure/ReefScore/" + key + "/" + reefLevel, initialValue),
                new LoggedTunableNumber(
                    "Superstructure/ReefScore/" + key + "Algae/" + reefLevel, initialValueAlgae)));
    }

    static {
        // Coral eject distance
        addInitialValue(ejectDistance, ReefLevel.L2, 0.15, 0.24, "EjectDistance");
        addInitialValue(ejectDistance, ReefLevel.L3, 0.15, 0.24, "EjectDistance");
        addInitialValue(ejectDistance, ReefLevel.L4, 0.12, 0.12, "EjectDistance");
        // Coral eject angles
        addInitialValue(ejectAngles, ReefLevel.L2, -20.0, 0.0, "EjectAngles");
        addInitialValue(ejectAngles, ReefLevel.L3, -20.0, 0.0, "EjectAngles");
        addInitialValue(ejectAngles, ReefLevel.L4, -48.0, -48.0, "EjectAngles");
        // Height fudges
        addInitialValue(
            heightFudges,
            ReefLevel.L2,
            Units.inchesToMeters(2.5),
            Units.inchesToMeters(2.5),
            "HeightFudges");
        addInitialValue(
            heightFudges,
            ReefLevel.L3,
            Units.inchesToMeters(2.5),
            Units.inchesToMeters(2.5),
            "HeightFudges");
        addInitialValue(
            heightFudges,
            ReefLevel.L4,
            Units.inchesToMeters(1.0),
            Units.inchesToMeters(1.0),
            "HeightFudges");
    }


    @Getter
    @RequiredArgsConstructor
    public enum CoralEndEffecterPose {
        L2(ReefLevel.L2, false),
        L3(ReefLevel.L3, false),
        L4(ReefLevel.L4, false),
        ALGAE_L2(ReefLevel.L2, true),
        ALGAE_L3(ReefLevel.L3, true),
        ALGAE_L4(ReefLevel.L4, true);

        private final Supplier<Pose2d> pose;
        private final ReefLevel reefLevel;
        private final boolean algae;

        CoralEndEffecterPose(ReefLevel reefLevel, boolean algae) {
            pose = () -> calculatePose(reefLevel, algae);
            this.reefLevel = reefLevel;
            this.algae = algae;
        }

        public double getElevatorHeight() {
            return (pose.get().getY());
        }

        public double getEndEffecterAngleDeg() {
            return pose.get().getRotation().getDegrees();
        }

        public Transform2d branchToRobot() {
            return new Transform2d();
        }

        public Transform2d robotToEndEffecter() {
            return new Transform2d();
        }

        public Pose2d calculatePose(ReefLevel reefLevel, boolean algae) {
            double endeffecterAngleDeg = 
                algae ? ejectAngles.get(reefLevel).getSecond().get() : ejectAngles.get(reefLevel).getFirst().get();
            return new Pose2d();
        }
    }

    // Read distances to branch from robot state to calculate postion
}
