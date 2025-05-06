package frc.robot.subsystems.superstructure.endeffecter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.RobotType;
import frc.robot.subsystems.rollers.RollerSystemIO;
import frc.robot.subsystems.rollers.RollerSystemIOInputsAutoLogged;
import frc.robot.subsystems.sensors.CoralSensorIO;
import frc.robot.subsystems.sensors.CoralSensorIOInputsAutoLogged;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTracer;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;

public class EndEffecter extends SubsystemBase {
    public static final Rotation2d minAngle = Rotation2d.fromDegrees(0.0);
    public static final Rotation2d maxAngle = Rotation2d.fromDegrees(0.0);

    // Tunable numbers
    private static final LoggedTunableNumber kP = new LoggedTunableNumber("EndEffecter/kP");
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("EndEffecter/kD");
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("EndEffecter/kS");
    private static final LoggedTunableNumber kG = new LoggedTunableNumber("EndEffecter/kG");
    private static final LoggedTunableNumber maxVelocityDegPerSec =
        new LoggedTunableNumber("EndEffecter/MaxVelocityDegreesPerSec", 1500.0);
    private static final LoggedTunableNumber maxAccelerationDegPerSec2 =
        new LoggedTunableNumber("EndEffecter/MaxAccelerationDegreesPerSec2", 2500.0);
    private static final LoggedTunableNumber algaeMaxVelocityDegPerSec =
        new LoggedTunableNumber("EndEffecter/AlgaeMaxVelocityDegreesPerSec", 800.0);
    private static final LoggedTunableNumber algaeMaxAccelerationDegPerSec2 =
        new LoggedTunableNumber("EndEffecter/AlgaeMaxAccelerationDegreesPerSec2", 1500.0);
    private static final LoggedTunableNumber slowMaxVelocityDegPerSec =
        new LoggedTunableNumber("EndEffecter/SlowMaxVelocityDegreesPerSec", 800.0);
    private static final LoggedTunableNumber slowMaxAccelerationDegPerSec2 =
        new LoggedTunableNumber("EndEffecter/SlowMaxAccelerationDegreesPerSec2", 1500.0);
    private static final LoggedTunableNumber staticCharacterizationVelocityThresh =
        new LoggedTunableNumber("EndEffecter/StaticCharacterizationVelocityThresh", 0.1);
    private static final LoggedTunableNumber staticCharacterizationRampRate =
        new LoggedTunableNumber("EndEffecter/StaticCharacterizationRampRate", 0.2);
    private static final LoggedTunableNumber algaeCurrentThresh =
        new LoggedTunableNumber("EndEffecter/AlgaeCurrentThreshold", 10.0);
    public static final LoggedTunableNumber gripperHoldVolts =
        new LoggedTunableNumber("EndEffecter/GripperHoldVolts", 0.8);
    public static final LoggedTunableNumber gripperIntakeVolts =
        new LoggedTunableNumber("EndEffecter/GripperIntakeVolts", 9.0);
    public static final LoggedTunableNumber gripperEjectVolts =
        new LoggedTunableNumber("EndEffecter/GripperEjectVolts", -12.0);
    public static final LoggedTunableNumber gripperIdleReverseTime =
        new LoggedTunableNumber("EndEffecter/GripperIdleReverseTime", 0.4);
    public static final LoggedTunableNumber gripperHardstopVolts =
        new LoggedTunableNumber("EndEffecter/GripperHardstopVolts", -12.0);
    public static final LoggedTunableNumber gripperReverseHardstopVolts =
        new LoggedTunableNumber("EndEffecter/GripperReverseHardstopVolts", 12.0);
    public static final LoggedTunableNumber gripperCurrentLimit =
        new LoggedTunableNumber("EndEffecter/GripperCurrentLimit", 50.0);
    public static final LoggedTunableNumber tolerance =
        new LoggedTunableNumber("EndEffecter/Tolerance", 0.4);
    public static final LoggedTunableNumber simIntakingTime =
        new LoggedTunableNumber("EndEffecter/SimIntakingTime", 0.5);
    private static final LoggedTunableNumber coralEjectDebounceTime =
        new LoggedTunableNumber("EndEffecter/CoralEjectDebounceTime", 0.2);
    private static final LoggedTunableNumber algaeDebounceTime =
        new LoggedTunableNumber("Dispenser/AlgaeDebounceTime", 0.6);

    static {
        switch (Constants.getRobot()) {
        case SIMBOT -> {
            kP.initDefault(0);
            kD.initDefault(0);
            kS.initDefault(0);
            kG.initDefault(0);
        }
        default -> {
            kP.initDefault(0);
            kD.initDefault(0);
            kS.initDefault(0);
            kG.initDefault(0);
        }
        }
    }

    public enum GripperGoal {
        IDLE,
        GRIP,
        EJECT
    }

    // Hardware
    private final PivotIO pivotIO;
    private final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();
    private final RollerSystemIO gripperIO;
    private final RollerSystemIOInputsAutoLogged gripperInputs = new RollerSystemIOInputsAutoLogged();
    private final CoralSensorIO coralSensorIO;
    private final CoralSensorIOInputsAutoLogged coralSensorInputs = new CoralSensorIOInputsAutoLogged();

    // Overrides
    private BooleanSupplier coastOverride = () -> false;
    private BooleanSupplier disabledOverride = () -> false;

    @AutoLogOutput(key = "EndEffecter/PivotBrakeModeEnabled")
    private boolean brakeModeEnabled = true;

    private TrapezoidProfile profile;
    private TrapezoidProfile algaeProfile;
    private TrapezoidProfile slowProfile;
    @Getter private State setpoint = new State();
    private DoubleSupplier goal = () -> 0.0;
    private boolean stopProfile = false;
    @Getter private boolean shouldEStop = false;
    @Setter private boolean isEStopped = false;
    @Setter private boolean forceFastConstraints = false;
    @Setter private boolean forceEjectForward = false;

    @Accessors(fluent = true)
    @Setter
    private boolean forceSlowConstraints = false;

    @Getter
    @AutoLogOutput(key = "EndEffecter/Profile/AtGoal")
    private boolean atGoal = false;

    @AutoLogOutput private GripperGoal gripperGoal = GripperGoal.IDLE;
    private Timer gripperGoalTimer = new Timer();

    @AutoLogOutput
    @Accessors(fluent = true)
    @Getter
    private boolean hasCoral = false;

    @AutoLogOutput
    @Accessors(fluent = true)
    @Getter
    private boolean rawHasCoral = false;

    private boolean lastHasCoral = hasCoral;

    @AutoLogOutput
    @Accessors(fluent = true)
    @Getter()
    private boolean hasAlgae = false;

    @Getter private boolean doNotStopIntaking = false;

    private Debouncer coralEjectedDebouncer =
        new Debouncer(coralEjectDebounceTime.get(), DebounceType.kRising);
    private Debouncer algaeDebouncer = new Debouncer(algaeDebounceTime.get(), DebounceType.kRising);
    private Debouncer toleranceDebouncer = new Debouncer(0.25, DebounceType.kRising);

    // Disconnected alerts
    private final Alert pivotMotorDisconnectedAlert =
        new Alert("EndEffecter pivot motor disconnected!", Alert.AlertType.kWarning);
    private final Alert pivotEncoderDisconnectedAlert =
        new Alert("EndEffecter pivot encoder disconnected!", Alert.AlertType.kWarning);
    private final Alert gripperDisconnectedAlert =
        new Alert("EndEffecter gripper disconnected!", Alert.AlertType.kWarning);

    private boolean lastAlgaeButtonPressed = false;
    private boolean lastCoralButtonPressed = false;

    /** Creates a new EndEffecter. */
    public EndEffecter(
        PivotIO pivotIO,
        RollerSystemIO gripperIO,
        CoralSensorIO coralSensorIO) {
        this.pivotIO = pivotIO;
        this.gripperIO = gripperIO;
        this.coralSensorIO = coralSensorIO;

      profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                Units.degreesToRadians(maxVelocityDegPerSec.get()),
                Units.degreesToRadians(maxAccelerationDegPerSec2.get())));
      gripperGoalTimer.start();
    }

    @Override
    public void periodic() {
        pivotIO.updateInputs(pivotInputs);
        Logger.processInputs("EndEffecter/Pivot", pivotInputs);
        gripperIO.updateInputs(gripperInputs);
        Logger.processInputs("EndEffecter", gripperInputs);
        coralSensorIO.updateInputs(coralSensorInputs);
        Logger.processInputs("EndEffecter", coralSensorInputs);

        pivotMotorDisconnectedAlert.set(
            !pivotInputs.data.motorConnected()
            && Constants.getRobot() == RobotType.COMPBOT
            && !Robot.isJITing());
        pivotEncoderDisconnectedAlert.set(
            !pivotInputs.data.encoderConnected()
            && Constants.getRobot() == RobotType.COMPBOT
            && !Robot.isJITing());
        gripperDisconnectedAlert.set(
            !gripperInputs.data.connected()
            && Constants.getRobot() == RobotType.COMPBOT
            && !Robot.isJITing());

        // Update tunable numbers
        if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
            pivotIO.setPID(kP.get(), 0.0, kD.get());
        }
        if (maxVelocityDegPerSec.hasChanged(hashCode()) || maxAccelerationDegPerSec2.hasChanged(hashCode())) {
            profile =
                new TrapezoidProfile(
                    new TrapezoidProfile.Constraints(
                        Units.degreesToRadians(maxVelocityDegPerSec.get()), 
                        Units.degreesToRadians(maxAccelerationDegPerSec2.get())));
        }
        if (algaeMaxVelocityDegPerSec.hasChanged(hashCode())
            || algaeMaxAccelerationDegPerSec2.hasChanged(hashCode())) {
        algaeProfile =
            new TrapezoidProfile(
                new TrapezoidProfile.Constraints(
                    Units.degreesToRadians(algaeMaxVelocityDegPerSec.get()),
                    Units.degreesToRadians(algaeMaxAccelerationDegPerSec2.get())));
        }
        if (slowMaxVelocityDegPerSec.hasChanged(hashCode())
            || slowMaxAccelerationDegPerSec2.hasChanged(hashCode())) {
        slowProfile =
            new TrapezoidProfile(
                new TrapezoidProfile.Constraints(
                    Units.degreesToRadians(slowMaxVelocityDegPerSec.get()),
                    Units.degreesToRadians(slowMaxAccelerationDegPerSec2.get())));
        }
        if (gripperCurrentLimit.hasChanged(hashCode())) {
            gripperIO.setCurrentLimit(gripperCurrentLimit.get());
        }
        if (coralEjectDebounceTime.hasChanged(hashCode())) {
            coralEjectedDebouncer.setDebounceTime(coralEjectDebounceTime.get());
        }
        if (algaeDebounceTime.hasChanged(hashCode())) {
            algaeDebouncer.setDebounceTime(algaeDebounceTime.get());
        }

        // Set coast mode
        setBrakeMode(!coastOverride.getAsBoolean());

        // Run profile
        final boolean shouldRunProfile =
            !stopProfile
                && !coastOverride.getAsBoolean()
                && !disabledOverride.getAsBoolean()
                && !isEStopped
                && DriverStation.isEnabled();
        Logger.recordOutput("EndEffecter/RunningProfile", shouldRunProfile);

        // Check if out of tolerance
        boolean outOfTolerance =
            Math.abs(pivotInputs.data.internalPosition().getRadians() - setpoint.position)
                > tolerance.get();
        shouldEStop = toleranceDebouncer.calculate(outOfTolerance && shouldRunProfile);
        if (shouldRunProfile) {
            // Clamp goal
            var goalState =
                new State(
                    MathUtil.clamp(goal.getAsDouble(), minAngle.getRadians(), maxAngle.getRadians()),
                    0.0);
            setpoint =
                (forceSlowConstraints
                        ? slowProfile
                        : hasAlgae && !forceFastConstraints ? algaeProfile : profile)
                    .calculate(Constants.loopPeriodSecs, setpoint, goalState);
        pivotIO.runPosition(
            Rotation2d.fromRadians(setpoint.position),
            kS.get() * Math.signum(setpoint.velocity) // Magnitude irrelevant
                + kG.get() * pivotInputs.data.internalPosition().getCos());
        // Check at goal
        atGoal =
            EqualsUtil.epsilonEquals(setpoint.position, goalState.position)
                && EqualsUtil.epsilonEquals(setpoint.velocity, 0.0);

        // Log state
        Logger.recordOutput("EndEffecter/Profile/SetpointAngleRad", setpoint.position);
        Logger.recordOutput("EndEffecter/Profile/SetpointAngleRadPerSec", setpoint.velocity);
        Logger.recordOutput("EndEffecter/Profile/GoalAngleRad", goalState.position);
        } else {
            // Reset setpoint
            setpoint = new State(pivotInputs.data.internalPosition().getRadians(), 0.0);

            // Clear logs
            Logger.recordOutput("EndEffecter/Profile/SetpointAngleRad", 0.0);
            Logger.recordOutput("EndEffecter/Profile/SetpointAngleRadPerSec", 0.0);
            Logger.recordOutput("EndEffecter/Profile/GoalAngleRad", 0.0);
        }
        if (isEStopped) {
            pivotIO.stop();
        }

        // Check algae & coral states
        if (Constants.getRobot() != Constants.RobotType.SIMBOT) {
            if (Math.abs(gripperInputs.data.appliedVoltage()) >= 0.5 || DriverStation.isDisabled()) {
                hasAlgae =
                    algaeDebouncer.calculate(
                        gripperInputs.data.torqueCurrentAmps() >= algaeCurrentThresh.get());
            } else {
                algaeDebouncer.calculate(hasAlgae);
            }
        } else {
            boolean algaeButtonPressed = DriverStation.getStickButtonPressed(2, 1);
            boolean coralButtonPressed = DriverStation.getStickButtonPressed(2, 2);
            if (algaeButtonPressed && !lastAlgaeButtonPressed) {
                hasAlgae = !hasAlgae;
            }
            if (coralButtonPressed && !lastCoralButtonPressed) {
                hasCoral = !hasCoral;
            }
            lastAlgaeButtonPressed = algaeButtonPressed;
            lastCoralButtonPressed = coralButtonPressed;
        }

        // Run gripper
        if (forceEjectForward) {
            gripperIO.runVolts(gripperEjectVolts.get());
        } else if (!isEStopped) {
            if (hasCoral) {
                if (!lastHasCoral) {}
            }
            switch (gripperGoal) {
                case IDLE -> gripperIO.runVolts(
                    gripperGoalTimer.hasElapsed(gripperIdleReverseTime.get()) 
                    ? 0.0 : gripperHardstopVolts.get());
                case GRIP -> {
                    if (hasAlgae) {
                        gripperIO.runVolts(gripperHoldVolts.get());
                    } else {
                        gripperIO.runVolts(gripperIntakeVolts.get());
                    }
                }
                case EJECT -> gripperIO.runVolts(gripperEjectVolts.get());
            }
        } else {
            gripperIO.stop();
        }
        lastHasCoral = hasCoral;

        // Display hasCoral & hasAlgae
        SmartDashboard.putBoolean("Has Coral?", hasCoral);
        SmartDashboard.putBoolean("Has Algae?", hasAlgae);
        
        // Log state
        Logger.recordOutput("EndEffecter/CoastOverride", coastOverride.getAsBoolean());
        Logger.recordOutput("EndEffecter/DisabledOverride", disabledOverride.getAsBoolean());

        // Record cycle time
        LoggedTracer.record("EndEffecter");
    }

    public void setGoal(Supplier<Rotation2d> goal) {
        this.goal =
            () -> MathUtil.inputModulus(goal.get().getRadians(), -3.0 * Math.PI / 2.0, Math.PI / 2.0);
        atGoal = false;
    }

    public void setGripperGoal(GripperGoal goal) {
        if (goal == gripperGoal) return;
        gripperGoal = goal;
        gripperGoalTimer.restart();
    }

    public double getGoal() {
        return goal.getAsDouble();
    }

    @AutoLogOutput(key = "EndEffecter/MeasuredAngle")
    public Rotation2d getPivotAngle() {
        return pivotInputs.data.internalPosition();
    }

    public void resetHasCoral(boolean value) {
        hasCoral = value;
        lastHasCoral = value;
    }
    
    public void resetHasAlgae(boolean value) {
        hasAlgae = value;
        algaeDebouncer = new Debouncer(algaeDebounceTime.get(), DebounceType.kRising);
        algaeDebouncer.calculate(value);
    }

    public void setOverrides(BooleanSupplier coastOverride, BooleanSupplier disabledOverride) {
        this.coastOverride = coastOverride;
        this.disabledOverride = disabledOverride;
    }

    private void setBrakeMode(boolean enabled) {
        if (brakeModeEnabled == enabled) return;
        brakeModeEnabled = enabled;
        pivotIO.setBrakeMode(enabled);
    }

    public Command staticCharacterization() {
        final StaticCharacterizationState state = new StaticCharacterizationState();
        Timer timer = new Timer();
        return Commands.startRun(
                () -> {
                stopProfile = true;
                timer.restart();
                },
                () -> {
                state.characterizationOutput = staticCharacterizationRampRate.get() * timer.get();
                pivotIO.runOpenLoop(state.characterizationOutput);
                Logger.recordOutput(
                    "EndEffecter/StaticCharacterizationOutput", state.characterizationOutput);
                })
            .until(
                () ->
                    pivotInputs.data.velocityRadPerSec() >= staticCharacterizationVelocityThresh.get())
            .andThen(pivotIO::stop)
            .andThen(Commands.idle())
            .finallyDo(
                () -> {
                stopProfile = false;
                timer.stop();
                Logger.recordOutput("EndEffecter/CharacterizationOutput", state.characterizationOutput);
                });
    }

    private static class StaticCharacterizationState {
        public double characterizationOutput = 0.0;
    }
}
