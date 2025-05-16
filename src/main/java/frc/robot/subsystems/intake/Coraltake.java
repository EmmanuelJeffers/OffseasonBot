package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTracer;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.experimental.Accessors;

public class Coraltake {
    protected static final double maxAngle = Units.degreesToRadians(0);

    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Coraltake/kP");
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Coraltake/kD");
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Coraltake/kS");
    private static final LoggedTunableNumber kG = new LoggedTunableNumber("Coraltake/kG");
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Coraltake/kA");
    private static final LoggedTunableNumber maxVelocityMetersPerSec =
        new LoggedTunableNumber("Coraltake/MaxVelocityMetersPerSec", 6.0);
    private static final LoggedTunableNumber maxAccelerationMetersPerSec2 =
        new LoggedTunableNumber("Coraltake/MaxAccelerationMetersPerSec2", 20.0);
    private static final LoggedTunableNumber homingVolts =
        new LoggedTunableNumber("Coraltake/HomingVolts", -3.0);
    private static final LoggedTunableNumber homingTimeSecs =
        new LoggedTunableNumber("Coraltake/HomingTimeSecs", 0.4);
    private static final LoggedTunableNumber homingVelocityThresh =
        new LoggedTunableNumber("Coraltake/HomingVelocityThresh", 0.1);

    static {
        switch (Constants.getRobot()) {
            default -> {
                kP.initDefault(1800);
                kD.initDefault(50);
                kS.initDefault(0);
                kG.initDefault(20);
                kA.initDefault(0);
            }
            case SIMBOT -> {
                kP.initDefault(1000.0);
                kD.initDefault(0);
                kS.initDefault(0);
                kG.initDefault(0);
                kA.initDefault(0);
            }
        }
    }

    @RequiredArgsConstructor
    public enum Goal {
        DEPLOY(new LoggedTunableNumber("Coraltake/DeployedDegrees", 8.0)),
        RETRACT(new LoggedTunableNumber("Coraltake/RetractedDegrees", 130.0)),
        REVERSE(new LoggedTunableNumber("Coraltake/ReverseDegrees", 18)),
        L1(new LoggedTunableNumber("Coraltake/L1Degrees", 90)),
        L1_EJECT(new LoggedTunableNumber("Coraltake/L1EjectDegrees", 90.0));

        private final DoubleSupplier positionDeg;

        double getAngleRad() {
            return Units.degreesToRadians(positionDeg.getAsDouble());
        }
    }

    private final CoraltakeIO io;
    private final CoraltakeIOInputsAutoLogged inputs = new CoraltakeIOInputsAutoLogged();

    private final Alert motorDisconnectedAlert = new Alert("Coraltake motor disconnected!", Alert.AlertType.kWarning);

    @AutoLogOutput(key = "Intake/Coraltake/BrakeModeEnabled")
    private boolean brakeModeEnabled = true;

    private TrapezoidProfile profile;
    @Getter private State setpoint = new State();
    @Getter private Goal goal = Goal.RETRACT;
    private boolean stopProfile = false;

    @AutoLogOutput(key = "Intake/Coraltake/HomedPositionRad")
    private double homedPosition = 0.0;

    @AutoLogOutput(key = "Intake/Coraltake/Homed")
    @Getter
    private boolean homed = false;

    private Debouncer homingDebouncer = new Debouncer(homingTimeSecs.get());
    private final Command homingCommand;

    @Getter
    @Accessors(fluent = true)
    @AutoLogOutput(key = "Intake/Coraltake/Profile/AtGoal")
    private boolean atGoal = false;

    @Getter
    @Accessors(fluent = true)
    private boolean wantsToDeploy = false;

    public Coraltake(CoraltakeIO io) {
        this.io = io;

        profile =
            new TrapezoidProfile(
                new TrapezoidProfile.Constraints(
                    maxVelocityMetersPerSec.get(), maxAccelerationMetersPerSec2.get()));

        homingCommand = homingSequence();
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake/Coraltake", inputs);

        motorDisconnectedAlert.set(!inputs.data.motorConnected() && !Robot.isJITing());

        // Update tunable numbers
        if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
            io.setPID(kP.get(), 0.0, kD.get());
          }
        if (maxVelocityMetersPerSec.hasChanged(hashCode())
            || maxAccelerationMetersPerSec2.hasChanged(hashCode())) {
        profile =
            new TrapezoidProfile(
                new TrapezoidProfile.Constraints(
                    maxVelocityMetersPerSec.get(), maxAccelerationMetersPerSec2.get()));
        }

        // Tell intake to deploy or not when disabled
        wantsToDeploy = !homed || (getMeasuredAngleRad() < maxAngle / 2.0);

        // Home on enable
        if (DriverStation.isEnabled() && !homed && !homingCommand.isScheduled()) {
            homingCommand.schedule();
        }

        // Run profile
        final boolean shouldRunProfile =
            !stopProfile
                && brakeModeEnabled
                && (homed || Constants.getRobot() == Constants.RobotType.SIMBOT)
                && DriverStation.isEnabled();
        Logger.recordOutput("Intake/Coraltake/RunningProfile", shouldRunProfile);

        if (shouldRunProfile) {
            // Clamp goal
            var goalState = new State(MathUtil.clamp(goal.getAngleRad(), 0.0, maxAngle), 0.0);
            double previousVelocity = setpoint.velocity;
            setpoint = profile.calculate(Constants.loopPeriodSecs, setpoint, goalState);
            if (setpoint.position < 0.0 || setpoint.position > maxAngle) {
                setpoint = new State(MathUtil.clamp(setpoint.position, 0.0, maxAngle), 0.0);
            }

            // Check at goal
            atGoal =
                EqualsUtil.epsilonEquals(setpoint.position, goalState.position)
                    && EqualsUtil.epsilonEquals(setpoint.velocity, goalState.velocity);

            // Run
            double accel = (setpoint.velocity - previousVelocity) / Constants.loopPeriodSecs;
            io.runPosition(
                setpoint.position + homedPosition,
                kS.get() * Math.signum(setpoint.velocity) // Magnitude irrelevant
                    + kG.get() * Math.cos(setpoint.position)
                    + kA.get() * accel);

            // Log state
            Logger.recordOutput("Intake/Coraltake/Profile/SetpointPositionMeters", setpoint.position);
            Logger.recordOutput("Intake/Coraltake/Profile/SetpointVelocityMetersPerSec", setpoint.velocity);
            Logger.recordOutput("Intake/Coraltake/Profile/GoalPositionMeters", goalState.position);
            Logger.recordOutput("Intake/Coraltake/Profile/GoalVelocityMetersPerSec", goalState.velocity);
        } else {
            // Reset setpoint
            setpoint = new State(getMeasuredAngleRad(), 0.0);

            // Clear logs
            Logger.recordOutput("Intake/Coraltake/Profile/SetpointPositionMeters", 0.0);
            Logger.recordOutput("Intake/Coraltake/Profile/SetpointVelocityMetersPerSec", 0.0);
            Logger.recordOutput("Intake/Coraltake/Profile/GoalPositionMeters", 0.0);
            Logger.recordOutput("Intake/Coraltake/Profile/GoalVelocityMetersPerSec", 0.0);
        }

        // Log state
        Logger.recordOutput("Intake/Coraltake/MeasuredVelocityMetersPerSec", inputs.data.velocityRadPerSec());

        // Record cycle time
        LoggedTracer.record("Coraltake");
    }

    void setGoal(Goal goal) {
        if (goal == this.goal) return;
        this.goal = goal;
        atGoal = false;
    }

    void setBrakeMode(boolean enabled) {
        if (brakeModeEnabled == enabled) return;
        brakeModeEnabled = enabled;
        io.setBrakeMode(brakeModeEnabled);
    }
    
    /** Set current position of slam to home. */
    public void setHome() {
        homedPosition = inputs.data.positionRad();
        homed = true;
    }

    private Command homingSequence() {
        return Commands.startRun(
            () -> {
                stopProfile = true;
                homed = false;
                homingDebouncer = new Debouncer(homingTimeSecs.get());
                homingDebouncer.calculate(false);
            },
            () -> {
                if (!brakeModeEnabled) return;
                io.runVolts(homingVolts.get());
                homed =
                    homingDebouncer.calculate(
                        Math.abs(inputs.data.velocityRadPerSec()) <= homingVelocityThresh.get()
                            && Math.abs(inputs.data.appliedVolts()) >= homingVolts.get() * 0.7);
            })
        .until(() -> homed)
        .andThen(this::setHome)
        .finallyDo(
            () -> {
                stopProfile = false;
            });
    }

    void overrideHoming() {
        homed = false;
    }

    /** Get position of slam with maxAngle at home */
  @AutoLogOutput(key = "Intake/Slam/MeasuredAngleRads")
  public double getMeasuredAngleRad() {
    return inputs.data.positionRad() - homedPosition;
  }
}