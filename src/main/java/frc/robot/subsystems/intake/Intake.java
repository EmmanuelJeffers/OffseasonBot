package frc.robot.subsystems.intake;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.rollers.RollerSystemIO;
import frc.robot.subsystems.sensors.CoralSensorIO;
import frc.robot.subsystems.sensors.CoralSensorIOInputsAutoLogged;
import frc.robot.util.LoggedTracer;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.Setter;

public class Intake extends SubsystemBase {

  private static final LoggedTunableNumber rollerIntakeVolts =
    new LoggedTunableNumber("Intake/Intake/Roller", 12.0);
  private static final LoggedTunableNumber indexerIntakeVolts =
    new LoggedTunableNumber("Intake/Intake/Indexer", 12.0);
  private static final LoggedTunableNumber rollerOuttakeVolts =
    new LoggedTunableNumber("Intake/Outtake/Roller", -8.0);
  private static final LoggedTunableNumber rollerL1EjectVolts =
    new LoggedTunableNumber("Intake/L1Eject/Roller", -2.0);
  private static final LoggedTunableNumber indexerOuttakeVolts =
    new LoggedTunableNumber("Intake/Outtake/Indexer", -6.0);
  private static final LoggedTunableNumber indexerIndexVolts =
    new LoggedTunableNumber("Intake/Index/Indexer", 8.0);
  private static final LoggedTunableNumber rollerReverseVolts =
    new LoggedTunableNumber("Intake/Reverse/Roller", -3.0);
  private static final LoggedTunableNumber indexerReverseVolts =
    new LoggedTunableNumber("Intake/Reverse/Indexer", -3.0);
  private static final LoggedTunableNumber l1ReverseTimeSecs =
    new LoggedTunableNumber("Intake/Reverse/L1ReverseTimeSecs", 0.43);
  private static final LoggedTunableNumber coralProximity =
    new LoggedTunableNumber("Intake/CoralProximity", 0.05);

  private final Coraltake coraltake;
  private final RollerSystem roller;
  private final RollerSystem indexer;
  private final CoralSensorIO coralSensorIO;
  private final CoralSensorIOInputsAutoLogged coralSensorInputs = new CoralSensorIOInputsAutoLogged();

  @AutoLogOutput private Goal goal = Goal.RETRACT;
  private final Timer goalTimer = new Timer();
  @Getter @AutoLogOutput private boolean coralIndexed;
  @Getter @AutoLogOutput private boolean coralIndexedRaw;
  private final Debouncer coralIndexedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  @Setter private boolean hasCoral;
  @AutoLogOutput @Setter private boolean superstructureReady;
  @Setter private BooleanConsumer coralIndexedConsumer;
  private boolean shouldIndex = true;

  @Setter private BooleanSupplier coastOverride = () -> false;

  /** Creates a new Intake. */
  public Intake(
    CoraltakeIO coraltakeIO,
    RollerSystemIO rollerIO,
    RollerSystemIO indexerIO,
    CoralSensorIO coralSensorIO) {
      this.coraltake = new Coraltake(coraltakeIO);
      this.roller = new RollerSystem("Intake roller", "Intake/Roller", rollerIO);
      this.indexer = new RollerSystem("Intake indexer", "Intake/Indexer", indexerIO);
      this.coralSensorIO = coralSensorIO;
      goalTimer.start();
      roller.setBrakeMode(true);
      roller.setBrakeMode(true);
  }

  @Override
  public void periodic() {
    // Set coast mode
    coraltake.setBrakeMode(!coastOverride.getAsBoolean());

    coraltake.periodic();
    roller.periodic();
    indexer.periodic();
    coralSensorIO.updateInputs(coralSensorInputs);
    Logger.processInputs("Intake/CoralSensor", coralSensorInputs);

    // Calculate goal
    Coraltake.Goal coraltakeGoal = null;
    double rollerVolts = 0.0;
    double indexerVolts = 0.0;
    switch (goal) {
      case RETRACT -> {
        coraltakeGoal = Coraltake.Goal.RETRACT;
        if (coraltake.getGoal() == Coraltake.Goal.DEPLOY && coraltake.atGoal()) {
          if (hasCoral) {
            rollerVolts = rollerOuttakeVolts.get();
            indexerVolts = indexerOuttakeVolts.get();
          } else if (coralIndexed) {
            rollerVolts = rollerOuttakeVolts.get();
          }
        }
      }
      case DEPLOY -> {
        coraltakeGoal = Coraltake.Goal.DEPLOY;
        if (coralIndexed && hasCoral) {
          indexerVolts = indexerOuttakeVolts.get();
        }
      }
      case INTAKE -> {
        coraltakeGoal = Coraltake.Goal.DEPLOY;
        if (coraltake.getGoal() != Coraltake.Goal.DEPLOY || !coraltake.atGoal()) break;
        if (hasCoral || coralIndexed) {
          rollerVolts = rollerOuttakeVolts.get();
        } else {
          rollerVolts = rollerIntakeVolts.get();
        }

        if (hasCoral) {
          indexerVolts = indexerOuttakeVolts.get();
        } else {
          if (coralIndexed) {
            indexerVolts = 0.0;
          } else {
            indexerVolts = indexerIntakeVolts.get();
          }
        }
      }
      case OUTTAKE -> {
        coraltakeGoal = Coraltake.Goal.DEPLOY;
        if (coraltake.getGoal() != Coraltake.Goal.DEPLOY || !coraltake.atGoal()) break;
        rollerVolts = rollerOuttakeVolts.get();
        indexerVolts = indexerOuttakeVolts.get();
      }
      case REVERSE -> {
        coraltakeGoal = Coraltake.Goal.REVERSE;
        if (coraltake.getGoal() != Coraltake.Goal.REVERSE || !coraltake.atGoal()) break;
        rollerVolts = rollerReverseVolts.get();
        indexerVolts = indexerReverseVolts.get();
      }
      case L1 -> coraltakeGoal = Coraltake.Goal.L1;
      case L1_EJECT -> {
        coraltakeGoal = Coraltake.Goal.L1_EJECT;
        if (coraltake.getGoal() != Coraltake.Goal.L1_EJECT || !coraltake.atGoal()) break;
        rollerVolts = rollerL1EjectVolts.get();
      }
    }

    // Handle indexing to superstructure
    if (coralIndexed && !hasCoral && superstructureReady && shouldIndex && goal != Goal.OUTTAKE) {
      indexerVolts = indexerIndexVolts.get();
    }

    // Run state
    coraltake.setGoal(coraltakeGoal);
    roller.setVolts(rollerVolts);
    indexer.setVolts(indexerVolts);

    // Get default goal
    if (DriverStation.isDisabled()) {
      goal = coraltake.wantsToDeploy() ? Goal.DEPLOY : Goal.RETRACT;
    }

    // Record cycle time
    LoggedTracer.record("Intake");
  }

  public Command intake() {
    return startEnd(() -> setGoal(Goal.INTAKE), () -> setGoal(Goal.DEPLOY));
  }

  public Command intakeTeleop() {
    return startEnd(() -> setGoal(Goal.INTAKE), () -> setGoal(Goal.RETRACT));
  }

  public Command outtake() {
    return startEnd(() -> setGoal(Goal.OUTTAKE), () -> setGoal(Goal.DEPLOY));
  }

  public Command retract() {
    return setGoalCommand(Goal.RETRACT);
  }

  public Command deploy() {
    return runOnce(() -> setGoal(Goal.DEPLOY));
  }

  public Command prepareToClimb() {
    return setGoalCommand(Goal.CLIMB);
  }

  public Command l1Sequence(BooleanSupplier eject, BooleanSupplier hasCoral) {
    Timer reverseTimer = new Timer();
    return intake()
        .until(hasCoral)
        .andThen(
            setGoalCommand(Goal.REVERSE),
            Commands.waitUntil(() -> coralIndexedRaw),
            Commands.waitUntil(() -> !coralIndexedRaw),
            Commands.runOnce(reverseTimer::restart),
            setGoalCommand(Goal.REVERSE),
            Commands.waitUntil(() -> reverseTimer.hasElapsed(l1ReverseTimeSecs.get())),
            setGoalCommand(Goal.L1),
            Commands.waitUntil(eject),
            setGoalCommand(Goal.L1_EJECT))
        .deadlineFor(Commands.startEnd(() -> shouldIndex = false, () -> shouldIndex = true));
  }

  private Command setGoalCommand(Goal goal) {
    return runOnce(() -> setGoal(goal));
  }

  private void setGoal(Goal goal) {
    if (this.goal == goal) return;
    this.goal = goal;
    goalTimer.restart();
  }

  public Command runHomingSequence() {
    return Commands.runOnce(coraltake::overrideHoming);
  }

  public enum Goal {
    RETRACT,
    DEPLOY,
    INTAKE,
    OUTTAKE,
    REVERSE,
    L1,
    L1_EJECT,
    CLIMB
  }
}
