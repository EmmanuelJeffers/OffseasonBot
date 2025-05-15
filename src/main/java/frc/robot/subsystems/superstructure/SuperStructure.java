package frc.robot.subsystems.superstructure;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.endeffecter.EndEffecter;
import lombok.Getter;
import lombok.Setter;

public class Superstructure extends SubsystemBase {
  private final Elevator elevator;
  private final EndEffecter endEffecter;

  @AutoLogOutput(key = "Superstructure/EStopped")
  private boolean isEStopped = false;

  private LoggedNetworkBoolean characterizationModeOn = new LoggedNetworkBoolean("/SmartDashboard/Characterization Mode On", false);

  @Setter private BooleanSupplier disabledOverride = () -> false;
  private final Alert driverDisableAlert = 
    new Alert("Superstructer disabled due to driver override", Alert.AlertType.kWarning);
  private final Alert emergencyDisableAlert =
    new Alert(
        "Superstructure emergency disabled due to high position error. Disable the superstructure manually and reenable to reset.",
        Alert.AlertType.kError);

  private final Debouncer intakeReadyDebouncer = new Debouncer(0.0, DebounceType.kRising);
  private final Timer readyToHomeTimer = new Timer();
  private boolean hasHomed = false;

  // Intake interface
  private BooleanConsumer intakeReadyConsumer = a -> {};
  private BooleanConsumer hasCoralConsumer = a -> {};

  /** Creates a new Superstructure. */
  public Superstructure(Elevator elevator, EndEffecter endEffecter) {
    this.elevator = elevator;
    this.endEffecter = endEffecter;
    readyToHomeTimer.start();

    // Updating E stop beased on disbled override
    new Trigger(() -> disabledOverride.getAsBoolean()).onFalse(Commands.runOnce(() -> isEStopped = false).ignoringDisable(true));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    elevator.periodic();
    endEffecter.periodic();

    // E Stop EndEffecter and Elevator if necessary
    isEStopped = (isEStopped || elevator.isShouldEStop() || endEffecter.isShouldEStop()) && Constants.getMode() == Mode.REAL;
    elevator.setEStopped(isEStopped);
    endEffecter.setEStopped(isEStopped);

    // TODO: add sim intake here

    // TODO: Log State
  }

  public void setIntakeInterface(BooleanConsumer intakeReadyConsumer, BooleanConsumer hasCoralConsumer) {
    this.intakeReadyConsumer = intakeReadyConsumer;
    this.hasCoralConsumer = hasCoralConsumer;
  }

  public boolean hasAlage() {
    return endEffecter.hasAlgae();
  }

  public boolean hasCoral() {
    return endEffecter.hasCoral();
  }

  public void resetHasCoral() {
    endEffecter.resetHasCoral(false);
  }

  public void resetHasAlgae() {
    endEffecter.resetHasAlgae(false);
  }

  public void setGoal() {}

  public Command runGoal() {
    return runOnce(() -> setGoal()).andThen(Commands.idle(this));
  }

  public Command setCharacterizationMode() {
    return Commands.none();
  }

  private Command runElevator(DoubleSupplier elevatorHeight) {
    return Commands.runOnce(() -> elevator.setGoal(elevatorHeight));
  }

  private Command runEndEffecterPivot(Supplier<Rotation2d> pivotAngle) {
    return Commands.runOnce(() -> endEffecter.setGoal(pivotAngle));
  }

  /** Runs elevator and pivot to {@link SuperstructurePose} pose. Ends immediately. */
  private Command runSuperstructurePose(SuperstructurePose pose) {
    return runElevator(pose.elevatorHeight()).alongWith(runEndEffecterPivot(pose.pivotAngle()));
  }

  private boolean mechanismsAtGoal() {
    return elevator.isAtGoal() && (endEffecter.isAtGoal());
  }
}
