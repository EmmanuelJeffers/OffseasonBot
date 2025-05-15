package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.rollers.RollerSystemIO;
import frc.robot.subsystems.rollers.RollerSystemIOSim;
import frc.robot.subsystems.rollers.RollerSystemIOTalonFX;
import frc.robot.subsystems.sensors.CoralSensorIO;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorIO;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOSim;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.superstructure.endeffecter.EndEffecter;
import frc.robot.subsystems.superstructure.endeffecter.PivotIO;
import frc.robot.subsystems.superstructure.endeffecter.PivotIOSim;
import frc.robot.subsystems.superstructure.endeffecter.PivotIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private Drive drive;
  private Vision vision;
  private final Superstructure superstructure;
  private Intake intake;

  // Controller
  private final CommandPS5Controller controller = new CommandPS5Controller(0);

  private final Alert controllerDisconnected = new Alert("Controller disconnected (port 0).", AlertType.kWarning);
//   private final Alert deadInTheWaterAlert = new Alert("Please select an auto routine!!! 😳", AlertType.kWarning);
  private final Trigger superstructureCoast = new Trigger(() -> controller.getHID().getRawButton(15));

  private boolean coastOverride = false;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    Elevator elevator = null;
    EndEffecter endEffecter = null;

    if (Constants.getMode() != Constants.Mode.REPLAY) {
        switch (Constants.getRobot()) {
          case COMPBOT -> {
            drive =
                new Drive(
                    new GyroIOPigeon2(),
                    new ModuleIOTalonFX(TunerConstants.FrontLeft),
                    new ModuleIOTalonFX(TunerConstants.FrontRight),
                    new ModuleIOTalonFX(TunerConstants.BackLeft),
                    new ModuleIOTalonFX(TunerConstants.BackRight));
            vision =
                new Vision(
                    drive::addVisionMeasurement,
                    new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation));
            elevator = new Elevator(new ElevatorIOTalonFX());
            endEffecter =
                new EndEffecter(
                    new PivotIOTalonFX(), 
                    new RollerSystemIOTalonFX(1, "", 0, coastOverride, false, 1.0),
                    new CoralSensorIO() {});
            intake = new Intake();

          }
          case DEVBOT -> {
            drive =
                new Drive(
                    new GyroIOPigeon2(),
                    new ModuleIOTalonFX(TunerConstants.FrontLeft),
                    new ModuleIOTalonFX(TunerConstants.FrontRight),
                    new ModuleIOTalonFX(TunerConstants.BackLeft),
                    new ModuleIOTalonFX(TunerConstants.BackRight));
            vision =
                new Vision(
                    drive::addVisionMeasurement,
                    new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation));
            elevator = new Elevator(new ElevatorIOTalonFX());
            endEffecter =
                new EndEffecter(
                    new PivotIOTalonFX(), 
                    new RollerSystemIOTalonFX(1, "", 0, coastOverride, false, 1.0),
                    new CoralSensorIO() {});
            intake = new Intake();
          }
          case SIMBOT -> {
            drive =
                new Drive(
                    new GyroIO() {},
                    new ModuleIOSim(TunerConstants.FrontLeft),
                    new ModuleIOSim(TunerConstants.FrontRight),
                    new ModuleIOSim(TunerConstants.BackLeft),
                    new ModuleIOSim(TunerConstants.BackRight));
            vision =
                new Vision(
                    drive::addVisionMeasurement,
                    new VisionIOPhotonVisionSim(
                        VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose),
                    new VisionIOPhotonVisionSim(
                        VisionConstants.camera1Name, VisionConstants.robotToCamera1, drive::getPose));
            elevator = new Elevator(new ElevatorIOSim());
            endEffecter =
                new EndEffecter(
                    new PivotIOSim(), 
                    new RollerSystemIOSim(DCMotor.getKrakenX60Foc(1), 1.0, 0.0), 
                    new CoralSensorIO() {});
            intake = new Intake();
        }
      }
    }
  
    // No-op implementations for replay
    if (drive == null) {
    drive =
        new Drive(
            new GyroIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {});
    }
    if (vision == null) {
        switch (Constants.getRobot()) {
            case COMPBOT ->
                vision =
                    new Vision(
                        drive::addVisionMeasurement,
                        new VisionIO() {},
                        new VisionIO() {},
                        new VisionIO() {});
            case DEVBOT -> vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});
            default -> vision = new Vision(drive::addVisionMeasurement);
        }
    }
    if (elevator == null) {
        elevator = new Elevator(new ElevatorIO() {});
    }
    if (endEffecter == null) {
        endEffecter = 
            new EndEffecter(new PivotIO() {}, 
            new RollerSystemIO() {},
            new CoralSensorIO() {});
    }
    if (intake == null) {
        intake = 
            new Intake();
    }
    superstructure = new Superstructure(elevator, endEffecter);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Elevator Static Up",
        superstructure.setCharacterizationMode().andThen(elevator.upStaticCharacterization()));
    autoChooser.addOption(
        "Elevator Static Down",
        superstructure.setCharacterizationMode().andThen(elevator.downStaticCharacterization()));
    autoChooser.addOption(
        "Pivot static",
        superstructure.setCharacterizationMode().andThen(endEffecter.staticCharacterization()));

    // Set up overrides
    superstructure.setDisabledOverride(controller.PS());
    elevator.setOverrides(() -> coastOverride, controller.PS());
    endEffecter.setOverrides(() -> coastOverride, controller.PS());

    // TODO: Set up communictaion for superstructure and intake

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    controller
        .R3()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> new Rotation2d()).withName("Snap Robot to 0°"));

    // Switch to X pattern when X button is pressed
    controller.L3().onTrue(Commands.runOnce(drive::stopWithX, drive).withName("Lock Wheels"));

    // Reset gyro to 0° when B button is pressed
    controller
        .options()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true).withName("Gyro Reset to 0°"));

    // Superstructure coast
    superstructureCoast
        .onTrue(
            Commands.runOnce(
                () -> {
                    if (DriverStation.isDisabled()) {
                        coastOverride = true;
                    }
                })
                .withName("Superstructure Coast")
                .ignoringDisable(true))
        .onFalse(
            Commands.runOnce(
                () -> {
                    coastOverride = false;
                })
                .withName("Superstructure Uncoast")
                .ignoringDisable(true));
    RobotModeTriggers.disabled()
        .onFalse(
            Commands.runOnce(
                () -> {
                    coastOverride = false;
                })
                .ignoringDisable(true));

    // Coral grabbed alert
    new Trigger(superstructure::hasCoral)
        .onTrue(controllerRumbleCommand()
        .withTimeout(0.25));

    // Algae grabbed alert
    new Trigger(superstructure::hasAlage)
        .onTrue(controllerRumbleCommand()
        .withTimeout(0.25));
  }

  // Creates controller rumble command
  private Command controllerRumbleCommand() {
    return Commands.startEnd(
        () -> {
            controller.getHID().setRumble(RumbleType.kBothRumble, 1.0);
        }, 
        () -> {
            controller.getHID().setRumble(RumbleType.kBothRumble, 0.0);
        });
  }

  // Update dashboard data
  public void updateDashboardOutputs() {
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
  }

  public void updateAlerts() {
    // Controller disconnect alerts
    controllerDisconnected.set(!DriverStation.isJoystickConnected(controller.getHID().getPort()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
