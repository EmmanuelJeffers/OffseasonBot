// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.endeffecter;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLogOutput;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.rollers.RollerSystemIO;
import frc.robot.subsystems.rollers.RollerSystemIOInputsAutoLogged;
import frc.robot.subsystems.sensors.CoralSensorIO;
import frc.robot.subsystems.sensors.CoralSensorIOInputsAutoLogged;
import frc.robot.subsystems.superstructure.pivot.PivotIOInputsAutoLogged;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
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
  private static final LoggedTunableNumber coralProxThreshold =
      new LoggedTunableNumber("EndEffecter/CoralProxThresh", 0.15);
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
  private BooleanSupplier diableOverride = () -> false;

  @AutoLogOutput(key = "EndEffecter/PivotBrakeModeEnabled")
  private boolean brakeModeEnabled = true;

  private TrapezoidProfile profile;

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
    
  }
}
