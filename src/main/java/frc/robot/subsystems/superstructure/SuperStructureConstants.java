package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTunableNumber;

/** Add your docs here. */
public class SuperStructureConstants {
    public static final double G = 9.807;

    public static double firstStageHeight = Units.inchesToMeters(0.0);
    public static double stageHeight = Units.inchesToMeters(0.0);
    public static double stageThickness = Units.inchesToMeters(0.0);
    public static double endeffecterToTop = Units.inchesToMeters(0.0);
    public static double endeffecterToBottom = Units.inchesToMeters(0.0);
    public static double stageToStage = Units.inchesToMeters(0.0);

    // 2d position of superstructure orgin on robot (x forward from center, y off the ground)
    public static final Rotation2d elevatorAngle = Rotation2d.fromDegrees(0.0);
    public static final Translation2d superstructureOrigin2d = new Translation2d();
    public static final Translation3d superstructureOrigin3d =
        new Translation3d(superstructureOrigin2d.getX(), 0.0, superstructureOrigin2d.getY());

    // TODO: get elevator travel distances
    public static final double elevatorMaxTravel = Units.inchesToMeters(0.0);

    public static double stage1ToStage2Height = Units.inchesToMeters(0.0);
    public static double stage2ToStage3Height = Units.inchesToMeters(0.0);

    public static final LoggedTunableNumber pivotMaxSafeAngleDeg =
        new LoggedTunableNumber("Superstructure/PivotMaxSafeAngleDegrees", 0.0);
    public static final LoggedTunableNumber elevatorL4ClearHeight =
        new LoggedTunableNumber("Superstructure/ElevatorL4ClearHeight", 1.65);
}
