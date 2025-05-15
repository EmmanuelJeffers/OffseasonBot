package frc.robot.subsystems.superstructure.endeffecter;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.superstructure.SuperstructureConstants;

public class PivotIOSim implements PivotIO {
  private static final double reduction = 64.5;

  public static final double moi = 0.0;
  private static final double cgRadius = Units.inchesToMeters(0.0);
  public static final DCMotor gearbox = DCMotor.getKrakenX60Foc(1).withReduction(reduction);
  public static final Matrix<N2, N2> A =
    MatBuilder.fill(
      Nat.N2(), 
      Nat.N2(), 
      0,
      1,
      0,
      -gearbox.KtNMPerAmp / (gearbox.KvRadPerSecPerVolt * gearbox.rOhms * moi));
  public static final Vector<N2> B = VecBuilder.fill(0, gearbox.KtNMPerAmp / moi);

  // State given by pivot angle position and velocity
  // Input given by torque current to motor
  private Vector<N2> simState;
  private double inputTorqueCurrent = 0.0;
  private double pivotAppliedVolts = 0.0;

  private final PIDController controller = new PIDController(0.0, 0.0, 0.0);
  private double feedforward = 0.0;
  private boolean closedLoop = false;

  public PivotIOSim() {
    simState = VecBuilder.fill(EndEffecter.maxAngle.getRadians() - 0.1, 0.0);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    if (!closedLoop) {
      controller.reset();
      update(Constants.loopPeriodSecs);
    } else {
      // Run control at 1khz
      for (int i = 0; i < Constants.loopPeriodSecs / (1.0 / 1000.0); i++) {
        setInputVoltage(controller.calculate(simState.get(0)) + feedforward);
        update(1.0 / 1000.0);
      }
    }

    // Pivot
    inputs.data =
      new PivotIOData(
        true,
        true, 
        Rotation2d.fromRadians(simState.get(0) - EndEffecter.maxAngle.getRadians()), 
        Rotation2d.fromRadians(simState.get(0)), 
        simState.get(0), 
        simState.get(1), 
        pivotAppliedVolts, 
        (pivotAppliedVolts / 12.0) * inputTorqueCurrent, 
        inputTorqueCurrent, 
        0.0);
  }

  @Override
  public void runOpenLoop(double output) {
    closedLoop = false;
    setInputTorqueCurrent(output);
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    controller.setPID(kP, kI, kD);
  }

  private void setInputTorqueCurrent(double torqueCurrent) {
    inputTorqueCurrent = torqueCurrent;
    pivotAppliedVolts =
        gearbox.getVoltage(gearbox.getTorque(inputTorqueCurrent), simState.get(1, 0));
    pivotAppliedVolts = MathUtil.clamp(pivotAppliedVolts, -12.0, 12.0);
  }

  private void setInputVoltage(double voltage) {
    setInputTorqueCurrent(gearbox.getCurrent(simState.get(1, 0), voltage));
  }

  private void update(double dt) {
    inputTorqueCurrent = MathUtil.clamp(inputTorqueCurrent, -40.0, 40.0);
    Matrix<N2, N1> updatedState =
        NumericalIntegration.rkdp(
            (Matrix<N2, N1> x, Matrix<N1, N1> u) -> {
              Matrix<N2, N1> xdot = A.times(x).plus(B.times(u));
              // Add gravity
              xdot.plus(
                  -SuperstructureConstants.G
                      * cgRadius
                      * Rotation2d.fromRadians(simState.get(0))
                          .minus(SuperstructureConstants.elevatorAngle)
                          .getCos()
                      / moi);
              return xdot;
            },
            simState,
            VecBuilder.fill(inputTorqueCurrent),
            dt);
    // Apply limits
    simState = VecBuilder.fill(updatedState.get(0, 0), updatedState.get(1, 0));
    if (simState.get(0) <= EndEffecter.minAngle.getRadians()) {
      simState.set(1, 0, 0.0);
      simState.set(0, 0, EndEffecter.minAngle.getRadians());
    }
    if (simState.get(0) >= EndEffecter.maxAngle.getRadians()) {
      simState.set(1, 0, 0.0);
      simState.set(0, 0, EndEffecter.maxAngle.getRadians());
    }
  }
}