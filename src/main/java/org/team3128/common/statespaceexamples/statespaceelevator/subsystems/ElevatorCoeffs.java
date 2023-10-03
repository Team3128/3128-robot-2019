package org.team3128.common.statespaceexamples.statespaceelevator.subsystems;

//CHECKSTYLE.OFF: AvoidStarImport
import org.team3128.common.controller.StateSpaceControllerCoeffs;
import org.team3128.common.controller.StateSpaceLoop;
import org.team3128.common.controller.StateSpaceObserverCoeffs;
import org.team3128.common.controller.StateSpacePlantCoeffs;
import org.team3128.common.util.math.*;
import org.team3128.common.util.math.numbers.*;
//CHECKSTYLE.ON

public final class ElevatorCoeffs {
  private ElevatorCoeffs() {
  }

  @SuppressWarnings({ "LocalVariableName", "JavadocMethod" })
  public static StateSpacePlantCoeffs<N2, N1, N1> makeElevatorPlantCoeffs() {
    Matrix<N2, N2> A = MatrixUtils.mat(Nat.N2(), Nat.N2()).fill(1.0, 0.0034323689390278016, 0.0, 0.4363739579808415);
    Matrix<N2, N1> B = MatrixUtils.mat(Nat.N2(), Nat.N1()).fill(0.00021137763582757403, 0.07364963688398798);
    Matrix<N1, N2> C = MatrixUtils.mat(Nat.N1(), Nat.N2()).fill(1.0, 0.0);
    Matrix<N1, N1> D = MatrixUtils.mat(Nat.N1(), Nat.N1()).fill(0.0);
    return new StateSpacePlantCoeffs<N2, N1, N1>(Nat.N2(), Nat.N1(), Nat.N1(), A, B, C, D);
  }

  @SuppressWarnings({ "LocalVariableName", "JavadocMethod" })
  public static StateSpaceControllerCoeffs<N2, N1, N1> makeElevatorControllerCoeffs() {
    Matrix<N1, N2> K = MatrixUtils.mat(Nat.N1(), Nat.N2()).fill(232.76812610676956, 5.540693882702349);
    Matrix<N1, N2> Kff = MatrixUtils.mat(Nat.N1(), Nat.N2()).fill(12.902151500051774, 11.23863895630079);
    Matrix<N1, N1> Umin = MatrixUtils.mat(Nat.N1(), Nat.N1()).fill(-12.0);
    Matrix<N1, N1> Umax = MatrixUtils.mat(Nat.N1(), Nat.N1()).fill(12.0);
    return new StateSpaceControllerCoeffs<N2, N1, N1>(K, Kff, Umin, Umax);
  }

  @SuppressWarnings({ "LocalVariableName", "JavadocMethod" })
  public static StateSpaceObserverCoeffs<N2, N1, N1> makeElevatorObserverCoeffs() {
    Matrix<N2, N1> K = MatrixUtils.mat(Nat.N2(), Nat.N1()).fill(0.9999960231492777, 0.7347579419051207);
    return new StateSpaceObserverCoeffs<N2, N1, N1>(K);
  }

  @SuppressWarnings({ "LocalVariableName", "JavadocMethod" })
  public static StateSpaceLoop<N2, N1, N1> makeElevatorLoop() {
    return new StateSpaceLoop<N2, N1, N1>(makeElevatorPlantCoeffs(), makeElevatorControllerCoeffs(),
        makeElevatorObserverCoeffs());
  }
}
