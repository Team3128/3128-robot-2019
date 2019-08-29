package org.team3128.common.util.math;

import org.team3128.common.util.math.numbers.N1;

/**
 * A specialization of {@link MatBuilder} for constructing vectors (Nx1
 * matrices).
 *
 * @param <N> The dimension of the vector to be constructed.
 */
public class VecBuilder<N extends Num> extends MatBuilder<N, N1> {
  public VecBuilder(Nat<N> rows) {
    super(rows, Nat.N1());
  }
}
