"""Atlas's soft posture objective for the core whole-body IK solver."""

import dartpy as dart
import numpy as np


class RelaxedPosture(dart.optimizer.Function):
    def __init__(self, ideal_posture, lower, upper, weights):
        super().__init__()
        arrays = [
            np.array(value, dtype=float, copy=True)
            for value in (ideal_posture, lower, upper, weights)
        ]
        if any(value.ndim != 1 for value in arrays):
            raise ValueError("posture vectors must be one-dimensional")
        ideal, lower, upper, weights = arrays
        if any(value.size != ideal.size for value in arrays):
            raise ValueError("posture vectors must have equal lengths")
        if (
            not np.isfinite(ideal).all()
            or not np.isfinite(weights).all()
            or (weights < 0).any()
            or np.isnan(lower).any()
            or np.isnan(upper).any()
            or (lower > upper).any()
        ):
            raise ValueError(
                "posture must be finite, with nonnegative weights and ordered bounds"
            )
        self._ideal, self._lower, self._upper, self._weights = arrays
        self.enforce_ideal_posture = False

    def _posture(self, posture):
        posture = np.asarray(posture, dtype=float)
        if (
            posture.ndim != 1
            or posture.size != self._ideal.size
            or not np.isfinite(posture).all()
        ):
            raise ValueError("posture must contain one finite value per DOF")
        return posture

    def _residual(self, posture):
        posture = np.asarray(posture, dtype=float)
        count = min(len(posture), self._ideal.size)
        result = np.zeros(self._ideal.size)
        x = posture[:count]
        weights = self._weights[:count]
        if self.enforce_ideal_posture:
            result[:count] = weights * (x - self._ideal[:count])
        else:
            lower, upper = self._lower[:count], self._upper[:count]
            below, above = x < lower, upper < x
            result[:count][below] = weights[below] * (x[below] - lower[below])
            result[:count][above] = weights[above] * (x[above] - upper[above])
        return result

    def eval(self, posture):
        residual = self._residual(self._posture(posture))
        return float(0.5 * residual.dot(residual))

    def eval_gradient(self, posture):
        return self._residual(self._posture(posture))

    def evalGradient(self, posture, gradient):
        # Preserve the original weighted residual used by the Atlas solver.
        residual = self._residual(posture)
        gradient[:] = 0
        count = min(residual.size, gradient.size)
        gradient[:count] = residual[:count]
