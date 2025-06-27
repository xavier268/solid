package solid

// Mat represents a 3x3 matrix used for linear transformations in 3D space.
// Stored in row-major order: Mat[row][column].
// Commonly used for rotation matrices, inertia tensors, and coordinate transformations.
type Mat [3][3]float64

// MulV performs matrix-vector multiplication: M * v.
// Transforms vector v by the linear transformation represented by matrix M.
// Each component r[i] = sum_j(M[i][j] * v[j]).
// Essential for applying rotations and coordinate transformations.
func (m Mat) MulV(v Vect) Vect {
	var r Vect
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			r[i] += m[i][j] * v[j]
		}
	}
	return r
}

// MulM performs matrix-matrix multiplication: M * N.
// Composes two linear transformations: applying N then M.
// Each element r[i][j] = sum_k(M[i][k] * N[k][j]).
// Used for combining rotations and other transformations.
func (m Mat) MulM(n Mat) Mat {
	var r Mat
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			for k := 0; k < 3; k++ {
				r[i][j] += m[i][k] * n[k][j]
			}
		}
	}
	return r
}

// IdentityM returns the 3x3 identity matrix.
// The identity matrix leaves vectors unchanged when multiplied: I * v = v.
// Represents "no transformation" and is the neutral element for matrix multiplication.
func IdentityM() Mat {
	return Mat{
		{1, 0, 0},
		{0, 1, 0},
		{0, 0, 1},
	}
}

// ScaleM multiplies each element of the matrix by scalar s: s * M.
// Scales the transformation represented by the matrix.
// Useful for scaling inertia tensors or damping matrices.
func (m Mat) ScaleM(s float64) Mat {
	var result Mat
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			result[i][j] = m[i][j] * s
		}
	}
	return result
}

// Add performs element-wise matrix addition: M + N.
// Each element result[i][j] = M[i][j] + N[i][j].
// Used for combining transformations additively (e.g., adding corrections to matrices).
func (m Mat) Add(n Mat) Mat {
	var result Mat
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			result[i][j] = m[i][j] + n[i][j]
		}
	}
	return result
}
