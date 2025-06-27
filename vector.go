package solid

import (
	"fmt"
	"math"
)

// Vect represents a 3D vector with x, y, z components.
// Used for positions, velocities, forces, torques, and angular velocities.
// Components are accessed as v[0], v[1], v[2] for x, y, z respectively.
type Vect [3]float64

// OuterProduct computes the outer (tensor) product u ⊗ v of two vectors.
// Returns a 3x3 matrix where result[i][j] = u[i] * v[j].
// Useful for constructing matrices from vector operations in mechanics.
func OuterProduct(u, v Vect) Mat {
	var result Mat
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			result[i][j] = u[i] * v[j]
		}
	}
	return result
}

// CrossMatrix returns the skew-symmetric (antisymmetric) matrix [u]× corresponding to vector u.
// This matrix satisfies [u]× * v = u × v (cross product) for any vector v.
// Used to convert cross products into matrix multiplications in rigid body dynamics.
func CrossMatrix(u Vect) Mat {
	return Mat{
		{0, -u[2], u[1]},
		{u[2], 0, -u[0]},
		{-u[1], u[0], 0},
	}
}

// Add performs vector addition: v + w.
// Returns a new vector with component-wise sum.
func (v Vect) Add(w Vect) Vect {
	return Vect{v[0] + w[0], v[1] + w[1], v[2] + w[2]}
}

// Sub performs vector subtraction: v - w.
// Returns a new vector with component-wise difference.
func (v Vect) Sub(w Vect) Vect {
	return Vect{v[0] - w[0], v[1] - w[1], v[2] - w[2]}
}

// Dot computes the dot (scalar) product: v · w.
// Returns the sum of products of corresponding components.
// Result is |v| * |w| * cos(θ) where θ is the angle between vectors.
func (v Vect) Dot(w Vect) float64 {
	return v[0]*w[0] + v[1]*w[1] + v[2]*w[2]
}

// Scale multiplies the vector by a scalar: s * v.
// Returns a new vector with each component multiplied by s.
func (v Vect) Scale(s float64) Vect {
	return Vect{v[0] * s, v[1] * s, v[2] * s}
}

// CrossProduct computes the cross (vector) product: v × w.
// Returns a vector perpendicular to both v and w, following the right-hand rule.
// Magnitude is |v| * |w| * sin(θ) where θ is the angle between vectors.
// Essential for computing torques and angular momentum in rigid body dynamics.
func (v Vect) CrossProduct(w Vect) Vect {
	return Vect{
		v[1]*w[2] - v[2]*w[1], // i component
		v[2]*w[0] - v[0]*w[2], // j component
		v[0]*w[1] - v[1]*w[0], // k component
	}
}

// Mod computes the magnitude (length) of the vector: |v| = √(x² + y² + z²).
// Returns the Euclidean norm of the vector.
func (v Vect) Mod() float64 {
	return math.Sqrt(v.Dot(v))
}

// ModSq computes the squared magnitude of the vector: |v|² = x² + y² + z².
// More efficient than Mod() when only comparing magnitudes or for energy calculations.
func (v Vect) ModSq() float64 {
	return v.Dot(v)
}

// Inv computes the component-wise inverse: (1/x, 1/y, 1/z).
// WARNING: Will panic if any component is zero. Use with caution.
// Useful for scaling operations in physics calculations.
func (v Vect) Inv() Vect {
	return Vect{1.0 / v[0], 1.0 / v[1], 1.0 / v[2]}
}

// String returns a formatted string representation of the vector.
// Format: "(x.xxxe+xx, y.yyye+yy, z.zzze+zz)" in scientific notation.
func (v Vect) String() string {
	return fmt.Sprintf("(%3e, %3e, %3e)", v[0], v[1], v[2])
}

// AngleRad computes the angle between two vectors in radians.
// Uses the formula: θ = arccos((u · v) / (|u| * |v|)).
// Returns a value in the range [0, π].
// WARNING: Will panic if either vector has zero magnitude.
func AngleRad(u, v Vect) float64 {
	// Add safety check for zero-length vectors
	uMod := u.Mod()
	vMod := v.Mod()
	if uMod < 1e-15 || vMod < 1e-15 {
		return 0.0 // Convention: angle with zero vector is 0
	}
	
	// Clamp dot product to [-1, 1] to handle numerical errors
	cosTheta := u.Dot(v) / (uMod * vMod)
	if cosTheta > 1.0 {
		cosTheta = 1.0
	} else if cosTheta < -1.0 {
		cosTheta = -1.0
	}
	
	return math.Acos(cosTheta)
}

// AngleDeg computes the angle between two vectors in degrees.
// Wrapper around AngleRad() with conversion to degrees.
// Returns a value in the range [0, 180].
func AngleDeg(u, v Vect) float64 {
	return AngleRad(u, v) * 180.0 / math.Pi
}
