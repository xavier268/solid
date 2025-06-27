package solid

import "math"

// Quaternion represents a unit quaternion used for 3D rotations.
// A quaternion is a hypercomplex number of the form: q = w + xi + yj + zk
// where:
//   - w is the real (scalar) part
//   - x, y, z are the imaginary (vector) parts
//   - i, j, k are the quaternion units with relations:
//     i² = j² = k² = ijk = -1
//     ij = k, jk = i, ki = j
//     ji = -k, kj = -i, ik = -j
//
// For rotations, quaternions are stored as [w, x, y, z] where:
//   - w = cos(θ/2)
//   - (x, y, z) = sin(θ/2) * axis_unit
//   - θ is the rotation angle
//   - axis is the rotation axis
//
// Quaternions provide a singularity-free representation of 3D rotations
// and are more efficient than rotation matrices for composition and interpolation.
type Quaternion [4]float64 // [w, x, y, z]

// Re returns the real (scalar) part of the quaternion.
// For rotation quaternions, this is cos(θ/2) where θ is the rotation angle.
func (q Quaternion) Re() float64 {
	return q[0]
}

// Im returns the imaginary (vector) part of the quaternion as a 3D vector.
// For rotation quaternions, this is sin(θ/2) * axis_unit.
func (q Quaternion) Im() Vect {
	return Vect{q[1], q[2], q[3]}
}

// Quat constructs a quaternion from real and imaginary parts.
// Takes a scalar real part and a 3D vector imaginary part.
func Quat(re float64, im Vect) Quaternion {
	return Quaternion{re, im[0], im[1], im[2]}
}

// Add performs quaternion addition: q + r.
// Component-wise addition of quaternions.
// Note: Addition of rotation quaternions doesn't correspond to rotation composition.
func (q Quaternion) Add(r Quaternion) Quaternion {
	return Quaternion{q[0] + r[0], q[1] + r[1], q[2] + r[2], q[3] + r[3]}
}

// Sub performs quaternion subtraction: q - r.
// Component-wise subtraction of quaternions.
func (q Quaternion) Sub(r Quaternion) Quaternion {
	return Quaternion{q[0] - r[0], q[1] - r[1], q[2] - r[2], q[3] - r[3]}
}

// Scale multiplies the quaternion by a scalar: s * q.
// Scales all components uniformly.
func (q Quaternion) Scale(s float64) Quaternion {
	return Quaternion{q[0] * s, q[1] * s, q[2] * s, q[3] * s}
}

// Mul performs quaternion multiplication: q ⊗ r.
// The quaternion product represents composition of rotations: applying r then q.
// Formula: q₁ ⊗ q₂ = (w₁w₂ - v₁·v₂, w₁v₂ + w₂v₁ + v₁×v₂)
// where w is the scalar part and v is the vector part.
// NOTE: Quaternion multiplication is NOT commutative: q*r ≠ r*q in general.
func (q Quaternion) Mul(r Quaternion) Quaternion {
	w1, x1, y1, z1 := q[0], q[1], q[2], q[3]
	w2, x2, y2, z2 := r[0], r[1], r[2], r[3]

	return Quaternion{
		w1*w2 - x1*x2 - y1*y2 - z1*z2, // real part
		w1*x2 + x1*w2 + y1*z2 - z1*y2, // i component
		w1*y2 - x1*z2 + y1*w2 + z1*x2, // j component
		w1*z2 + x1*y2 - y1*x2 + z1*w2, // k component
	}
}

// Conj returns the quaternion conjugate: q* = (w, -v).
// For unit quaternions, the conjugate represents the inverse rotation.
// Property: q * q* = |q|² (a real number).
func (q Quaternion) Conj() Quaternion {
	return Quaternion{q[0], -q[1], -q[2], -q[3]}
}

// NormSq returns the squared norm of the quaternion: |q|² = w² + x² + y² + z².
// More efficient than Norm() when only comparing magnitudes.
// For unit quaternions (rotations), this should equal 1.
func (q Quaternion) NormSq() float64 {
	return q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]
}

// Norm returns the magnitude of the quaternion: |q| = √(w² + x² + y² + z²).
// For rotation quaternions, this should be 1 (unit quaternion).
func (q Quaternion) Norm() float64 {
	return math.Sqrt(q.NormSq())
}

// Normalize returns the unit quaternion q/|q|.
// Essential for rotation quaternions which must have unit norm.
// Returns identity quaternion if the input has near-zero norm.
func (q Quaternion) Normalize() Quaternion {
	norm := q.Norm()
	if norm < 1e-10 {
		return Quaternion{1, 0, 0, 0} // identity quaternion (no rotation)
	}
	return q.Scale(1.0 / norm)
}

// Inv returns the quaternion inverse: q⁻¹ = q*/|q|².
// For unit quaternions (rotations): q⁻¹ = q* (conjugate).
// The inverse represents the opposite rotation.
// Returns zero quaternion if input has near-zero norm.
func (q Quaternion) Inv() Quaternion {
	normSq := q.NormSq()
	if normSq < 1e-20 {
		return Quaternion{0, 0, 0, 0} // zero quaternion
	}
	return q.Conj().Scale(1.0 / normSq)
}

// === ROTATION CONSTRUCTORS ===

// QuatIdentity returns the identity quaternion representing no rotation.
// This is the multiplicative identity: q * I = I * q = q for any quaternion q.
func QuatIdentity() Quaternion {
	return Quaternion{1, 0, 0, 0}
}

// QuatFromAxisAngle creates a rotation quaternion from an axis and angle.
// The axis will be normalized automatically. Angle is in radians.
// Formula: q = (cos(θ/2), sin(θ/2) * axis_unit)
// Returns identity quaternion if axis has near-zero length.
func QuatFromAxisAngle(axis Vect, angle float64) Quaternion {
	axisNorm := axis.Norm()
	if axisNorm < 1e-10 {
		return QuatIdentity()
	}

	unitAxis := axis.Scale(1.0 / axisNorm)
	halfAngle := angle * 0.5
	sinHalf := math.Sin(halfAngle)
	cosHalf := math.Cos(halfAngle)

	return Quaternion{
		cosHalf,
		unitAxis[0] * sinHalf,
		unitAxis[1] * sinHalf,
		unitAxis[2] * sinHalf,
	}
}

// QuatFromRotVec creates a rotation quaternion from a rotation vector.
// The rotation vector encodes both axis and angle: direction is the axis, magnitude is the angle.
// This is commonly used in robotics and control systems.
// Formula: axis = rotVec/|rotVec|, angle = |rotVec|
func QuatFromRotVec(rotVec Vect) Quaternion {
	angle := rotVec.Norm()
	if angle < 1e-10 {
		return QuatIdentity()
	}
	axis := rotVec.Scale(1.0 / angle)
	return QuatFromAxisAngle(axis, angle)
}

// === ROTATION APPLICATION ===

// RotateVec applies the rotation represented by quaternion q to vector v.
// This implements the rotation formula: v' = q * v * q* without explicit quaternion multiplication.
// Uses the optimized Rodrigues rotation formula for efficiency:
// v' = v + 2w(qv × v) + 2(qv × (qv × v))
// where qv = (x, y, z) is the vector part of q and w is the scalar part.
// This is more efficient than converting to matrix form.
func (q Quaternion) RotateVec(v Vect) Vect {
	w, x, y, z := q[0], q[1], q[2], q[3]

	// Optimized rotation formula avoiding quaternion multiplications
	// v' = v + 2w(qv × v) + 2(qv × (qv × v))
	// where qv = (x, y, z) is the vector part of q
	qv := Vect{x, y, z}
	cross1 := qv.CrossProduct(v)
	cross2 := qv.CrossProduct(cross1)

	return v.Add(cross1.Scale(2.0 * w)).Add(cross2.Scale(2.0))
}

// === CONVERSION ===

// ToAxisAngle extracts the rotation axis and angle from the quaternion.
// Returns the axis as a unit vector and the rotation angle in radians.
// For zero rotation, returns an arbitrary axis (1,0,0) and angle 0.
//
// Returns:
//
//	axis: Unit vector representing the rotation axis
//	angle: Rotation angle in radians [0, π]
func (q Quaternion) ToAxisAngle() (axis Vect, angle float64) {
	// Normalize first
	qn := q.Normalize()

	// Ensure w ≥ 0 to avoid sign ambiguity (q and -q represent same rotation)
	if qn[0] < 0 {
		qn = qn.Scale(-1)
	}

	// w = cos(θ/2), so θ = 2*arccos(w)
	angle = 2.0 * math.Acos(math.Min(1.0, math.Abs(qn[0])))

	// axis = imaginary_part / sin(θ/2)
	sinHalf := math.Sin(angle * 0.5)
	if sinHalf < 1e-10 {
		// No rotation, return arbitrary axis
		return Vect{1, 0, 0}, 0
	}

	axis = Vect{qn[1], qn[2], qn[3]}.Scale(1.0 / sinHalf)
	return axis, angle
}

// ToMatrix converts the quaternion to a 3x3 rotation matrix.
// The resulting matrix can be used to rotate vectors: v' = M * v
// Uses the standard quaternion-to-matrix conversion formula.
//
// Returns:
//
//	Mat: 3x3 rotation matrix equivalent to the quaternion rotation
func (q Quaternion) ToMatrix() Mat {
	// Normalize first to ensure unit quaternion
	qn := q.Normalize()
	w, x, y, z := qn[0], qn[1], qn[2], qn[3]

	// Rotation matrix from unit quaternion using standard formula
	return Mat{
		{1 - 2*(y*y+z*z), 2 * (x*y - w*z), 2 * (x*z + w*y)},
		{2 * (x*y + w*z), 1 - 2*(x*x+z*z), 2 * (y*z - w*x)},
		{2 * (x*z - w*y), 2 * (y*z + w*x), 1 - 2*(x*x+y*y)},
	}
}

// === TEMPORAL INTEGRATION ===

// Integrate updates the quaternion by integrating angular velocity over time.
// Implements the quaternion differential equation: dq/dt = 0.5 * q * (0, ω)
// Uses exact integration via rotation vector for better accuracy.
//
// Parameters:
//
//	omega: Angular velocity vector in the same frame as quaternion [rad/s]
//	dt: Time step for integration [s]
//
// Returns:
//
//	Quaternion: Updated quaternion after integration, automatically normalized
func (q Quaternion) Integrate(omega Vect, dt float64) Quaternion {
	// Incremental rotation quaternion from angular velocity
	deltaQ := QuatFromRotVec(omega.Scale(dt))

	// Composition: q_new = delta_q * q_old (rotation composition)
	return deltaQ.Mul(q).Normalize()
}

// === INTERPOLATION ===

// Slerp performs Spherical Linear Interpolation between two quaternions.
// Provides smooth interpolation along the shortest path on the quaternion sphere.
//
// Parameters:
//
//	r: Target quaternion to interpolate towards
//	t: Interpolation parameter ∈ [0, 1], where t=0 returns q, t=1 returns r
//
// Returns:
//
//	Quaternion: Interpolated quaternion between q and r
func (q Quaternion) Slerp(r Quaternion, t float64) Quaternion {
	// Normalize input quaternions
	q1 := q.Normalize()
	q2 := r.Normalize()

	// Calculate dot product
	dot := q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3]

	// Choose shortest path (q and -q represent same rotation)
	if dot < 0 {
		q2 = q2.Scale(-1)
		dot = -dot
	}

	// If quaternions are very close, use linear interpolation
	if dot > 0.9995 {
		result := q1.Scale(1 - t).Add(q2.Scale(t))
		return result.Normalize()
	}

	// SLERP standard
	theta := math.Acos(math.Abs(dot))
	sinTheta := math.Sin(theta)

	factor1 := math.Sin((1-t)*theta) / sinTheta
	factor2 := math.Sin(t*theta) / sinTheta

	return q1.Scale(factor1).Add(q2.Scale(factor2))
}

// QuatFromMatrix converts a rotation matrix to a quaternion.
// Uses Shepperd's algorithm for numerical stability, which avoids
// near-zero denominators by choosing the largest diagonal element.
//
// Parameters:
//
//	m: 3x3 rotation matrix (should be orthogonal with determinant 1)
//
// Returns:
//
//	Quaternion: Unit quaternion representing the same rotation as the matrix
func QuatFromMatrix(m Mat) Quaternion {
	trace := m[0][0] + m[1][1] + m[2][2]

	var q Quaternion

	if trace > 0 {
		// Standard case: positive trace
		s := math.Sqrt(trace+1.0) * 2  // s = 4 * qw
		q[0] = 0.25 * s                // w
		q[1] = (m[2][1] - m[1][2]) / s // x
		q[2] = (m[0][2] - m[2][0]) / s // y
		q[3] = (m[1][0] - m[0][1]) / s // z
	} else if m[0][0] > m[1][1] && m[0][0] > m[2][2] {
		// m[0][0] is the largest diagonal element
		s := math.Sqrt(1.0+m[0][0]-m[1][1]-m[2][2]) * 2 // s = 4 * qx
		q[0] = (m[2][1] - m[1][2]) / s                  // w
		q[1] = 0.25 * s                                 // x
		q[2] = (m[0][1] + m[1][0]) / s                  // y
		q[3] = (m[0][2] + m[2][0]) / s                  // z
	} else if m[1][1] > m[2][2] {
		// m[1][1] is the largest diagonal element
		s := math.Sqrt(1.0+m[1][1]-m[0][0]-m[2][2]) * 2 // s = 4 * qy
		q[0] = (m[0][2] - m[2][0]) / s                  // w
		q[1] = (m[0][1] + m[1][0]) / s                  // x
		q[2] = 0.25 * s                                 // y
		q[3] = (m[1][2] + m[2][1]) / s                  // z
	} else {
		// m[2][2] is the largest diagonal element
		s := math.Sqrt(1.0+m[2][2]-m[0][0]-m[1][1]) * 2 // s = 4 * qz
		q[0] = (m[1][0] - m[0][1]) / s                  // w
		q[1] = (m[0][2] + m[2][0]) / s                  // x
		q[2] = (m[1][2] + m[2][1]) / s                  // y
		q[3] = 0.25 * s                                 // z
	}

	return q.Normalize()
}
