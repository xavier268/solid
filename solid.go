package solid

import (
	"fmt"
	"math"
	"strings"
)

// Solid represents a rigid body in 3D space with translational and rotational dynamics.
// All physical properties and state variables are stored here.
type Solid struct {
	// === TRANSLATIONAL STATE ===
	Position Vect    // Position of center of mass in global coordinates [m]
	Speed    Vect    // Linear velocity in global coordinates [m/s]
	Mass     float64 // Mass of the rigid body [kg]

	// === ROTATIONAL STATE ===
	Orientation Quaternion // Current orientation as unit quaternion (global → local rotation)
	Omega       Vect       // Angular velocity in local body frame [rad/s]
	Inertia     Vect       // Diagonal inertia tensor [Ixx, Iyy, Izz] in local frame [kg·m²]
}

// ApplyLocalTorque applies a torque in the local body frame and integrates the rotational motion.
// This implements Euler's rotation equations for rigid body dynamics:
//
//	α = I⁻¹ · (τ - ω × (I·ω))
//
// where α is angular acceleration, I is inertia tensor, τ is applied torque,
// ω is angular velocity, and ω × (I·ω) is the gyroscopic term.
//
// Parameters:
//
//	localTorque: Applied torque in local body coordinates [N·m]
//	dt: Time step for integration [s]
func (s *Solid) ApplyLocalTorque(localTorque Vect, dt float64) {
	// Safety check: avoid division by zero for massless rotations
	if s.Inertia[0] < 1e-10 || s.Inertia[1] < 1e-10 || s.Inertia[2] < 1e-10 {
		return // Skip if any inertia component is too small
	}

	// 1. Compute angular acceleration using Euler's rotation equations
	//    α = I⁻¹ · (τ - ω × (I·ω))

	// First, compute I·ω (angular momentum in body frame)
	Jomega := Vect{
		s.Inertia[0] * s.Omega[0],
		s.Inertia[1] * s.Omega[1],
		s.Inertia[2] * s.Omega[2],
	}

	// Compute gyroscopic term: ω × (I·ω)
	// This represents the coupling between rotation axes due to inertia asymmetry
	gyroscopic := s.Omega.CrossProduct(Jomega)

	// Net torque = applied torque - gyroscopic torque
	netTorque := localTorque.Sub(gyroscopic)

	// Compute angular acceleration: α = I⁻¹ · netTorque
	alpha := Vect{
		netTorque[0] / s.Inertia[0],
		netTorque[1] / s.Inertia[1],
		netTorque[2] / s.Inertia[2],
	}

	// 2. Integrate angular velocity: ω_new = ω_old + α·dt
	s.Omega = s.Omega.Add(alpha.Scale(dt))

	// 3. Integrate orientation using quaternion integration
	//    This updates the orientation based on the angular velocity
	s.Orientation = s.Orientation.Integrate(s.Omega, dt)
}

// ApplyLocalForce applies a force at the center of mass in local body coordinates.
// This affects only translational motion (no rotation since force passes through COM).
// The force is first transformed to global coordinates, then Newton's laws are applied.
//
// Parameters:
//
//	localForce: Force vector in local body coordinates [N]
//	dt: Time step for integration [s]
func (s *Solid) ApplyLocalForce(localForce Vect, dt float64) {
	// Safety check for massless objects
	if s.Mass < 1e-10 {
		return // Skip if mass is too small
	}

	// 1. Transform force from local body frame to global frame
	//    F_global = R * F_local, where R is the rotation matrix (quaternion)
	globalForce := s.Orientation.RotateVec(localForce)

	// 2. Apply Newton's second law: F = ma, so a = F/m
	acceleration := globalForce.Scale(1.0 / s.Mass)

	// 3. Integrate linear velocity: v_new = v_old + a·dt
	s.Speed = s.Speed.Add(acceleration.Scale(dt))

	// 4. Integrate position using current velocity
	//    Using semi-implicit Euler: x_new = x_old + v_new·dt
	//    This is more stable than explicit Euler for long time steps
	s.Position = s.Position.Add(s.Speed.Scale(dt))
}

// ApplyLocalForceAtPoint applies a force at a specific point in local body coordinates.
// This generates both translational and rotational motion due to the lever arm effect.
// The force creates a torque about the center of mass: τ = r × F
//
// Parameters:
//
//	localForce: Force vector in local body coordinates [N]
//	localPoint: Point of application relative to COM in local coordinates [m]
//	dt: Time step for integration [s]
func (s *Solid) ApplyLocalForceAtPoint(localForce Vect, localPoint Vect, dt float64) {
	// 1. Transform force and point to global coordinates
	globalForce := s.Orientation.RotateVec(localForce)

	// 2. Apply translation: force acts at center of mass for linear motion
	s.ApplyGlobalForce(globalForce, dt)

	// 3. Compute torque about center of mass: τ = r × F
	//    Use local coordinates directly for more direct calculation
	leverArm := s.Orientation.RotateVec(localPoint) // Transform lever arm to global
	globalTorque := leverArm.CrossProduct(globalForce)

	// 4. Convert global torque back to local frame and apply
	//    Local torque = R^T * global_torque (where R^T = conjugate for unit quaternions)
	localTorque := s.Orientation.Conj().RotateVec(globalTorque)
	s.ApplyLocalTorque(localTorque, dt)
}

// ToLocal converts a vector from global coordinates to local body coordinates.
// Uses the conjugate (inverse) of the orientation quaternion for the transformation.
// Formula: local_vec = q* * global_vec * q (implemented efficiently)
func (s Solid) ToLocal(globalVect Vect) Vect {
	return s.Orientation.Conj().RotateVec(globalVect)
}

// ToGlobal converts a vector from local body coordinates to global coordinates.
// Uses the orientation quaternion directly for the transformation.
// Formula: global_vec = q * local_vec * q* (implemented efficiently)
func (s Solid) ToGlobal(localVect Vect) Vect {
	return s.Orientation.RotateVec(localVect)
}

// ApplyGlobalForce applies a force in global coordinates at the center of mass.
// This affects only translational motion (no rotation since force passes through COM).
// Uses Newton's second law F = ma for acceleration calculation.
//
// Parameters:
//
//	globalForce: Force vector in global coordinates [N]
//	dt: Time step for integration [s]
func (s *Solid) ApplyGlobalForce(globalForce Vect, dt float64) {
	if s.Mass < 1e-10 {
		return // Avoid division by zero for massless objects
	}

	// Calculate acceleration: a = F/m
	acceleration := globalForce.Scale(1.0 / s.Mass)

	// Integrate velocity and position using semi-implicit Euler
	s.Speed = s.Speed.Add(acceleration.Scale(dt))
	s.Position = s.Position.Add(s.Speed.Scale(dt))
}

// ApplyGlobalForceAtPoint applies a force in global coordinates at a specific global point.
// This generates both translational and rotational motion due to the lever arm effect.
// The force creates a torque about the center of mass: τ = r × F
//
// Parameters:
//
//	globalForce: Force vector in global coordinates [N]
//	globalPoint: Point of application in global coordinates [m]
//	dt: Time step for integration [s]
func (s *Solid) ApplyGlobalForceAtPoint(globalForce Vect, globalPoint Vect, dt float64) {
	// 1. Apply translational effect
	s.ApplyGlobalForce(globalForce, dt)

	// 2. Calculate torque: τ = r × F
	leverArm := globalPoint.Sub(s.Position)
	globalTorque := leverArm.CrossProduct(globalForce)

	// 3. Apply torque (converted to local frame)
	localTorque := s.Orientation.Conj().RotateVec(globalTorque)
	s.ApplyLocalTorque(localTorque, dt)
}

// String returns a detailed string representation of the solid's physical state.
// Includes mass, inertia, position, velocity, orientation, and angular velocity information.
// Useful for debugging and monitoring simulation state.
func (s Solid) String() string {
	sb := new(strings.Builder)
	fmt.Fprintf(sb, "Solid        :    Mass  %3e kg\t inertia : %s\n", s.Mass, s.Inertia.String())
	fmt.Fprintf(sb, "Position (m) :    Global %s\t Distance %.1e\n", s.Position.String(), s.Position.Norm())
	fmt.Fprintf(sb, "Speed (m/s)  :    Global %s\t Local %s\t Speed %.1e m/s\n", s.Speed.String(), s.ToLocal(s.Speed).String(), s.Speed.Norm())

	// Local coordinate system basis vectors
	i, j, k := Vect{1, 0, 0}, Vect{0, 1, 0}, Vect{0, 0, 1} // Local basis in local coordinates
	I, J, K := s.ToGlobal(i), s.ToGlobal(j), s.ToGlobal(k) // Local basis in global coordinates
	fmt.Fprintf(sb, "Orientation  :    I : %.1f°, J : %.1f°, K : %.1f°\n", AngleDeg(I, i), AngleDeg(J, j), AngleDeg(K, k))
	localSpeed := s.ToLocal(s.Speed)
	fmt.Fprintf(sb, "Velocity Dir :    I : %.1f°, J : %.1f°, K : %.1f°\n", AngleDeg(I, localSpeed), AngleDeg(J, localSpeed), AngleDeg(K, localSpeed))
	fmt.Fprintf(sb, "Ang. speed   :    Local %s\t Speed (%.1e Rad/s, %.1e °/s)\n", s.Omega.String(), s.Omega.Norm(), s.Omega.Norm()*180/math.Pi)
	return sb.String()
}

// NewCylinder creates a new cylindrical solid with the specified mass, radius, and height.
// The cylinder's main axis is aligned with the Z-axis in the local coordinate system.
// Uses the correct inertia tensor for a solid cylinder.
//
// Parameters:
//
//	mass: Mass of the cylinder [kg], must be > 0
//	radius: Radius of the cylinder [m], must be > 0
//	height: Height of the cylinder [m], can be 0 for a disk
//
// Returns:
//
//	*Solid: Pointer to the created cylinder with proper inertia tensor
//
// Inertia tensor components:
//
//	Iz = (1/2) * m * r²           (around cylinder axis)
//	Ix = Iy = Iz/2 + (1/12) * m * h²  (perpendicular axes)
func NewCylinder(mass, radius, height float64) *Solid {
	iz := 0.5 * mass * radius * radius
	ix := iz/2. + 1./12.*mass*height*height
	return &Solid{
		Mass:        mass,
		Inertia:     Vect{ix, ix, iz},
		Orientation: Quaternion{1, 0, 0, 0},
	}
}

// NewSphere creates a new spherical solid with the specified mass and radius.
// Uses the correct inertia tensor for a solid sphere.
//
// Parameters:
//
//	mass: Mass of the sphere [kg], must be > 0
//	radius: Radius of the sphere [m], must be > 0
//
// Returns:
//
//	*Solid: Pointer to the created sphere with isotropic inertia tensor
//
// Inertia tensor: I = (2/5) * m * r² for all three principal axes
func NewSphere(mass, radius float64) *Solid {
	i := 0.4 * mass * radius * radius
	return &Solid{
		Mass:        mass,
		Inertia:     Vect{i, i, i},
		Orientation: Quaternion{1, 0, 0, 0},
	}
}

// NewBox creates a new rectangular box (cuboid) solid with the specified dimensions.
// Uses the correct inertia tensor for a rectangular solid.
//
// Parameters:
//
//	mass: Mass of the box [kg], must be > 0
//	a: Width along X-axis [m], must be >= 0
//	b: Height along Y-axis [m], must be >= 0
//	c: Depth along Z-axis [m], must be >= 0
//
// Returns:
//
//	*Solid: Pointer to the created box with proper inertia tensor
//
// Note: One dimension can be 0 to create a thin plate.
// Inertia tensor components:
//
//	Ix = (1/12) * m * (b² + c²)
//	Iy = (1/12) * m * (a² + c²)
//	Iz = (1/12) * m * (a² + b²)
func NewBox(mass, a, b, c float64) *Solid {
	ix := 1. / 12. * mass * (b*b + c*c)
	iz := 1. / 12. * mass * (a*a + c*c)
	iy := 1. / 12. * mass * (a*a + b*b)
	return &Solid{
		Mass:        mass,
		Inertia:     Vect{ix, iy, iz},
		Orientation: Quaternion{1, 0, 0, 0},
	}
}
