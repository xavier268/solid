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

// Application d'une force globale (plus simple)
func (s *Solid) ApplyGlobalForce(globalForce Vect, dt float64) {

	if s.Mass < 1e-10 {
		return // éviter division par zéro
	}

	// Calculer l'accélération : a = F/m
	acceleration := globalForce.Scale(1.0 / s.Mass)

	// Intégrer la vitesse et position
	s.Speed = s.Speed.Add(acceleration.Scale(dt))
	s.Position = s.Position.Add(s.Speed.Scale(dt))
}

// Application d'une force globale à un point global
func (s *Solid) ApplyGlobalForceAtPoint(globalForce Vect, globalPoint Vect, dt float64) {
	// 1. Translation
	s.ApplyGlobalForce(globalForce, dt)

	// 2. Rotation : couple = r × F
	leverArm := globalPoint.Sub(s.Position)
	globalTorque := leverArm.CrossProduct(globalForce)

	// 3. Appliquer le couple (converti en local)
	localTorque := s.Orientation.Conj().RotateVec(globalTorque)
	s.ApplyLocalTorque(localTorque, dt)
}

// Affiche les informations sur le solide
func (s Solid) String() string {

	sb := new(strings.Builder)
	fmt.Fprintf(sb, "Solid        :    Mass  %3e kg\t inertia : %s\n", s.Mass, s.Inertia.String())
	//fmt.Fprintln(sb)
	fmt.Fprintf(sb, "Position (m) :    Global %s\t Distance %.1e\n", s.Position.String(), s.Position.Mod())
	fmt.Fprintf(sb, "Speed (m/s)  :    Global %s\t Local %s\t Speed %.1e m/s\n", s.Speed.String(), s.ToLocal(s.Speed).String(), s.Speed.Mod())
	//fmt.Fprintln(sb)

	i, j, k := Vect{1, 0, 0}, Vect{0, 1, 0}, Vect{0, 0, 1} // local base in local coord
	I, J, K := s.ToGlobal(i), s.ToGlobal(j), s.ToGlobal(k) // local base in global coord
	fmt.Fprintf(sb, "Orientation  :    I : %.1f°, J : %.1f°, K : %.1f°\n", AngleDeg(I, i), AngleDeg(J, j), AngleDeg(K, k))
	lspeed := s.ToLocal(s.Speed)
	fmt.Fprintf(sb, "Derive       :    I : %.1f°, J : %.1f°, K : %.1f°\n", AngleDeg(I, lspeed), AngleDeg(J, lspeed), AngleDeg(K, lspeed))
	fmt.Fprintf(sb, "Ang. speed   :    Local %s\t Speed (%.1e Rad/s, %.1e °/s)\n", s.Omega.String(), s.Omega.Mod(), s.Omega.Mod()*180/math.Pi)
	return sb.String()
}

// Create a cylinder with the given parameters.
// Main cylinder axis is 0z
// radius must be > 0
// It is ok for height to be 0 (disk)
func NewCylinder(mass, radius, height float64) *Solid {
	iz := 0.5 * mass * radius * radius
	ix := iz/2. + 1./12.*mass*height*height
	return &Solid{
		Mass:        mass,
		Inertia:     Vect{ix, ix, iz},
		Orientation: Quaternion{1, 0, 0, 0},
	}
}

// Create a sphere.
// radius must be > 0
func NewSphere(mass, radius float64) *Solid {
	i := 0.4 * mass * radius * radius
	return &Solid{
		Mass:        mass,
		Inertia:     Vect{i, i, i},
		Orientation: Quaternion{1, 0, 0, 0},
	}
}

// Create a box (cubold).
// it is ok if ONE dimension is 0 (thin plate)
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
