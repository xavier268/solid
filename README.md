# Solid - 3D Rigid Body Physics Engine

[![Go Reference](https://pkg.go.dev/badge/github.com/xavier268/solid.svg)](https://pkg.go.dev/github.com/xavier268/solid)
[![Go Report Card](https://goreportcard.com/badge/github.com/xavier268/solid)](https://goreportcard.com/report/github.com/xavier268/solid)
[![Build Status](https://github.com/xavier268/solid/workflows/CI/badge.svg)](https://github.com/xavier268/solid/actions)
[![Coverage Status](https://coveralls.io/repos/github/xavier268/solid/badge.svg?branch=main)](https://coveralls.io/github/xavier268/solid?branch=main)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![GitHub release](https://img.shields.io/github/release/xavier268/solid.svg)](https://github.com/xavier268/solid/releases)
[![Go version](https://img.shields.io/badge/go-1.19+-blue.svg)](https://golang.org/dl/)

A comprehensive Go package for simulating rigid body dynamics in 3D space with accurate physics calculations, quaternion-based rotations, and efficient force/torque integration.

## Overview

The `solid` package provides a complete physics simulation framework for rigid bodies, implementing:

- **Translational dynamics** using Newton's laws
- **Rotational dynamics** using Euler's rotation equations  
- **Quaternion-based orientation** for singularity-free rotations
- **Gyroscopic effects** for realistic spinning object behavior
- **Force and torque application** at arbitrary points
- **Batch action processing** for complex interactions
- **Geometric primitives** (sphere, cylinder, box) with correct inertia tensors

## Key Features

- 🎯 **Accurate Physics**: Implements proper rigid body equations with gyroscopic terms
- 🔄 **Quaternion Rotations**: Stable, efficient 3D rotations without gimbal lock
- ⚡ **Performance Optimized**: Efficient vector/matrix operations and quaternion integration
- 🛡️ **Robust**: Handles edge cases like zero mass/inertia safely
- 🧮 **Mathematical Foundation**: Based on classical mechanics and numerical integration
- 📐 **Geometry Support**: Built-in support for common shapes with realistic inertia properties

## Installation

```bash
go get github.com/xavier268/solid
```

## Quick Start

```go
package main

import (
    "fmt"
    "math"
    "solid"
)

func main() {
    // Create a sphere (1 kg, 0.5 m radius)
    sphere := solid.NewSphere(1.0, 0.5)
    
    // Set initial position and velocity
    sphere.Position = solid.Vect{0, 0, 1}
    sphere.Speed = solid.Vect{1, 0, 0}
    
    // Apply gravity for 1 second
    dt := 0.01 // 10ms time step
    for i := 0; i < 100; i++ {
        gravity := solid.Vect{0, 0, -9.81}
        sphere.ApplyGlobalForce(gravity.Scale(sphere.Mass), dt)
        
        fmt.Printf("t=%.2fs: Position %s\n", 
                   float64(i)*dt, sphere.Position.String())
    }
}
```

## Core Types

### Solid
Represents a rigid body with complete physical state:

```go
type Solid struct {
    // Translational state
    Position solid.Vect    // Center of mass position [m]
    Speed    solid.Vect    // Linear velocity [m/s]  
    Mass     float64       // Mass [kg]
    
    // Rotational state
    Orientation solid.Quaternion // Orientation quaternion
    Omega       solid.Vect       // Angular velocity in body frame [rad/s]
    Inertia     solid.Vect       // Principal inertia tensor [kg⋅m²]
}
```

### Vector (Vect)
3D vector for positions, velocities, forces:

```go
v := solid.Vect{1.0, 2.0, 3.0}
magnitude := v.Mod()
normalized := v.Scale(1.0 / magnitude)
```

### Quaternion
Represents 3D rotations without singularities:

```go
// 90° rotation around Z axis
q := solid.QuatFromAxisAngle(solid.Vect{0, 0, 1}, math.Pi/2)
rotatedVector := q.RotateVec(solid.Vect{1, 0, 0}) // Results in {0, 1, 0}
```

## Force and Torque Application

### Basic Force Application

```go
solid := solid.NewBox(1.0, 1.0, 1.0, 0.1) // 1kg, 1×1×0.1m plate

// Apply force at center of mass (pure translation)
force := solid.Vect{10, 0, 0} // 10N in X direction
solid.ApplyGlobalForce(force, 0.016) // 60 FPS

// Apply local force (affected by orientation)
localForce := solid.Vect{0, 5, 0} // 5N in body Y direction  
solid.ApplyLocalForce(localForce, 0.016)
```

### Force at Specific Points

```go
// Apply force at offset point (causes translation + rotation)
force := solid.Vect{0, 1, 0}        // 1N upward force
point := solid.Vect{0.5, 0, 0}      // Applied 0.5m from center in X
solid.ApplyLocalForceAtPoint(force, point, dt)

// This creates:
// - Translation from the force
// - Rotation from torque = point × force = (0.5,0,0) × (0,1,0) = (0,0,0.5)
```

### Pure Torques

```go
// Apply pure rotation without translation
torque := solid.Vect{0, 0, 1} // 1 N⋅m around Z axis
solid.ApplyLocalTorque(torque, dt)
```

### Batch Operations

```go
// Apply multiple forces/torques simultaneously for better accuracy
actions := solid.MultipleActions{
    Forces: []solid.ForceAction{
        {Force: solid.Vect{0, 0, -9.81}, Point: solid.Vect{0, 0, 0}, IsLocal: false}, // Gravity
        {Force: solid.Vect{1, 0, 0}, Point: solid.Vect{0, 1, 0}, IsLocal: true},      // Thrust
    },
    Torques: []solid.TorqueAction{
        {Torque: solid.Vect{0, 0, 0.1}, IsLocal: true}, // Stabilizing torque
    },
}
solid.ApplyMultipleActions(actions, dt)
```

## Geometric Primitives

### Sphere
```go
sphere := solid.NewSphere(mass, radius)
// Inertia: I = (2/5) * m * r²
```

### Cylinder  
```go
cylinder := solid.NewCylinder(mass, radius, height)
// Main axis along Z, disk if height=0
// Inertia: Iz = (1/2) * m * r², Ix = Iy = Iz/2 + (1/12) * m * h²
```

### Box/Cuboid
```go
box := solid.NewBox(mass, width, height, depth)  
// Inertia: Ix = (1/12) * m * (h² + d²), etc.
```

## Coordinate Systems

The package uses consistent coordinate conventions:

- **Global coordinates**: Fixed world reference frame
- **Local coordinates**: Body-fixed frame that rotates with the object
- **Force application**: Forces can be specified in either frame
- **Point coordinates**: Always relative to center of mass in local frame

## Advanced Features

### Gyroscopic Effects

The package correctly implements Euler's rotation equations including gyroscopic terms:

```
α = I⁻¹ ⋅ (τ - ω × (I⋅ω))
```

This means spinning objects exhibit realistic gyroscopic behavior:

```go
// Create a spinning wheel
wheel := solid.NewCylinder(1.0, 0.3, 0.05)
wheel.Omega = solid.Vect{0, 0, 100} // 100 rad/s spin

// Apply small torque - will cause precession, not simple tilt
torque := solid.Vect{0.1, 0, 0}
wheel.ApplyLocalTorque(torque, dt)
```

### Energy Conservation

The integration scheme conserves energy in the absence of external forces:

```go
// Calculate kinetic energy
translational := 0.5 * solid.Mass * solid.Speed.ModSq()
rotational := 0.5 * (solid.Inertia[0]*solid.Omega[0]*solid.Omega[0] + 
                     solid.Inertia[1]*solid.Omega[1]*solid.Omega[1] + 
                     solid.Inertia[2]*solid.Omega[2]*solid.Omega[2])
totalKE := translational + rotational
```

### Coordinate Transformations

```go
// Convert between coordinate systems
globalVec := solid.Vect{1, 0, 0}
localVec := solid.ToLocal(globalVec)   // Global → Local
backToGlobal := solid.ToGlobal(localVec) // Local → Global
```

## Mathematical Background

### Translational Dynamics
Uses Newton's second law: **F = ma**

Integration uses semi-implicit Euler for stability:
```
v(t+dt) = v(t) + a⋅dt
x(t+dt) = x(t) + v(t+dt)⋅dt  
```

### Rotational Dynamics  
Uses Euler's rotation equations in body frame:
```
τ = I⋅α + ω × (I⋅ω)
```
Where:
- τ = applied torque
- I = inertia tensor (diagonal)
- α = angular acceleration  
- ω = angular velocity
- ω × (I⋅ω) = gyroscopic term

### Quaternion Integration
Orientation updates use exact quaternion integration:
```
q(t+dt) = q_delta ⊗ q(t)
q_delta = exp(0.5 ⋅ ω⋅dt)
```

## Performance Considerations

- **Vector operations**: Optimized for 3D with minimal allocations
- **Quaternion rotations**: Use efficient Rodrigues formula avoiding matrix conversion
- **Batch processing**: `ApplyMultipleActions` reduces coupling errors
- **Memory**: Minimal heap allocations in physics loop

## Testing

Run the comprehensive test suite:

```bash
go test ./solid -v
```

Tests cover:
- ✅ Basic force/torque application
- ✅ Coordinate system transformations  
- ✅ Gyroscopic effects
- ✅ Energy conservation
- ✅ Edge cases (zero mass/inertia)
- ✅ Numerical stability
- ✅ Performance benchmarks

## Examples

### Bouncing Ball
```go
ball := solid.NewSphere(0.1, 0.05) // 100g, 5cm radius
ball.Position = solid.Vect{0, 0, 2} // Start 2m high

for ball.Position[2] > 0 {
    gravity := solid.Vect{0, 0, -9.81 * ball.Mass}
    ball.ApplyGlobalForce(gravity, 0.001)
    
    // Collision with ground
    if ball.Position[2] <= 0.05 {
        ball.Speed[2] *= -0.8 // 80% restitution
        ball.Position[2] = 0.05
    }
}
```

### Spacecraft Attitude Control
```go
spacecraft := solid.NewBox(1000, 2, 2, 4) // 1000kg satellite
spacecraft.Omega = solid.Vect{0.1, 0.05, 0.02} // Initial tumble

// Reaction wheel control
target := solid.QuatIdentity()
for !spacecraft.Orientation.AlmostEqual(target) {
    // Calculate error and apply corrective torque
    error := target.Mul(spacecraft.Orientation.Conj())
    axis, angle := error.ToAxisAngle()
    
    controlTorque := axis.Scale(-angle * 0.1) // Proportional control
    spacecraft.ApplyLocalTorque(controlTorque, 0.1)
}
```

## Contributing

Contributions welcome! Please ensure:

1. All tests pass: `go test ./...`
2. Add tests for new features
3. Follow Go conventions
4. Document physics assumptions
5. Include usage examples

## License

MIT License - see LICENSE file for details.

## References

- Rigid Body Dynamics: Goldstein, Classical Mechanics
- Quaternion Mathematics: Kuipers, Quaternions and Rotation Sequences  
- Numerical Integration: Hairer & Wanner, Solving Ordinary Differential Equations
- Game Physics: Ericson, Real-Time Collision Detection