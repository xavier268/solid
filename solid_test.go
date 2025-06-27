package solid

import (
	"fmt"
	"math"
	"testing"
)

const (
	epsilon = 1e-9 // Tolérance pour les comparaisons de flottants
)

// Helper functions pour les tests
func almostEqual(a, b float64) bool {
	return math.Abs(a-b) < epsilon
}

func vecAlmostEqual(a, b Vect) bool {
	return almostEqual(a[0], b[0]) && almostEqual(a[1], b[1]) && almostEqual(a[2], b[2])
}

func quatAlmostEqual(a, b Quaternion) bool {
	// Les quaternions q et -q représentent la même rotation
	diff1 := math.Abs(a[0]-b[0]) + math.Abs(a[1]-b[1]) + math.Abs(a[2]-b[2]) + math.Abs(a[3]-b[3])
	diff2 := math.Abs(a[0]+b[0]) + math.Abs(a[1]+b[1]) + math.Abs(a[2]+b[2]) + math.Abs(a[3]+b[3])
	return diff1 < epsilon || diff2 < epsilon
}

// Crée un solide test standard
func createTestSolid() *Solid {
	return &Solid{
		Position:    Vect{0, 0, 0},
		Speed:       Vect{0, 0, 0},
		Mass:        1.0,
		Orientation: QuatIdentity(),
		Omega:       Vect{0, 0, 0},
		Inertia:     Vect{1, 1, 1},
	}
}

// Crée un solide test avec des propriétés spécifiques
func createCustomSolid(mass float64, inertia Vect) *Solid {
	return &Solid{
		Position:    Vect{0, 0, 0},
		Speed:       Vect{0, 0, 0},
		Mass:        mass,
		Orientation: QuatIdentity(),
		Omega:       Vect{0, 0, 0},
		Inertia:     inertia,
	}
}

// Test 1: Force globale pure (translation)
func TestApplyGlobalForce(t *testing.T) {
	s := createTestSolid()
	dt := 1.0

	// Appliquer une force de 1N dans la direction X pendant 1s
	force := Vect{1, 0, 0}
	s.ApplyGlobalForce(force, dt)

	// Vérifications
	expectedSpeed := Vect{1, 0, 0} // v = F*dt/m = 1*1/1 = 1
	expectedPos := Vect{1, 0, 0}   // x = v*dt = 1*1 = 1

	if !vecAlmostEqual(s.Speed, expectedSpeed) {
		t.Errorf("Speed incorrecte. Attendu: %v, Obtenu: %v", expectedSpeed, s.Speed)
	}

	if !vecAlmostEqual(s.Position, expectedPos) {
		t.Errorf("Position incorrecte. Attendu: %v, Obtenu: %v", expectedPos, s.Position)
	}

	// L'orientation ne doit pas changer
	if !quatAlmostEqual(s.Orientation, QuatIdentity()) {
		t.Errorf("L'orientation ne devrait pas changer avec une force pure")
	}
	fmt.Println(s)
}

// Test 2: Force locale au centre de masse
func TestApplyLocalForce(t *testing.T) {
	s := createTestSolid()
	dt := 1.0

	// Rotation initiale de 90° autour de Z
	s.Orientation = QuatFromAxisAngle(Vect{0, 0, 1}, math.Pi/2)

	// Force locale dans X (qui devient Y global après rotation)
	localForce := Vect{1, 0, 0}
	s.ApplyLocalForce(localForce, dt)

	// Après rotation de 90°, X local devient Y global
	expectedSpeed := Vect{0, 1, 0}
	expectedPos := Vect{0, 1, 0}

	if !vecAlmostEqual(s.Speed, expectedSpeed) {
		t.Errorf("Speed incorrecte avec rotation. Attendu: %v, Obtenu: %v", expectedSpeed, s.Speed)
	}

	if !vecAlmostEqual(s.Position, expectedPos) {
		t.Errorf("Position incorrecte avec rotation. Attendu: %v, Obtenu: %v", expectedPos, s.Position)
	}
	fmt.Println(s)

}

// Test 3: Couple pur (rotation)
func TestApplyLocalTorque(t *testing.T) {
	s := createTestSolid()
	dt := 1.0

	// Appliquer un couple autour de Z
	torque := Vect{0, 0, 1} // 1 N·m autour de Z
	s.ApplyLocalTorque(torque, dt)

	// Vérifications
	expectedOmega := Vect{0, 0, 1} // ω = τ*dt/J = 1*1/1 = 1 rad/s

	if !vecAlmostEqual(s.Omega, expectedOmega) {
		t.Errorf("Vitesse angulaire incorrecte. Attendu: %v, Obtenu: %v", expectedOmega, s.Omega)
	}

	// La position ne doit pas changer
	if !vecAlmostEqual(s.Position, Vect{0, 0, 0}) {
		t.Errorf("La position ne devrait pas changer avec un couple pur")
	}

	// L'orientation doit avoir changé
	if quatAlmostEqual(s.Orientation, QuatIdentity()) {
		t.Errorf("L'orientation devrait avoir changé avec un couple")
	}
	fmt.Println(s)

}

// Test 4: Force hors centre (translation + rotation)
func TestApplyLocalForceAtPoint(t *testing.T) {
	s := createTestSolid()
	dt := 1.0

	// Force Y appliquée à un point décalé en X
	force := Vect{0, 1, 0} // Force dans Y
	point := Vect{1, 0, 0} // Point à 1m en X du centre

	s.ApplyLocalForceAtPoint(force, point, dt)

	// Vérifications de la translation
	expectedSpeed := Vect{0, 1, 0} // Translation due à la force
	if !vecAlmostEqual(s.Speed, expectedSpeed) {
		t.Errorf("Translation incorrecte. Attendu: %v, Obtenu: %v", expectedSpeed, s.Speed)
	}

	// Vérifications de la rotation
	// Couple = r × F = (1,0,0) × (0,1,0) = (0,0,1)
	expectedOmega := Vect{0, 0, 1} // Rotation autour de Z
	if !vecAlmostEqual(s.Omega, expectedOmega) {
		t.Errorf("Rotation incorrecte. Attendu: %v, Obtenu: %v", expectedOmega, s.Omega)
	}
	fmt.Println(s)

}

// Test 5: Force globale à un point global
func TestApplyGlobalForceAtPoint(t *testing.T) {
	s := createTestSolid()
	s.Position = Vect{5, 5, 0} // Décaler le centre de masse
	dt := 1.0

	globalForce := Vect{0, 1, 0} // Force Y
	globalPoint := Vect{6, 5, 0} // Point décalé de 1m en X du centre

	s.ApplyGlobalForceAtPoint(globalForce, globalPoint, dt)

	// Translation
	expectedSpeed := Vect{0, 1, 0}
	if !vecAlmostEqual(s.Speed, expectedSpeed) {
		t.Errorf("Translation incorrecte. Attendu: %v, Obtenu: %v", expectedSpeed, s.Speed)
	}

	// Rotation (même calcul que le test précédent)
	expectedOmega := Vect{0, 0, 1}
	if !vecAlmostEqual(s.Omega, expectedOmega) {
		t.Errorf("Rotation incorrecte. Attendu: %v, Obtenu: %v", expectedOmega, s.Omega)
	}
	fmt.Println(s)

}

// Test 6: Terme gyroscopique
func TestGyroscopicTerm(t *testing.T) {
	// Solide avec inertie asymétrique
	s := createCustomSolid(1.0, Vect{1, 2, 3})
	s.Omega = Vect{1, 0, 0} // Rotation initiale autour de X
	dt := 0.1               // Petit pas de temps

	// Appliquer un couple nul
	s.ApplyLocalTorque(Vect{0, 0, 0}, dt)

	// Le terme gyroscopique doit modifier Omega même sans couple externe
	// ω × (J·ω) = (1,0,0) × (1,0,0) = (0,0,0) pour cet exemple
	// Mais avec asymétrie, il devrait y avoir un effet

	// Ce test vérifie que le code ne crash pas et que les calculs sont cohérents
	if s.Omega.Mod() > 10 { // Vérification de stabilité numérique
		t.Errorf("Instabilité numérique détectée dans le terme gyroscopique")
	}
	fmt.Println(s)

}

// Test 7: Conservation de l'énergie (approximative)
func TestEnergyConservation(t *testing.T) {
	s := createTestSolid()

	// Donner une vitesse et rotation initiales
	s.Speed = Vect{1, 0, 0}
	s.Omega = Vect{0, 0, 1}

	// Calculer l'énergie initiale
	kineticLinear := 0.5 * s.Mass * s.Speed.ModSq()
	kineticRotational := 0.5 * (s.Inertia[0]*s.Omega[0]*s.Omega[0] +
		s.Inertia[1]*s.Omega[1]*s.Omega[1] +
		s.Inertia[2]*s.Omega[2]*s.Omega[2])
	initialEnergy := kineticLinear + kineticRotational

	// Simuler sans forces externes (juste l'intégration)
	dt := 0.001
	for range 1000 {
		s.ApplyLocalTorque(Vect{0, 0, 0}, dt)
		s.ApplyGlobalForce(Vect{0, 0, 0}, dt)
	}

	// Calculer l'énergie finale
	finalKineticLinear := 0.5 * s.Mass * s.Speed.ModSq()
	finalKineticRotational := 0.5 * (s.Inertia[0]*s.Omega[0]*s.Omega[0] +
		s.Inertia[1]*s.Omega[1]*s.Omega[1] +
		s.Inertia[2]*s.Omega[2]*s.Omega[2])
	finalEnergy := finalKineticLinear + finalKineticRotational

	// L'énergie devrait être approximativement conservée
	energyDiff := math.Abs(finalEnergy - initialEnergy)
	if energyDiff > 0.01 { // Tolérance pour les erreurs numériques
		t.Errorf("Énergie mal conservée. Initiale: %f, Finale: %f, Diff: %f",
			initialEnergy, finalEnergy, energyDiff)
	}
	fmt.Println(s)

}

// Test 8: Cas limites et sécurité
func TestEdgeCases(t *testing.T) {
	// Test avec masse nulle (ne devrait pas crasher)
	s := createCustomSolid(0, Vect{1, 1, 1})
	s.ApplyGlobalForce(Vect{1, 0, 0}, 1.0)
	// Le solide ne devrait pas bouger
	if !vecAlmostEqual(s.Speed, Vect{0, 0, 0}) {
		t.Errorf("Un solide de masse nulle ne devrait pas accélérer")
	}
	fmt.Println(s)

	// Test avec inertie nulle (ne devrait pas crasher)
	s = createCustomSolid(1, Vect{0, 0, 0})
	s.ApplyLocalTorque(Vect{1, 0, 0}, 1.0)
	// Le solide ne devrait pas tourner
	if !vecAlmostEqual(s.Omega, Vect{0, 0, 0}) {
		t.Errorf("Un solide d'inertie nulle ne devrait pas avoir de vitesse angulaire")
	}
	fmt.Println(s)

}

// Test 9: Cohérence entre méthodes locales et globales
func TestLocalGlobalConsistency(t *testing.T) {
	// Créer deux solides identiques
	s1 := createTestSolid()
	s2 := createTestSolid()

	dt := 1.0
	force := Vect{1, 0, 0}

	// Appliquer la même force en local et global
	s1.ApplyLocalForce(force, dt)  // Force locale X
	s2.ApplyGlobalForce(force, dt) // Force globale X

	// Les résultats doivent être identiques (car pas de rotation initiale)
	if !vecAlmostEqual(s1.Speed, s2.Speed) {
		t.Errorf("Incohérence local/global pour la vitesse. Local: %v, Global: %v", s1.Speed, s2.Speed)
	}

	if !vecAlmostEqual(s1.Position, s2.Position) {
		t.Errorf("Incohérence local/global pour la position. Local: %v, Global: %v", s1.Position, s2.Position)
	}
}

// Test 10: Simulation complexe
func TestComplexSimulation(t *testing.T) {
	s := createCustomSolid(2.0, Vect{0.5, 1.0, 1.5}) // Solide asymétrique
	dt := 0.01

	// Simulation de 1 seconde avec forces variées
	for i := range 100 {
		time := float64(i) * dt

		// Gravité
		gravity := Vect{0, 0, -9.81 * s.Mass}
		s.ApplyGlobalForce(gravity, dt)

		// Force sinusoïdale
		oscillatingForce := Vect{math.Sin(time * 10), 0, 0}
		s.ApplyLocalForce(oscillatingForce, dt)

		// Couple périodique
		torque := Vect{0, 0, 0.1 * math.Cos(time*5)}
		s.ApplyLocalTorque(torque, dt)

		// Vérifications de stabilité
		if s.Position.Mod() > 100 {
			t.Errorf("Position explosive détectée: %v", s.Position)
			break
		}

		if s.Speed.Mod() > 100 {
			t.Errorf("Vitesse explosive détectée: %v", s.Speed)
			break
		}

		if s.Omega.Mod() > 100 {
			t.Errorf("Vitesse angulaire explosive détectée: %v", s.Omega)
			break
		}
	}
	fmt.Println(s)

}

// Test 11: Benchmark de performance
func BenchmarkSolidPhysics(b *testing.B) {
	s := createTestSolid()
	dt := 0.016 // ~60 FPS

	force := Vect{1, 0, 0}
	point := Vect{0.5, 0.5, 0}
	torque := Vect{0, 0, 0.1}

	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		s.ApplyLocalForceAtPoint(force, point, dt)
		s.ApplyLocalTorque(torque, dt)
	}
}

func TestString(t *testing.T) {
	s := createTestSolid()
	fmt.Println(s)
}

// Fonction principale de test
func TestMain(m *testing.M) {
	// Ici vous pourriez ajouter une initialisation globale si nécessaire
	m.Run()
}
