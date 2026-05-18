package dis

import (
	"math"
)

// ==========================================
// 1. COSTANTI WGS84
// ==========================================
const (
	A       = 6378137.0           // Raggio equatoriale terrestre (metri)
	F       = 1.0 / 298.257223563 // Schiacciamento
	B       = A * (1.0 - F)
	E2      = (A*A - B*B) / (A * A)
	EPrime2 = (A*A - B*B) / (B * B)
)

// ==========================================
// 2. STRUTTURE DATI
// ==========================================
type ECEF struct{ X, Y, Z float64 }
type LLA struct{ Lat, Lon, Alt float64 }
type LocalPhysics struct{ X, Y, Z float64 }

// PilotCommand rappresenta l'input di controllo del veicolo
type PilotCommand struct {
	CourseDegrees float64 // Da 0.0 a 360.0 (0=Nord, 90=Est)
	Speed         float64 // Velocità orizzontale in m/s
	TargetAlt     float64 // Quota desiderata in metri
}

// SimulatedEntity rappresenta l'oggetto nel mondo 3D locale
type SimulatedEntity struct {
	Position LocalPhysics
}

// ==========================================
// 3. FUNZIONI DI TRASFORMAZIONE
// ==========================================

// ToLLA converte le coordinate ECEF in coordinate geografiche LLA
func (ecef ECEF) ToLLA() LLA {
	p := math.Sqrt(ecef.X*ecef.X + ecef.Y*ecef.Y)
	if p == 0 {
		lat := 90.0
		if ecef.Z < 0 {
			lat = -90.0
		}
		return LLA{Lat: lat, Lon: 0.0, Alt: math.Abs(ecef.Z) - B}
	}
	lonRad := math.Atan2(ecef.Y, ecef.X)
	theta := math.Atan2(ecef.Z*A, p*B)
	latRad := math.Atan2(
		ecef.Z+EPrime2*B*math.Pow(math.Sin(theta), 3),
		p-E2*A*math.Pow(math.Cos(theta), 3),
	)
	n := A / math.Sqrt(1.0-E2*math.Pow(math.Sin(latRad), 2))
	return LLA{
		Lat: latRad * 180.0 / math.Pi,
		Lon: lonRad * 180.0 / math.Pi,
		Alt: (p / math.Cos(latRad)) - n,
	}
}

// ToECEF converte le coordinate geografiche LLA in coordinate ECEF
// usando le formule standard del WGS84
func (lla LLA) ToECEF() ECEF {
	latRad := lla.Lat * math.Pi / 180.0
	lonRad := lla.Lon * math.Pi / 180.0
	sinLat, cosLat := math.Sin(latRad), math.Cos(latRad)
	sinLon, cosLon := math.Sin(lonRad), math.Cos(lonRad)
	n := A / math.Sqrt(1.0-E2*sinLat*sinLat)
	return ECEF{
		X: (n + lla.Alt) * cosLat * cosLon,
		Y: (n + lla.Alt) * cosLat * sinLon,
		Z: (n*(1.0-E2) + lla.Alt) * sinLat,
	}
}

// ToLocalPhysics converte le coordinate geografiche LLA in coordinate locali di un'entità
// usando un punto di ancoraggio come riferimento (origine del sistema locale)
func (lla LLA) ToLocalPhysics(anchor LLA) LocalPhysics {
	latRad := anchor.Lat * math.Pi / 180.0
	metersPerLat := (math.Pi * A) / 180.0
	metersPerLon := metersPerLat * math.Cos(latRad)
	return LocalPhysics{
		X: (lla.Lon - anchor.Lon) * metersPerLon, // Est/Ovest
		Y: lla.Alt - anchor.Alt,                  // Su/Giù
		Z: (lla.Lat - anchor.Lat) * metersPerLat, // Nord/Sud
	}
}

// ToLLA converte le coordinate locali di un'entità in coordinate geografiche LLA
// usando un punto di ancoraggio come riferimento (origine del sistema locale)
func (local LocalPhysics) ToLLA(anchor LLA) LLA {
	latRad := anchor.Lat * math.Pi / 180.0
	metersPerLat := (math.Pi * A) / 180.0
	metersPerLon := metersPerLat * math.Cos(latRad)
	return LLA{
		Lat: anchor.Lat + (local.Z / metersPerLat),
		Lon: anchor.Lon + (local.X / metersPerLon),
		Alt: anchor.Alt + local.Y,
	}
}
