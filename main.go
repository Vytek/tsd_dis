package main

import (
	"fmt"
	"math"
)

// ==========================================
// 1. COSTANTI WGS84
// ==========================================
const (
	A       = 6378137.0             // Raggio equatoriale terrestre (metri)
	F       = 1.0 / 298.257223563   // Schiacciamento
	B       = A * (1.0 - F)
	E2      = (A*A - B*B) / (A * A)
	EPrime2 = (A*A - B*B) / (B * B)
)

// ==========================================
// 2. STRUTTURE DATI
// ==========================================
type ECEF struct { X, Y, Z float64 }
type LLA struct { Lat, Lon, Alt float64 }
type LocalPhysics struct { X, Y, Z float64 }

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
func (ecef ECEF) ToLLA() LLA {
	p := math.Sqrt(ecef.X*ecef.X + ecef.Y*ecef.Y)
	if p == 0 {
		lat := 90.0
		if ecef.Z < 0 { lat = -90.0 }
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

// ==========================================
// 4. LOGICA DI CONTROLLO E DEAD RECKONING
// ==========================================

// UpdateState applica i comandi di navigazione per calcolare il Dead Reckoning
func (ent *SimulatedEntity) UpdateState(cmd PilotCommand, anchor LLA, dt float64) {
	// 1. Calcolo del vettore velocità orizzontale in base al Course (0-360)
	// Essendo 0 Nord (+Z) e 90 Est (+X), invertiamo seno e coseno rispetto alla trigonometria classica
	courseRad := cmd.CourseDegrees * math.Pi / 180.0
	
	velX := cmd.Speed * math.Sin(courseRad) // Componente Est (X)
	velZ := cmd.Speed * math.Cos(courseRad) // Componente Nord (Z)

	// 2. Controllore di Quota (Altimetro)
	// Calcoliamo l'altitudine attuale nel mondo geografico
	currentLLA := ent.Position.ToLLA(anchor)
	
	// Errore di quota: positivo se dobbiamo salire, negativo se dobbiamo scendere
	altError := cmd.TargetAlt - currentLLA.Alt 
	
	// Un semplice rateo di salita/discesa proporzionale (max 15 m/s)
	velY := altError * 0.5 
	if velY > 15.0 { velY = 15.0 }
	if velY < -15.0 { velY = -15.0 }

	// 3. Applicazione del Dead Reckoning locale: P(t) = P0 + V * dt
	ent.Position.X += velX * dt
	ent.Position.Y += velY * dt
	ent.Position.Z += velZ * dt
}

// ==========================================
// 5. MAIN: PIPELINE COMPLETA
// ==========================================
func main() {
	// Origine della mappa 3D
	anchorMap := LLA{Lat: 41.9000, Lon: 12.5000, Alt: 0.0}

	// Fase 1: Creazione dell'entità da coordinate geografiche note
	startLLA := LLA{Lat: 41.9050, Lon: 12.5050, Alt: 1000.0}
	
	mezzo := SimulatedEntity{
		Position: startLLA.ToLocalPhysics(anchorMap),
	}

	fmt.Println("--- STATO INIZIALE ---")
	fmt.Printf("LLA: Lat %.5f, Lon %.5f, Alt %.1fm\n", startLLA.Lat, startLLA.Lon, startLLA.Alt)
	fmt.Printf("Local 3D: X=%.1f (Est), Y=%.1f (Quota), Z=%.1f (Nord)\n", mezzo.Position.X, mezzo.Position.Y, mezzo.Position.Z)

	// Fase 2: Input dei comandi di guida (Course, Speed, Quota)
	comandoPilota := PilotCommand{
		CourseDegrees: 45.0,   // Va verso Nord-Est
		Speed:         200.0,  // 200 m/s
		TargetAlt:     1200.0, // Vuole salire a 1200 metri
	}

	dt := 5.0 // Simuliamo un aggiornamento di 5 secondi nel motore di gioco

	fmt.Printf("\n--- COMANDI APPLICATI (dt = %.1fs) ---\n", dt)
	fmt.Printf("Prua: %.1f°, Velocità: %.1f m/s, Quota Target: %.1fm\n", comandoPilota.CourseDegrees, comandoPilota.Speed, comandoPilota.TargetAlt)

	// Fase 3: Esecuzione Fisica/Dead Reckoning nel Game Engine
	mezzo.UpdateState(comandoPilota, anchorMap, dt)

	fmt.Println("\n--- STATO DOPO DEAD RECKONING LOCALE ---")
	fmt.Printf("Nuovo Local 3D: X=%.1f, Y=%.1f, Z=%.1f\n", mezzo.Position.X, mezzo.Position.Y, mezzo.Position.Z)

	// Fase 4: Outbound (Riconversione verso la rete DIS)
	aggiornataLLA := mezzo.Position.ToLLA(anchorMap)
	fmt.Printf("Nuova LLA: Lat %.5f, Lon %.5f, Alt %.1fm (Salita progressiva)\n", aggiornataLLA.Lat, aggiornataLLA.Lon, aggiornataLLA.Alt)

	pacchettoDIS := aggiornataLLA.ToECEF()
	fmt.Println("\n--- PACCHETTO PRONTO PER LA RETE DIS ---")
	fmt.Printf("ECEF: X=%.1f, Y=%.1f, Z=%.1f\n", pacchettoDIS.X, pacchettoDIS.Y, pacchettoDIS.Z)
}