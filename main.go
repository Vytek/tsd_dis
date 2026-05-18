package main

import (
	"fmt"
	"math"
	"tsd_dis/dis" // Importa il pacchetto dis che contiene tutte le strutture e funzioni di trasformazione
)

// ==========================================
// 4. LOGICA DI CONTROLLO E DEAD RECKONING
// ==========================================

// UpdateState applica i comandi di navigazione per calcolare il Dead Reckoning
func UpdateState(ent *dis.SimulatedEntity, cmd dis.PilotCommand, anchor dis.LLA, dt float64) {
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
	if velY > 15.0 {
		velY = 15.0
	}
	if velY < -15.0 {
		velY = -15.0
	}

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
	anchorMap := dis.LLA{Lat: 41.9000, Lon: 12.5000, Alt: 0.0}

	// Fase 1: Creazione dell'entità da coordinate geografiche note
	startLLA := dis.LLA{Lat: 41.9050, Lon: 12.5050, Alt: 1000.0}

	mezzo := dis.SimulatedEntity{
		Position: startLLA.ToLocalPhysics(anchorMap),
	}

	fmt.Println("--- STATO INIZIALE ---")
	fmt.Printf("LLA: Lat %.5f, Lon %.5f, Alt %.1fm\n", startLLA.Lat, startLLA.Lon, startLLA.Alt)
	fmt.Printf("Local 3D: X=%.1f (Est), Y=%.1f (Quota), Z=%.1f (Nord)\n", mezzo.Position.X, mezzo.Position.Y, mezzo.Position.Z)

	// Fase 2: Input dei comandi di guida (Course, Speed, Quota)
	comandoPilota := dis.PilotCommand{
		CourseDegrees: 45.0,   // Va verso Nord-Est
		Speed:         200.0,  // 200 m/s
		TargetAlt:     1200.0, // Vuole salire a 1200 metri
	}

	dt := 5.0 // Simuliamo un aggiornamento di 5 secondi nel motore di gioco

	fmt.Printf("\n--- COMANDI APPLICATI (dt = %.1fs) ---\n", dt)
	fmt.Printf("Prua: %.1f°, Velocità: %.1f m/s, Quota Target: %.1fm\n", comandoPilota.CourseDegrees, comandoPilota.Speed, comandoPilota.TargetAlt)

	// Fase 3: Esecuzione Fisica/Dead Reckoning nel Game Engine
	UpdateState(&mezzo, comandoPilota, anchorMap, dt)

	fmt.Println("\n--- STATO DOPO DEAD RECKONING LOCALE ---")
	fmt.Printf("Nuovo Local 3D: X=%.1f, Y=%.1f, Z=%.1f\n", mezzo.Position.X, mezzo.Position.Y, mezzo.Position.Z)

	// Fase 4: Outbound (Riconversione verso la rete DIS)
	aggiornataLLA := mezzo.Position.ToLLA(anchorMap)
	fmt.Printf("Nuova LLA: Lat %.5f, Lon %.5f, Alt %.1fm (Salita progressiva)\n", aggiornataLLA.Lat, aggiornataLLA.Lon, aggiornataLLA.Alt)

	pacchettoDIS := aggiornataLLA.ToECEF()
	fmt.Println("\n--- PACCHETTO PRONTO PER LA RETE DIS ---")
	fmt.Printf("ECEF: X=%.1f, Y=%.1f, Z=%.1f\n", pacchettoDIS.X, pacchettoDIS.Y, pacchettoDIS.Z)
}
