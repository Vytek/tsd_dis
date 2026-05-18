package dis_net

import (
	"bytes"
	"encoding/binary"
	"tsd_dis/dis"
)

// ==========================================
// SERIALIZZAZIONE PACCHETTI DI RETE
// ==========================================
// Struttura del pacchetto: [ID (1 byte)] [X (8 byte)] [Y (8 byte)] [Z (8 byte)] = 25 bytes totali

// SerializePDU converte un'entità simulata in un pacchetto di dati da inviare in rete
func SerializePDU(id byte, ecef dis.ECEF) []byte {
	buf := new(bytes.Buffer)
	buf.WriteByte(id)
	_ = binary.Write(buf, binary.LittleEndian, &ecef.X)
	_ = binary.Write(buf, binary.LittleEndian, &ecef.Y)
	_ = binary.Write(buf, binary.LittleEndian, &ecef.Z)
	return buf.Bytes()
}

// DeserializePDU converte un pacchetto di dati ricevuto in rete in un'entità simulata
func DeserializePDU(data []byte) (byte, dis.ECEF) {
	reader := bytes.NewReader(data)
	var id byte
	var ecef dis.ECEF
	id, _ = reader.ReadByte()
	_ = binary.Read(reader, binary.LittleEndian, &ecef.X)
	_ = binary.Read(reader, binary.LittleEndian, &ecef.Y)
	_ = binary.Read(reader, binary.LittleEndian, &ecef.Z)
	return id, ecef
}
