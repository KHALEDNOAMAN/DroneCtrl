package mavlink

import (
	"bytes"
	"errors"
	"io"
	"math"
	"os"
	"testing"

	"github.com/KHALEDNOAMAN/DroneCtrl/groundstation/internal/tlog"
)

func TestCRCCheckValue(t *testing.T) {
	if got := crcOf([]byte("123456789")); got != 0x6F91 {
		t.Fatalf("CRC-16/MCRF4XX check value: got %#04x, want 0x6f91", got)
	}
}

// Bytes produced by pymavlink, the same reference the C++ tests use
// (firmware/test/test_telemetry). Three implementations, one answer.
func TestHeartbeatMatchesPymavlink(t *testing.T) {
	want := []byte{0xFD, 0x09, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00,
		0x02, 0x01, 0x00, 0x00, 0x02, 0x00, 0x51, 0x03, 0x03, 0x9D, 0x9A}
	e := Encoder{SystemID: 1, ComponentID: 1}
	got, err := e.Frame(MsgHeartbeat, Heartbeat{CustomMode: 0x0102, Type: 2, BaseMode: 81, SystemStatus: 3, Version: 3}.Encode())
	if err != nil {
		t.Fatal(err)
	}
	if !bytes.Equal(got, want) {
		t.Fatalf("heartbeat\n got % X\nwant % X", got, want)
	}
}

func TestAttitudeMatchesPymavlink(t *testing.T) {
	want := []byte{0xFD, 0x1C, 0x00, 0x00, 0x07, 0x01, 0x01, 0x1E, 0x00, 0x00, 0xD2, 0x04, 0x00, 0x00,
		0xCD, 0xCC, 0xCC, 0x3D, 0xCD, 0xCC, 0x4C, 0xBE, 0x00, 0x00, 0x00, 0x00, 0x0A, 0xD7,
		0x23, 0x3C, 0x0A, 0xD7, 0xA3, 0x3C, 0x8F, 0xC2, 0xF5, 0xBC, 0x04, 0xB4}
	e := Encoder{SystemID: 1, ComponentID: 1, seq: 7}
	got, _ := e.Frame(MsgAttitude, Attitude{TimeBootMS: 1234, Roll: 0.1, Pitch: -0.2,
		RollSpeed: 0.01, PitchSpeed: 0.02, YawSpeed: -0.03}.Encode())
	if !bytes.Equal(got, want) {
		t.Fatalf("attitude\n got % X\nwant % X", got, want)
	}
}

func TestParserRecoversFrameInsideFalseStart(t *testing.T) {
	stream := []byte{0xFD, 200, 0, 0, 0, 1, 1, 0, 0, 0}
	e := Encoder{SystemID: 1, ComponentID: 1}
	for i := 0; i < 3; i++ {
		f, _ := e.Frame(MsgAttitude, Attitude{TimeBootMS: uint32(100 + i), Roll: 0.5}.Encode())
		stream = append(stream, f...)
	}
	stream = append(stream, bytes.Repeat([]byte{0x55}, 200)...)

	p := NewParser()
	var got []Frame
	p.Push(stream, func(f Frame) { got = append(got, f) })
	if len(got) != 3 {
		t.Fatalf("recovered %d frames, want 3", len(got))
	}
	if p.Stats.CRCErrors != 1 {
		t.Fatalf("crc errors %d, want 1", p.Stats.CRCErrors)
	}
	if a := DecodeAttitude(got[2].Payload); a.TimeBootMS != 102 {
		t.Fatalf("last frame time %d, want 102", a.TimeBootMS)
	}
}

func TestParserTruncatedPayloadIsZeroExtended(t *testing.T) {
	e := Encoder{SystemID: 1, ComponentID: 1}
	f, _ := e.Frame(MsgAttitude, Attitude{TimeBootMS: 9}.Encode())
	if f[1] != 1 {
		t.Fatalf("wire length %d, want 1 after truncation", f[1])
	}
	p := NewParser()
	var got Frame
	p.Push(f, func(fr Frame) { got = fr })
	if len(got.Payload) != 28 || DecodeAttitude(got.Payload).TimeBootMS != 9 {
		t.Fatalf("truncated frame not restored: %+v", got)
	}
}

func TestParserSequenceGapsAndWrap(t *testing.T) {
	e := Encoder{SystemID: 1, ComponentID: 1, seq: 250}
	p := NewParser()
	for i := 0; i < 12; i++ {
		f, _ := e.Frame(MsgHeartbeat, Heartbeat{}.Encode())
		if i == 8 {
			continue // lose one, after the wrap
		}
		p.Push(f, func(Frame) {})
	}
	if p.Stats.SeqGaps != 1 {
		t.Fatalf("seq gaps %d, want 1", p.Stats.SeqGaps)
	}
}

func TestParserConsumesUnknownAndSigned(t *testing.T) {
	stream := []byte{0xFD, 4, 0, 0, 0, 255, 190, 76, 0, 0, 0xFD, 0xFD, 0xFD, 0xFD, 0x12, 0x34}
	e := Encoder{SystemID: 1, ComponentID: 1}
	hb, _ := e.Frame(MsgHeartbeat, Heartbeat{Type: 2}.Encode())
	signed := append([]byte(nil), hb...)
	signed[2] |= 0x01
	signed = append(signed, bytes.Repeat([]byte{0xFD}, 13)...)
	stream = append(stream, signed...)
	stream = append(stream, hb...)

	p := NewParser()
	n := 0
	p.Push(stream, func(Frame) { n++ })
	if n != 1 || p.Stats.UnknownIDs != 1 || p.Stats.SignedDropped != 1 || p.Stats.CRCErrors != 0 {
		t.Fatalf("frames %d stats %+v", n, p.Stats)
	}
}

// Telemetry written by the firmware's own C++ code, through the SIL harness.
// Every frame must verify, and the content must be the flight it describes.
func TestDecodesFirmwareTelemetry(t *testing.T) {
	f, err := os.Open("../../testdata/hover.tlog")
	if err != nil {
		t.Fatal(err)
	}
	defer f.Close()
	r := tlog.NewReader(f)
	p := NewParser()
	counts := map[uint32]int{}
	var lastHB Heartbeat
	var maxRoll float64
	for {
		rec, err := r.Next()
		if errors.Is(err, io.EOF) {
			break
		}
		if err != nil {
			t.Fatal(err)
		}
		p.Push(rec.Frame, func(fr Frame) {
			counts[fr.MsgID]++
			switch fr.MsgID {
			case MsgHeartbeat:
				lastHB = DecodeHeartbeat(fr.Payload)
			case MsgAttitude:
				a := DecodeAttitude(fr.Payload)
				if r := math.Abs(float64(a.Roll)); r > maxRoll && a.TimeBootMS > 5000 {
					maxRoll = r
				}
			}
		})
	}
	if p.Stats.CRCErrors != 0 || p.Stats.SeqGaps != 0 {
		t.Fatalf("firmware telemetry did not verify: %+v", p.Stats)
	}
	// 20 s of flight: 1 Hz heartbeat, 25 Hz attitude, 2 Hz status, 10 Hz named values.
	for id, want := range map[uint32]int{MsgHeartbeat: 20, MsgAttitude: 500, MsgSysStatus: 40, MsgNamedValueFloat: 200} {
		if counts[id] != want {
			t.Errorf("message %d: %d frames, want %d", id, counts[id], want)
		}
	}
	if !lastHB.Armed() || lastHB.CustomMode&0xFF != 2 {
		t.Errorf("hover should end ARMED, got custom_mode %#x base_mode %#x", lastHB.CustomMode, lastHB.BaseMode)
	}
	if maxRoll > 5*math.Pi/180 {
		t.Errorf("hover roll reached %.2f deg, the flight log says it stays level", maxRoll*180/math.Pi)
	}
}
