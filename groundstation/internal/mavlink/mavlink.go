// Package mavlink is the ground station's MAVLink v2 codec.
//
// It is the third implementation of the same seven messages in this
// repository, after firmware/include/mavlink_lite.h and the simulator's
// src/hil/mavlink.ts, and it follows them deliberately: the same truncation
// rule, the same resynchronising parser, the same counters. The tests decode
// telemetry the C++ firmware code wrote (testdata/*.tlog, produced by the SIL
// harness), so a disagreement between the two shows up here.
//
// Standard library only, so the service builds anywhere with nothing to
// fetch.
package mavlink

import (
	"encoding/binary"
	"errors"
	"math"
)

// Message ids.
const (
	MsgHeartbeat           uint32 = 0
	MsgSysStatus           uint32 = 1
	MsgAttitude            uint32 = 30
	MsgRcChannelsOverride  uint32 = 70
	MsgHilActuatorControls uint32 = 93
	MsgHilSensor           uint32 = 107
	MsgNamedValueFloat     uint32 = 251
)

type info struct {
	crcExtra byte
	length   int // full payload length, extensions included
}

var messages = map[uint32]info{
	MsgHeartbeat:           {50, 9},
	MsgSysStatus:           {124, 43},
	MsgAttitude:            {39, 28},
	MsgRcChannelsOverride:  {124, 38},
	MsgHilActuatorControls: {47, 81},
	MsgHilSensor:           {108, 65},
	MsgNamedValueFloat:     {170, 18},
}

const (
	stx       = 0xFD
	headerLen = 10
	crcLen    = 2
	sigLen    = 13
)

// CRCAccumulate is CRC-16/MCRF4XX, one byte at a time.
func CRCAccumulate(b byte, crc uint16) uint16 {
	tmp := b ^ byte(crc&0xFF)
	tmp ^= tmp << 4
	return (crc >> 8) ^ (uint16(tmp) << 8) ^ (uint16(tmp) << 3) ^ (uint16(tmp) >> 4)
}

func crcOf(data []byte) uint16 {
	crc := uint16(0xFFFF)
	for _, b := range data {
		crc = CRCAccumulate(b, crc)
	}
	return crc
}

// Frame is one verified frame, payload zero extended to the full length.
type Frame struct {
	MsgID       uint32
	Seq         uint8
	SystemID    uint8
	ComponentID uint8
	WireLength  int
	Payload     []byte
}

// Stats counts what the parser saw.
type Stats struct {
	FramesOK      uint64
	CRCErrors     uint64
	UnknownIDs    uint64
	SignedDropped uint64
	SeqGaps       uint64
}

// Parser is a byte stream to frame decoder. Not safe for concurrent use.
type Parser struct {
	buf     []byte
	Stats   Stats
	lastSeq map[uint8]uint8
}

// NewParser returns a parser with empty state.
func NewParser() *Parser {
	return &Parser{buf: make([]byte, 0, 512), lastSeq: map[uint8]uint8{}}
}

// Push feeds bytes and calls fn for every verified frame.
//
// On a bad checksum only the STX is dropped and the buffered bytes after it
// are rescanned, so a stray 0xFD cannot swallow a real frame inside its
// declared length. Unknown and signed frames are consumed whole.
func (p *Parser) Push(data []byte, fn func(Frame)) {
	for _, b := range data {
		if len(p.buf) == 0 && b != stx {
			continue
		}
		p.buf = append(p.buf, b)
		p.process(fn)
	}
}

func (p *Parser) consume(n int) {
	p.buf = append(p.buf[:0], p.buf[n:]...)
}

func (p *Parser) process(fn func(Frame)) {
	for len(p.buf) > 0 {
		if p.buf[0] != stx {
			p.consume(1)
			continue
		}
		if len(p.buf) < 3 {
			return
		}
		plen := int(p.buf[1])
		signed := p.buf[2]&0x01 != 0
		need := headerLen + plen + crcLen
		if signed {
			need += sigLen
		}
		if len(p.buf) < need {
			return
		}
		msgid := uint32(p.buf[7]) | uint32(p.buf[8])<<8 | uint32(p.buf[9])<<16
		inf, known := messages[msgid]
		if signed {
			p.Stats.SignedDropped++
			p.consume(need)
			continue
		}
		if !known {
			p.Stats.UnknownIDs++
			p.consume(need)
			continue
		}
		crc := crcOf(p.buf[1 : headerLen+plen])
		crc = CRCAccumulate(inf.crcExtra, crc)
		got := binary.LittleEndian.Uint16(p.buf[headerLen+plen:])
		if crc != got || plen > inf.length {
			p.Stats.CRCErrors++
			p.consume(1)
			continue
		}
		payload := make([]byte, inf.length)
		copy(payload, p.buf[headerLen:headerLen+plen])
		f := Frame{
			MsgID: msgid, Seq: p.buf[4], SystemID: p.buf[5], ComponentID: p.buf[6],
			WireLength: plen, Payload: payload,
		}
		if last, ok := p.lastSeq[f.SystemID]; ok {
			p.Stats.SeqGaps += uint64(f.Seq - (last + 1))
		}
		p.lastSeq[f.SystemID] = f.Seq
		p.Stats.FramesOK++
		p.consume(need)
		fn(f)
	}
}

// Encoder frames payloads for one sender.
type Encoder struct {
	SystemID, ComponentID uint8
	seq                   uint8
}

// ErrUnknownMessage is returned for a message id this codec does not carry.
var ErrUnknownMessage = errors.New("mavlink: unknown message id")

// Frame wraps a payload, truncating trailing zeros as v2 requires.
func (e *Encoder) Frame(msgid uint32, payload []byte) ([]byte, error) {
	inf, ok := messages[msgid]
	if !ok || len(payload) > inf.length {
		return nil, ErrUnknownMessage
	}
	n := len(payload)
	for n > 1 && payload[n-1] == 0 {
		n--
	}
	out := make([]byte, headerLen+n+crcLen)
	out[0] = stx
	out[1] = byte(n)
	out[4] = e.seq
	e.seq++
	out[5] = e.SystemID
	out[6] = e.ComponentID
	out[7] = byte(msgid)
	out[8] = byte(msgid >> 8)
	out[9] = byte(msgid >> 16)
	copy(out[headerLen:], payload[:n])
	crc := CRCAccumulate(inf.crcExtra, crcOf(out[1:headerLen+n]))
	binary.LittleEndian.PutUint16(out[headerLen+n:], crc)
	return out, nil
}

// ----------------------------------------------------------------- messages

// Heartbeat is HEARTBEAT (0).
type Heartbeat struct {
	CustomMode   uint32
	Type         uint8
	Autopilot    uint8
	BaseMode     uint8
	SystemStatus uint8
	Version      uint8
}

// DecodeHeartbeat reads a HEARTBEAT payload.
func DecodeHeartbeat(p []byte) Heartbeat {
	return Heartbeat{
		CustomMode: binary.LittleEndian.Uint32(p[0:]), Type: p[4], Autopilot: p[5],
		BaseMode: p[6], SystemStatus: p[7], Version: p[8],
	}
}

// Encode writes a HEARTBEAT payload.
func (h Heartbeat) Encode() []byte {
	p := make([]byte, 9)
	binary.LittleEndian.PutUint32(p, h.CustomMode)
	p[4], p[5], p[6], p[7], p[8] = h.Type, h.Autopilot, h.BaseMode, h.SystemStatus, h.Version
	return p
}

// Armed reports MAV_MODE_FLAG_SAFETY_ARMED.
func (h Heartbeat) Armed() bool { return h.BaseMode&128 != 0 }

// SysStatus is the part of SYS_STATUS (1) DroneCtrl fills in.
type SysStatus struct {
	SensorsPresent, SensorsEnabled, SensorsHealth uint32
	Load                                          uint16 // 0.1 percent
	VoltageMV                                     uint16 // UINT16_MAX unknown
	CurrentCA                                     int16
	DropRateComm, ErrorsComm                      uint16
	ErrorsCount                                   [4]uint16
	BatteryRemaining                              int8
}

// DecodeSysStatus reads a SYS_STATUS payload.
func DecodeSysStatus(p []byte) SysStatus {
	le := binary.LittleEndian
	s := SysStatus{
		SensorsPresent: le.Uint32(p[0:]), SensorsEnabled: le.Uint32(p[4:]), SensorsHealth: le.Uint32(p[8:]),
		Load: le.Uint16(p[12:]), VoltageMV: le.Uint16(p[14:]), CurrentCA: int16(le.Uint16(p[16:])),
		DropRateComm: le.Uint16(p[18:]), ErrorsComm: le.Uint16(p[20:]),
		BatteryRemaining: int8(p[30]),
	}
	for i := range s.ErrorsCount {
		s.ErrorsCount[i] = le.Uint16(p[22+2*i:])
	}
	return s
}

// Attitude is ATTITUDE (30).
type Attitude struct {
	TimeBootMS                      uint32
	Roll, Pitch, Yaw                float32
	RollSpeed, PitchSpeed, YawSpeed float32
}

// DecodeAttitude reads an ATTITUDE payload.
func DecodeAttitude(p []byte) Attitude {
	f := func(o int) float32 { return math.Float32frombits(binary.LittleEndian.Uint32(p[o:])) }
	return Attitude{
		TimeBootMS: binary.LittleEndian.Uint32(p), Roll: f(4), Pitch: f(8), Yaw: f(12),
		RollSpeed: f(16), PitchSpeed: f(20), YawSpeed: f(24),
	}
}

// Encode writes an ATTITUDE payload.
func (a Attitude) Encode() []byte {
	p := make([]byte, 28)
	binary.LittleEndian.PutUint32(p, a.TimeBootMS)
	for i, v := range []float32{a.Roll, a.Pitch, a.Yaw, a.RollSpeed, a.PitchSpeed, a.YawSpeed} {
		binary.LittleEndian.PutUint32(p[4+4*i:], math.Float32bits(v))
	}
	return p
}

// NamedValueFloat is NAMED_VALUE_FLOAT (251).
type NamedValueFloat struct {
	TimeBootMS uint32
	Value      float32
	Name       string
}

// DecodeNamedValueFloat reads a NAMED_VALUE_FLOAT payload.
func DecodeNamedValueFloat(p []byte) NamedValueFloat {
	name := p[8:18]
	n := 0
	for n < len(name) && name[n] != 0 {
		n++
	}
	return NamedValueFloat{
		TimeBootMS: binary.LittleEndian.Uint32(p),
		Value:      math.Float32frombits(binary.LittleEndian.Uint32(p[4:])),
		Name:       string(name[:n]),
	}
}
