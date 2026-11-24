// Package tlog reads telemetry logs in the format QGroundControl and
// MAVProxy write: each record is an 8 byte big endian timestamp in
// microseconds followed by one MAVLink frame. The SIL harness writes the
// same format (firmware/sil/logs/*.tlog), which is what lets a simulated
// flight be replayed into this service exactly as a live one would arrive.
package tlog

import (
	"bufio"
	"encoding/binary"
	"errors"
	"fmt"
	"io"
)

// Record is one timestamped frame, raw bytes included.
type Record struct {
	Usec  uint64
	Frame []byte
}

// Reader yields records from a tlog stream.
type Reader struct {
	r *bufio.Reader
}

// NewReader wraps r.
func NewReader(r io.Reader) *Reader { return &Reader{r: bufio.NewReader(r)} }

// Next returns the next record, or io.EOF at a clean end of file.
func (t *Reader) Next() (Record, error) {
	var ts [8]byte
	if _, err := io.ReadFull(t.r, ts[:]); err != nil {
		if errors.Is(err, io.ErrUnexpectedEOF) {
			return Record{}, fmt.Errorf("tlog: truncated timestamp: %w", err)
		}
		return Record{}, err
	}
	head := make([]byte, 3)
	if _, err := io.ReadFull(t.r, head); err != nil {
		return Record{}, fmt.Errorf("tlog: truncated frame header: %w", err)
	}
	if head[0] != 0xFD {
		return Record{}, fmt.Errorf("tlog: expected a MAVLink v2 frame, got 0x%02X", head[0])
	}
	n := 10 + int(head[1]) + 2
	if head[2]&0x01 != 0 {
		n += 13
	}
	frame := make([]byte, n)
	copy(frame, head)
	if _, err := io.ReadFull(t.r, frame[3:]); err != nil {
		return Record{}, fmt.Errorf("tlog: truncated frame: %w", err)
	}
	return Record{Usec: binary.BigEndian.Uint64(ts[:]), Frame: frame}, nil
}
