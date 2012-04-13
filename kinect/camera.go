package kinect

import (
	"bytes"
	"encoding/binary"
)

type Frame struct {
	Magic   [2]byte // RB
	Control uint8
	Command uint8
	SeqNum  uint8
	PktSeq  uint8
	LengthH uint8
	LengthL uint8
	Time    uint32
}

type Control struct {
	Magic   [2]byte // RB for input GM for output
	Words   uint16
	Command uint16 // 3=SET 2=GET
	Tag     uint16
}

func SetParam(param uint16, value uint16) []byte {
	hdr := Control{
		Magic:   [2]byte{'G', 'M'},
		Words:   2,
		Command: 3,
	}

	buf := new(bytes.Buffer)
	binary.Write(buf, binary.LittleEndian, hdr)
	binary.Write(buf, binary.LittleEndian, param)
	binary.Write(buf, binary.LittleEndian, value)
	return buf.Bytes()
}

func ReadParam(param uint16) []byte {
	hdr := Control{
		Magic:   [2]byte{'G', 'M'},
		Words:   1,
		Command: 2,
	}

	buf := new(bytes.Buffer)
	binary.Write(buf, binary.LittleEndian, hdr)
	binary.Write(buf, binary.LittleEndian, param)
	return buf.Bytes()
}

func DecodeControl(b []byte) (hdr Control, values []uint16) {
	buf := bytes.NewBuffer(b)
	binary.Read(buf, binary.LittleEndian, &hdr)
	for i := uint16(0); i < hdr.Words; i++ {
		var val uint16
		binary.Read(buf, binary.LittleEndian, &val)
		values = append(values, val)
	}
	return hdr, values
}
