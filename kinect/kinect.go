package kinect

import (
	"fmt"
	"time"

	"github.com/kylelemons/gousb/usb"
)

type Kinect struct {
	ctx *usb.Context
	cam *usb.Device
	mic *usb.Device
	mot *usb.Device

	depth usb.Endpoint
}

func New() (_k *Kinect, _e error) {
	ctx := usb.NewContext()
	defer func() {
		// Be sure to close the context if we return an error
		if _e != nil {
			ctx.Close()
		}
	}()

	dev, err := ctx.ListDevices(func(desc *usb.Descriptor) bool {
		if desc.Vendor != 0x045e { // Microsoft Corp.
			return false
		}
		switch desc.Product {
		case 0x02ae, 0x02ad, 0x02b0:
			return true
		}
		return false
	})
	if err != nil {
		return nil, err
	}
	k := &Kinect{
		ctx: ctx,
	}

	for _, d := range dev {
		switch d.Product {
		case 0x02ae: // XBOX NUI Camera
			k.cam = d
		case 0x02ad: // XBOX NUI Audio
			k.mic = d
		case 0x02b0: // XBOX NUI Motor
			k.mot = d
		default:
			d.Close()
		}
	}

	switch {
	case k.cam == nil:
		return nil, fmt.Errorf("kinect: failed to detect camera")
	case k.mic == nil:
		return nil, fmt.Errorf("kinect: failed to detect mic")
	case k.mot == nil:
		return nil, fmt.Errorf("kinect: failed to detect motor")
	}

	// Perform initialization
	resp := []byte{0xFF}
	if _, err := k.mot.Control(0xC0, 0x10, 0, 0, resp); err != nil {
		return nil, fmt.Errorf("kinect: motor init: %s", err)
	} else if resp[0] != 0x22 {
		return nil, fmt.Errorf("kinect: motor init failed (code %#x)", resp[0])
	}

	//if err := k.SetAngle(+20); err != nil { return nil, err }
	if err := k.SetAngle(-20); err != nil {
		return nil, err
	}
	if err := k.SetAngle(0); err != nil {
		return nil, err
	}

	fmt.Println("Kinect: ", k.cam.Configs)
	if k.depth, err = k.cam.OpenEndpoint(1, 0, 0, 0x82); err != nil {
		return nil, fmt.Errorf("kinect: open depth: %s", err)
	}

	if err := k.initDepth(); err != nil {
		return nil, fmt.Errorf("kinect: init depth: %s", err)
	}

	return k, nil
}

func (k *Kinect) SetAngle(angle int) error {
	// Set LED to yellow
	k.mot.Control(0x40, 0x06, 3, 0, nil)

	// Set LED to green at the end
	defer func() {
		k.mot.Control(0x40, 0x06, 1, 0, nil)
	}()

	if angle > 90 || angle < -90 {
		return fmt.Errorf("kinect: motor angle out of range: %v (must be +/-90)", angle)
	}

	angle16 := uint16(angle * 128 / 90)
	if _, err := k.mot.Control(0x40, 0x31, angle16, 0, nil); err != nil {
		return fmt.Errorf("kinect: set motor angle: %s", err)
	}

	data := [10]byte{}
	for i := 0; i < 1000; i++ {
		time.Sleep(10 * time.Millisecond)
		if n, err := k.mot.Control(0xC0, 0x32, 0, 0, data[:]); err != nil {
			return err
		} else if n != 10 {
			return fmt.Errorf("kinect: motor control short read")
		}

		//fmt.Printf("Data: %x", data[:])
		status := data[9]
		strain := data[1]
		moving := data[8]

		if strain > 0x30 {
			k.mot.Control(0x40, 0x31, 0, 0, nil)
			return fmt.Errorf("kinect: motor strain threshold exceeded turning to %+d", angle)
		}

		//fmt.Printf(", status: %x, strain %x\n", status, strain)
		if status == 0 && moving != 0x80 {
			break
		}
	}

	return nil
}

func (k *Kinect) initDepth() error {
	recv := make([]byte, 200)

	// send_cmd(dev, cmd=3, buf=cmd, len=4, rbuf=reply, rlen=4)
	init0 := SetParam(0x105, 0x00)
	init1 := SetParam(0x06, 0x00)
	// depth format?
	init2 := SetParam(0x13, 0x01)
	init3 := SetParam(0x14, 0x1e)
	init4 := SetParam(0x06, 0x02)
	init5 := SetParam(0x17, 0x00)

	for i, init := range [][]byte{init0, init1, init2, init3, init4, init5} {
		n, err := k.Command(init, recv)
		if err != nil {
			return err
		}
		hdr, val := DecodeControl(recv[:n])
		//fmt.Printf("Command [ %x ] returned %#v %v [ %x ]\n", init, hdr, val, recv[:n])
		if len(val) != 1 || val[0] != 0 {
			return fmt.Errorf("kinect: camera init #%d failed: %v", i, val)
		}
	}
	return nil
}

func (k *Kinect) Command(send, recv []byte) (n int, err error) {
	//fmt.Printf("Command:  %x\n", send)
	if _, err = k.cam.Control(0x40, 0, 0, 0, send); err != nil {
		return 0, err
	}

	tries := 0
	for n == 0 && tries < 1e4 {
		if n, err = k.cam.Control(0xC0, 0, 0, 0, recv); err != nil {
			return n, err
		}
		tries++
	}
	return n, err
}

func (k *Kinect) Close() error {
	k.mot.Control(0x40, 0x06, 2, 0, nil)
	k.mot.Close()
	k.mic.Close()
	k.cam.Close()
	k.ctx.Close()
	return nil
}
