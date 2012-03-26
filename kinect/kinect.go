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
}

func New() (_k *Kinect, _e error) {
	ctx := usb.NewContext()
	defer func() {
		// Be sure to close the context if we return an error
		if _e != nil {
			ctx.Close()
		}
	}()

	dev, err := ctx.ListDevices(func(bus, addr int, desc *usb.Descriptor) bool {
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
	defer func() {
		// Be sure to close the devices
		for _, d := range dev {
			d.Close()
		}
	}()

	k := &Kinect{
		ctx: ctx,
	}

	for _, d := range dev {
		desc, err := d.Descriptor()
		if err != nil {
			return nil, err
		}

		switch desc.Product {
		case 0x02ae: // XBOX NUI Camera
			k.cam, err = d.Open()
		case 0x02ad: // XBOX NUI Audio
			k.mic, err = d.Open()
		case 0x02b0: // XBOX NUI Motor
			k.mot, err = d.Open()
		}
		if err != nil {
			return nil, err
		}
	}

	switch {
	case k.cam == nil: return nil, fmt.Errorf("kienct: failed to detect camera")
	case k.mic == nil: return nil, fmt.Errorf("kinect: failed to detect mic")
	case k.mot == nil: return nil, fmt.Errorf("kinect: failed to detect motor")
	}

	// Perform initialization
	resp := []byte{0xFF}
	if _, err := k.mot.Control(0xC0, 0x10, 0, 0, resp); err != nil {
		return nil, fmt.Errorf("kinect: motor init: %s", err)
	} else if resp[0] != 0x22 {
		return nil, fmt.Errorf("kinect: motor init failed (code %#x)", resp[0])
	}

	if err := k.SetAngle(+20); err != nil { return nil, err }
	if err := k.SetAngle(-20); err != nil { return nil, err }
	if err := k.SetAngle(0); err != nil { return nil, err }

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

	angle16 := uint16(angle*128/90)
	if _, err := k.mot.Control(0x40, 0x31, angle16, 0, nil); err != nil {
		return fmt.Errorf("kinect: set motor angle: %s", err)
	}

	data := [10]byte{}
	for i := 0; i < 1000; i++ {
		time.Sleep(10*time.Millisecond)
		if n, err := k.mot.Control(0xC0, 0x32, 0, 0, data[:]); err != nil {
			return err
		} else if n != 10 {
			return fmt.Errorf("kinect: motor control short read")
		}

		fmt.Printf("Data: %x", data[:])
		status := data[9]
		strain := data[1]
		moving := data[8]

		if strain > 0x30 {
			k.mot.Control(0x40, 0x31, 0, 0, nil)
			return fmt.Errorf("kinect: motor strain threshold exceeded turning to %+d", angle)
		}

		fmt.Printf(", status: %x, strain %x\n", status, strain)
		if status == 0 && moving != 0x80 {
			break
		}
	}

	return nil
}

func (k *Kinect) Close() error {
	k.mot.Close()
	k.mic.Close()
	k.cam.Close()
	k.ctx.Close()
	return nil
}
