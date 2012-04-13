package kinect

import (
	"testing"
)

var kinect *Kinect

func TestNew(t *testing.T) {
	k, err := New()
	if err != nil {
		t.Fatalf("new(): %s", err)
	}
	kinect = k
}

func TestDepthFrame(t *testing.T) {
	if kinect == nil {
		return
	}
	//kinect.GetDepthFrame()
}

func TestClose(t *testing.T) {
	if kinect == nil {
		return
	}
	kinect.Close()
}
