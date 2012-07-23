package kinect

import (
	"runtime/debug"
	"testing"
	"time"
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
	t.Logf("depth is %s", kinect.depth.Info())
	time.Sleep(1 * time.Second)
	if err := kinect.GetDepthFrame(); err != nil {
		t.Errorf("depth: %s", err)
	}
}

func TestClose(t *testing.T) {
	if kinect == nil {
		return
	}
	defer func() {
		if r := recover(); r != nil {
			t.Errorf("panic: %v\n%s", r, debug.Stack())
		}
	}()
	kinect.Close()
}
