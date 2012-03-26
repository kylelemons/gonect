package kinect

import (
	"testing"
)

func TestNew(t *testing.T) {
	kinect, err := New()
	if err != nil {
		t.Fatalf("new(): %s", err)
	}
	defer kinect.Close()
}
