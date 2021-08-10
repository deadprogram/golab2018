/*
You must have ffmpeg and OpenCV installed in order to run this code. It will connect to the Tello
and then open a window using OpenCV showing the streaming video.

How to run

	go run demo/facetracker/main.go ~/Downloads/res10_300x300_ssd_iter_140000.caffemodel ~/Development/opencv/samples/dnn/face_detector/deploy.prototxt opencv cpu
*/

package main

import (
	"bufio"
	"fmt"
	"image"
	"image/color"
	"io"
	"math"
	"os"
	"os/exec"
	"strconv"
	"strings"
	"sync/atomic"
	"time"

	serial "go.bug.st/serial.v1"
	"gobot.io/x/gobot"
	"gobot.io/x/gobot/platforms/dji/tello"
	"gocv.io/x/gocv"
)

type pair struct {
	x int
	y int
}

const (
	frameX    = 400
	frameY    = 300
	frameSize = frameX * frameY * 3
	offset    = 32767
	detente   = 500
)

var (
	// drone
	drone = tello.NewDriver("8890")

	// joystick
	sp                           serial.Port
	leftX, leftY, rightX, rightY atomic.Value
)

func init() {
	leftX.Store(int(0))
	leftY.Store(int(0))
	rightX.Store(int(0))
	rightY.Store(int(0))

	// process drone events in separate goroutine for concurrency
	go func() {
		// process joystick events
		setupJoystick()

		if err := ffmpeg.Start(); err != nil {
			fmt.Println(err)
			return
		}

		// process drone events
		setupDrone()

		robot := gobot.NewRobot("tello",
			[]gobot.Device{drone},
		)
		robot.Start()
	}()
}

func setupJoystick() {
	sp, _ = serial.Open("/dev/ttyACM0", &serial.Mode{BaudRate: 115200})
	reader := bufio.NewReader(sp)

	button1, button2, button3, button4 := false, false, false, false

	gobot.Every(50*time.Millisecond, func() {
		reply, err := reader.ReadBytes('\r')
		if err != nil {
			fmt.Println(err)
		}

		// remove return & new line
		stickdata := strings.Replace(string(reply), "\r", "", -1)
		stickdata = strings.Replace(stickdata, "\n", "", -1)

		// parse data
		data := strings.Split(stickdata, " ")
		if len(data) != 7 {
			fmt.Println("Joystick data underrun error")
			return
		}

		x, _ := strconv.Atoi(data[0])
		y, _ := strconv.Atoi(data[1])

		b1, b2, b3, b4, b5 := false, false, false, false, false
		if data[2] == "true" {
			b1 = true
		}
		if data[3] == "true" {
			b2 = true
		}
		if data[4] == "true" {
			b3 = true
		}
		if data[5] == "true" {
			b4 = true
		}
		if data[6] == "true" {
			b5 = true
		}

		stickmode := "right"

		// handle buttons
		switch {
		case b1:
			// CV system
			if !button1 {
				fmt.Println("CV system")
				button1 = b1
				toggleCVSystem()
			}
		case b2:
			// land
			if !button2 {
				fmt.Println("Land")
				drone.Land()
				//drone.PalmLand()
				button2 = b2
			}
		case b3:
			// flip
			if !button3 {
				fmt.Println("Flip")
				drone.FrontFlip()
				button3 = b3
			}
		case b4:
			// take off
			if !button4 {
				fmt.Println("Takeoff")
				drone.TakeOff()
				//drone.ThrowTakeOff()
				button4 = b4
			}
		case b5:
			stickmode = "left"
			fallthrough
		default:
			// reset store values
			button1 = b1
			button2 = b2
			button3 = b3
			button4 = b4
		}

		// handle control stick
		if stickmode == "right" {
			// set left to center position
			leftX.Store(offset)
			leftY.Store(offset)

			// set right x,y to stick values
			rightX.Store(x)
			rightY.Store(y)

		} else {
			// set left x,y to stick values
			leftX.Store(x)
			leftY.Store(y)

			// set right to center position
			rightX.Store(offset)
			rightY.Store(offset)
		}
	})
}

func getLeftStick() pair {
	s := pair{x: 0, y: 0}
	s.x = leftX.Load().(int)
	s.y = leftY.Load().(int)
	return s
}

func getRightStick() pair {
	s := pair{x: 0, y: 0}
	s.x = rightX.Load().(int)
	s.y = rightY.Load().(int)
	return s
}

func main() {
	if len(os.Args) < 3 {
		fmt.Println("How to run:\ngo run tinyfaces.go [model] [config] ([backend] [device])")
		return
	}

	model := os.Args[1]
	config := os.Args[2]
	backend := gocv.NetBackendDefault
	if len(os.Args) > 3 {
		backend = gocv.ParseNetBackend(os.Args[3])
	}

	target := gocv.NetTargetCPU
	if len(os.Args) > 4 {
		target = gocv.ParseNetTarget(os.Args[4])
	}

	n := gocv.ReadNet(model, config)
	if n.Empty() {
		fmt.Printf("Error reading network model from : %v %v\n", model, config)
		return
	}
	net = &n
	defer net.Close()
	net.SetPreferableBackend(gocv.NetBackendType(backend))
	net.SetPreferableTarget(gocv.NetTargetType(target))

	for {
		// get next frame from stream
		buf := make([]byte, frameSize)
		if _, err := io.ReadFull(ffmpegOut, buf); err != nil {
			fmt.Println(err)
			continue
		}
		img, _ := gocv.NewMatFromBytes(frameY, frameX, gocv.MatTypeCV8UC3, buf)
		if img.Empty() {
			continue
		}

		trackFace(&img)

		window.IMShow(img)
		if window.WaitKey(1) == 27 {
			break
		}
	}
}

// drone related code
func setupDrone() {
	drone.On(tello.ConnectedEvent, func(data interface{}) {
		fmt.Println("Connected")
		drone.StartVideo()
		drone.SetVideoEncoderRate(tello.VideoBitRateAuto)
		drone.SetExposure(0)
		gobot.Every(100*time.Millisecond, func() {
			drone.StartVideo()
		})
	})

	drone.On(tello.VideoFrameEvent, func(data interface{}) {
		pkt := data.([]byte)
		if _, err := ffmpegIn.Write(pkt); err != nil {
			fmt.Println(err)
		}
	})
}

// GoCV related code
var (
	// ffmpeg command to decode video stream from drone
	ffmpeg = exec.Command("ffmpeg", "-hwaccel", "auto", "-hwaccel_device", "opencl", "-i", "pipe:0",
		"-pix_fmt", "bgr24", "-s", strconv.Itoa(frameX)+"x"+strconv.Itoa(frameY), "-f", "rawvideo", "pipe:1")
	ffmpegIn, _  = ffmpeg.StdinPipe()
	ffmpegOut, _ = ffmpeg.StdoutPipe()

	// gocv
	window = gocv.NewWindow("Tello")
	net    *gocv.Net
	green  = color.RGBA{0, 255, 0, 0}

	// tracking
	tracking                 = false
	detected                 = false
	detectSize               = false
	distTolerance            = 0.05 * dist(0, 0, frameX, frameY)
	refDistance              float64
	left, top, right, bottom float64
)

func toggleCVSystem() {
	drone.Forward(0)
	drone.Up(0)
	drone.Clockwise(0)
	tracking = !tracking
	if tracking {
		detectSize = true
		println("tracking")
	} else {
		detectSize = false
		println("not tracking")
	}
}

func trackFace(frame *gocv.Mat) {
	W := float64(frame.Cols())
	H := float64(frame.Rows())

	blob := gocv.BlobFromImage(*frame, 1.0, image.Pt(300, 300), gocv.NewScalar(104, 177, 123, 0), false, false)
	defer blob.Close()

	net.SetInput(blob, "data")

	detBlob := net.Forward("detection_out")
	defer detBlob.Close()

	detections := gocv.GetBlobChannel(detBlob, 0, 0)
	defer detections.Close()

	for r := 0; r < detections.Rows(); r++ {
		confidence := detections.GetFloatAt(r, 2)
		if confidence < 0.5 {
			continue
		}

		left = float64(detections.GetFloatAt(r, 3)) * W
		top = float64(detections.GetFloatAt(r, 4)) * H
		right = float64(detections.GetFloatAt(r, 5)) * W
		bottom = float64(detections.GetFloatAt(r, 6)) * H

		left = math.Min(math.Max(0.0, left), W-1.0)
		right = math.Min(math.Max(0.0, right), W-1.0)
		bottom = math.Min(math.Max(0.0, bottom), H-1.0)
		top = math.Min(math.Max(0.0, top), H-1.0)

		detected = true
		rect := image.Rect(int(left), int(top), int(right), int(bottom))
		gocv.Rectangle(frame, rect, green, 3)
	}

	if !tracking || !detected {
		return
	}

	if detectSize {
		detectSize = false
		refDistance = dist(left, top, right, bottom)
	}

	distance := dist(left, top, right, bottom)

	// x axis
	switch {
	case right < W/2:
		drone.CounterClockwise(50)
	case left > W/2:
		drone.Clockwise(50)
	default:
		drone.Clockwise(0)
	}

	// y axis
	switch {
	case top < H/10:
		drone.Up(25)
	case bottom > H-H/10:
		drone.Down(25)
	default:
		drone.Up(0)
	}

	// z axis
	switch {
	case distance < refDistance-distTolerance:
		drone.Forward(20)
	case distance > refDistance+distTolerance:
		drone.Backward(20)
	default:
		drone.Forward(0)
	}
}

func dist(x1, y1, x2, y2 float64) float64 {
	return math.Sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1))
}
