// +build example
//
// Do not build by default.

/*
How to setup
You must be using the TinyGo serial flight controller, along with a DJI Tello drone to run this example.

You run the Go program on your computer and communicate wirelessly via WiFi with the Tello.

How to run

	go run examples/tinydrone.go
*/

package main

import (
	"bufio"
	"fmt"
	"strconv"
	"strings"
	"sync/atomic"
	"time"

	serial "go.bug.st/serial.v1"
	"gobot.io/x/gobot"
	"gobot.io/x/gobot/platforms/dji/tello"
)

type pair struct {
	x int
	y int
}

var leftX, leftY, rightX, rightY atomic.Value

var sp serial.Port

const offset = 32767
const detente = 500

func main() {
	sp, _ = serial.Open("/dev/ttyACM0", &serial.Mode{BaudRate: 115200})
	reader := bufio.NewReader(sp)

	drone := tello.NewDriver("8888")

	work := func() {
		leftX.Store(int(0))
		leftY.Store(int(0))
		rightX.Store(int(0))
		rightY.Store(int(0))

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

		gobot.Every(50*time.Millisecond, func() {
			rightStick := getRightStick()

			switch {
			case rightStick.y < offset-detente:
				drone.Backward(tello.ValidatePitch(float64(rightStick.y-offset), float64(offset)))

			case rightStick.y > offset+detente:
				drone.Forward(tello.ValidatePitch(float64(rightStick.y-offset), float64(offset)))

			default:
				drone.Forward(0)
			}

			switch {
			case rightStick.x > offset+detente:
				drone.Right(tello.ValidatePitch(float64(rightStick.x-offset), float64(offset)))

			case rightStick.x < offset-detente:
				drone.Left(tello.ValidatePitch(float64(rightStick.x-offset), float64(offset)))

			default:
				drone.Right(0)
			}
		})

		gobot.Every(50*time.Millisecond, func() {
			leftStick := getLeftStick()

			switch {
			case leftStick.y < offset-detente:
				drone.Down(tello.ValidatePitch(float64(leftStick.y-offset), float64(offset)))

			case leftStick.y > offset+detente:
				drone.Up(tello.ValidatePitch(float64(leftStick.y-offset), float64(offset)))

			default:
				drone.Up(0)
			}

			switch {
			case leftStick.x > offset+detente:
				drone.Clockwise(tello.ValidatePitch(float64(leftStick.x-offset), float64(offset)))

			case leftStick.x < offset-detente:
				drone.CounterClockwise(tello.ValidatePitch(float64(leftStick.x-offset), float64(offset)))

			default:
				drone.Clockwise(0)
			}
		})
	}

	robot := gobot.NewRobot("tello",
		[]gobot.Device{drone},
		work,
	)

	robot.Start()
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
