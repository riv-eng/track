package track

import (
	"math"
	"fyne.io/fyne/canvas"
)

type Vec2D struct {
	X float64
	Y float64
}

type Circle2D struct {
	O Vec2D
	R float64
}

const NUM_TRACKER_BEACONS int = 4
const NUM_TRACKER_POINTS int = 20

const NUM_ROOM_BEACONS int = 20

type BeaconData struct {
	Major uint16
	Minor uint16
	Pos   Vec2D
	Hot   bool
}

type BScanData struct {
	BD        BeaconData
	Rssi     int8
	Distance float64
}

type RoomData struct {
	Width  float64
	Height float64

	Bcns     [NUM_ROOM_BEACONS]BeaconData
	Bcns_num int
}

type RoomDataGFX struct {
	RD   RoomData
	W      int
	H      int
	Offs_x int
	Offs_y int

	GFX     canvas.Rectangle
	Bcn_gfx [NUM_ROOM_BEACONS]canvas.Rectangle
}

type TrackerData struct {
	Pos       [NUM_TRACKER_POINTS]Vec2D
	Pos_index int
	BS      [NUM_TRACKER_BEACONS]BScanData
	Scan_num  int
}

type TrackerDataGFX struct {
	TD  TrackerData
	GFX [NUM_TRACKER_POINTS]canvas.Circle
}

func Vec_2d_mean(vecs [NUM_TRACKER_BEACONS]Vec2D, num int) Vec2D {
	var sum_x float64 = 0
	var sum_y float64 = 0

	var res Vec2D

	for i := 0; i < num; i++ {
		sum_x += vecs[i].X
		sum_y += vecs[i].Y
	}

	res.X = sum_x / float64(num)
	res.Y = sum_y / float64(num)

	return res
}

func Circle_intersection(c1 Circle2D, c2 Circle2D, right_side bool) Vec2D {

	var res, p1, p2, c2_, v_tmp Vec2D
	var d2, d, a, h, z float64

	res.X = -1
	res.Y = -1

	//go to local sys - c1 = {0;0}

	c2_.X = c2.O.X - c1.O.X
	c2_.Y = c2.O.Y - c1.O.Y

	d2 = c2_.X*c2_.X + c2_.Y*c2_.Y
	d = math.Sqrt(d2)

	if (d > (c1.R + c2.R + 0.01)) || (d < (math.Abs(c1.R-c2.R) + 0.01)) {
		return res
	}

	a = (c1.R*c1.R - c2.R*c2.R + d2) / (2 * d)

	h = math.Sqrt(c1.R*c1.R - a*a)

	v_tmp.X = (a / d) * (c2_.X)
	v_tmp.Y = (a / d) * (c2_.Y)

	p1.X = v_tmp.X + h*c2_.Y/d
	p1.Y = v_tmp.X - h*c2_.X/d

	p2.X = v_tmp.X - h*c2_.Y/d
	p2.Y = v_tmp.Y + h*c2_.X/d

	z = c2_.X*p1.Y - p1.X*c2_.Y

	if right_side {
		if z < 0 {
			res = p1
		} else {
			res = p2
		}
	} else {
		if z < 0 {
			res = p2
		} else {
			res = p1
		}
	}

	res.X = res.X + c1.O.X
	res.Y = res.Y + c1.O.Y

	return res
}

func TrackerScanUpd(td *TrackerData, scan_data []BScanData, num int) {

	td.Scan_num = num

	for i := 0; i < num; i++ {
		td.BS[i] = scan_data[i]
		td.BS[i].Distance = (-0.1895064*float64(scan_data[i].Rssi) - 11.62893)
	}
}

////float meters = 0.17397*exp((float)rssi*(-0.04715));//.5555;
func TrackerPosUpd(td *TrackerData, room *RoomData) {

	var c_buf [NUM_TRACKER_BEACONS]Circle2D
	var c_buf_index int = 0

	for s := 0; s < td.Scan_num; s++ {
		for b := 0; b < room.Bcns_num; b++ {
			if (td.BS[s].BD.Major == room.Bcns[b].Major) && (td.BS[s].BD.Minor == room.Bcns[b].Minor) {
				c_buf[c_buf_index].O = room.Bcns[b].pos
				c_buf[c_buf_index].R = td.BS[s].Distance
				c_buf_index++
			}
		}
	}

	if c_buf_index > 2 {
		var cross [NUM_TRACKER_BEACONS]Vec2D

		for i := 0; i < (c_buf_index - 1); i++ {
			cross[i] = Circle_intersection(c_buf[i], c_buf[i+1], true)
		}

		cross[c_buf_index-1] = Circle_intersection(c_buf[c_buf_index-1], c_buf[0], true)

		td.Pos[td.Pos_index] = Vec_2d_mean(cross, c_buf_index)
		td.Pos_index++

		if td.Pos_index >= NUM_TRACKER_POINTS {
			td.Pos_index = 0
		}
	}
}

func TrackerGFXUpd(tg *TrackerDataGFX, rg *RoomDataGFX) {

}
