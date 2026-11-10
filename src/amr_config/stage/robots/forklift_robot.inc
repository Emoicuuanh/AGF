define laser ranger
(
  sensor
  (
    range [0 100] # Distance laser detect [min max] (m)
    fov 360       # Field Of View (deg)
    samples 720   # Number of samples in field of view
    # size [ 0.05 0.05 0.05 ]
  )

	block(
		points 4
		point[0] [0 1]
		point[1] [1 1]
		point[2] [1 0]
		point[3] [0 0]
		z [0 1]
	)

  # generic model properties
  color_rgba [ 0 0 1 1 ]
  size [ 0.05 0.05 0.05 ]

  # pose [0.0 0.0 0.3 90]
)

#
# Robot model:
# footprint (counter-clockwise): [-0.1,-0.125], [0.5,-0.125], [0.5,0.125], [-0.1,0.125]
# center of rotation: [0,0]
# wheelbase: 0.4

define forklift_robot position
(
  pose [ 0.0 0.0 0.0 0.0 ]

  odom_error [0.03 0.03 999999 999999 999999 0.02]

  block
  (
    points 12
    point[0] [1529 525]
    point[1] [683 525]
    point[2] [683 325]
    point[3] [-467 325]
    point[4] [-467 15]
    point[5] [683 15]
    point[6] [683 -15]
    point[7] [-467 -15]
    point[8] [-467 -325]
    point[9] [683 -325]
    point[10] [683 -525]
    point[11] [1529 -525]
    z [0 1]
  )

  size [ 1.0 0.5 0.1 ] # this models the footprint (rectangular), but shifted such that the bottom-left corner is in [0,0]. The center of rotation now here at [0.3, 0.125, 0.2]
  # correct center of rotation:
  origin [ 0.0 0.0 0.0 0.0]
  gui_nose 1
  color "red"

  # kinematics
  drive "car"
  wheelbase 1.311 # distance between both axles

  # spawn sensors
  laser(pose [ 0.0 0.0 0.0 0.0 ])
)
