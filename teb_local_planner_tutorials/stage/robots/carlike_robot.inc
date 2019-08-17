define laser ranger
(
  sensor
  (
    range_max 999
    fov 270.0 # the angular field of view of the scanner, in degrees.
    samples 640
  )
  # generic model properties
  color "black"
  size [ 0.06 0.15 0.03 ]
)

#
# Robot model:
# footprint (counter-clockwise): [-0.1,-0.125], [0.5,-0.125], [0.5,0.125], [-0.1,0.125]
# center of rotation: [0,0]
# wheelbase: 0.58

define carlike_robot position
(
  pose [ 0.0 0.0 0.0 0.0 ]

  odom_error [0.03 0.03 999999 999999 999999 0.02]

  size [ 0.85 0.54 0.40 ] # this models the footprint (rectangular), but shifted such that the bottom-left corner is in [0,0]. The center of rotation now here at [0.425, 0.27, 0.2]
  # correct center of rotation:
  origin [ 0.0 0.0 0.0 0.0]
  gui_nose 1
  color "red"

  # kinematics
  drive "car"
  wheelbase 0.58 # distance between both axles
  
  # spawn sensors
  laser(pose [ 0.3 0.0 0.0 0.0 ])
)
