#  Coverts ROS sensor_msgs/Image data string
#  to `.ppm' string (rgb-8 encoding).
#
#  Jason Price
#  Version: 1.2


def RosImageToPPMString(SENSOR_IMAGE):
    IMAGE = SENSOR_IMAGE
    IM_CONV = ''
    IM_CONV = IM_CONV + 'P6\n' + \
        str(IMAGE.width) + ' ' + str(IMAGE.height) + '\n'
    IM_CONV = IM_CONV + '255\n'
    print 'sensor_msgs/Image to .ppm:\n [resolution: ' + \
        str(IMAGE.width) + 'x' + str(IMAGE.height) + ']'
    for y in xrange(IMAGE.height):
        for x in xrange(IMAGE.width):
            RED_IDX = int(y * int(IMAGE.step) + 3 * x)
            GREEN_IDX = RED_IDX + 1
            BLUE_IDX = RED_IDX + 2
            IM_CONV = IM_CONV + str(IMAGE.data[RED_IDX])
            IM_CONV = IM_CONV + str(IMAGE.data[GREEN_IDX])
            IM_CONV = IM_CONV + str(IMAGE.data[BLUE_IDX])
    print 'Conversion complete.'
    return IM_CONV

if __name__ == "__main__":
    pass
