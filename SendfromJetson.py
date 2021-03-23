from vidgear.gears import VideoGear
from vidgear.gears import NetGear
import pyrealsense2 as rs
import numpy as np
import cv2
import imagiz
import rospy
from geometry_msgs.msg import Point

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 5)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 5)

# Start streaming
pipeline.start(config)


#vidgear part
options = {"flag": 0, "copy": False, "track": False}

server = NetGear(
    address="192.168.1.213",
    port="3456",
    protocol="tcp",
    pattern=0,
    logging=True,
    **options
)

#imagiz part
client=imagiz.Client("cc1",server_ip="192.168.1.213")
encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]

rospy.init_node('hypotenuse_length')
pub = rospy.Publisher('hypotenuse', Point, queue_size=10)
def callback(data):
    x = int(data.x)
    y = int(data.y)
    theta = int(data.z) 
    hypotenuse = round(depth_frame.get_distance(x, y), 1)
    point2 = Point()
    point2.x = hypotenuse
    point2.y = theta
    point2.z = 0
    pub.publish(point2)

rospy.Subscriber("coordinates", Point, callback)

# infinite loop until [Ctrl+C] is pressed
while True:
    try:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        r, image = cv2.imencode('.png', color_image, encode_param)
        frame = depth_colormap
        #frame2 = color_image
        # read frames

        # check if frame is None
        if frame is None:
            #if True break the infinite loop
            break

        # do something with frame here

        # send frame to server
        server.send(frame)
        client.send(image)

    except KeyboardInterrupt:
        #break the infinite loop
        break

# safely close video stream
stream.stop()
# safely close server
writer.close()
