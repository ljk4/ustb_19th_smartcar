#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from qr_msgs.srv import qr, qrResponse
from pyzbar.pyzbar import decode
from std_msgs.msg import Int8

class QRController:
    def __init__(self):
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber("/cam", Image, self.img_callback)
        self.srv = rospy.Service("/qr/scan_qr", qr, self.serve)
        self.img = None
        
        # Mapping from QR content string to ID
        self.QR_MAPPING = {
            "fruit;dessert": 1,
            "fruit;vegetable": 2,
            "vegetable;dessert": 3,
            "A:dessert, B:fruit, C:vegetable": 10,
            "A:dessert, B:vegetable, C:fruit": 11,
            "A:fruit, B:dessert, C:vegetable": 12,
            "A:fruit, B:vegetable, C:dessert": 13,
            "A:vegetable, B:dessert, C:fruit": 14,
            "A:vegetable, B:fruit, C:dessert": 15,
            "A:dessert, B:fruit, D:vegetable": 16,
            "A:dessert, B:vegetable, D:fruit": 17,
            "A:fruit, B:dessert, D:vegetable": 18,
            "A:fruit, B:vegetable, D:dessert": 19,
            "A:vegetable, B:dessert, D:fruit": 20,
            "A:vegetable, B:fruit, D:dessert": 21,
            "A:dessert, C:fruit, D:vegetable": 22,
            "A:dessert, C:vegetable, D:fruit": 23,
            "A:fruit, C:dessert, D:vegetable": 24,
            "A:fruit, C:vegetable, D:dessert": 25,
            "A:vegetable, C:dessert, D:fruit": 26,
            "A:vegetable, C:fruit, D:dessert": 27,
            "B:dessert, C:fruit, D:vegetable": 28,
            "B:dessert, C:vegetable, D:fruit": 29,
            "B:fruit, C:dessert, D:vegetable": 30,
            "B:fruit, C:vegetable, D:dessert": 31,
            "B:vegetable, C:dessert, D:fruit": 32,
            "B:vegetable, C:fruit, D:dessert": 33,
        }

    def img_callback(self, img_msg):
        try:
            self.img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except Exception as e:
            rospy.logerr(e)

    def serve(self, req):
        if req.start_calling:
            if self.img is None:
                response = qrResponse()
                response.success = False
                response.id = Int8()
                response.id.data = -1
                return response

            decoded_objects = decode(self.img)
            
            for obj in decoded_objects:
                data = obj.data.decode("utf-8")
                rospy.loginfo("QR Content: " + data)
                
                # Check exact match
                if data in self.QR_MAPPING:
                    response = qrResponse()
                    response.success = True
                    response.id = Int8()
                    response.id.data = self.QR_MAPPING[data]
                    return response
                
                # Try stripping whitespace
                data_stripped = data.strip()
                if data_stripped in self.QR_MAPPING:
                    response = qrResponse()
                    response.success = True
                    response.id = Int8()
                    response.id.data = self.QR_MAPPING[data_stripped]
                    return response

            # If no valid QR found
            response = qrResponse()
            response.success = False
            response.id = Int8()
            response.id.data = -1
            return response
        
        response = qrResponse()
        response.success = False
        response.id = Int8()
        response.id.data = -1
        return response

if __name__ == '__main__':
    rospy.init_node("qr_node")
    rospy.loginfo("qr node started")
    
    node = QRController()
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        rospy.spin()
