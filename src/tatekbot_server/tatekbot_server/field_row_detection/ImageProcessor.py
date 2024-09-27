import rclpy
from rclpy.node import Node
import rclpy.qos
import rclpy.time
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import numpy as np
import message_filters

# Import necessary pre-built packages
import Camera as cam
import imageProc as imc
from featureMatching import featureMatching
import image_geometry

class CropRowImageProcessingNode(Node):
    def __init__(self):
        super().__init__('crop_row_image_processing')

        # Initialize CvBridge to convert ROS images to OpenCV format
        self.bridge = CvBridge()
        
        # Params

        self.scannerParams = {
            "scanSteps": 5,
            "scanEndPoint": 1280,
            "scanStartPoint": 0,
            "scanWindowWidth": 128
        }
        #  in case of using bigger size image size, we suggest to set ROI 
        self.roiParams = {
            "enable_roi": True,
            "p1": [0, 0],
            "p2": [350, 0],
            "p3": [10, 720],
            "p4": [0, 720],
            "p5": [830, 0],
            "p6": [1280, 0],
            "p7": [1280, 720],
            "p8": [1270, 720]
        }

        self.contourParams = {
            "imgResizeRatio": 100,
            "minContourArea": 0,
        }

        self.featureParams = {
            "linesToPass": 1,
            "minKeypointNum": 50,
            "maxMatchingDifference": 100,
            "minMatchingDifference": 0,
        }

        self.trackerParams = {
            "sacleRatio": 0.4,
            "topOffset": 0,
            "bottomOffset": 0,
            "trackingBoxWidth": 230,
        }


        
        sub_qos = rclpy.qos.QoSProfile(depth=1)

        self.frontColor_topic = '/intel_realsense_r200_rgb/image_raw'
        self.frontDepth_topic = '/intel_realsense_r200_depth/depth/image_raw'
        self.frontInfo_topic =  '/intel_realsense_r200_rgb/camera_info'

        self.frontColor_sub = message_filters.Subscriber(self, Image, self.frontColor_topic,  qos_profile=sub_qos)
        self.frontDepth_sub = message_filters.Subscriber(self,  Image, self.frontDepth_topic, qos_profile=sub_qos)
        self.frontCameraInfo_sub = message_filters.Subscriber(self, CameraInfo, self.frontInfo_topic,  qos_profile=sub_qos)
        
    
        
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.frontColor_sub, self.frontDepth_sub, self.frontCameraInfo_sub], queue_size=1, slop=0.5)
        self.ts.registerCallback(self.frontSyncCallback)
        
        self.create_subscription
        
        
        self.graphic_pub = self.create_publisher(Image,'vs_nav/graphic', 5)
        self.mask_pub = self.create_publisher(Image, 'vs_nav/mask', 5)
        self.exg_pub = self.create_publisher(Image, 'vs_nav/ExG', 5)

        self.navigationMode = 1

        self.primaryCamera = True

        self.frontImg = None
        self.frontDepth = None
        self.backImg = None
        self.backDepth = None




        
        self.camera = cam.Camera(1,1.2,0,1.151,1.27,0.96,0,0,1)
        self.imageProcessor = imc.imageProc(self.scannerParams,
                                            self.contourParams,
                                            self.roiParams,
                                            self.trackerParams)

        self.cameraModel = image_geometry.PinholeCameraModel()
        self.featureMatcher = featureMatching(self.featureParams)



    def start_image_proccess(self):
        self.camera = cam.Camera(1,1.2,0,1.151,1.27,0.96,0,0,1)
        self.imageProcessor = imc.imageProc(self.scannerParams,
                                            self.contourParams,
                                            self.roiParams,
                                            self.trackerParams)

        self.cameraModel = image_geometry.PinholeCameraModel()
        self.featureMatcher = featureMatching(self.featureParams)
        primaryRGB, primaryDepth = self.getProcessingImage(self.frontImg, self.frontDepth, self.backImg,self.backDepth)

        if not self.imageProcessor.findCropLane(primaryRGB, primaryDepth, mode='RGB-D'):
            # this is only False if the initialization in 'setImage' was unsuccessful
            print("The initialization was unsuccessful!! ")
            self.imageProcessor = imc.imageProc(self.scannerParams,
                                            self.contourParams,
                                            self.roiParams,
                                            self.trackerParams)
            return
        else:
           
            # if the robot is currently following a line and is not turning just compute the controls
            if self.isFollowingLane() :
                try:
                    self.imageProcessor.trackCropLane(self.navigationMode)
                
                except:
                    print("Can not track rows restarting...")
                    return

                
                                
                    

            else: 
                # test if the condition for the row switching is fulfilled
                foundNewCropLane = self.featureMatcher.detectNewCropLane(self.navigationMode,
                                                                  self.imageProcessor.primaryRGBImg,
                                                                  self.imageProcessor.greenIDX,
                                                                  self.imageProcessor.mask, 
                                                                  self.imageProcessor.rowTrackingBoxes,
                                                                  self.imageProcessor.numOfCropRows)
                

        self.publishImageTopics()

        print("Processed image has been published!!")
        
    def publishImageTopics(self):
        # Publish the Graphics image
        self.imageProcessor.drawGraphics()
        graphic_img = self.bridge.cv2_to_imgmsg(self.imageProcessor.graphicsImg, encoding='bgr8')
        self.graphic_pub.publish(graphic_img)
        # publish predicted Mask
        mask_msg = CvBridge().cv2_to_imgmsg(self.imageProcessor.mask, encoding="mono8")
        mask_msg.header.stamp = self.get_clock().now().to_msg()
        self.mask_pub.publish(mask_msg)
        # publish Exg image 
        exg_msg = CvBridge().cv2_to_imgmsg(self.imageProcessor.greenIDX, encoding="mono8")
        exg_msg.header.stamp = self.get_clock().now().to_msg()
        self.exg_pub.publish(exg_msg)
    
        

    def frontSyncCallback(self, rgbImage, depthImage, camera_info_msg):
        self.cameraModel.fromCameraInfo(camera_info_msg)
        try:
            # Convert your ROS Image message to OpenCV2
           
            self.frontImg = self.bridge.imgmsg_to_cv2(rgbImage, "bgr8")
            
            
        except CvBridgeError as e:
            print(e)
        try:
            # Convert your ROS Image message to OpenCV2
            # The depth image is a single-channel float32 image
            # the values is the distance in mm in z axis
            
            cv_depth = self.bridge.imgmsg_to_cv2(depthImage, "passthrough")
            # Convert the depth image to a Numpy array since most cv functions
            # require Numpy arrays.
            self.frontDepth = np.array(cv_depth, dtype=np.float32)
            # Normalize the depth image to fall between 0 (black) and 1 (white)
            cv.normalize(self.frontDepth, self.frontDepth,
                          0.0, 1.0, cv.NORM_MINMAX)
            

        except CvBridgeError as e:
            print(e)

        # get image size
        
        if self.frontImg is None:
            return
        self.imgHeight, self.imgWidth, self.imgCh = self.frontImg.shape
        # if the image is not empty
        if self.frontImg is not None :
            # compute and publish robot controls if the image is currently used
            print("Front image has beeen recieved")
            if self.primaryCamera:
                print("Starting to proccess image")
                self.start_image_proccess()
    
    def backSyncCallback(self, rgbImage, depthImage, camera_info_msg):
        self.cameraModel.fromCameraInfo(camera_info_msg)
        # print("here")
        try:
            # Convert your ROS Image message to OpenCV2
            self.backImg = self.bridge.imgmsg_to_cv2(rgbImage, "bgr8")
        except CvBridgeError as e:
            print(e)
        try:
            # Convert your ROS Image message to OpenCV2
            # The depth image is a single-channel float32 image
            # the values is the distance in mm in z axis
            cv_depth = self.bridge.imgmsg_to_cv2(depthImage, "passthrough")
            # Convert the depth image to a Numpy array since most cv functions
            # require Numpy arrays.
            self.backDepth = np.array(cv_depth, dtype=np.float32)
            # Normalize the depth image to fall between 0 (black) and 1 (white)
            cv.normalize(self.backDepth, self.backDepth,
                          0.0, 1.0, cv.NORM_MINMAX)
        except CvBridgeError as e:
            print(e)

        # get image size
        if self.backImg is None:
            return
        self.imgHeight, self.imgWidth, self.imgCh = self.backImg.shape
        # if the image is not empty
        if self.frontImg is not None and self.backImg is not None:
            # compute and publish robot controls if the image is currently used
            if not self.primaryCamera:
                self.start_image_proccess()

    def front_camera_callback(self, data):
        """Function to deal with the front image, called by the subscriber

        Args:
            data (_type_): _description_
        """
        # get and set new image from the ROS topic
        self.frontImg = self.bridge.imgmsg_to_cv2(data, desired_encoding='rgb8')
        # get image size
        self.imgHeight, self.imgWidth, self.imgCh = self.frontImg.shape
        # if the image is not empty
        if self.frontImg is not None and self.backImg is not None:
            # compute and publish robot controls if the image is currently used
            if self.primaryCamera:
                self.start_image_proccess()
                       
    def back_camera_callback(self, data):
        """Function to deal with the back image

        Args:
            data (_type_): _description_
        """
        # get and set new image from the ROS topic
        self.backImg = self.bridge.imgmsg_to_cv2(data, desired_encoding='rgb8')
        # get image size
        self.imgHeight, self.imgWidth, self.imgCh = self.backImg.shape
        # if the image is not empty
        if self.frontImg is not None and self.backImg is not None:
            # compute and publish robot controls if the image is currently used
            if not self.primaryCamera:
                self.start_image_proccess()




    
    def isFollowingLane(self):
        """if following a lane, modes 1, 4

        Returns:
            _type_: _description_
        """
        if self.navigationMode in [1,2,4,5]:
            return True
        else:
            return False

    def getProcessingImage(self, frontRgb, frontDepth, backRgb, backDepth):
        """Function to set the currently used image

        Args:
            frontImg (_type_): _description_
            backImg (_type_): _description_

        Returns:
            _type_: _description_
        """
        # The front image is used
        if self.primaryCamera:
            primaryRgb = frontRgb
            primaryDepth = frontDepth
        # The back image is used
        else:
            primaryRgb = backRgb
            primaryDepth = backDepth
        return primaryRgb, primaryDepth
    
    def getMainLineFeatures(self):
        try:
            return self.imageProcessor.P, self.imageProcessor.ang
        except:
            return None , None


    

        



def main(args=None):
    rclpy.init(args=args)
    crop_row_image_processing_node = CropRowImageProcessingNode()
    
    rclpy.spin(crop_row_image_processing_node)
    crop_row_image_processing_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()