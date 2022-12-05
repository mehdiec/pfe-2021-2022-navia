from sys import maxsize
import olympe
import cv2
from multiprocessing import Lock
from olympe.messages.ardrone3.Piloting import TakeOff, Landing, PCMD, moveBy
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged, SpeedChanged, AttitudeChanged
from olympe.messages.ardrone3.PilotingSettings import MaxTilt
from olympe.messages.ardrone3.SpeedSettings import MaxVerticalSpeed
from olympe.messages.ardrone3.SpeedSettings import MaxRotationSpeed

from numpy import pi

class Anafi:
    def __init__(self, frame_cb, max_tilt,max_rotation_speed,max_vertical_speed,ip='192.168.42.1'):

        ## set Ip,and frame
        self.DRONE_IP      = ip
        self.drone = olympe.Drone(self.DRONE_IP)
        self.frame_cb = frame_cb
        self.max_tilt = max_tilt
        self.max_vertical_speed = max_vertical_speed
        self.max_rotation_speed = max_rotation_speed
        ## set maxtilt in radian
        self.set_max_tilt()
        self.set_max_vertical_speed()
        self.set_max_rotation_speed()

        self.measured_speed    = None
        self.measured_attitude = None

        self.frame_mutex = Lock()
        self.img = None
        self.width = None
        self.height = None,
        self.cv2_cvt_color_flag = None

        self.stamp = 0


    def __str__(self):
        if self.measured_speed is None:
            return '???'
        else:
            return '[vx = {}, vy = {}, vz = {}, roll = {}, pitch = {}, yaw = {}]'.format(self.measured_speed['speedX'],
                                                                                         self.measured_speed['speedY'],
                                                                                         self.measured_speed['speedZ'],
                                                                                         self.measured_attitude['roll'],
                                                                                         self.measured_attitude['pitch'],
                                                                                         self.measured_attitude['yaw'])
                                                                                         
        
    def connect(self):
        self.drone.connect()
        self.drone.streaming.set_callbacks(raw_cb=self.on_yuv)
        self.drone.streaming.start()
        
    def disconnect(self):
        self.drone.streaming.stop()
        self.drone.disconnect()

    def takeoff(self):
        assert self.drone(
    	    TakeOff()
    	    >> FlyingStateChanged(state="hovering", _timeout=5)).wait().success()
        
    def land(self):
        assert self.drone(Landing()).wait().success()

    def hover(self):
        assert self.drone(FlyingStateChanged(state="hovering", _timeout=5)).wait().success()
        
    def get_image(self):

        return self.img

    def get_speed(self):
        return self.drone.get_state(SpeedChanged)

    def get_attitude(self):
        return self.drone.get_state(AttitudeChanged)

    def set_max_tilt(self):
        ## maxtilt is converted into degrees
        assert self.drone(MaxTilt(self.max_tilt*180/pi)).wait().success()

    def set_max_vertical_speed(self):
        assert self.drone(MaxVerticalSpeed(self.max_vertical_speed)).wait().success()
    
    def set_max_rotation_speed(self):
        ## radian/s coverted into degree/s
        assert self.drone(MaxRotationSpeed(self.max_rotation_speed*180/pi)).wait().success()

    def move(self,roll,pitch,yaw,gaz,flag = 1):

        '''
        roll, pitch are in radian
        yaw is in radian/s  ( can be negative )

        '''
        

        assert self.drone(PCMD(flag = 1, roll =roll*100/self.max_tilt, pitch = pitch*100/self.max_tilt,
                         yaw = yaw*100/self.max_rotation_speed, gaz= gaz, timestampAndSeqNum = self.stamp))

    def set_pitch(self,pitch): 
        '''
        pitch must be in radian
        if pitch is higher than maxtilt it is set to maxtilt

        '''
        if pitch > self.max_tilt:
            pitch = self.max_tilt
        
        roll,_,yaw,_,_ = self.get_attitude()

        ## -100 is equivalent to 0m/s
        gaz = self.get_speed()[2] /self.max_vertical_speed*200 - 100

        ### issue yaw is in radian ???
        self.move(roll,pitch,yaw,gaz)

        

    def set_roll(self,roll):

        '''
        roll must be in radian
        if roll is higher than maxtilt it is set to maxtilt
        
        '''
        if roll > self.max_tilt:
            roll = self.max_tilt
        
        _,pitch,yaw,_,_ = self.get_attitude()

        ## -100 is equivalent to 0m/s
        gaz = self.get_speed()[2] /self.max_vertical_speed*200 - 100

        self.move(roll,pitch,yaw,gaz)

    def set_yaw(self,yaw):
        '''
        yaw must be in radian/s
        if yaw is higher than max_rotation_speed it is set to max_rotation_speed
        
        '''
        if yaw > self.max_rotation_speed:
            yaw = self.max_rotation_speed
        
        roll,pitch,_,_,_ = self.get_attitude()

        ## -100 is equivalent to 0m/s
        gaz = self.get_speed()[2] /self.max_vertical_speed*200 - 100

        self.move(roll,pitch,yaw,gaz)

    def set_gaz(self,gaz):
        '''
        gaz must be in m/s
        
        '''
        if gaz > self.max_vertical_speed:
            yaw = self.max_vertical_speed
        
        ## -100 is equivalent to 0m/s
        gaz = gaz /self.max_vertical_speed*200 - 100

        roll,pitch,yaw,_,_ = self.get_attitude()
        self.move(roll,pitch,yaw,gaz)


    def on_yuv(self, yuv_frame):
        yuv_frame.ref()
        if self.width is None:
            info = yuv_frame.info()
            self.height, self.width = ( 
                info["raw"]["frame"]["info"]["height"],
                info["raw"]["frame"]["info"]["width"],
            )
            self.cv2_cvt_color_flag = {
                olympe.VDEF_I420: cv2.COLOR_YUV2BGR_I420,
                olympe.VDEF_NV12: cv2.COLOR_YUV2BGR_NV12,
            }[yuv_frame.format()]
        with self.frame_mutex:
            self.img = cv2.cvtColor(yuv_frame.as_ndarray(), self.cv2_cvt_color_flag)
        yuv_frame.unref()
        
