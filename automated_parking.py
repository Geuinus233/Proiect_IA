import math
import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

class AutomatedParkingSystem:
    def __init__(self):
        print("Connecting to CoppeliaSim...", flush=True)
        self.client = RemoteAPIClient()
        self.sim = self.client.require('sim')
        print("Connected! Initializing variables...", flush=True)
        
        # Initialize variables
        self.t = 0
        self.t1 = 0
        self.t2 = 0
        self.t3 = 0
        self.L = 0
        self.v = [0, 0, 0] # Linear velocity
        self.a = 0
        self.b = 0
        
        self.d = 0.1416 # 2*d = distance between left and right wheels
        self.l = 0.4832 # l = distance between front and rear wheels
        
        print("Setting up handles...", flush=True)
        self.setup_handles()
        print("Init complete.", flush=True)
        
    def setup_handles(self):
        self.steeringLeft = self.sim.getObjectHandle('nakedCar_steeringLeft')
        self.steeringRight = self.sim.getObjectHandle('nakedCar_steeringRight')
        self.motorLeft = self.sim.getObjectHandle('nakedCar_motorLeft')
        self.motorRight = self.sim.getObjectHandle('nakedCar_motorRight')
        self.IrFl = self.sim.getObjectHandle('Proximity_sensor')
        self.IrRl = self.sim.getObjectHandle('Proximity_sensor0')
        self.front = self.sim.getObjectHandle('Proximity_sensor1')
        self.back = self.sim.getObjectHandle('Proximity_sensor2')
        self.IrFr = self.sim.getObjectHandle('Proximity_sensor3')
        self.IrRr = self.sim.getObjectHandle('Proximity_sensor4')
        self.midl = self.sim.getObjectHandle('Proximity_sensor5')
        self.midr = self.sim.getObjectHandle('Proximity_sensor6')
        
        # Backward compatibility for Distance handles
        if hasattr(self.sim, 'getDistanceHandle'):
            self.dist_handle = self.sim.getDistanceHandle('Distance1')
        else:
            self.dist_handle = self.sim.getObjectHandle('Distance1')
            
        self.wheel = self.sim.getObjectHandle('Cylinder4')

    def get_sensor(self, handle):
        # The API returns (res, distance, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector)
        result = self.sim.readProximitySensor(handle)
        res = result[0]
        dist = result[1] if res == 1 else None
        return res, dist

    def read_sensors(self):
        self.current_front, self.dist_fl = self.get_sensor(self.IrFl)
        self.current_rear, self.dist_rl = self.get_sensor(self.IrRl)
        self.current_frontR, self.dist_fr = self.get_sensor(self.IrFr)
        self.current_rearR, self.dist_rr = self.get_sensor(self.IrRr)
        self.current_midL, self.dist_ml = self.get_sensor(self.midl)
        self.current_midR, self.dist_mr = self.get_sensor(self.midr)
        
        self.ul_front, self.dist_ful = self.get_sensor(self.front)
        self.ul_back, self.dist_bul = self.get_sensor(self.back)

    def measure(self):
        while True:
            self.read_sensors()
            
            if self.current_front == 1 and self.current_rear == 1:
                if self.dist_fl is not None and self.dist_rl is not None and (self.dist_fl - self.dist_rl > 0.4):
                    self.t = self.sim.getSimulationTime()
            
                    while True:
                        self.read_sensors()
                        if self.dist_rl is not None and self.dist_fl is not None and (self.dist_rl - self.dist_fl > 0.4):
                            self.t1 = self.sim.getSimulationTime()
                            self.a = 1
                            break
            
            if self.a == 1:
                linearVel, angularVel = self.sim.getVelocity(self.wheel)
                self.v = linearVel
                self.length()
                return

    def length(self):
        # self.v[1] represents Y-axis velocity (Lua array indexing v[2] -> Python v[1])
        self.L = abs(self.v[1]) * (self.t1 - self.t) 
        self.a = 0
        
        while True: 
            self.read_sensors()
            
            if self.current_front == 1 and self.current_rear == 1:
                if self.L > 0.6:
                    if self.dist_rl is not None and self.dist_fl is not None and (self.dist_rl - self.dist_fl > 0.4):
                        self.t2 = self.sim.getSimulationTime()
                        
                        while True:
                            self.read_sensors()
                            if self.dist_fl is not None and self.dist_rl is not None and (self.dist_fl - self.dist_rl > 0.4):
                                self.t3 = self.sim.getSimulationTime()
                                self.b = 1
                                break
                else:
                    desiredWheelRotSpeed = 90 * math.pi / 180
                    self.sim.setJointTargetVelocity(self.motorLeft, desiredWheelRotSpeed)
                    self.sim.setJointTargetVelocity(self.motorRight, desiredWheelRotSpeed)
                    self.measure()
                    return
            
            if self.b == 1:
                linearVel, angularVel = self.sim.getVelocity(self.wheel)
                self.v = linearVel
                
                S = abs(self.v[1]) * (self.t3 - self.t2)
                print("Measured space (S):", S)
                
                if S > 0.55 and self.L > 0.9:
                    self.parallel()
                elif S > 0.45:
                    self.perpendicular()
                else: 
                    self.measure()
                return

    def parallel(self):
        count = 0
        self.b = 0
        self.read_sensors()
    
        if self.dist_fl is not None or self.dist_rl is not None:
            # Start Looping for Making A Turn Towards the Parking
            while True:
                self.read_sensors()
                
                desiredSteeringAngle = 43 * math.pi / 180
                try:
                    r = self.l / math.tan(desiredSteeringAngle)
                except ZeroDivisionError:
                    r = 9999.0
                    
                steeringAngleDx = 5 * math.pi / 180
                desiredWheelRotSpeed = -90 * math.pi / 180
                
                # Condition Used For Breaking The Loop And Enter the Next Loop
                if self.ul_back == 0:
                    desiredSteeringAngle += steeringAngleDx
                    if desiredSteeringAngle > 45 * math.pi / 180:
                        desiredSteeringAngle = 45 * math.pi / 180
                    
                    steeringAngleLeft = math.atan(self.l / (-self.d + r))
                    steeringAngleRight = math.atan(self.l / (self.d + r))
                    
                    self.sim.setJointTargetPosition(self.steeringLeft, steeringAngleLeft)
                    self.sim.setJointTargetPosition(self.steeringRight, steeringAngleRight)
                    self.sim.setJointTargetVelocity(self.motorLeft, desiredWheelRotSpeed)
                    self.sim.setJointTargetVelocity(self.motorRight, desiredWheelRotSpeed)
                else:
                    break
            
            # Start Looping For Opposite Turn
            while True:
                self.read_sensors()
                
                steeringAngleDx = 5 * math.pi / 180
                desiredWheelRotSpeed = -45 * math.pi / 180
                desiredSteeringAngle -= steeringAngleDx
                
                if desiredSteeringAngle < -45 * math.pi / 180:
                    desiredSteeringAngle = -45 * math.pi / 180
                    desiredWheelRotSpeed = -90 * math.pi / 180
                    self.sim.setJointTargetVelocity(self.motorLeft, desiredWheelRotSpeed)
                    self.sim.setJointTargetVelocity(self.motorRight, desiredWheelRotSpeed)
                
                # This Condition Is For Making The Car Straight Just After Entering The Parking Space
                if self.dist_fl is not None and self.dist_rl is not None:
                    if self.dist_fl < 0.35 and self.dist_rl < 0.35:
                        if -0.005 < (self.dist_rl - self.dist_fl) < 0.005:
                            desiredSteeringAngle = 0
                            
                            self.sim.setJointTargetPosition(self.steeringLeft, 0.0)
                            self.sim.setJointTargetPosition(self.steeringRight, 0.0)
                            
                            # Condition Used To Move The Car To The Center If It's Not
                            if self.dist_ful is not None and self.dist_bul is not None:
                                if self.dist_ful > self.dist_bul:
                                    desiredWheelRotSpeed = 10 * math.pi / 180
                                    self.sim.setJointTargetVelocity(self.motorLeft, desiredWheelRotSpeed)
                                    self.sim.setJointTargetVelocity(self.motorRight, desiredWheelRotSpeed)
                                elif self.dist_ful < self.dist_bul:
                                    desiredWheelRotSpeed = -10 * math.pi / 180
                                    self.sim.setJointTargetVelocity(self.motorLeft, desiredWheelRotSpeed)
                                    self.sim.setJointTargetVelocity(self.motorRight, desiredWheelRotSpeed)
                                elif self.dist_ful == self.dist_bul:
                                    desiredWheelRotSpeed = 0
                                    self.sim.setJointTargetVelocity(self.motorLeft, desiredWheelRotSpeed)
                                    self.sim.setJointTargetVelocity(self.motorRight, desiredWheelRotSpeed)
                                    self.sim.stopSimulation()
                                    return
                                    
                            simTime = self.sim.getSimulationTime()
                            if simTime > self.t + 70:
                                self.parallelremoval()
                                return
                
                if desiredSteeringAngle != 0:
                    r = self.l / math.tan(desiredSteeringAngle)
                    steeringAngleLeft = math.atan(self.l / (-self.d + r))
                    steeringAngleRight = math.atan(self.l / (self.d + r))
                else:
                    steeringAngleLeft = 0.0
                    steeringAngleRight = 0.0
            
                self.sim.setJointTargetPosition(self.steeringLeft, steeringAngleLeft)
                self.sim.setJointTargetPosition(self.steeringRight, steeringAngleRight)
                self.sim.setJointTargetVelocity(self.motorLeft, desiredWheelRotSpeed)
                self.sim.setJointTargetVelocity(self.motorRight, desiredWheelRotSpeed)
                
        else:
            self.measure()

    def perpendicular(self):
        self.read_sensors()
        self.b = 0
        if self.dist_fl is not None and self.dist_rl is not None:
            B = self.dist_fl - self.dist_rl
            print("Perpendicular parameter B:", B)
            
        count = 0
        if self.L > 0.6:
            # This Loop Is For Turning Of The Car Into The Parking Space
            while True:
                self.read_sensors()
                
                desiredSteeringAngle = 60 * math.pi / 180
                try:
                    r = self.l / math.tan(desiredSteeringAngle)
                    steeringAngleLeft = math.atan(self.l / (-self.d + r))
                    steeringAngleRight = math.atan(self.l / (self.d + r))
                except ZeroDivisionError:
                    steeringAngleLeft = 0.0
                    steeringAngleRight = 0.0
                    
                steeringAngleDx = 5 * math.pi / 180
                desiredWheelRotSpeed = -90 * math.pi / 180
                
                self.sim.setJointTargetPosition(self.steeringLeft, steeringAngleLeft)
                self.sim.setJointTargetPosition(self.steeringRight, steeringAngleRight)
                self.sim.setJointTargetVelocity(self.motorLeft, desiredWheelRotSpeed)
                self.sim.setJointTargetVelocity(self.motorRight, desiredWheelRotSpeed)
                
                # Condition Used To Make The Car Straight And Move Into The Parking Space
                if ((self.current_rear == 0) or (self.current_rearR == 1)) and (self.ul_back == 1):
                    desiredSteeringAngle = 0
                    print(self.dist_rl, self.dist_fl)
                    self.sim.setJointTargetPosition(self.steeringLeft, 0.0)
                    self.sim.setJointTargetPosition(self.steeringRight, 0.0)
                    break
                    
            # This Loop Is Used To Make The Car Move Straight Until The Condition Given Below
            while True:
                self.read_sensors()
                
                try:
                    r = self.l / math.tan(desiredSteeringAngle)
                    steeringAngleLeft = math.atan(self.l / (-self.d + r))
                    steeringAngleRight = math.atan(self.l / (self.d + r))
                except ZeroDivisionError:
                    steeringAngleLeft = 0.0
                    steeringAngleRight = 0.0
                    
                steeringAngleDx = 5 * math.pi / 180
                desiredWheelRotSpeed = -40 * math.pi / 180
                
                # This Condition Is Used To Stop The Car Leaving Some Space
                if self.dist_bul is not None and self.dist_bul < 0.125:
                    desiredWheelRotSpeed = 0
                    desiredSteeringAngle = 0
                    self.sim.setJointTargetVelocity(self.motorLeft, desiredWheelRotSpeed)
                    self.sim.setJointTargetVelocity(self.motorRight, desiredWheelRotSpeed)
                    count = 1
                    
                    simTime1 = self.sim.getSimulationTime()
                    if simTime1 > self.t + 70:
                        self.perpendicularremoval()
                        return
                        
                self.sim.setJointTargetVelocity(self.motorLeft, desiredWheelRotSpeed)
                self.sim.setJointTargetVelocity(self.motorRight, desiredWheelRotSpeed)
                self.sim.setJointTargetPosition(self.steeringLeft, steeringAngleLeft)
                self.sim.setJointTargetPosition(self.steeringRight, steeringAngleRight)
        else:
            self.measure()

    def perpendicularremoval(self):
        desiredWheelRotSpeed = -40 * math.pi / 180
        self.sim.setJointTargetVelocity(self.motorLeft, desiredWheelRotSpeed)
        self.sim.setJointTargetVelocity(self.motorRight, desiredWheelRotSpeed)

        while True:
            self.read_sensors()
            
            # Following Condition Is Used So That Car Starts Turning After That
            if self.current_front == 0 and self.current_midL == 0:
                while True:
                    self.read_sensors()
                    
                    desiredSteeringAngle = 45 * math.pi / 180
                    steeringAngleDx = 5 * math.pi / 180
                    desiredWheelRotSpeed = 60 * math.pi / 180
                    desiredSteeringAngle += steeringAngleDx
                    
                    if desiredSteeringAngle > 45 * math.pi / 180:
                        desiredSteeringAngle = 45 * math.pi / 180
                        
                    # Condition Used To Stop Turning And Go Into The Next Loop
                    if self.dist_rl is not None and self.dist_fl is not None and self.dist_ml is not None:
                        if self.dist_ml - self.dist_rl < 0.1 and self.dist_rl - self.dist_ml > 0:
                            break
                            
                    try:
                        r = self.l / math.tan(desiredSteeringAngle)
                        steeringAngleLeft = math.atan(self.l / (-self.d + r))
                        steeringAngleRight = math.atan(self.l / (self.d + r))
                    except ZeroDivisionError:
                        steeringAngleLeft = 0.0
                        steeringAngleRight = 0.0
                        
                    self.sim.setJointTargetPosition(self.steeringLeft, steeringAngleLeft)
                    self.sim.setJointTargetPosition(self.steeringRight, steeringAngleRight)
                    self.sim.setJointTargetVelocity(self.motorLeft, desiredWheelRotSpeed)
                    self.sim.setJointTargetVelocity(self.motorRight, desiredWheelRotSpeed)
                    
                # In This Loop The Car Stops Turning, Moves Straight And Again Goes To Measure Function
                while True:
                    self.read_sensors()
                    if self.dist_rl is not None and self.dist_fl is not None:
                        print(self.dist_rl - self.dist_fl)
                        
                    desiredSteeringAngle = 0
                    steeringAngleDx = 5 * math.pi / 180
                    desiredWheelRotSpeed = 60 * math.pi / 180
                    desiredSteeringAngle += steeringAngleDx
                    
                    if desiredSteeringAngle < -45 * math.pi / 180:
                        desiredSteeringAngle = -45 * math.pi / 180
                        
                    try:
                        r = self.l / math.tan(desiredSteeringAngle)
                        steeringAngleLeft = math.atan(self.l / (-self.d + r))
                        steeringAngleRight = math.atan(self.l / (self.d + r))
                    except ZeroDivisionError:
                        steeringAngleLeft = 0.0
                        steeringAngleRight = 0.0
                        
                    self.sim.setJointTargetPosition(self.steeringLeft, steeringAngleLeft)
                    self.sim.setJointTargetPosition(self.steeringRight, steeringAngleRight)
                    self.sim.setJointTargetVelocity(self.motorLeft, desiredWheelRotSpeed)
                    self.sim.setJointTargetVelocity(self.motorRight, desiredWheelRotSpeed)
                    self.measure()
                    return
            else:
                desiredWheelRotSpeed = 60 * math.pi / 180
                self.sim.setJointTargetVelocity(self.motorLeft, desiredWheelRotSpeed)
                self.sim.setJointTargetVelocity(self.motorRight, desiredWheelRotSpeed)

    def parallelremoval(self):
        desiredWheelRotSpeed = -40 * math.pi / 180
        self.sim.setJointTargetVelocity(self.motorLeft, desiredWheelRotSpeed)
        self.sim.setJointTargetVelocity(self.motorRight, desiredWheelRotSpeed)
        
        while True:
            self.read_sensors()
            
            # Following Condition Is Used So That Car Starts Moving In Backward Direction To Gain Some Space For Turning
            if self.dist_bul is not None and self.dist_bul < 0.08:
                # When The Condition Gets Satisfied The Looping Takes Place Where Car Starts Turning
                while True:
                    self.read_sensors()
                    
                    desiredSteeringAngle = -50 * math.pi / 180
                    steeringAngleDx = 5 * math.pi / 180
                    desiredWheelRotSpeed = 90 * math.pi / 180
                    desiredSteeringAngle += steeringAngleDx
                    
                    if desiredSteeringAngle > 45 * math.pi / 180:
                        desiredSteeringAngle = 45 * math.pi / 180
                        
                    # This is The Condition Where It Breaks The Loop And Enters Other Loop For Opposite Turning
                    if self.current_front == 0:
                        break
                        
                    try:
                        r = self.l / math.tan(desiredSteeringAngle)
                        steeringAngleLeft = math.atan(self.l / (-self.d + r))
                        steeringAngleRight = math.atan(self.l / (self.d + r))
                    except ZeroDivisionError:
                        steeringAngleLeft = 0.0
                        steeringAngleRight = 0.0
                        
                    self.sim.setJointTargetPosition(self.steeringLeft, steeringAngleLeft)
                    self.sim.setJointTargetPosition(self.steeringRight, steeringAngleRight)
                    self.sim.setJointTargetVelocity(self.motorLeft, desiredWheelRotSpeed)
                    self.sim.setJointTargetVelocity(self.motorRight, desiredWheelRotSpeed)
                    
                # In This Loop The Car Reverses It's Steering To Move Out
                while True:
                    self.read_sensors()
                    
                    desiredSteeringAngle = 50 * math.pi / 180
                    steeringAngleDx = 5 * math.pi / 180
                    desiredWheelRotSpeed = 60 * math.pi / 180
                    desiredSteeringAngle += steeringAngleDx
                    
                    if desiredSteeringAngle > 45 * math.pi / 180:
                        desiredSteeringAngle = 45 * math.pi / 180
                        
                    if self.dist_rl is not None and self.dist_fl is not None and self.dist_ml is not None:
                        if self.dist_rl - self.dist_fl > 0.57:
                            break
                            
                    try:
                        r = self.l / math.tan(desiredSteeringAngle)
                        steeringAngleLeft = math.atan(self.l / (-self.d + r))
                        steeringAngleRight = math.atan(self.l / (self.d + r))
                    except ZeroDivisionError:
                        steeringAngleLeft = 0.0
                        steeringAngleRight = 0.0
                        
                    self.sim.setJointTargetPosition(self.steeringLeft, steeringAngleLeft)
                    self.sim.setJointTargetPosition(self.steeringRight, steeringAngleRight)
                    self.sim.setJointTargetVelocity(self.motorLeft, desiredWheelRotSpeed)
                    self.sim.setJointTargetVelocity(self.motorRight, desiredWheelRotSpeed)
                    
                # This Is The Loop Where The Car Starts Moving In Straight Direction
                while True:
                    self.read_sensors()
                    if self.dist_rl is not None and self.dist_fl is not None:
                        print(self.dist_rl - self.dist_fl)
                        
                    desiredSteeringAngle = 0
                    steeringAngleDx = 5 * math.pi / 180
                    desiredWheelRotSpeed = 60 * math.pi / 180
                    desiredSteeringAngle += steeringAngleDx
                    
                    if desiredSteeringAngle < -45 * math.pi / 180:
                        desiredSteeringAngle = -45 * math.pi / 180
                        
                    try:
                        r = self.l / math.tan(desiredSteeringAngle)
                        steeringAngleLeft = math.atan(self.l / (-self.d + r))
                        steeringAngleRight = math.atan(self.l / (self.d + r))
                    except ZeroDivisionError:
                        steeringAngleLeft = 0.0
                        steeringAngleRight = 0.0
                        
                    self.sim.setJointTargetPosition(self.steeringLeft, steeringAngleLeft)
                    self.sim.setJointTargetPosition(self.steeringRight, steeringAngleRight)
                    self.sim.setJointTargetVelocity(self.motorLeft, desiredWheelRotSpeed)
                    self.sim.setJointTargetVelocity(self.motorRight, desiredWheelRotSpeed)
                    self.measure()
                    return
            else:
                desiredWheelRotSpeed = -40 * math.pi / 180
                self.sim.setJointTargetVelocity(self.motorLeft, desiredWheelRotSpeed)
                self.sim.setJointTargetVelocity(self.motorRight, desiredWheelRotSpeed)

    def run(self):
        print("Pornire simulare...")
        self.sim.startSimulation()
        
        desiredWheelRotSpeed = 90 * math.pi / 180
        self.sim.setJointTargetVelocity(self.motorLeft, desiredWheelRotSpeed)
        self.sim.setJointTargetVelocity(self.motorRight, desiredWheelRotSpeed)
        print(f"Am setat viteza motoarelor la {desiredWheelRotSpeed} rad/s")
        
        while True:
            self.read_sensors()
            
            # Condition Used For Measuring Time And Velocity And Then Calling "Length" Function
            if self.current_front == 1 and self.current_rear == 1:
                if self.dist_fl is not None and self.dist_rl is not None and (self.dist_fl - self.dist_rl > 0.4):
                    self.t = self.sim.getSimulationTime()
                    print(f"Conditie de masurare indeplinita la t={self.t}")
                    
                    while True:
                        self.read_sensors()
                        if self.dist_rl is not None and self.dist_fl is not None and (self.dist_rl - self.dist_fl > 0.4):
                            self.t1 = self.sim.getSimulationTime()
                            self.a = 1
                            break

            if self.a == 1:
                linearVel, angularVel = self.sim.getVelocity(self.wheel)
                self.v = linearVel
                self.length()
                
            # Yield slightly to avoid burning 100% CPU when not in an active maneuver loop
            time.sleep(0.01)

if __name__ == '__main__':
    system = AutomatedParkingSystem()
    print("Starting Automated Parking System...")
    system.run()
    
