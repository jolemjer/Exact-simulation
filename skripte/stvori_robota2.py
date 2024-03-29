# stavaramo robota, ono sto stvori stageros /base_pose_ground_truth

import numpy as np
import math
import rospy
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Point
from std_msgs.msg import Int8
import time
from tf_conversions import transformations


# hardkorirano za sliku 2.png
height = rospy.get_param('/height')
width  = rospy.get_param('/width')
step = rospy.get_param('/step')

# samo jedamput pokreni, probao sam s wait_for_message ali mi nije radilo
global run_once
run_once = 0


class robot():
    
    def __init__(self) -> None:
        rospy.init_node('robot2', anonymous=True)

        # kad se napravi array od mape onda pokreni objavu robota 
        self.odometry = Odometry()

        #vjerojatno je sve 0 na početku
        self.odometry.pose.pose.position.x = rospy.get_param('/position')[0] 
        self.odometry.pose.pose.position.y = rospy.get_param('/position')[1] 

        # orijentacija u quater
        self.orientation = 0 # nisam jos siguran s QUATERNION pa radim s kutovima RAD
        q = transformations.quaternion_from_euler(0, 0, self.orientation) # u radijanim, mijenjamo samo z-os
        self.odometry.pose.pose.orientation.x = q[0]
        self.odometry.pose.pose.orientation.y = q[1]
        self.odometry.pose.pose.orientation.z = q[2]
        self.odometry.pose.pose.orientation.w = q[3]

        # nariktaj header
        self.odometry.header.frame_id = 'map'
        self.odometry.header.stamp.nsecs = 20000000  # 50 Hz


# nesto mi ne stima, popravi da se robot vidi na slici

        self.pub1 = rospy.Publisher('robot_pose2', Odometry, queue_size=10)

        # bitno za radar
        self.kut = math.pi / 2 # u stupnjevima
        self.radijus = rospy.get_param('/radijus')  # u pikselima
        rospy.Subscriber('/all_grid', OccupancyGrid, callback=self.kut_grid)  # dobi grdi od robota
        velicina = np.arange(height* width, dtype=int) 
        self.map = np.full_like(velicina, -1, dtype=np.int8)

        # publisher za mapu od robota
        self.pub = rospy.Publisher('robot_grid2', OccupancyGrid, queue_size=10)
        self.occupancyGrid = OccupancyGrid()
        self.occupancyGrid.info.resolution = 1
        self.occupancyGrid.info.width = width
        self.occupancyGrid.info.height = height
        p = Point(0,0,0)
        self.occupancyGrid.info.origin.position = p

        # za key press
        rospy.Subscriber('key', Int8, callback=self.pomakni)

    def kut_grid(self, data):
        start = time.time()
        # glupi nacin, ali radi
        napravi_matricu(data)
# tu sam napravio gresku jer nisam imao gornju i donju vrijdnost 
# stavio sam da ide od pola doljnje i do pola gornje

        for kut in range(-math.floor(self.kut * 180 / math.pi / 2), math.floor(self.kut * 180 / math.pi / 2)):
            for korak in range(self.radijus):
                dx = math.cos(kut * math.pi / 180 + self.orientation) * korak
                dy = math.sin(kut * math.pi / 180 + self.orientation) * korak 
                x =  math.floor(dx + self.odometry.pose.pose.position.x)
                y = math.floor(dy + self.odometry.pose.pose.position.y)
                self.map[y*width+x] = matrica[y*width + x] 
                if matrica[y*width + x] == 100:
                    break

        self.occupancyGrid.data = self.map
        self.pub.publish(self.occupancyGrid)
        self.pub1.publish(self.odometry)
        print(time.time() - start)


    def dobi_novi_grid(self, data):
        start = time.time()
        for index, element in enumerate(list(matrica)):
            y = index / width
            x = index % width
            if abs(x-self.odometry.pose.pose.position.x) > self.radijus or abs(y-self.odometry.pose.pose.position.y) > self.radijus:
                continue
            if self.element_in_circle(x, y):

                self.map.put(index, element)
        self.occupancyGrid.data = self.map
        self.pub.publish(self.occupancyGrid)
        self.pub1.publish(self.odometry)
        print(time.time()- start)
    
    def element_in_circle(self, x, y):
        # tu cemo slati isto informaiciju dali je lijevo desno ....
        dx = x - self.odometry.pose.pose.position.x
        dy = y - self.odometry.pose.pose.position.y
        
        distance = dx ** 2 + dy ** 2
        if distance <= self.radijus**2:
            return True
        return False

    def pomakni(self, data):
        if data.data == 119:
            self.odometry.pose.pose.position.x += step * math.cos(self.orientation) 
            self.odometry.pose.pose.position.y += step * math.sin(self.orientation)
            return
        if data.data == 97:
            self.orientation += 0.1
            q = transformations.quaternion_from_euler(0, 0, self.orientation)
            self.odometry.pose.pose.orientation.x = q[0]
            self.odometry.pose.pose.orientation.y = q[1]
            self.odometry.pose.pose.orientation.z = q[2]
            self.odometry.pose.pose.orientation.w = q[3]
            return
        if data.data == 100:
            self.orientation -= 0.1
            q = transformations.quaternion_from_euler(0, 0, self.orientation)
            self.odometry.pose.pose.orientation.x = q[0]
            self.odometry.pose.pose.orientation.y = q[1]
            self.odometry.pose.pose.orientation.z = q[2]
            self.odometry.pose.pose.orientation.w = q[3]
            return 
        if data.data == 115:
            self.odometry.pose.pose.position.x -= step * math.cos(self.orientation) 
            self.odometry.pose.pose.position.y -= step * math.sin(self.orientation)
            return
        else:
            pass




velicina = np.arange(height* width, dtype=int)
matrica = np.full_like(velicina, -1, np.int8)
def napravi_matricu(data):
    global matrica
    matrica = data.data

if __name__ == '__main__':
    try:
        print('prosao')
        node = robot()
        while not rospy.is_shutdown():
          #  print(node.preprka)
            pass
    except rospy.ROSInterruptException:
        pass
