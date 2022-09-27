import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
import numpy as np

height = rospy.get_param('/height')
width  = rospy.get_param('/width')


def callback_metoda1(data):
    karta_prostora1.data = data.data

def callback_metoda2(data):
    karta_prostora2.data = data.data

karta_prostora1 = OccupancyGrid()
karta_prostora1.info.resolution = 1
karta_prostora1.info.width = width
karta_prostora1.info.height = height
karta_prostora1.info.origin.position = Point(0, 0, 0)
rospy.Subscriber('/robot_grid', OccupancyGrid, callback=callback_metoda1)

karta_prostora2 = OccupancyGrid()
karta_prostora2.info.resolution = 1
karta_prostora2.info.width = width
karta_prostora2.info.height = height
karta_prostora2.info.origin.position = Point(0, 0, 0)
rospy.Subscriber('/robot_grid2', OccupancyGrid, callback=callback_metoda2)

karta_prostora = OccupancyGrid()
karta_prostora.info.resolution = 1
karta_prostora.info.width = width
karta_prostora.info.height = height
karta_prostora.info.origin.position = Point(0, 0, 0)


pub = rospy.Publisher('/karta_prostora', OccupancyGrid, queue_size=10)

velicina = np.arange(height* width, dtype=int)

rospy.init_node('spoji_karte', anonymous=True)

while not rospy.is_shutdown():
    matrica = np.full_like(velicina, -1, np.int8)
    for i, (e1, e2) in enumerate(zip(karta_prostora1.data, karta_prostora2.data)):
        if e1 == 0:
            matrica[i] = 0
            continue
        if e2 == 0:
            matrica[i] = 0
            continue
        if e1 == 100:
            matrica[i] = 100
            continue
        if e2 == 100:
            matrica[i] = 100
            continue
        matrica[i] = -1
    karta_prostora.data = matrica
    pub.publish(karta_prostora)
    

