#!/usr/bin/env python
import rospy
import rosbag
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

print("Starting")
x_coord = list()
y_coord = list()
'''
img = mpimg.imread('/home/siddharth/SB#337.bag')
#print(img)

imgplot = plt.imshow(img)
#plt.show()
#print(imgplot) 
'''

bag = rosbag.Bag('/home/siddharth/catkin_ws/src/sahayak_bot/ebot_nav/bag_files/SB#337.bag') 
#print bag
print("Read file successfully")
for topic, msg, t in bag.read_messages(topics=['/odom']):
    x_coord.append(msg.pose.pose.position.x)
    y_coord.append(msg.pose.pose.position.y)
    #print("Hi")
bag.close()
x = x_coord
y = y_coord
fig, ax = plt.subplots()
ax.plot(x, y)
plt.show()



