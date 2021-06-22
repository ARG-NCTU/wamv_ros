#!/usr/bin/env python 
import rospy 
import matplotlib.pyplot as plt

from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

class PLOTWHISTLEDATA(object):
    def __init__(self):
        # Initulize
        self.plot_y_max = 10000
        self.plot_y_min = 1000
        self.get_F = False 
        self.get_T = False
        self.get_data = False

        #On Start up 
        self.OnStartUp()

        #Iterate Function
        rospy.Timer(rospy.Duration(0.5), self.Iterate)
        
        #Publisher
        #self.pub_bar = rospy.Publisher("FOO", Float64, queue_size=1)

        #Subscriber
        self.sub_plot_data = rospy.Subscriber("whistle_data", numpy_msg(Floats), self.callBack_data, queue_size=1)
        self.sub_plot_f = rospy.Subscriber("whistle_f", numpy_msg(Floats), self.callBack_f, queue_size=1)
        self.sub_plot_t = rospy.Subscriber("whistle_t", numpy_msg(Floats), self.callBack_t, queue_size=1)

    def OnStartUp(self):
        #Set the params at yaml file
        if rospy.has_param('~y_max'):
            self.plot_y_max = rospy.get_param('~y_max')
        if rospy.has_param('~y_min'):
            self.plot_y_min = rospy.get_param('~y_min')

    def Iterate(self, event):
        #do ur thing here
        #self.pub_bar.publish(data)
        if(self.get_data == True and self.get_F == True and self.get_T == True):
            self.plotInfo(self.whistle_data, self.whistle_f, self.whistle_t, self.plot_y_min, self.plot_y_max)

    def callBack_data(self, msg):
        self.whistle_data = msg.data
        self.get_data = True

    def callBack_t(self, msg):
        self.whistle_t = msg.data 
        self.get_T = True

    def callBack_f(self, msg):
        self.whistle_f = msg.data
        self.get_F = True

    def plotInfo(self, input_data, f, t, y_min=1000, y_max=10000, plot_pause=1e-10):
	plt.clf()
        plt.pcolormesh(t, f,input_data.reshape(len(f), len(t)))
        plt.draw()
        plt.ylim(y_min, y_max)
        plt.pause(plot_pause)

    def onShutdown(self):
        rospy.loginfo("[%s] Shutdown." %(self.node_name))

if __name__ == '__main__':
    rospy.init_node('plotWhistleData', anonymous=False)
    plot = PLOTWHISTLEDATA()
    rospy.on_shutdown(plot.onShutdown)
    rospy.spin()
