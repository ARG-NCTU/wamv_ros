#!/usr/bin/env python 
import rospy 
import os
import time
import numpy as np

from std_msgs.msg import Float32
from robotx_msgs.msg import RealTimeData

class REALDATASAVE(object):
    def __init__(self):
 
    # Initulize
        self.get_F = False 
        self.get_T = False
        self.get_data_c1 = False
        self.get_data_c2 = False
        
        self.folder_time = "./"+time.strftime("%Y_%m_%d_%H", time.localtime())
        self.folder_name = self.createFolder(self.folder_time)

        #On Start up 
        #self.OnStartUp()

        #Iterate Function
        rospy.Timer(rospy.Duration(1), self.Iterate)
        
        #Publisher
        #self.pub_bar = rospy.Publisher("FOO", Float64, queue_size=1)

        #Subscriber
        self.sub_test      = rospy.Subscriber("acoustic_real_data", RealTimeData, self.callBack, queue_size=10)

    def OnStartUp(self):
        tmp = 0

    def Iterate(self, event):
        tmp = 0

    def createFolder(self, folder_name):
        try:
            if not os.path.exists(folder_name):
                os.makedirs(folder_name)
                print("Create "+folder_name+" folder ")
        except OSError:
            print ('Error: Creating directory. ' +  folder_name)

        return folder_name

    def callBack(self, msg):
        print("Get real data.")
        self.saveData(self.folder_name, msg.data_ch1, msg.data_ch2, msg.fs, msg.angle)

    def saveData(self, folder, input_data_c1, input_data_c2, fs, angle):
        fileName_c1 = folder+"/c1_"+str(fs)+"_"+str(angle)+".csv"
        fileName_c2 = folder+"/c2_"+str(fs)+"_"+str(angle)+".csv"
        np.savetxt(fileName_c1, np.asarray(input_data_c1, dtype=np.float64), fmt='%1.13f', delimiter=',')
        print("Save "+fileName_c1+" file. ")
        np.savetxt(fileName_c2, np.asarray(input_data_c2, dtype=np.float64), fmt='%1.13f', delimiter=',')
        print("Save "+fileName_c2+" file. ")
        print("-------------------------------------")

    def onShutdown(self):
        rospy.loginfo("[%s] Shutdown." %(self.node_name))

if __name__ == '__main__':
    rospy.init_node('realDataSave', anonymous=False)
    realDataSave = REALDATASAVE()
    rospy.on_shutdown(realDataSave.onShutdown)
    rospy.spin()
