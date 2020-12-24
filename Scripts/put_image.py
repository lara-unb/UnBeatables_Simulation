# import naoqi
import vrep
import time

import numpy as np
import cv2
import array

if __name__ == "__main__":

    # #nao setup
    # naoIP = "localhost"
    # naoPort = 9559
    # videoProxy = naoqi.ALProxy("ALVideoDevice", naoIP, naoPort)

    #vrep setup
    vrep.simxFinish(-1)
    clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
    if (clientID != -1):


        #Top camera

        #Get the handle of the vision sensor
        res1, top_camera_handle = vrep.simxGetObjectHandle(
            clientID, "NAO_vision1", vrep.simx_opmode_oneshot_wait)
        #Get the image
        res2, resolution, image = vrep.simxGetVisionSensorImage(
            clientID, top_camera_handle, 0, vrep.simx_opmode_streaming)


        #bottom camera

        #Get the handle of the vision sensor
        res3, bottom_camera_handle = vrep.simxGetObjectHandle(
            clientID, "NAO_vision2", vrep.simx_opmode_oneshot_wait)
        #Get the image
        res4, resolution, image = vrep.simxGetVisionSensorImage(
            clientID, bottom_camera_handle, 0, vrep.simx_opmode_streaming)


        time.sleep(1)
        while (vrep.simxGetConnectionId(clientID) != -1):
            time.sleep(1 / 2)  # fps setter

            #Get the image of the vision sensor
            top_res, top_resolution, top_image = vrep.simxGetVisionSensorImage(
                clientID, top_camera_handle, 0, vrep.simx_opmode_buffer)
            top_image = np.array(top_image,dtype=np.uint8)
            top_image.resize([top_resolution[1],top_resolution[0],3])
            top_image = np.flip(top_image, axis=0)

            bot_res, bot_resolution, bot_image = vrep.simxGetVisionSensorImage(
                clientID, bottom_camera_handle, 0, vrep.simx_opmode_buffer)
            bot_image = np.array(bot_image,dtype=np.uint8)
            bot_image.resize([bot_resolution[1],bot_resolution[0],3])
            bot_image = np.flip(bot_image, axis=0)


            cv2.imshow('image',np.hstack((top_image, bot_image)))
            cv2.waitKey(0)
            cv2.destroyAllWindows()


            

            # videoProxy.putImage(0, 640, 480, top_image)
            # videoProxy.putImage(1, 640, 480, bot_image)
    else:
        print "Client id failed"