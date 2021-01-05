import b0RemoteApi
import cv2
import numpy as np


def test_function(msg):
    print(msg)


class VrepImage():
    def __init__(self, client, name="NAO_top_camera"):
        _, self.camera_handle = client.simxGetObjectHandle(
            name, client.simxServiceCall())
        self.client = client

    def get_camera_image(self):
        msg = self.client.simxGetVisionSensorImage( self.camera_handle, False, client.simxServiceCall())
        self.result = msg[0]
        self.resolution = msg[1]
        self._camera_image_raw = msg[2]
        imgarr = np.fromstring(self._camera_image_raw,
                               np.uint8).reshape(self.resolution[1],
                                                 self.resolution[0], 3)
        imgarr = np.flipud(imgarr)  #inverted image
        imgarr = cv2.cvtColor(imgarr, cv2.COLOR_RGB2BGR)
        return imgarr


if __name__ == "__main__":
    # Define the codec and create VideoWriter object.The output is stored in 'outpy.avi' file.
    out_top = cv2.VideoWriter('top_camera.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 20, (640,480))
    out_bottom = cv2.VideoWriter('bottom_camera.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 20, (640,480))

    client = b0RemoteApi.RemoteApiClient('b0RemoteApi_NAO_Node', 'b0RemoteApiNAO',
                                     60, setupSubscribersAsynchronously=True)
    top_vrep_image = VrepImage(client)
    bottom_vrep_image = VrepImage(client, "NAO_bottom_camera")
    while (True):
        top_frame = top_vrep_image.get_camera_image()
        bottom_frame = bottom_vrep_image.get_camera_image()
        # Write the frame into the file 'output.avi'
        out_top.write(top_frame)
        out_bottom.write(bottom_frame)

        cv2.imshow("iamges", np.hstack((top_frame, bottom_frame)) )
        cv2.waitKey(1)
    client.__exit__()
    cv2.destroyAllWindows()
    
    out_bottom.release()
    out_top.release()

    #TODO: gravar com angulos diferentes
    #TODO: gravar o campo inteiro. Todas as features.
    #TODO: graver