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

    client = b0RemoteApi.RemoteApiClient('b0RemoteApi_NAO_Node', 'b0RemoteApiNAO',
                                     60, setupSubscribersAsynchronously=True)
    vrepimage = VrepImage(client)
    cv2.namedWindow("image")
    while (True):
        cv2.imshow("image", vrepimage.get_camera_image())
        cv2.waitKey(1)
    client.__exit__()
    cv2.destroyAllWindows()