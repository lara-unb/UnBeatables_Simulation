import cv2
import naoqi

if __name__ == "__main__":
    nao_ip = "localhost"
    nao_port = 9559
    video_proxy = naoqi.ALProxy("ALVideoDevice",nao_ip, nao_port)

    opencv_video = cv2.VideoCapture("/home/paulo/nao/workspace/UnBeatables/videos/side1_2.avi")

    while(opencv_video.isOpened()):
        ret, frame = opencv_video.read()
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        put_ret = video_proxy.putImage(0, rgb.shape[1], rgb.shape[0], rgb.data )

        cv2.imshow('frame',rgb)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()