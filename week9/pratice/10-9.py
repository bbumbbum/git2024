import cv2

def main():
    camera = cv2.VideoCapture(0)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 800)
    camera.set(cv2.CAP_PROP_FPS, 20)

    while(camera.isOpened()) :
        _, image = camera.read()
        cv2.imshow('camera test', image)

        if cv2.waitKey(1) == ord('q') :
            break

    cv2.destroyAllWindows()

if __name__ == '__main__' :
    main()