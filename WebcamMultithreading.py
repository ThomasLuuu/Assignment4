import threading
import cv2

class WebcamMultithreading:
    def __init__(self, src=-1, width = 640, height = 480):
        # init to start camera and set dimensions
        self.stream = cv2.VideoCapture(src)
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        (self.grabbed, self.frame) = self.stream.read()

        self.started = False
        self.read_lock = threading.Lock()

    def start(self):
        # create and start a separate thread
        if self.started:
            print('[!] Asynchroneous video capturing has already been started.')
            return None
        self.started = True
        self.thread = threading.Thread(target=self.update, args=())
        self.thread.start()
        return self

    def update(self):
        # loop for reading frames until the thread is stopped
         while self.started:
            grabbed, frame = self.stream.read()
            with self.read_lock:
                self.grabbed = grabbed
                self.frame = frame

    def read(self):
        # copy the frame for processing
        with self.read_lock:
            frame = self.frame.copy()
            grabbed = self.grabbed
        return grabbed, frame

    def stop(self):
        # to stop the thread
        self.started = False
        self.thread.join()


    def __exit__(self, exec_type, exc_value, traceback):
        self.stream.release()