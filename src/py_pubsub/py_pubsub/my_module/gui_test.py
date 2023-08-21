from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QLabel
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtCore import QTimer
import cv2

class gui_work(QMainWindow):
    def __init__(self, subscriber):
        super().__init__()
        
        self.subscriber = subscriber
        self.current_frame = None

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)

        self.initUI()

    def initUI(self):
        self.label = QLabel(self)
        self.label.setGeometry(10, 10, 640, 480)

        start_button = QPushButton('Start', self)
        start_button.setGeometry(10, 500, 100, 30)
        start_button.clicked.connect(self.start_feed)

        stop_button = QPushButton('Stop', self)
        stop_button.setGeometry(120, 500, 100, 30)
        stop_button.clicked.connect(self.stop_feed)

        select_bbox_button = QPushButton('Select BBox', self)
        select_bbox_button.setGeometry(230, 500, 100, 30)
        select_bbox_button.clicked.connect(self.select_bbox)

        send_bbox_button = QPushButton('Send BBox', self)
        send_bbox_button.setGeometry(340, 500, 100, 30)
        send_bbox_button.clicked.connect(self.send_bbox)

        self.show()

    def start_feed(self):
        self.timer.start()

    def stop_feed(self):
        self.timer.stop()

    def select_bbox(self):
        if self.current_frame is not None:
            self.bbox = cv2.selectROI("Select Bounding Box", self.current_frame, False, False)

    def send_bbox(self):
        if self.bbox is not None:
            # Code to send bounding box coordinates to the publisher node
            pass

    def update_frame(self):
        self.current_frame = self.subscriber.get_frame()
        if self.current_frame is not None:
            q_img = QImage(self.current_frame.data, self.current_frame.shape[1], self.current_frame.shape[0], QImage.Format_RGB888)
            self.label.setPixmap(QPixmap.fromImage(q_img))
