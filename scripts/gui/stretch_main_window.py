#!/usr/bin/env python3

import os
import sys

from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QMainWindow
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtGui import QPixmap, QImage

import rospy
from geometry_msgs.msg import Point

import cv2

from stretch_uml.stretch_ui_main_window import Ui_StretchWindow

selected_x = None
selected_y = None

loading_icon = os.path.join(os.path.dirname(__file__), "assets/loading_icon.png")
current_map = os.path.join(os.path.dirname(__file__), "assets/current_map.png")
image_zoomed = os.path.join(os.path.dirname(__file__), "assets/image_zoomed.png")


class MapWorker(QThread):
    """
    Updates to the shown map image are to be made within this thread. Subscriber runs externally, but slicing of the
    image and relevant localization occurs here.
    """
    map_update = pyqtSignal(QPixmap)

    def __init__(self, parent=None, center=(0, 0)):
        QThread.__init__(self, parent)
        self.capture = None
        self.thread_active = False
        self.center = center

    def run(self):
        self.thread_active = True
        while self.thread_active:
            cv_image = cv2.imread(current_map)
            if cv_image is not None:
                x_dim = cv_image.shape[1]
                y_dim = cv_image.shape[0]

                # Check limits on self.center
                if abs(self.center[0]) > round(x_dim / 2):
                    if self.center[0] > 0:
                        self.center = (round(x_dim / 2), self.center[1])
                    else:
                        self.center = (-round(x_dim / 2), self.center[1])

                if abs(self.center[1]) > round(y_dim / 2):
                    if self.center[1] > 0:
                        self.center = (self.center[0], round(y_dim / 2))
                    else:
                        self.center = (self.center[0], -round(y_dim / 2))

                rel_center = (round(x_dim / 2) + self.center[0], round(y_dim / 2) + self.center[1])

                x_limits = int(round(x_dim / 4))
                y_limits = int(round(y_dim / 4))

                x_region = [rel_center[0] - int(round((x_limits / 2 - 1))),
                            rel_center[0] + int(round((x_limits / 2)))]
                y_region = [rel_center[1] - int(round((y_limits / 2 - 1))),
                            rel_center[1] + int(round((y_limits / 2)))]

                if x_region[0] < 0:
                    x_region = [0, x_limits - 1]
                elif x_region[1] > x_dim:
                    x_region = [(x_dim - x_limits - 1), x_dim]

                if y_region[0] < 0:
                    y_region = [0, y_limits - 1]
                elif y_region[1] > y_dim:
                    y_region = [y_dim - y_limits - 1, y_dim]

                cv_image = cv_image[y_region[0]:y_region[1], x_region[0]:x_region[1], :]

                cv_image = cv2.resize(cv_image, (x_dim, y_dim), interpolation=cv2.INTER_CUBIC)
                cv2.imwrite(image_zoomed, cv_image)
                updated_map = QPixmap(image_zoomed)
                self.map_update.emit(updated_map)
                rospy.sleep(0.1)
            else:
                rospy.loginfo("No Map publishing yet")
                # Longer sleep because I assume it won't be up for a bit
                self.map_update.emit(QPixmap(loading_icon))
                rospy.sleep(1)

    def stop(self):
        self.thread_active = False
        self.quit()


class VideoWorker(QThread):
    """
    Takes video feed from system camera and displays. Any necessary rotations or zooms will be done here.
    """
    image_update = pyqtSignal(QImage)

    def __init__(self, parent=None):
        QThread.__init__(self, parent)
        self.capture = None
        self.thread_active = False
        self.image = cv2.imread(loading_icon)

    def run(self):
        self.thread_active = True
        self.capture = cv2.VideoCapture(0)
        while self.thread_active:
            ret, frame = self.capture.read()
            if ret:
                self.image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                converted_to_qt = QImage(self.image.data, self.image.shape[1], self.image.shape[0],
                                         QImage.Format_RGB888)
                picture = converted_to_qt.scaled(960, 540, Qt.KeepAspectRatio)
                self.image_update.emit(picture)
        global selected_x, selected_y
        self.image = cv2.circle(self.image, (selected_x, selected_y), 10, (0, 255, 0), -1)
        cv2.imwrite(image_zoomed, self.image)
        self.capture.release()

    def stop(self):
        self.thread_active = False
        self.quit()


class MainWindow:
    def __init__(self):
        """
        Start up QT Instance. Stack all pages for the stacked page setup. Bind relevant callbacks and set initial
        conditions.
        """
        self.main_window = QMainWindow()
        self.ui = Ui_StretchWindow()
        self.ui.setupUi(self.main_window)

        # Set up thread for video feed
        self.video_worker = VideoWorker()
        self.video_worker.image_update.connect(self.video_frame_update)

        self.map_worker = MapWorker()
        self.map_worker.map_update.connect(self.map_frame_update)

        self.ui.PagesStackedWidget.setCurrentWidget(self.ui.page_1)

        self.ui.GraspButtonPage1.clicked.connect(self.go_to_page_2)
        self.ui.UpButtonPage1.clicked.connect(self.page_1_up)
        self.ui.LeftButtonPage1.clicked.connect(self.page_1_left)
        self.ui.RightButtonPage1.clicked.connect(self.page_1_right)
        self.ui.DownButtonPage1.clicked.connect(self.page_1_down)

        self.ui.BackButtonPage2.clicked.connect(self.go_to_page_1)
        self.ui.UpButtonPage2.clicked.connect(self.page_2_up)
        self.ui.LeftButtonPage2.clicked.connect(self.page_2_left)
        self.ui.RightButtonPage2.clicked.connect(self.page_2_right)
        self.ui.DownButtonPage2.clicked.connect(self.page_2_down)

        self.ui.CameraLabelPage2.mousePressEvent = self.publish_point

        self.ui.YesButtonPage3.clicked.connect(self.go_to_page_1)
        self.ui.NoButtonPage3.clicked.connect(self.go_to_page_2)

    def show(self):
        """
        Make QT instance visible.
        """
        self.main_window.show()

    def go_to_page_1(self):
        self.ui.PagesStackedWidget.setCurrentWidget(self.ui.page_1)
        # self.ui.MapLabelPage1 = QPixmap("map_zoomed.png")
        self.map_worker.start()
        self.video_worker.stop()

    def go_to_page_2(self):
        self.ui.CameraLabelPage2.setPixmap(QPixmap(loading_icon))
        self.ui.PagesStackedWidget.setCurrentWidget(self.ui.page_2)
        self.video_worker.start()
        self.map_worker.stop()

    def go_to_page_3(self):
        self.video_worker.stop()
        self.map_worker.stop()
        # Give the video a second to write the image
        rospy.sleep(0.1)
        image = cv2.imread(image_zoomed)
        converted_to_qt = QImage(image.data, image.shape[1], image.shape[0], QImage.Format_RGB888)
        self.ui.SelectedImageLabelPage3.setPixmap(QPixmap(converted_to_qt))
        self.ui.PagesStackedWidget.setCurrentWidget(self.ui.page_3)

    def video_frame_update(self, image):
        self.ui.CameraLabelPage2.setPixmap(QPixmap.fromImage(image))

    def map_frame_update(self, updated_map):
        self.ui.MapLabelPage1.setPixmap(updated_map)

    def page_1_up(self):
        """
        Inverted control scheme
        """
        self.map_worker.center = (self.map_worker.center[0], self.map_worker.center[1] - 100)
        rospy.logdebug("page 1 up: current center={}".format(self.map_worker.center))

    def page_1_down(self):
        """
        Inverted control scheme
        """
        self.map_worker.center = (self.map_worker.center[0], self.map_worker.center[1] + 100)
        rospy.logdebug("page 1 down: current center={}".format(self.map_worker.center))

    def page_1_left(self):
        """
        Inverted control scheme
        """
        self.map_worker.center = (self.map_worker.center[0] - 100, self.map_worker.center[1])
        rospy.logdebug("page 1 left: current center={}".format(self.map_worker.center))

    def page_1_right(self):
        """
        Inverted control scheme
        """
        self.map_worker.center = (self.map_worker.center[0] + 100, self.map_worker.center[1])
        rospy.logdebug("page 1 right: current center={}".format(self.map_worker.center))

    def publish_point(self, event):
        global selected_x, selected_y
        # X Scaling 960:640
        # Y Scaling 540:480
        selected_x = round(event.x() * 640.0 / 960.0)
        selected_y = round(event.y() * 480.0 / 540.0)
        rospy.loginfo("Point selected\nX:{} Y:{}".format(selected_x, selected_y))
        point_pub.publish(Point(selected_x, selected_y, 0))
        self.go_to_page_3()

    def page_2_up(self):
        # TODO Bind to Camera Driver
        rospy.logdebug("page 2 up")

    def page_2_down(self):
        # TODO Bind to Camera Driver
        rospy.logdebug("page 2 down")

    def page_2_left(self):
        # TODO Bind to Camera Driver
        rospy.logdebug("page 2 left")

    def page_2_right(self):
        # TODO Bind to Camera Driver
        rospy.logdebug("page 2 right")


if __name__ == "__main__":
    point_pub = rospy.Publisher("selected_point", Point, queue_size=10)
    rospy.init_node("gui_pub", anonymous=True)
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    main_window.go_to_page_1()
    sys.exit(app.exec_())
