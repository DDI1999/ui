import os
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QFileDialog
from test_ui import test_MainWindow
from Modify_param import Modify_param_MainWindow


# def open_Usb():
#     cv2.VideoCapture()
#     open = True
#     # while self.open:
#     # print("open")
#     if open:
#         threading.Thread(target=getUSB).start()


# def getUSB():
#     while open:
#         # 一帧一帧捕捉
#         ret, frame = cap.read()
#         # 我们对帧的操作在这里
#         # image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
#         if not ret:
#             break
#         image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
#         r_image = cv2.resize(image, (540, 320))
#         if not self.open:
#             break
#         # if self.start_predict:
#         classes, pro, self.predict_index = self.infer.detection(image)  # return class and pro
#         self.la_class.setText('类别: {:10}    概率: {:.3}'.format(classes, pro))
#
#         #  显示图片
#         # image_height, image_width, image_depth = r_image.shape  # 读取图像高宽深度
#         # self.image_show = QImage(r_image.data, image_width, image_height, image_width * image_depth,
#         #                          QImage.Format_RGB888)
#
#         # image_height, image_width = image.shape  # 读取图像高宽
#         # self.image_show = QImage(image.data, image_width, image_height,
#         #                          QImage.Format_Grayscale8)
#         # self.la_img.setPixmap(QPixmap.fromImage(self.image_show))
#
#         # 发送串口信号
#         self.out_serial()
#         # 显示返回的每帧
#         cv2.imshow('frame', r_image)
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break
#     cap.release()
#     cv.destroyAllWindows()

class Test(QMainWindow, test_MainWindow):
    def __init__(self):
        super(Test, self).__init__()
        self.setupUi(self)
        self.ln = None
        self.pushButton_2.clicked.connect(self.openFile)
        self.pushButton.clicked.connect(self.mo)

    def mo(self):
        print(self.ln)
        self.ln = self.lineEdit.text().strip()
        print(self.ln)

    def openFile(self):
        imgName, imgType = QFileDialog.getOpenFileName(
            self, "打开图片", "", "All Files(*)")
        print(imgName, imgType)


if __name__ == "__main__":
    # app = QApplication(sys.argv)
    # # main = Test()
    # # main.show()
    # sys.exit(app.exec_())
    sys_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    print(sys_path)
    print(os.path.dirname(os.path.abspath(__file__)))
    # print(os.path.dirname(os.path.abspath(__file__)) + "\\IR\\int8\\res34_int8_985.xml")
