import sys
import time
import cv2
import threading
import numpy as np
import serial.tools.list_ports
import serial
from PIL import Image
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox, QFileDialog
from PyQt5.QtGui import QPixmap, QImage
from Login import login_MainWindow
from Body import body_MainWindow
from Modify_param import Modify_param_MainWindow
from lib.share import SI
# 分类接口
from infer.class_infer_run import Infer  # openvino_runtime

# from infer.infer_onnx_class import Infer  # onnxruntime

sys.path.append("MvImport")
from MvImport.MvCameraControl_class import *


# 注意 这里选择的父类 要和你UI文件窗体一样的类型
# 主窗口是 QMainWindow， 表单是 QWidget， 对话框是 QDialog
class Login(QMainWindow, login_MainWindow):

    def __init__(self):
        super(Login, self).__init__()
        self.setupUi(self)
        # self.connect_and_emit_sendAddDeviceName()
        self.init()

    def init(self):
        self.bt_login.clicked.connect(self.sing_in)
        self.edu_password.returnPressed.connect(self.sing_in)

    def sing_in(self):
        USERNAME = "1"
        PASSWORD = "1"
        username = self.edu_login.text().strip()
        password = self.edu_password.text().strip()
        if username == USERNAME:
            if password == PASSWORD:
                SI.Body_Win = Body()
                SI.Body_Win.show()
                # SI.Login_Win.edt_password.setText("")
                # SI.Login_Win.hide()
                SI.Login_Win.close()
            else:
                QMessageBox.warning(
                    self,
                    '登录失败',
                    '密码错误,请重新输入!'
                )
                return
        else:
            QMessageBox.warning(
                self,
                '登录失败',
                '用户名错误,请重新输入!'
            )
            return


class Body(QMainWindow, body_MainWindow):

    def __init__(self):
        super(Body, self).__init__()
        self.setupUi(self)
        self.init()
        # ch:创建HK相机实例 | en:Creat Camera Object
        self.cam = MvCamera()
        # ch:创建推理实例
        self.infer = Infer()
        # ch:创建USB相机实例
        self.cap = cv2.VideoCapture(0)

        # 串口初始化
        self.ser = None
        self.init_ser()
        self.predict_index = 0  # 类别索引
        NO1_open = '01 05 00 00 FF 00 8C 3A'  # "上灯"
        NO1_down = "01 05 00 00 00 00 CD CA"

        NO2_open = "01 05 00 01 FF 00 DD FA"
        NO2_down = "01 05 00 01 00 00 9C 0A"

        NO3_open = "01 05 00 02 FF 00 2D FA"
        NO3_down = "01 05 00 02 00 00 6C 0A"

        self.light_num = 3
        self.predict_light = [NO1_open, NO2_open, NO3_open,
                              NO1_down, NO2_down, NO3_down]

        # HK相机初始化参数
        self.deviceList = MV_CC_DEVICE_INFO_LIST()  # 设备列表
        self.tlayerType = MV_GIGE_DEVICE | MV_USB_DEVICE  # gige/usb
        self.g_bExit = False
        self.open = False  # usb开启状态
        self.opencamera_flay = False  # HK开启状态
        self.HKCAM_init = False  # 工业相机参数初始化状态
        self.camera_information = False  # 是否有可用HK相机
        self.param_list = []

    def init(self):
        self.bt_param.clicked.connect(self.modify_param)  # 修改参数
        self.bt_version.clicked.connect(self.aboout_ver)  # 关于
        self.bt_openHK.clicked.connect(self.openHK)  # 工业摄像头
        self.bt_openUSB.clicked.connect(self.open_Usb)  # USB摄像头
        self.bt_close.clicked.connect(self.closeCamera)  # 关闭摄像头
        self.bt_close_usb.clicked.connect(self.openFile)  # 打开文件
        self.la_img.setScaledContents(True)
        self.la_class.setText("类别: **    概率: **")

    # 初始化串口设备
    def init_ser(self):
        if self.ser is None:
            port_list = list(serial.tools.list_ports.comports())
            index = None
            flag = False
            for i, com in enumerate(port_list):
                if "USB-to-Serial" in com.description:
                    flag = True
                    index = i
            if flag:
                self.ser = serial.Serial(port_list[index].device, 9600, timeout=0.5)
            else:
                QMessageBox.warning(
                    self,
                    '串口',
                    '未找到可用串口，请检查串口连接状态!'
                )
                return

    # 关于版本
    def aboout_ver(self):
        pass

    # 打开图片文件
    def openFile(self):
        if self.open or self.opencamera_flay:
            self.closeCamera()
        imgName, imgType = QFileDialog.getOpenFileName(
            self, "打开图片", "", "All Files(*)")
        # print(imgName, imgType)
        if imgName == "":
            return
        img = cv2.imread(imgName)
        image = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        r_image = cv2.resize(image, (540, 320))
        classes, pro, self.predict_index = self.infer.detection(image)  # return class and pro
        self.la_class.setText('类别: {:10}    概率: {:.3}'.format(classes, pro))

        #  显示图片
        image_height, image_width, image_depth = r_image.shape  # 读取图像高宽深度
        self.image_show = QImage(r_image.data, image_width, image_height, image_width * image_depth,
                                 QImage.Format_RGB888)

        # image_height, image_width = image.shape  # 读取图像高宽
        # self.image_show = QImage(image.data, image_width, image_height,
        #                          QImage.Format_Grayscale8)
        self.la_img.setPixmap(QPixmap.fromImage(self.image_show))

        # 发送串口信号
        self.out_serial()

    # 开始工业摄像头检测
    def openHK(self):
        if self.HKCAM_init:
            self.get_camera_information()
            self.openCamera()
        else:
            QMessageBox.warning(
                self,
                '打开失败',
                '请先将相机参数初始化后!'
            )
            return

    # 打开HK摄像头开始检测
    def openCamera(self):
        # print(self.param_list)
        if self.camera_information:
            if self.ser is None:
                QMessageBox.warning(
                    self,
                    '串口',
                    '未找到可用串口，请检查串口连接状态!'
                )
                return
            self.g_bExit = False
            # ch:选择设备并创建句柄 | en:Select device and create handle
            stDeviceList = cast(self.deviceList.pDeviceInfo[int(0)], POINTER(MV_CC_DEVICE_INFO)).contents
            ret = self.cam.MV_CC_CreateHandle(stDeviceList)
            if ret != 0:
                # print("create handle fail! ret[0x%x]" % ret)
                QMessageBox.critical(self, "错误", "创建句柄失败 ! ret[0x%x]" % ret)
                # sys.exit()
            # ch:打开设备 | en:Open device
            ret = self.cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
            if ret != 0:
                # print("open device fail! ret[0x%x]" % ret)
                QMessageBox.critical(self, "错误", "打开设备失败 ! ret[0x%x]" % ret)
                # sys.exit()
            else:
                # 设置获取的参数
                self.set_parameter(self.param_list)

            # ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal package size(It only works for the GigE camera)
            if stDeviceList.nTLayerType == MV_GIGE_DEVICE:
                nPacketSize = self.cam.MV_CC_GetOptimalPacketSize()
                if int(nPacketSize) > 0:
                    ret = self.cam.MV_CC_SetIntValue("GevSCPSPacketSize", nPacketSize)
                    if ret != 0:
                        # print("Warning: Set Packet Size fail! ret[0x%x]" % ret)
                        QMessageBox.warning(self, "警告", "报文大小设置失败 ! ret[0x%x]" % ret)
                else:
                    # print("Warning: Get Packet Size fail! ret[0x%x]" % nPacketSize)
                    QMessageBox.warning(self, "警告", "报文大小获取失败 ! ret[0x%x]" % nPacketSize)

            # ch:设置触发模式为off | en:Set trigger mode as off
            ret = self.cam.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF)
            if ret != 0:
                # print("set trigger mode fail! ret[0x%x]" % ret)
                QMessageBox.critical(self, "错误", "设置触发模式失败 ! ret[0x%x]" % ret)
                # sys.exit()

            # ch:获取数据包大小 | en:Get payload size
            stParam = MVCC_INTVALUE()
            memset(byref(stParam), 0, sizeof(MVCC_INTVALUE))
            ret = self.cam.MV_CC_GetIntValue("PayloadSize", stParam)
            if ret != 0:
                # print("get payload size fail! ret[0x%x]" % ret)
                QMessageBox.critical(self, "错误", "获取有效负载大小失败 ! ret[0x%x]" % ret)
                sys.exit()
            nDataSize = stParam.nCurValue
            pdata = (c_ubyte * nDataSize)()  # 8位

            # ch:开始取流 | en:Start grab image
            ret = self.cam.MV_CC_StartGrabbing()
            if ret != 0:
                # print("start grabbing fail! ret[0x%x]" % ret)
                QMessageBox.critical(self, "错误", "开始抓取图像失败 ! ret[0x%x]" % ret)
                # sys.exit()

            self.opencamera_flay = True
            try:

                hThreadHandle = threading.Thread(target=self.work_thread,
                                                 args=(self.cam, pdata, nDataSize))
                hThreadHandle.start()
            except:
                # print("error: unable to start thread")
                QMessageBox.critical(self, "错误", "无法启动线程 ! ")

        else:
            QMessageBox.critical(self, '错误', '获取相机信息失败！')
            return None

    # 调出参数界面设置相机参数
    def modify_param(self):
        self.get_camera_information()
        if self.camera_information:
            SI.Modify_param_Win = Modify_par()
            SI.Modify_param_Win.show()
            SI.Body_Win.hide()

    # 检索可用HK设备
    def get_camera_information(self):
        """
        选择所有能用的相机到列表中，
        gige相机需要配合 sdk 得到。
        """
        # 得到相机列表
        # tlayerType = MV_GIGE_DEVICE | MV_USB_DEVICE
        # ch:枚举设备 | en:Enum device
        ret = MvCamera.MV_CC_EnumDevices(self.tlayerType, self.deviceList)
        if ret != 0:
            print("enum devices fail! ret[0x%x]" % ret)
            # QMessageBox.critical(self, '错误', '读取设备驱动失败！')
            # sys.exit()
        if self.deviceList.nDeviceNum == 0:
            QMessageBox.critical(self, "错误", "没有发现设备 ！ ")
            # print("find no device!")
            # sys.exit()
        else:
            QMessageBox.information(self, "提示", "发现了 %d 个设备 !" % self.deviceList.nDeviceNum)
        # print("Find %d devices!" % self.deviceList.nDeviceNum)
        if self.deviceList.nDeviceNum == 0:
            return None

        for i in range(0, self.deviceList.nDeviceNum):
            mvcc_dev_info = cast(self.deviceList.pDeviceInfo[i], POINTER(MV_CC_DEVICE_INFO)).contents
            if mvcc_dev_info.nTLayerType == MV_GIGE_DEVICE:
                print("\ngige device: [%d]" % i)
                strModeName = ""
                for per in mvcc_dev_info.SpecialInfo.stGigEInfo.chModelName:
                    strModeName = strModeName + chr(per)
                print("device model name: %s" % strModeName)

        self.camera_information = True

    # 将更改的相机参数加载到相机
    def set_parameter(self, param_list):
        param_list_def = ["90", "RGB", "4000", "1440", "1080"]
        for i, elem in enumerate(param_list):
            if elem == "":
                param_list[i] = param_list_def[i]
        # print(param_list)
        AcquisitionFrameRate = int(param_list[0])  # 帧率
        Width = int(param_list[3])
        Height = int(param_list[4])
        if param_list[1] == "RGB":
            PixelFormat = PixelType_Gvsp_BayerRG8
        else:
            PixelFormat = PixelType_Gvsp_Mono8
        ExposureTime = int(param_list[2])

        ret_width = self.cam.MV_CC_SetIntValue("Width", Width)
        if ret_width != 0:
            QMessageBox.critical(self, "错误", "Width ! ret[0x%x]" % ret_width)
        ret_height = self.cam.MV_CC_SetIntValue("Height", Height)
        if ret_height != 0:
            QMessageBox.critical(self, "错误", "Height ! ret[0x%x]" % ret_height)
        ret_frameRate = self.cam.MV_CC_SetFloatValue("AcquisitionFrameRate", float(AcquisitionFrameRate))
        if ret_frameRate != 0:
            QMessageBox.critical(self, "错误", "AcquisitionFrameRate ! ret[0x%x]" % ret_frameRate)
        ret_pixelFormat = self.cam.MV_CC_SetEnumValue("PixelFormat", PixelFormat)
        if ret_pixelFormat != 0:
            QMessageBox.critical(self, "错误", "PixelFormat ! ret[0x%x]" % ret_pixelFormat)
        ret_exposureTime = self.cam.MV_CC_SetFloatValue("ExposureTime", float(ExposureTime))
        if ret_exposureTime != 0:
            QMessageBox.critical(self, "错误", "ExposureTime ! ret[0x%x]" % ret_exposureTime)

    # 逐帧处理数据流
    def work_thread(self, cam=0, pData=0, nDataSize=0):
        stFrameInfo = MV_FRAME_OUT_INFO_EX()
        memset(byref(stFrameInfo), 0, sizeof(stFrameInfo))
        while True:
            ret = cam.MV_CC_GetOneFrameTimeout(pData, nDataSize, stFrameInfo, 1000)
            if ret == 0:
                image = np.frombuffer(pData, dtype=np.uint8)  # 将c_ubyte_Array转化成ndarray得到(1555200,)
                # image = np.asarray(pData)  # 将c_ubyte_Array转化成ndarray得到（1555200，）
                image = self.image_control(image, stFrameInfo)

                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                show_img = cv2.resize(image, (image.shape[0] // 2, image.shape[1] // 2))

                # 分类接口
                classes, pro, self.predict_index = self.infer.detection(image)  # return class and pro
                self.la_class.setText('类别: {:10}    概率: {:.3}'.format(classes, pro))

                #  显示图片

                image_height, image_width, image_depth = show_img.shape  # 读取图像高宽深度
                self.image_show = QImage(show_img.data, image_width, image_height, image_width * image_depth,
                                         QImage.Format_RGB888)

                # image_height, image_width = image.shape  # 读取图像高宽
                # self.image_show = QImage(image.data, image_width, image_height,
                #                          QImage.Format_Grayscale8)
                self.la_img.setPixmap(QPixmap.fromImage(self.image_show))

                # 发送串口信号
                self.out_serial()

            if self.g_bExit:
                del pData
                break

    # 串口控制
    def out_serial(self):
        # 背景不开
        if self.predict_index == 0 or not self.ser:
            return
        k = bytes.fromhex(self.predict_light[self.predict_index])
        d = bytes.fromhex(self.predict_light[self.predict_index + self.light_num])

        # 串口发送数据
        success_bytes = self.ser.write(k)
        end_code = success_bytes // 2 + 4
        data = []
        for _ in range(end_code):
            tmp = self.ser.read(1)
            data.append(tmp.hex())
        time.sleep(2)
        success_bytes = self.ser.write(d)
        data = []
        end_code = success_bytes // 2 + 4
        for _ in range(end_code):
            tmp = self.ser.read(1)
            data.append(tmp.hex())

    # 关闭相机
    def closeCamera(self):
        self.open = False
        self.la_img.clear()  # 清除label组件上的图片
        self.la_class.setText("类别: **    概率: **")
        if self.opencamera_flay:
            self.g_bExit = True
            # ch:停止取流 | en:Stop grab image
            ret = self.cam.MV_CC_StopGrabbing()
            if ret != 0:
                QMessageBox.critical(self, "错误", "停止抓取图像失败 ! ret[0x%x]" % ret)
            # ch:关闭设备 | Close device
            ret = self.cam.MV_CC_CloseDevice()
            if ret != 0:
                # print("close deivce fail! ret[0x%x]" % ret)
                QMessageBox.critical(self, "错误", "停止设备失败 ! ret[0x%x]" % ret)
            # ch:销毁句柄 | Destroy handle
            ret = self.cam.MV_CC_DestroyHandle()
            if ret != 0:
                QMessageBox.critical(self, "错误", "销毁处理失败 ! ret[0x%x]" % ret)
            self.la_img.clear()  # 清除label组件上的图片
            self.la_class.setText("类别: **    概率: **")
            self.camera_information = False
            self.opencamera_flay = False

    # 将获取的数据帧转换成RGB返回
    def image_control(self, data, stFrameInfo):
        if stFrameInfo.enPixelType == 17301505:
            return data.reshape((stFrameInfo.nHeight, stFrameInfo.nWidth))
            # self.image_show_(image=image, name=stFrameInfo.nHeight)
        elif stFrameInfo.enPixelType == 17301513:
            data = data.reshape(stFrameInfo.nHeight, stFrameInfo.nWidth, -1)
            return cv2.cvtColor(data, cv2.COLOR_BAYER_RG2RGB)
            # self.image_show_(image=image, name=stFrameInfo.nHeight)
        elif stFrameInfo.enPixelType == 35127316:
            data = data.reshape(stFrameInfo.nHeight, stFrameInfo.nWidth, -1)
            return cv2.cvtColor(data, cv2.COLOR_RGB2BGR)
            # self.image_show_(image=image, name=stFrameInfo.nHeight)
        elif stFrameInfo.enPixelType == 34603039:
            data = data.reshape(stFrameInfo.nHeight, stFrameInfo.nWidth, -1)
            return cv2.cvtColor(data, cv2.COLOR_YUV2BGR_Y422)
            # self.image_show_(image=image, name=stFrameInfo.nHeight)
        elif stFrameInfo.enPixelType == 17301505:
            data = data.reshape(stFrameInfo.nHeight, stFrameInfo.nWidth)
            return data

    # 开始USB检测
    def open_Usb(self):
        if self.ser is None:
            QMessageBox.warning(
                self,
                '串口',
                '未找到可用串口，请检查串口连接状态!'
            )
            return
        self.open = True
        threading.Thread(target=self.getUSB).start()

    # 检测帧
    def getUSB(self):
        while self.open:
            # 一帧一帧捕捉
            ret, frame = self.cap.read()
            # 我们对帧的操作在这里
            # image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            if not ret:
                break
            image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            r_image = cv2.resize(image, (540, 320))
            if not self.open:
                break
            # if self.start_predict:
            classes, pro, self.predict_index = self.infer.detection(image)  # return class and pro
            self.la_class.setText('类别: {:10}    概率: {:.3}'.format(classes, pro))

            #  显示图片
            image_height, image_width, image_depth = r_image.shape  # 读取图像高宽深度
            self.image_show = QImage(r_image.data, image_width, image_height, image_width * image_depth,
                                     QImage.Format_RGB888)

            # image_height, image_width = image.shape  # 读取图像高宽
            # self.image_show = QImage(image.data, image_width, image_height,
            #                          QImage.Format_Grayscale8)
            self.la_img.setPixmap(QPixmap.fromImage(self.image_show))

            # 发送串口信号
            self.out_serial()


class Modify_par(QMainWindow, Modify_param_MainWindow):

    def __init__(self):
        super(Modify_par, self).__init__()
        self.h = None
        self.w = None
        self.et = None
        self.mode = None
        self.fps = None
        self.param_list = None
        self.HKCAM_init = True  # 是否初始化
        self.setupUi(self)
        self.init()

    def init(self):
        self.pushButton.clicked.connect(self.singout)

    def singout(self):
        # 保存并退出
        self.fps = self.edu_Fps.text().strip()
        self.mode = self.edu_mode.text().strip()
        self.et = self.edu_etime.text().strip()
        self.w = self.edu_width.text().strip()
        self.h = self.edu_height.text().strip()
        self.param_list = [self.fps, self.mode, self.et, self.w, self.h]
        print(self.param_list)
        SI.Body_Win.HKCAM_init = self.HKCAM_init  # 成功初始化相机参数后标志 True
        SI.Body_Win.param_list = self.param_list  # 获取设置的参数 [帧率, 模式, 曝光时间, 帧宽, 帧高]
        SI.Body_Win.show()
        SI.Modify_param_Win.hide()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    SI.Login_Win = Login()
    SI.Login_Win.show()
    sys.exit(app.exec_())
