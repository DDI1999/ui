import sys
import os
import logging as log

import cv2
import torch
import numpy as np
from PIL import Image
from torchvision import transforms
import openvino.runtime as ov

from infer.myTransforms import ResizeTo224

'''
openvino infer int8
'''


class Infer:
    def __init__(self):
        device = "CPU"
        model_xml_path = "./IR/int8/res34_int8_985.xml"
        model_bin_path = "./IR/int8/res34_int8_985.bin"
        # model_xml_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + "\\IR\\int8\\res34_int8_985.xml"
        # model_bin_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + "\\IR\\int8\\res34_int8_985.bin"
        # set log format
        log.basicConfig(format="[ %(levelname)s ] %(message)s", level=log.INFO, stream=sys.stdout)

        assert os.path.exists(model_xml_path), ".xml file does not exist..."
        assert os.path.exists(model_bin_path), ".bin file does not exist..."

        # classes
        self.class_indict = ["background", "female", "male"]
        self.class_indict_ch = ["背景", "母鸡", "公鸡"]
        core = ov.Core()

        self.model = core.read_model(model=model_xml_path)
        self.compiled_model = core.compile_model(model=self.model, device_name=device)
        self.input_layer = self.compiled_model.input(0)
        self.output_layer = self.compiled_model.output(0)
        self.request = self.compiled_model.create_infer_request()

    def detection(self, img):
        img = Image.fromarray(img)
        data_transform = transforms.Compose([
            ResizeTo224(),
            transforms.ToTensor(),
        ])
        img = data_transform(img)
        img = torch.unsqueeze(img, dim=0)

        # model.input(0).any_name

        self.request.infer(inputs={self.input_layer.any_name: img})
        result = self.request.get_output_tensor(self.output_layer.index).data
        # [[]] -> []
        prediction = np.squeeze(result)
        predict = torch.softmax(torch.tensor(prediction), dim=0)

        # for i in range(len(predict)):
        #     print("class: {:10}   prob: {:.3}".format(class_indict[str(i)],
        #                                               predict[i].numpy()))

        index = np.array(predict).argmax()
        print("class: {} pro: {:.4}".format(self.class_indict_ch[index], predict[index].numpy()))
        return self.class_indict_ch[index], predict[index].numpy(), index


if __name__ == '__main__':
    img_path = 'E:/work/test/data/background/13.PNG'
    img = cv2.imread(img_path)
    image = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    infer = Infer()
    infer.detection(img)
