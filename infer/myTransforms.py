from random import random

import numpy as np
from PIL import Image
import os


class ResizeTo224(object):
    def __init__(self, size=224):  # ...是要传入的多个参数

        # 对多参数进行传入
        # 如 self.p = p 传入概率
        # ...
        self.desired_size = size

    def __call__(self, img):  # __call__函数还是只有一个参数传入
        # 该自定义transforms方法的具体实现过程

        old_size = img.size

        ratio = float(self.desired_size) / max(old_size)
        new_size = [int(x * ratio) for x in old_size]
        im = img.resize((new_size[0], new_size[1]), Image.BILINEAR)
        new_im = Image.new('L', (self.desired_size, self.desired_size))
        new_im.paste(im, ((self.desired_size - new_size[0]) // 2,
                          (self.desired_size - new_size[1]) // 2))
        return new_im

