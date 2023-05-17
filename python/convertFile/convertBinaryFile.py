# -*- coding: utf-8 -*-
import os
import binascii
from functools import partial
import sys

file_writePath = "./"



if __name__ == "__main__":
    i = 0
    argvstr = sys.argv
    if len(argvstr) < 2:
        print ("need input file name")
        # with open(argvstr[1], 'rb') as f:
        #     map = f.read()
        exit()
    else:
        #转为二进制字符文件
        print("need to be converted Binary file  name",argvstr[1])
        f = open(argvstr[1], 'rb')
        writeFileName = "converted_"
        writeFileName += argvstr[1]
        print(writeFileName)

        f2 = open(writeFileName, 'w')
        records = iter(partial(f.read,1), b'')
        for r in records:
            r_int = int.from_bytes(r, byteorder='big')  #将 byte转化为 int
            str_bin = bin(r_int).lstrip('0b')  #将int转化为二进制字符
            if r_int.bit_length() < 8 :  #以8bit为单位，不足8bit的补零
                str_bin = (8 - r_int.bit_length()) * '0' + str_bin
            f2.write(str_bin)
            i += 1
            if i == 4 :              #以32bit为单位分行
                f2.write('\n')
                i = 0
        f.close
        f2.close

        #
        # with open(argvstr[1], "r", encoding='UTF-8')as f:
        #     # res = f.read()
        #     res = f.read_csv(path, encoding='ISO-8859-1')
        #     print(res)
        f = open(argvstr[1],'rb+')
        print(f.read())
