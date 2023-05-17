import sys
import os
#显示相关
import matplotlib.pyplot as plt
import numpy as np
#字体相关
import matplotlib.font_manager as fm

###########################导入相关的文件路径:############################
#https://blog.csdn.net/cocos2dGirl/article/details/119907503
# sys.path.append(r'C:\AmyPython\Test1')
#python import模块时，是在sys.path里按顺序查找的。
#sys.path是一个列表，里面以字符串的形式存储了许多路径。
#使用A.py文件中的函数需要先将他的文件路径放到sys.path中

baseDir = os.path.abspath(os.path.dirname(__file__))    #当前文件夹的绝对路径
print(baseDir)
os.chdir(baseDir)                                       #change当前所处的目录到baseDir
sys.path.append("..")                                   #父目录
# sys.path.append(baseDir)

import base.base_veh                                  #导入base目录下的base_veh.py

baseDir += "/log/"
os.chdir(baseDir)                                       #change回log目录下，否则因为目录变了后面执行脚本会报错找不到文件
#########################################################################


# for content in file_contents:     #逐行读取
#     if  'pH' in content:          #检查包含pH的那行数据
#         print(content)            #打印，看效果
#         pH_label_file.write(content)   #将符合要求的内容写入文件

def test():
    print("this is func test")
    return 0

decision_dr_writePath = "./decision_dr_trim.txt"     #剪贴后的txt
key_words_traks = ""
key_words_dr = "alg_dr:"     #包括了planInit和Alg两个过程,init时为0

if __name__ == "__main__":
    # myfont = fm.FontProperties(fname=r'C:\Windows\Fonts\Arial.ttf') # 设置字体:windows下可使用,wsl下目录不正确
    # # 坐标名称显示
    # plt.xlabel ( u"x_coordinate ", fontproperties=myfont)
    # plt.ylabel ( u"y_coordinate", fontproperties=myfont)
    # pl.title('dr_show',fontproperties=myfont,fontsize=32) #图像标题
    
    argvstr = sys.argv
    if len(argvstr) < 2:
        print ("need input file name")
        # with open(argvstr[1], 'rb') as f:
        #     map = f.read()
        exit()
    else:        
        if argvstr[1][:11] == "mv_decision":    #判断两个str的前11个字符是否完全一致
            print("The file is decision log")
        else:
            print("The file is not decision log")

        with open(argvstr[1], 'rb') as f:
            # totalData = f.read()
            file_contents=f.readlines() 
            # f.closed

    fWrite_decision_dr = open(decision_dr_writePath,'w')
    # num = fWrite_decision_dr.write("this is my test")
    # print(num)

    for content in file_contents:   #content为bytes
        if key_words_dr.encode() in content:
            # print("key_words_dr: ",key_words_dr)
            # print(content.decode().split(key_words_dr))
            str_decision_dr_split1 = content.decode().split(key_words_dr)
            str_decision_dr_split2 = str_decision_dr_split1[1].split(',')               #以','分割结果;split()默认是以空格进行分割，将每行内容添加到列表
            # print(str_decision_dr_split1[1].split(','))

            num = fWrite_decision_dr.write(str_decision_dr_split2[0])
            num = fWrite_decision_dr.write(' ')
            num = fWrite_decision_dr.write(str_decision_dr_split2[1])
            num = fWrite_decision_dr.write(' ')
            num = fWrite_decision_dr.write(str_decision_dr_split2[2])

            fWrite_decision_dr.write('\r\n')            #换行
            # num = fWrite_decision_dr.write(content.decode())    #write str
    
    #调用其他文件接口
    base.base_veh.C_base_veh.CalCornerCoordinate()  #调用其他文件中的函数(目录.文件.类.函数)
    print("AXIS_DISTANCE222: ",base.base_veh.AXIS_DISTANCE)
    #结构体使用
    # base.base_veh.C_base_veh()     #定义结构对象
    # print(a.x,a.y,a.yaw)


    #决策泊入dr显示
    data = np.loadtxt(decision_dr_writePath)
    print("decision_dr_writePath:",decision_dr_writePath)
    plt.plot(data[:,0],data[:,1])
    plt.show()


    fWrite_decision_dr.close()

    f.closed


    test()
    print("end")
    
    

    

    
            