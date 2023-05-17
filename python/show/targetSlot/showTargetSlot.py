import sys
import os
#显示相关
import matplotlib.pyplot as plt
import numpy as np
#字体相关
import matplotlib.font_manager as fm

import time

# 全局变量
# global cnt
cnt = 0

target_slot_writePath = "./target_slot.txt"     #剪贴后的txt
key_words_target_slot_init = "TargetSolt"     #planInit
key_words_target_slot_alg = "TargetSolt "     #planAlg

def write_target_slot(content,keyWords,fwrite):
    str_split1 = content.decode().split(keyWords)

    str_split2 = str_split1[1].split(' ')

    str_split_tmp = str_split2[0][1:-1]  #截取字符串第二个~倒数第二个的数据
    str_split_3 = str_split_tmp.split(',')
    num = fwrite.write(str_split_3[0])
    num = fwrite.write(' ')
    num = fwrite.write(str_split_3[1])
    num = fwrite.write(' ')

    str_split_tmp = str_split2[1][1:-1]  #截取字符串第二个~倒数第二个的数据
    str_split_3 = str_split_tmp.split(',')
    num = fwrite.write(str_split_3[0])
    num = fwrite.write(' ')
    num = fwrite.write(str_split_3[1])
    num = fwrite.write(' ')

    str_split_tmp = str_split2[2][1:-1]  #截取字符串第二个~倒数第二个的数据
    str_split_3 = str_split_tmp.split(',')
    num = fwrite.write(str_split_3[0])
    num = fwrite.write(' ')
    num = fwrite.write(str_split_3[1])
    num = fwrite.write(' ')

    str_split_tmp = str_split2[3][1:-2]  #截取字符串第二个~倒数第三个的数据
    str_split_3 = str_split_tmp.split(',')
    num = fwrite.write(str_split_3[0])
    num = fwrite.write(' ')
    num = fwrite.write(str_split_3[1])

    fwrite.write('\r\n')

    return 0



def showTargetSlot(plt):
    global cnt              #声明用的是全局变量cnt,不是局变

    if not os.path.exists(target_slot_writePath):
        print(target_slot_writePath,"is not exist")
        return -1
    
    with open(target_slot_writePath, 'rb') as fread_target_slot:
            file_contents_target_slot = fread_target_slot.readlines() 

    

    for content in file_contents_target_slot:
         str_split1 = content.decode().split(" ")
         point0_x = str_split1[0]
         point1_x = str_split1[2]
         point2_x = str_split1[4]
         point3_x = str_split1[6]
         point0_y = str_split1[1]
         point1_y = str_split1[3]
         point2_y = str_split1[5]
         point3_y = str_split1[7]
         
         targetSlotX = np.array([point0_x,point1_x,point2_x,point3_x,point0_x])
         targetSlotY = np.array([point0_y,point1_y,point2_y,point3_y,point0_y])

        # 静态显示一张
        #  plt.plot(targetSlotX, targetSlotY)
        # #  plt.plot(slot_psd_x, slot_psd_y)
        #  plt.show()

        # 动态显示变化情况
         plt.clf()  #清除上一幅图像
         plt.plot(targetSlotX,targetSlotY)
         plt.pause(0.1)  # 暂停0.01秒
         plt.ioff()  # 关闭画图的窗口

         cnt += 1
         print(point0_x,point0_y,point1_x,point1_y,point2_x,point2_y,point3_x,point3_y,"showTargetSlot:",cnt,end='\n')

        #  time.sleep(0.5)        #休眠
    plt.show()                    #结束循环时调用,否则可能闪退
    return 0



if __name__ == "__main__":
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

    if os.path.exists(target_slot_writePath):       #文件存在则删除
        os.remove(target_slot_writePath)

    fwrite_target_slot = open(target_slot_writePath,'w')

    b_first_data_target_slot = True  #第一个目标车位数据
    
    #截取target slot数据并保存到特定文件中去
    for content in file_contents:
        if key_words_target_slot_init.encode() in content:
                if b_first_data_target_slot:    #init
                    print("init data")
                    write_target_slot(content,key_words_target_slot_init,fwrite_target_slot)
                    b_first_data_target_slot = False
                else:
                    print("alg data")
                    write_target_slot(content,key_words_target_slot_alg,fwrite_target_slot)                                        

    fwrite_target_slot.close()

    f.closed

    # 显示
    plt.figure('Targetslot')     
    plt.axis('equal')       #x,y轴等比

    #设置坐标轴范围
    plt.xlim((-5, 5))
    plt.ylim((-2, 2))
    #设置坐标轴名称
    plt.xlabel('xxxxxxxxxxx')
    plt.ylabel('yyyyyyyyyyy')
    #设置坐标轴刻度
    my_x_ticks = np.arange(-5, 5, 0.5)
    my_y_ticks = np.arange(-2, 2, 0.3)
    plt.xticks(my_x_ticks)
    plt.yticks(my_y_ticks)

    #设置左上角为坐标轴原点
    # ax = plt.gca()                                 #获取到当前坐标轴信息
    # ax.xaxis.set_ticks_position('top')             #将X坐标轴移到上面
    # ax.invert_yaxis()                              #反转Y坐标轴



    showTargetSlot(plt)


    print("end")