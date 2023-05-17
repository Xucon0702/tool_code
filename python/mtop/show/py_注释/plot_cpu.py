# perf utils plot_process.py

import sys
import pandas as pd
import matplotlib as mpl
import matplotlib.pyplot as plt
from mplcursors import cursor
from optparse import OptionParser

#matplotlib.use('TkAgg')

#list
WHITE_NAME = [
        'mv_avm',
        'mv_psd',
        'mv_plan',
        'mv_fusion',
        'mv_decision'
        ]

def plot(csv_file):
    # load mtop result(csv_file->文件路径;sep->定界符使,默认'\s'使用python解析引擎解析;parse_dates->将'TS'解析为单独的列表;names->列名列表)
    df = pd.read_csv(csv_file, sep='\s+', parse_dates=['TS'], names=["TS", "PID", "PRI", "STATE",\
            "HH:MM:SS", "CPU", "LASTCPU", "COMMAND", "THREAD_NAME"])

    # filter rows which contain thread info
    df = df[df['TS'].str.match(r'[0-9][0-9]:[0-9][0-9]:[0-9][0-9]')] #TS中的时间表每个数做0~9的匹配1 6 :5 3 :4 3;符合要求的保留
    df['TS'] = pd.to_datetime(df['TS'], format='%H:%M:%S')  #转换为pandas datetime对象

    df = df[df['COMMAND'].isin(WHITE_NAME)]     #判断进程名是否在关注的list中,在的话保留;这句话删除的话就是包含所有的进程不做过滤

    # accum cpu info by proc name
    df['CPU'] = df['CPU'].str.rstrip('%').astype('float')  #rstrip 剥离字符串中特定的字符:'%'
    result = df.groupby(['TS','COMMAND'])['CPU'].sum().unstack()    #groupby对DataFrame进行分组,用于对大量数据进行分组并在这些组上计算操作;unstack将整合数据列表成值在一排
    result['TOTAL'] = df.groupby('TS')['CPU'].sum()

    # change plot layout when needed
    parser = OptionParser()     #处理命令行参数
    parser.add_option('-v', action='store_true', dest='verbose', default=False) #dest是存储的变量
    opts, args = parser.parse_args()    #解析并获得选项，默认使用 sys.argv[:1]
    layout = 10, 4                      #tuple初始化,10行4列，如果进程数多了需要适当增加
    if opts.verbose:
        result.plot(subplots=True, layout=layout, legend=True) #subplots=True为每个列创建单独的子图;layout用于子图的布局;将图例放在轴子图上
    else:
        result.plot(subplots=False, layout=layout, legend=False)

    #plt.axhline(y=50, color='r')
    #plt.axhline(y=45, color='g')
    #plt.axhline(y=40, color='b')
    cursor(hover=True)              #鼠标悬停在热图上时,显示热图中特定元素的值




    '''
    mpl.rcParams['font.sans-serif'] = 'WenQuanYi Zen Hei'
    plt.title("0807版本寻库/泊入整体CPU使用情况\n(卡度尼楼下垂直线车位, 003车辆, S.E02.02底软，关闭memtester, 关闭调试线程，关闭gaclog终端实时打印)")
    '''
    plt.show()                      #显示

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("usage: plot_csv.py cpu.txt")
    else:
        plot(sys.argv[1])
