# perf utils plot_process.py

import sys
import pandas as pd
import matplotlib as mpl
import matplotlib.pyplot as plt
from mplcursors import cursor
from optparse import OptionParser

WHITE_NAME = [
        'mv_avm',
        'mv_psd',
        'mv_plan',
        'mv_fusion',
        'mv_decision'
        ]

def plot(csv_file):
    # load mtop result
    df = pd.read_csv(csv_file, sep='\s+', parse_dates=['TS'], names=["TS", "PID", "PRI", "STATE",\
            "HH:MM:SS", "CPU", "LASTCPU", "COMMAND", "THREAD_NAME"])

    # filter rows which contain thread info
    df = df[df['TS'].str.match(r'[0-9][0-9]:[0-9][0-9]:[0-9][0-9]')]
    df['TS'] = pd.to_datetime(df['TS'], format='%H:%M:%S')

    df = df[df['COMMAND'].isin(WHITE_NAME)]

    # accum cpu info by proc name
    df['CPU'] = df['CPU'].str.rstrip('%').astype('float')
    result = df.groupby(['TS','COMMAND'])['CPU'].sum().unstack()
    result['TOTAL'] = df.groupby('TS')['CPU'].sum()

    # change plot layout when needed
    parser = OptionParser()
    parser.add_option('-v', action='store_true', dest='verbose', default=False)
    opts, args = parser.parse_args()
    layout = 10, 4
    if opts.verbose:
        result.plot(subplots=True, layout=layout, legend=True)
    else:
        result.plot(subplots=False, layout=layout, legend=False)

    #plt.axhline(y=50, color='r')
    #plt.axhline(y=45, color='g')
    #plt.axhline(y=40, color='b')
    cursor(hover=True)

    '''
    mpl.rcParams['font.sans-serif'] = 'WenQuanYi Zen Hei'
    plt.title("0807版本寻库/泊入整体CPU使用情况\n(卡度尼楼下垂直线车位, 003车辆, S.E02.02底软，关闭memtester, 关闭调试线程，关闭gaclog终端实时打印)")
    '''
    plt.show()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("usage: plot_csv.py cpu.txt")
    else:
        plot(sys.argv[1])
