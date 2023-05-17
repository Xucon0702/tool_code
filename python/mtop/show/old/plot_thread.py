# motovis perf utils mtop
# 2022

import sys
import pandas as pd
import matplotlib.pyplot as plt

def plot(csv_file):
    data = pd.read_csv(csv_file, sep='\s+', names=["TS", "PID", "PRI", "STATE",\
            "HH:MM:SS", "CPU", "LASTCPU", "COMMAND", "THREAD_NAME"])

    '''
    df = pd.read_csv(csv_file, sep='\s+', names=["TS", "PID", "PRI", "STATE",\
            "HH:MM:SS", "CPU", "LASTCPU", "COMMAND", "THREAD_NAME"])
    '''

    data['CPU'] = data['CPU'].str.rstrip('%').astype('float')
    result = data.groupby(['TS', 'THREAD_NAME'])['CPU'].sum().unstack()

    result.plot(subplots=True)
    plt.legend(loc='best')
    plt.show()

if __name__ == "__main__":
    if (len(sys.argv) != 2):
        print("usage: plot_csv.py data.csv")
    else:
        plot(sys.argv[1])
