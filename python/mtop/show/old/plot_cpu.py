# perf utils plot_process.py

import sys
import pandas as pd
import matplotlib as mpl
import matplotlib.pyplot as plt
from mplcursors import cursor
from optparse import OptionParser

WHITE_NAME_MV = [
        'mv_avm',
        'mv_psd',
        'mv_plan',
        'mv_fusion',
        'mv_decision'
        ]

WHITE_NAME_HOLO = [
        'eol_calibration_cmw',
        'hmi_converter_cmw',
        'navigator_parking_cmw',
        'obstacle_fusion_parking_app_cmw',
        'parking_manager_cmw',
        'parkingslot_park_cmw_app',
        'path_optimization_hpp_app',
        'planning_hpp_cmw',
        'renderer_app',
        'srv_freespace_app_cmw',
        'srv_multitask_parking_app_cmw',
        'vision_fd_pps_app_cmw',
        'vision_od_pps_app_cmw',
        'vslam_cmw',
        ]

WHITE_NAME_PLATFORM = [
        'ErrorManagement',
        'IpcAdapterSVC_A58_S.0_0928',
        'avm3d_gui_A58_V1.57',
        'devb-sdmmc-am65x',
        'devc-pty',
        'io-pkt-v4-hc',
        'kernel',
        'pipe',
        'pps',
        'screen',
        'sh',
        'slogger2',
        'sshd',
        'syslogd',
        'tiipc-mgr',
        'uds_app_bootloader',
        'update_service',
        'vx_desay_capture_display_csitx_front_view.out',
        'vx_desay_dispatch.out',
        ]

def sum_thread(df, name):
    # accum cpu info by proc name
    df['CPU'] = df['CPU'].str.rstrip('%').astype('float')
    result = df.groupby(['TS','COMMAND'])['CPU'].sum().unstack()
    result[name] = df.groupby('TS')['CPU'].sum()
    return result


def plot(csv_file):
    # load mtop result
    df = pd.read_csv(csv_file, sep='\s+', parse_dates=['TS'], names=["TS", "PID", "PRI", "STATE",\
            "HH:MM:SS", "CPU", "LASTCPU", "COMMAND", "THREAD_NAME"])

    # filter rows which contain thread info
    df = df[df['TS'].str.match(r'[0-9][0-9]:[0-9][0-9]:[0-9][0-9]')]
    df['TS'] = pd.to_datetime(df['TS'], format='%H:%M:%S')

    #df = df[df['COMMAND'].isin(WHITE_NAME_MV+WHITE_NAME_HOLO)]
    df_platform = df[df['COMMAND'].isin(WHITE_NAME_PLATFORM)]
    df_mv = df[df['COMMAND'].isin(WHITE_NAME_MV)]
    df_holo = df[df['COMMAND'].isin(WHITE_NAME_HOLO)]
    #df = df[df['COMMAND'].isin(WHITE_NAME_AVP)]

    result_total = sum_thread(df, 'total')
    result_mv = sum_thread(df_mv, 'motovis')
    result_holo = sum_thread(df_holo, 'holo')
    result_platform = sum_thread(df_platform, 'platform')

    # change plot layout when needed
    parser = OptionParser()
    parser.add_option('-v', action='store_true', dest='verbose', default=False)
    opts, args = parser.parse_args()
    layout = 10, 6
    if opts.verbose:
        result_total.plot(subplots=True, layout=layout, legend=True)
    else:
        ax = result_mv.plot(subplots=False, layout=layout, legend=False, style='.-')
        result_holo.plot(ax=ax, subplots=False, layout=layout, legend=False, style='.-')
        result_platform.plot(ax=ax, subplots=False, layout=layout, legend=False, style='.-')
        result_total.plot(ax=ax, subplots=False, layout=layout, legend=False, style='.-')

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
