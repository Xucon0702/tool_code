附件是mtop的源码(基于openqnx top程序修改)，可以从中了解操作系统统计cpu占用的原理
记录 cpu loading 数据方法：
将附件 mtop 工具传到设备端/gac_code/目录下；
单独建ssh连接，执行以下命令：
cd /gac_code/ && chmod a+x mtop  # 只需要第一次拷贝到板端时执行一次
 ./mtop   >/dev/shmem/mtop_log.txt
执行后，cpu loading 数据会持续记录到mtop_log.txt中
在准备测试前，运行以上命令。测试几分钟后，可以导出该数据。
