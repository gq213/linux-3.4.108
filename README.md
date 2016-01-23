# linux-3.4.108
for tq210

编译内核
cp bak.config .config
make ARCH=arm CROSS_COMPILE=arm-none-linux-gnueabi- menuconfig
make ARCH=arm CROSS_COMPILE=arm-none-linux-gnueabi- uImage

编译模块
make ARCH=arm CROSS_COMPILE=arm-none-linux-gnueabi- modules
make ARCH=arm CROSS_COMPILE=arm-none-linux-gnueabi- modules_install INSTALL_MOD_PATH=/home/work/nfs/rootfs

清除
make ARCH=arm CROSS_COMPILE=arm-none-linux-gnueabi- distclean
