# linux-3.4.108
for tq210

编译内核
cp bak.config .config
make ARCH=arm CROSS_COMPILE=arm-buildroot-linux-gnueabihf- menuconfig
make ARCH=arm CROSS_COMPILE=arm-buildroot-linux-gnueabihf- uImage

编译模块
make ARCH=arm CROSS_COMPILE=arm-buildroot-linux-gnueabihf- modules
make ARCH=arm CROSS_COMPILE=arm-buildroot-linux-gnueabihf- modules_install INSTALL_MOD_PATH=/mnt/qiang/work/210/nfs/rootfs

清除
make ARCH=arm CROSS_COMPILE=arm-buildroot-linux-gnueabihf- distclean
