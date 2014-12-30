#!/bin/sh
export KERNELDIR=`greadlink -f .`
export PARENT_DIR=`greadlink -f ..`
export USE_SEC_FIPS_MODE=true
export CROSS_COMPILE=/usr/tools/arm-linux-eabi/bin/arm-none-linux-gnueabi-
export PATH=/usr/local/bin:$PATH

if [ "${1}" != "" ];then
  export KERNELDIR=`greadlink -f ${1}`
fi

if [ ! -f $KERNELDIR/.config ];
then
  make -j4 x4_defconfig
fi

. $KERNELDIR/.config

export ARCH=arm

cd $KERNELDIR/
make INSTALL_HDR_PATH=../linux headers_install -j4 || exit 1
