#!/bin/bash
[ ! -d "toolchain" ] && echo  "installing toolchain..." && bash init_clang.sh

# Basic
export KBUILD_BUILD_USER=Samw662
export KBUILD_BUILD_HOST=DaturaStramonium
PATH=$PWD/toolchain/bin:$PATH
export LLVM_DIR=$PWD/toolchain/bin
export LLVM=1
export AnyKernel3=AnyKernel3
export TIME="$(date "+%d-%m-%Y-%H:%M:%S")"
export modpath=${AnyKernel3}/modules/vendor/lib/modules
export ARCH=arm64

# Export bangkk device
if [ -z "$DEVICE" ]; then
export DEVICE=bangkk
fi
rm -rf out

ARGS='
CC=clang
LD='${LLVM_DIR}/ld.lld'
ARCH=arm64
AR='${LLVM_DIR}/llvm-ar'
NM='${LLVM_DIR}/llvm-nm'
AS='${LLVM_DIR}/llvm-as'
OBJCOPY='${LLVM_DIR}/llvm-objcopy'
OBJDUMP='${LLVM_DIR}/llvm-objdump'
READELF='${LLVM_DIR}/llvm-readelf'
OBJSIZE='${LLVM_DIR}/llvm-size'
STRIP='${LLVM_DIR}/llvm-strip'
LLVM_AR='${LLVM_DIR}/llvm-ar'
LLVM_DIS='${LLVM_DIR}/llvm-dis'
LLVM_NM='${LLVM_DIR}/llvm-nm'
LLVM=1
'

make ${ARGS} O=out ${DEVICE}_defconfig moto.config -j$(nproc --all)
make ${ARGS} O=out -j$(nproc --all)

[ ! -e "out/arch/arm64/boot/Image" ] && \
echo "  ERROR : image binary not found in any of the specified locations , fix compile!" && \
exit 1

make O=out ${ARGS} -j$(nproc --all) INSTALL_MOD_PATH=modules INSTALL_MOD_STRIP=1 modules_install

#Clean Up
rm -rf ${modpath}/*
rm -rf ${AnyKernel3}/{Image, dtb, dtbo.img}
rm -rf ${AnyKernel3}/*.zip

#Setup
mkdir -p ${modpath}
kver=$(make kernelversion)
kmod=$(echo ${kver} | awk -F'.' '{print $3}')

#Copy stuff
cp out/.config ${AnyKernel3}/config
cp out/arch/arm64/boot/Image ${AnyKernel3}/Image
cp out/arch/arm64/boot/dtb.img ${AnyKernel3}/dtb
cp out/arch/arm64/boot/dtbo.img ${AnyKernel3}/dtbo.img
#cp build.sta/${DEVICE}_modules.blocklist ${modpath}/modules.blocklist
cp $(find out/modules/lib/modules/5.4* -name '*.ko') ${modpath}/
cp out/modules/lib/modules/5.4*/modules.{alias,dep,softdep} ${modpath}/
cp out/modules/lib/modules/5.4*/modules.order ${modpath}/modules.load

#Edit
sed -i 's/\(kernel\/[^: ]*\/\)\([^: ]*\.ko\)/\/vendor\/lib\/modules\/\2/g' ${modpath}/modules.dep
#sed -i 's/.*\//.ko/g' ${AnyKernel3}/modules/vendor/lib/modules/modules.load
#sed -i 's#.*/##; s/\.ko$//' ${AnyKernel3}/modules/vendor/lib/modules/modules.load
sed -i 's/.*\///; s/\.ko$//' ${modpath}/modules.load

#Zip
cd ${AnyKernel3}
zip -r9 WearyStars-R1+_${DEVICE}^${KBUILD_BUILD_USER}-${TIME}.zip * -x .git README.md *placeholder
