#!/bin/bash
# Copyright 2020-2020, Huawei Technologies Co. Ltd.
#
# ALL RIGHTS RESERVED
#
# Compile mpp/sample project, this is the entrance script

# error out on errors
set -e
OUT_DIR="$1"
BOARD_NAME="$2"
HOS_KERNEL_TYPE="$3"
HOS_BUILD_COMPILER="$4"
COMPILER_DIR=$5

function main(){
    ROOT_DIR=$(cd $(dirname "$0");pwd)

    if [ "$HOS_BUILD_COMPILER" = "gcc" ];then
        cd $ROOT_DIR/../../camera/middleware/source
        ./build.sh $OUT_DIR $BOARD_NAME $HOS_KERNEL_TYPE "gcc"
    else
        cd $ROOT_DIR/source
        ./build.sh $OUT_DIR $BOARD_NAME $HOS_KERNEL_TYPE "llvm" $COMPILER_DIR
    fi
    cd $ROOT_DIR
}

main "$@"
