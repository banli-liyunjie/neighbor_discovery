#! /bin/bash

OUTDIR="./out"
if [ -e "${OUTDIR}" ]; then
    rm -rf ${OUTDIR}
fi

mkdir ${OUTDIR}

INCLUDE_PATH="-I ./uav_node -I ./uav_system"

gcc -c ./uav_node/uav_node.cpp -o ${OUTDIR}/uav_node.o
gcc -c ./uav_system/uav_system.cpp -o ${OUTDIR}/uav_system.o ${INCLUDE_PATH}
gcc -c main.cpp -o ${OUTDIR}/main.o ${INCLUDE_PATH}

gcc ${OUTDIR}/uav_node.o ${OUTDIR}/uav_system.o ${OUTDIR}/main.o -o ${OUTDIR}/neighbor_discovery -lstdc++ -lm

