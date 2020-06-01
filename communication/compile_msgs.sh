#!/bin/bash
SRC_DIR="./protos"
DST_DIR="./msgs"

protoc -I=$SRC_DIR --cpp_out=$DST_DIR $SRC_DIR/imagem.proto
protoc -I=$SRC_DIR --cpp_out=$DST_DIR $SRC_DIR/nuvem.proto
protoc -I=$SRC_DIR --cpp_out=$DST_DIR $SRC_DIR/arquivos.proto
protoc -I=$SRC_DIR --cpp_out=$DST_DIR $SRC_DIR/nvm.proto


