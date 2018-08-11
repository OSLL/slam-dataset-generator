#! /bin/bash

REQUIRED_PACKAGES="git cmake g++ libfltk1.1-dev libjpeg8-dev libpng12-dev libglu1-mesa-dev libltdl-dev"
PACKAGES_TO_INSTALL=""

read -ra PACKAGES <<< $REQUIRED_PACKAGES
for pkg in "${PACKAGES[@]}"; do
  if ! dpkg-query -W -f'${Status}' "$pkg" 2>/dev/null | grep -q "ok installed"; then
    PACKAGES_TO_INSTALL=$PACKAGES_TO_INSTALL" $pkg"
  fi
done

if [[ ! -z $PACKAGES_TO_INSTALL ]]; then
  echo "Installing required packages ..."
  sudo apt-get install $PACKAGES_TO_INSTALL
fi

if [ ! -d Stage ]; then
  git clone https://github.com/rtv/Stage.git Stage
  if [ ! -d Stage ]; then
    echo "Error: unable to clone Stage"
    exit -1
  fi
fi

cd Stage

git reset --hard
git checkout 731fc69
git apply ../stage.patch

if [ -d build ]; then
  rm -rf build
fi

mkdir build
cd build

if [ ! -d ../../install ]; then
  mkdir ../../install
fi

INSTALL_DIR=`cd ../../install; pwd`

CXX=g++-5 CC=gcc-5 cmake -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR" ..
make && make install
