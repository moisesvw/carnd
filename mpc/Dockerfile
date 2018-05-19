FROM ubuntu:18.04

COPY docker.sh .
RUN sh docker.sh
RUN apt-get install gcc-6 g++-6 -y
# To Compile project remember use Gcc 6
# cmake -DCMAKE_C_COMPILER="/usr/bin/gcc-6" -DCMAKE_CXX_COMPILER="/usr/bin/g++-6" .. && make