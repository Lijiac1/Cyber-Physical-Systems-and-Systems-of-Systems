##################################################
# Section 1: Build the application
FROM ubuntu:18.04 as builder

RUN apt-get update -y && \
    apt-get upgrade -y

RUN apt-get install -y --no-install-recommends \
        cmake \
        build-essential

ADD . /opt/sources
WORKDIR /opt/sources

RUN cd /opt/sources && \
    mkdir build && \
    cd build && \
    cmake -D CMAKE_BUILD_TYPE=Release .. && \
    make && make test && cp helloworld /tmp &&\
    cd CMakeFiles/helloworld-Runner.dir && gcov PrimeChecker.cpp.gcno && cp PrimeChecker.cpp.gcov /tmp
    

    

##################################################
# Section 2: Bundle the application.
FROM ubuntu:18.04

RUN apt-get update -y && \
    apt-get upgrade -y && \
    apt-get dist-upgrade -y

WORKDIR /opt
COPY --from=builder /tmp/helloworld .
COPY --from=builder /tmp/PrimeChecker.cpp.gcov .

RUN cat PrimeChecker.cpp.gcov
ENTRYPOINT ["/opt/helloworld"]
