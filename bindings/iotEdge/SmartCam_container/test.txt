FROM ubuntu:latest

RUN apt-get update

RUN mkdir -p /home/HELLOWORLD

RUN cp . /home/HELLOWORLD

CMD echo robi >> test.txt
