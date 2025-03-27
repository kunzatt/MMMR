# /home/ubuntu/jenkins-data/workspace/Dockerfile

FROM jenkins/jenkins:latest-jdk17

USER root

RUN apt-get update &&\
    apt-get upgrade -y &&\
    apt-get install -y openssh-client docker.io && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*
    
    # Docker Compose 설치 (공식 방식)
RUN curl -L "https://github.com/docker/compose/releases/download/v2.24.5/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose \
    && chmod +x /usr/local/bin/docker-compose \
    && ln -s /usr/local/bin/docker-compose /usr/bin/docker-compose
    
# 도커 그룹에 jenkins 유저 추가
RUN usermod -aG docker jenkins

USER jenkins
