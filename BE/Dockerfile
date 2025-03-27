# /home/ubuntu/jenkins-data/workspace/mmmr-backend-develop/BE/Dockerfile
FROM openjdk:17-alpine
WORKDIR /app
COPY ./build/libs/*SNAPSHOT.jar ./app.jar
EXPOSE 8088
ENTRYPOINT ["sh", "-c", "java -jar app.jar"]

