FROM openjdk:17-alpine
WORKDIR /app
COPY ./build/libs/*SNAPSHOT.jar /app.jar
EXPOSE 8088
ENTRYPOINT ["java", "-jar", "app.jar"]