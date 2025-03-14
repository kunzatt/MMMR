package com.ssafy.mmmr;

import org.springframework.batch.core.configuration.annotation.EnableBatchProcessing;
import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;

@SpringBootApplication
@EnableBatchProcessing
public class MmmrApplication {

	public static void main(String[] args) {
		SpringApplication.run(MmmrApplication.class, args);
	}

}
