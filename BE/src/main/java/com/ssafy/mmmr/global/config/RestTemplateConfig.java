package com.ssafy.mmmr.global.config;

import java.time.Duration;

import org.springframework.boot.web.client.RestTemplateBuilder;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import org.springframework.http.client.SimpleClientHttpRequestFactory;
import org.springframework.web.client.RestTemplate;

@Configuration
public class RestTemplateConfig {

	@Bean
	public RestTemplate restTemplate() {
		SimpleClientHttpRequestFactory factory = new SimpleClientHttpRequestFactory();
		factory.setConnectTimeout(5000);
		factory.setReadTimeout(5000);
		return new RestTemplate(factory);
	}

	@Bean
	public RestTemplate weatherRestTemplate(RestTemplateBuilder builder) {
		return builder
			.setConnectTimeout(Duration.ofMillis(5000))
			.setReadTimeout(Duration.ofMillis(5000))
			.build();
	}
}
