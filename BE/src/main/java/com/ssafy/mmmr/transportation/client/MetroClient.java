package com.ssafy.mmmr.transportation.client;

import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Component;
import org.springframework.web.client.RestTemplate;

import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;

@Component
@RequiredArgsConstructor
@Slf4j
public class MetroClient {
	private final RestTemplate restTemplate;

	@Value("${seoul.metro.api.key}")
	private String apiKey;

	@Value("${seoul.metro.api.url}")
	private String apiUrl;

	// Methods for interacting with the Seoul Metro API will be added here
}