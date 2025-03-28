package com.ssafy.mmmr.weather.client;

import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.cache.annotation.Cacheable;
import org.springframework.http.HttpEntity;
import org.springframework.http.HttpHeaders;
import org.springframework.http.HttpMethod;
import org.springframework.http.ResponseEntity;
import org.springframework.stereotype.Component;
import org.springframework.web.client.RestTemplate;
import org.springframework.web.util.UriComponentsBuilder;

import java.net.URI;

import com.ssafy.mmmr.global.error.code.ErrorCode;
import com.ssafy.mmmr.global.error.exception.GeocoderException;

@Component
@RequiredArgsConstructor
public class GeocoderClient {

	private final RestTemplate weatherRestTemplate;

	@Value("${vworld.api.key}")
	private String apiKey;

	@Value("${vworld.api.geocoder.url}")
	private String geocoderUrl;

	@Cacheable(value = "geocoderCache", key = "#address")
	public String geocodeAddress(String address) {
		try {
			// 주소 전처리 (공백 제거, URL 인코딩 문제 해결)
			String processedAddress = address.trim();

			URI uri = UriComponentsBuilder.fromHttpUrl(geocoderUrl)
				.queryParam("service", "address")
				.queryParam("request", "getcoord")
				.queryParam("version", "2.0")
				.queryParam("crs", "epsg:4326")
				.queryParam("address", processedAddress)
				.queryParam("refine", "true")
				.queryParam("simple", "false")
				.queryParam("format", "json")
				.queryParam("type", "road")
				.queryParam("key", apiKey)
				.build()
				.encode() // URL 인코딩 적용
				.toUri();

			HttpHeaders headers = new HttpHeaders();
			HttpEntity<String> entity = new HttpEntity<>(headers);
			ResponseEntity<String> response = weatherRestTemplate.exchange(uri, HttpMethod.GET, entity, String.class);

			if (response.getBody() == null) {
				throw new GeocoderException(ErrorCode.FAIL_TO_CONVERT);
			}

			return response.getBody();
		} catch (Exception e) {
			throw new GeocoderException(ErrorCode.FAIL_TO_CONVERT);
		}
	}
}