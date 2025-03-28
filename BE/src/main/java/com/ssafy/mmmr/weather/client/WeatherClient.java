package com.ssafy.mmmr.weather.client;

import lombok.RequiredArgsConstructor;
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
import com.ssafy.mmmr.global.error.exception.WeatherException;

@Component
@RequiredArgsConstructor
public class WeatherClient {

	private final RestTemplate weatherRestTemplate;

	@Value("${openweather.api.key}")
	private String apiKey;

	@Value("${openweather.api.url}")
	private String apiUrl;

	@Cacheable(value = "weatherCache", key = "#latitude + '-' + #longitude")
	public String getCurrentWeather(double latitude, double longitude) {
		URI uri = UriComponentsBuilder.fromHttpUrl(apiUrl + "/weather")
			.queryParam("lat", latitude)
			.queryParam("lon", longitude)
			.queryParam("appid", apiKey)
			.queryParam("units", "metric")
			.queryParam("lang", "en")
			.build()
			.toUri();

		try {
			HttpHeaders headers = new HttpHeaders();
			HttpEntity<String> entity = new HttpEntity<>(headers);

			ResponseEntity<String> response = weatherRestTemplate.exchange(uri, HttpMethod.GET, entity, String.class);

			String responseBody = response.getBody();

			if (responseBody == null) {
				throw new WeatherException(ErrorCode.FAIL_GETTING_WEATHER);
			}

			return responseBody;
		} catch (Exception e) {
			throw new WeatherException(ErrorCode.FAIL_GETTING_WEATHER);
		}
	}

	@Cacheable(value = "forecastCache", key = "#latitude + '-' + #longitude")
	public String getDailyForecast(double latitude, double longitude) {
		URI uri = UriComponentsBuilder.fromHttpUrl(apiUrl + "/forecast")
			.queryParam("lat", latitude)
			.queryParam("lon", longitude)
			.queryParam("appid", apiKey)
			.queryParam("units", "metric")
			.queryParam("lang", "en")
			.queryParam("cnt", 8) // 24시간(3시간 간격 8개)의 데이터만 가져옴
			.build()
			.toUri();

		try {
			HttpHeaders headers = new HttpHeaders();
			HttpEntity<String> entity = new HttpEntity<>(headers);

			ResponseEntity<String> response = weatherRestTemplate.exchange(uri, HttpMethod.GET, entity, String.class);

			String responseBody = response.getBody();

			if (responseBody == null) {
				throw new WeatherException(ErrorCode.FAIL_GETTING_WEATHER);
			}

			return responseBody;
		} catch (Exception e) {
			throw new WeatherException(ErrorCode.FAIL_GETTING_WEATHER);
		}
	}
}