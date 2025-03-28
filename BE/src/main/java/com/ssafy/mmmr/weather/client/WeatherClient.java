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

import java.io.UnsupportedEncodingException;
import java.net.URI;
import java.net.URLDecoder;
import java.nio.charset.StandardCharsets;
import java.time.LocalDate;
import java.time.LocalDateTime;
import java.time.LocalTime;
import java.time.format.DateTimeFormatter;

import com.ssafy.mmmr.global.error.code.ErrorCode;
import com.ssafy.mmmr.global.error.exception.WeatherException;

@Slf4j
@Component
@RequiredArgsConstructor
public class WeatherClient {

	private final RestTemplate weatherRestTemplate;

	@Value("${weather.api.key}")
	private String apiKey;

	@Value("${weather.api.forecast.url}")
	private String forecastUrl;

	@Value("${weather.api.alert.url}")
	private String alertUrl;

	@Value("${weather.api.typhoon.url}")
	private String typhoonUrl;

	@Cacheable(value = "forecastCache", key = "#nx + '-' + #ny")
	public String getUltraSrtFcst(int nx, int ny) {
		// 현재 시간 기준으로 가장 최근 발표 시각 구하기
		LocalDateTime now = LocalDateTime.now();
		LocalDate today = now.toLocalDate();

		// 매 시간 정각에 생성되는 단기예보 기준시각 계산
		LocalDateTime baseTime;
		int hour = now.getHour();
		int minute = now.getMinute();

		// 정각 발표 후 10분이 지나야 API 데이터가 준비됨
		// 10분이 지나지 않았으면 한 시간 전 데이터 사용
		if (minute < 10) {
			if (hour == 0) {
				// 자정 직후라면 전날 마지막 예보
				baseTime = LocalDateTime.of(today.minusDays(1), LocalTime.of(23, 0));
			} else {
				baseTime = LocalDateTime.of(today, LocalTime.of(hour - 1, 0));
			}
		} else {
			// 그 외에는 현재 시간대 정각 예보 사용
			baseTime = LocalDateTime.of(today, LocalTime.of(hour, 0));
		}

		String baseDate = baseTime.format(DateTimeFormatter.ofPattern("yyyyMMdd"));
		String baseTimeStr = baseTime.format(DateTimeFormatter.ofPattern("HH00"));

		// URL 직접 구성 (인코딩된 키를 그대로 사용)
		String url = forecastUrl + "/getUltraSrtFcst" +
			"?serviceKey=" + apiKey +
			"&numOfRows=60" +
			"&pageNo=1" +
			"&dataType=XML" +
			"&base_date=" + baseDate +
			"&base_time=" + baseTimeStr +
			"&nx=" + nx +
			"&ny=" + ny;

		URI uri = URI.create(url);

		try {
			HttpHeaders headers = new HttpHeaders();
			HttpEntity<String> entity = new HttpEntity<>(headers);

			log.debug("기상청 API 요청 URL: {}", uri.toString());
			ResponseEntity<String> response = weatherRestTemplate.exchange(uri, HttpMethod.GET, entity, String.class);

			// 응답 로깅 추가
			log.debug("기상청 초단기예보 API 응답 상태 코드: {}", response.getStatusCode());
			String responseBody = response.getBody();

			// 응답 내용 로깅
			if (responseBody != null) {
				if (responseBody.length() > 500) {
					log.debug("기상청 API 응답 시작 부분: {}", responseBody.substring(0, 500) + "...");
				} else {
					log.debug("기상청 API 응답: {}", responseBody);
				}
			}

			// API 오류 응답 확인
			if (responseBody != null && responseBody.contains("SERVICE_KEY_IS_NOT_REGISTERED_ERROR")) {
				log.error("API 키 인증 오류: 기상청 API 키가 유효하지 않거나 등록되지 않았습니다.");
				throw new WeatherException(ErrorCode.FAIL_GETTING_WEATHER);
			}

			return responseBody;
		} catch (Exception e) {
			log.error("기상청 초단기예보 API 호출 실패: {}", e.getMessage(), e);
			throw new WeatherException(ErrorCode.FAIL_GETTING_WEATHER);
		}
	}

	@Cacheable(value = "alertCache", key = "#areaCode")
	public String getWthrWrnList(String areaCode) {
		// URL 직접 구성 (인코딩된 키를 그대로 사용)
		String url = alertUrl + "/getWthrWrnList" +
			"?serviceKey=" + apiKey +
			"&numOfRows=10" +
			"&pageNo=1" +
			"&dataType=XML" +
			"&stnId=" + areaCode;

		URI uri = URI.create(url);

		try {
			HttpHeaders headers = new HttpHeaders();
			HttpEntity<String> entity = new HttpEntity<>(headers);

			log.debug("기상 특보 API 요청 URL: {}", uri.toString());
			ResponseEntity<String> response = weatherRestTemplate.exchange(uri, HttpMethod.GET, entity, String.class);

			// 응답 로깅 추가
			log.debug("기상청 특보 API 응답 상태 코드: {}", response.getStatusCode());
			String responseBody = response.getBody();

			// 응답 내용 로깅
			if (responseBody != null) {
				if (responseBody.length() > 500) {
					log.debug("기상 특보 API 응답 시작 부분: {}", responseBody.substring(0, 500) + "...");
				} else {
					log.debug("기상 특보 API 응답: {}", responseBody);
				}
			}

			// API 오류 응답 확인
			if (responseBody != null && responseBody.contains("SERVICE_KEY_IS_NOT_REGISTERED_ERROR")) {
				log.error("API 키 인증 오류: 기상청 API 키가 유효하지 않거나 등록되지 않았습니다.");
				throw new WeatherException(ErrorCode.FAIL_GETTING_URGENT_WEATHER);
			}

			return responseBody;
		} catch (Exception e) {
			log.error("기상청 특보 API 호출 실패: {}", e.getMessage(), e);
			throw new WeatherException(ErrorCode.FAIL_GETTING_URGENT_WEATHER);
		}
	}

	@Cacheable(value = "typhoonCache")
	public String getTyphoonInfo() {
		int currentYear = LocalDate.now().getYear();

		// URL 직접 구성 (인코딩된 키를 그대로 사용)
		String url = typhoonUrl + "/getTyphoonInfoList" +
			"?serviceKey=" + apiKey +
			"&numOfRows=10" +
			"&pageNo=1" +
			"&dataType=XML" +
			"&year=" + currentYear;

		URI uri = URI.create(url);

		try {
			HttpHeaders headers = new HttpHeaders();
			HttpEntity<String> entity = new HttpEntity<>(headers);

			log.debug("태풍 정보 API 요청 URL: {}", uri.toString());
			ResponseEntity<String> response = weatherRestTemplate.exchange(uri, HttpMethod.GET, entity, String.class);

			// 응답 로깅 추가
			log.debug("기상청 태풍 API 응답 상태 코드: {}", response.getStatusCode());
			String responseBody = response.getBody();

			// 응답 내용 로깅
			if (responseBody != null) {
				if (responseBody.length() > 500) {
					log.debug("태풍 정보 API 응답 시작 부분: {}", responseBody.substring(0, 500) + "...");
				} else {
					log.debug("태풍 정보 API 응답: {}", responseBody);
				}
			}

			// API 오류 응답 확인
			if (responseBody != null && responseBody.contains("SERVICE_KEY_IS_NOT_REGISTERED_ERROR")) {
				log.error("API 키 인증 오류: 기상청 API 키가 유효하지 않거나 등록되지 않았습니다.");
				throw new WeatherException(ErrorCode.FAIL_GETTING_TYPHOON);
			}

			return responseBody;
		} catch (Exception e) {
			log.error("기상청 태풍 API 호출 실패: {}", e.getMessage(), e);
			throw new WeatherException(ErrorCode.FAIL_GETTING_TYPHOON);
		}
	}
}