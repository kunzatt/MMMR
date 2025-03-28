package com.ssafy.mmmr.weather.controller;

import com.ssafy.mmmr.weather.dto.WeatherRequestDto;
import com.ssafy.mmmr.weather.dto.WeatherResponseDto;
import com.ssafy.mmmr.weather.service.WeatherService;
import io.swagger.v3.oas.annotations.Operation;
import io.swagger.v3.oas.annotations.tags.Tag;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

@RestController
@RequestMapping("/api/weather")
@RequiredArgsConstructor
@Tag(name = "날씨 API", description = "OpenWeatherMap API를 사용한 날씨 정보, 옷차림 추천 및 우산 필요 여부 관련 API")
public class WeatherController {

	private final WeatherService weatherService;

	@Operation(
		summary = "종합 날씨 정보 조회",
		description = "주소를 기반으로 OpenWeatherMap API를 사용하여 현재 날씨, 기온, 습도, 옷차림 추천, 우산 필요 여부 정보를 제공합니다."
	)
	@PostMapping
	public ResponseEntity<WeatherResponseDto> getWeather(@RequestBody WeatherRequestDto request) {
		WeatherResponseDto response = weatherService.getComprehensiveWeather(request.getAddress());
		return ResponseEntity.ok(response);
	}
}