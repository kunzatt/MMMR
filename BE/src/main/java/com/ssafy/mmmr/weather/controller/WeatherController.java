package com.ssafy.mmmr.weather.controller;

import com.ssafy.mmmr.global.error.dto.ErrorResponse;
import com.ssafy.mmmr.global.response.ApiResponse;
import com.ssafy.mmmr.weather.dto.WeatherRequestDto;
import com.ssafy.mmmr.weather.dto.WeatherResponseDto;
import com.ssafy.mmmr.weather.service.WeatherService;
import io.swagger.v3.oas.annotations.Operation;
import io.swagger.v3.oas.annotations.Parameter;
import io.swagger.v3.oas.annotations.media.Content;
import io.swagger.v3.oas.annotations.media.ExampleObject;
import io.swagger.v3.oas.annotations.media.Schema;
import io.swagger.v3.oas.annotations.responses.ApiResponses;
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
	@ApiResponses({
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "200",
			description = "날씨 정보 조회 성공",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ApiResponse.class),
				examples = @ExampleObject(value = """
					{
						"message": "날씨 정보를 성공적으로 조회했습니다.",
						"data": {
							"temperature": 23.5,
							"humidity": 60,
							"weatherDescription": "맑음",
							"clothingRecommendation": "가벼운 긴팔과 바지가 적당합니다.",
							"umbrellaNeeded": false,
							"currentDateTime": "2024-03-20T15:30:00"
						}
					}
					""")
			)
		),
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "400",
			description = "잘못된 요청",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ErrorResponse.class),
				examples = @ExampleObject(value = """
					{
						"timestamp": "2024-03-20T10:00:00",
						"status": 400,
						"message": "주소 정보가 올바르지 않습니다",
						"errors": []
					}
					""")
			)
		),
		@io.swagger.v3.oas.annotations.responses.ApiResponse(
			responseCode = "500",
			description = "서버 오류",
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = ErrorResponse.class),
				examples = @ExampleObject(value = """
					{
						"timestamp": "2024-03-20T10:00:00",
						"status": 500,
						"message": "날씨 정보를 가져오는 중 오류가 발생했습니다",
						"errors": []
					}
					""")
			)
		)
	})
	@PostMapping
	public ResponseEntity<ApiResponse> getWeather(
		@Parameter(description = "날씨 정보 요청 데이터", required = true)
		@io.swagger.v3.oas.annotations.parameters.RequestBody(
			description = "날씨 정보를 요청할 주소",
			required = true,
			content = @Content(
				mediaType = "application/json",
				schema = @Schema(implementation = WeatherRequestDto.class),
				examples = @ExampleObject(
					value = """
						{
							"address": "서울특별시 강남구 테헤란로 212"
						}
						"""
				)
			)
		)
		@RequestBody WeatherRequestDto request) {
		WeatherResponseDto response = weatherService.getComprehensiveWeather(request.getAddress());
		return ResponseEntity.ok(new ApiResponse("날씨 정보를 성공적으로 조회했습니다.", response));
	}
}