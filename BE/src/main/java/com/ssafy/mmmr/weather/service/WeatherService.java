package com.ssafy.mmmr.weather.service;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.ssafy.mmmr.global.error.code.ErrorCode;
import com.ssafy.mmmr.global.error.exception.GeocoderException;
import com.ssafy.mmmr.global.error.exception.WeatherException;
import com.ssafy.mmmr.weather.client.GeocoderClient;
import com.ssafy.mmmr.weather.client.WeatherClient;
import com.ssafy.mmmr.weather.dto.WeatherResponseDto;

import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;

import org.springframework.stereotype.Service;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

@Slf4j
@Service
@RequiredArgsConstructor
public class WeatherService {

	private final WeatherClient weatherClient;
	private final GeocoderClient geocoderClient;
	private final ObjectMapper objectMapper;

	public WeatherResponseDto getComprehensiveWeather(String address) {
		try {
			// 1. 주소를 좌표로 변환
			Map<String, Object> addressInfo = getLocationInfoFromAddress(address);
			double latitude = (double)addressInfo.get("latitude");
			double longitude = (double)addressInfo.get("longitude");

			// 2. 현재 날씨 정보 조회
			String currentWeatherResponse = weatherClient.getCurrentWeather(latitude, longitude);
			JsonNode currentWeatherJson = objectMapper.readTree(currentWeatherResponse);

			// 3. 일일 예보 정보 조회 (최저/최고 기온)
			String forecastResponse = weatherClient.getDailyForecast(latitude, longitude);
			JsonNode forecastJson = objectMapper.readTree(forecastResponse);

			// 4. 현재 날씨 상태 및 온도 추출
			String weatherStatus = extractWeatherStatus(currentWeatherJson);
			int currentTemp = (int)Math.round(extractCurrentTemperature(currentWeatherJson));
			int humidity = extractHumidity(currentWeatherJson);
			int precipitation = calculatePrecipitation(currentWeatherJson, forecastJson);

			// 5. 최저/최고 기온 추출
			int minTemp = (int)Math.round(extractMinTemperature(forecastJson));
			int maxTemp = (int)Math.round(extractMaxTemperature(forecastJson));

			// 6. 온도 논리적 일관성 확인 및 수정
			// 최고 기온이 현재 기온보다 낮으면 현재 기온으로 설정
			if (maxTemp < currentTemp) {
				maxTemp = currentTemp;
			}
			// 최저 기온이 현재 기온보다 높으면 현재 기온으로 설정
			if (minTemp > currentTemp) {
				minTemp = currentTemp;
			}

			// 7. 옷차림 추천 및 우산 필요 여부 결정
			String clothingAdvice = recommendClothing(minTemp, maxTemp, weatherStatus);
			String umbrellaAdvice = recommendUmbrella(weatherStatus, precipitation);

			// 8. 응답 구성
			return WeatherResponseDto.builder()
				.currentWeather(weatherStatus)
				.currentTemperature(currentTemp)
				.minTemperature(minTemp)
				.maxTemperature(maxTemp)
				.humidity(humidity)
				.precipitation(precipitation)
				.clothingAdvice(clothingAdvice)
				.umbrellaAdvice(umbrellaAdvice)
				.build();

		} catch (Exception e) {
			log.error("OpenWeatherMap 날씨 정보 조회 중 오류 발생", e);
			throw new WeatherException(ErrorCode.FAIL_GETTING_WEATHER);
		}
	}

	private Map<String, Object> getLocationInfoFromAddress(String address) throws IOException {
		String geocoderResponse = geocoderClient.geocodeAddress(address);
		JsonNode geocoderJson = objectMapper.readTree(geocoderResponse);

		// 응답 상태 확인
		if (!geocoderJson.path("response").path("status").asText().equals("OK")) {
			throw new GeocoderException(ErrorCode.FAIL_TO_CONVERT);
		}

		Map<String, Object> result = new HashMap<>();

		// 위도, 경도 추출
		JsonNode point = geocoderJson.path("response").path("result").path("point");
		double longitude = Double.parseDouble(point.path("x").asText());
		double latitude = Double.parseDouble(point.path("y").asText());

		result.put("longitude", longitude);
		result.put("latitude", latitude);

		return result;
	}

	private String extractWeatherStatus(JsonNode weatherJson) {
		try {
			JsonNode weatherArray = weatherJson.path("weather");
			JsonNode firstWeather = weatherArray.get(0);
			String description = firstWeather.path("description").asText().toLowerCase();
			String main = firstWeather.path("main").asText().toLowerCase();

			// OpenWeatherMap 날씨 상태를 7가지 기본 상태로 매핑
			if (main.contains("thunderstorm") || description.contains("thunderstorm") ||
				description.contains("tornado") || description.contains("hurricane") ||
				description.contains("storm")) {
				return "태풍";
			}

			if (main.contains("rain") || main.contains("drizzle") ||
				description.contains("rain") || description.contains("drizzle") ||
				description.contains("shower")) {
				return "비";
			}

			if (main.contains("snow") || description.contains("snow") ||
				description.contains("sleet")) {
				return "눈";
			}

			if (description.contains("wind") || description.contains("breeze") ||
				description.contains("gale") || description.contains("squall")) {
				return "바람";
			}

			if (description.contains("overcast") || main.contains("fog") ||
				main.contains("mist") || main.contains("smoke") ||
				main.contains("haze") || description.contains("fog")) {
				return "흐림";
			}

			if (main.contains("clouds") || description.contains("clouds") ||
				description.contains("few clouds") || description.contains("scattered clouds") ||
				description.contains("broken clouds")) {
				return "약간 흐림";
			}

			return "맑음"; // 기본값
		} catch (Exception e) {
			log.warn("날씨 상태 추출 실패", e);
			return "맑음"; // 오류 발생 시 기본값
		}
	}

	private int extractCurrentTemperature(JsonNode weatherJson) {
		try {
			return weatherJson.path("main").path("temp").asInt();
		} catch (Exception e) {
			log.warn("현재 온도 추출 실패", e);
			return 0;
		}
	}

	private int extractHumidity(JsonNode weatherJson) {
		try {
			return weatherJson.path("main").path("humidity").asInt();
		} catch (Exception e) {
			log.warn("습도 추출 실패", e);
			return 0;
		}
	}

	private int calculatePrecipitation(JsonNode currentWeatherJson, JsonNode forecastJson) {
		try {
			// 현재 날씨에서 비/눈 등 강수 여부 확인
			JsonNode weatherArray = currentWeatherJson.path("weather");
			if (weatherArray.isArray() && weatherArray.size() > 0) {
				String mainWeather = weatherArray.get(0).path("main").asText();

				// 비나 눈이 이미 내리고 있으면 100% 반환
				if (mainWeather.equals("Rain") || mainWeather.equals("Snow") ||
					mainWeather.equals("Drizzle") || mainWeather.equals("Thunderstorm")) {
					return 100;
				}
			}

			// 예보 데이터에서 강수 확률 확인 (3시간 이내의 데이터만 고려)
			JsonNode forecastList = forecastJson.path("list");
			if (forecastList.isArray() && forecastList.size() > 0) {
				int maxPop = 0;

				// 첫 2개 항목(약 6시간)에서 최대 강수 확률 찾기
				int count = Math.min(2, forecastList.size());
				for (int i = 0; i < count; i++) {
					JsonNode forecast = forecastList.get(i);
					int pop = forecast.path("pop").asInt();
					if (pop > maxPop) {
						maxPop = pop;
					}
				}

				// 0~1 사이의 값을 백분율로 변환
				return (int)(maxPop * 100);
			}

			return 0;
		} catch (Exception e) {
			log.warn("강수 확률 계산 실패", e);
			return 0;
		}
	}

	private int extractMinTemperature(JsonNode forecastJson) {
		try {
			int minTemp = 100; // 초기값을 높은 값으로 설정

			JsonNode forecastList = forecastJson.path("list");
			if (forecastList.isArray()) {
				// 오늘의 예보만 확인 (약 24시간)
				int count = Math.min(8, forecastList.size());
				for (int i = 0; i < count; i++) {
					JsonNode forecast = forecastList.get(i);
					int temp = forecast.path("main").path("temp_min").asInt();
					if (temp < minTemp) {
						minTemp = temp;
					}
				}
			}

			return minTemp == 100 ? 0 : minTemp;
		} catch (Exception e) {
			log.warn("최저 온도 추출 실패", e);
			return 0;
		}
	}

	private int extractMaxTemperature(JsonNode forecastJson) {
		try {
			int maxTemp = -100; // 초기값을 낮은 값으로 설정

			JsonNode forecastList = forecastJson.path("list");
			if (forecastList.isArray()) {
				// 오늘의 예보만 확인 (약 24시간)
				int count = Math.min(8, forecastList.size());
				for (int i = 0; i < count; i++) {
					JsonNode forecast = forecastList.get(i);
					int temp = forecast.path("main").path("temp_max").asInt();
					if (temp > maxTemp) {
						maxTemp = temp;
					}
				}
			}

			return maxTemp == -100 ? 0 : maxTemp;
		} catch (Exception e) {
			log.warn("최고 온도 추출 실패", e);
			return 0;
		}
	}

	private String recommendClothing(int minTemp, int maxTemp, String weatherStatus) {
		// 최고 온도를 기준으로 옷차림 추천
		int highTemp = maxTemp;

		// 일교차 계산
		int tempRange = maxTemp - minTemp;
		boolean isLargeTempRange = tempRange >= 8;

		// 기본 온도 기반 추천
		String recommendation;

		if (highTemp >= 24) {
			recommendation = "더운 날씨입니다. 반팔, 반바지, 민소매 등 얇고 시원한 옷차림을 추천합니다.";
			if (isLargeTempRange) {
				recommendation += " 일교차가 " + tempRange + "℃로 크니 얇은 겉옷을 하나 챙기세요.";
			}
		} else if (highTemp >= 17) {
			recommendation = "따뜻한 날씨입니다. 얇은 긴팔, 면바지, 가디건 등을 추천합니다.";
			if (isLargeTempRange) {
				recommendation += " 아침저녁으로는 쌀쌀할 수 있으니 겉옷을 준비하세요.";
			}
		} else if (highTemp >= 12) {
			recommendation = "선선한 날씨입니다. 가디건, 맨투맨, 청바지, 얇은 자켓을 추천합니다.";
			if (isLargeTempRange) {
				recommendation += " 일교차가 크니 겹쳐입기를 권장합니다.";
			}
		} else if (highTemp >= 5) {
			recommendation = "쌀쌀한 날씨입니다. 코트, 니트, 기모 소재 옷, 목도리를 추천합니다.";
			if (isLargeTempRange) {
				recommendation += " 체온 유지를 위해 따뜻한 내의를 입는 것이 좋습니다.";
			}
		} else {
			recommendation = "추운 날씨입니다. 패딩, 두꺼운 코트, 장갑, 목도리, 모자 등 보온에 신경쓰세요. 여러 겹 껴입기로 추위를 대비하세요.";
		}

		// 날씨 상태에 따른 추가 권장사항
		String weatherAdvice = switch (weatherStatus) {
			case "비" -> " 비가 오니 방수 기능이 있는 아우터와 신발을 착용하고, 우산을 꼭 챙기세요.";
			case "눈" -> " 눈이 오니 방한용 방수 신발과 따뜻한 겨울용 아우터를 준비하세요.";
			case "태풍" -> " 태풍이 예상되니 외출을 자제하고, 꼭 나가야 한다면 방수 의류와 견고한 신발을 착용하세요.";
			case "흐림" -> highTemp <= 10 ? " 흐린 날씨는 체감온도가 더 낮을 수 있으니 평소보다 따뜻하게 입으세요." : "";
			case "바람" -> " 바람이 강하니 날림이 적은 밀착된 의류와 모자를 착용하세요.";
			default -> "";
		};

		return recommendation + weatherAdvice;
	}

	private String recommendUmbrella(String weatherStatus, int precipitation) {
		// 날씨 상태별 권고사항
		return switch (weatherStatus) {
			case "비" -> precipitation >= 80
				? "강한 비가 예상됩니다. 우산만으로는 비를 피하기 어려울 수 있으니, 가능하면 외출을 자제하고 방수 의류와 신발을 착용하세요."
				: "비가 내리고 있습니다. 우산을 필수로 챙기고, 미끄러운 길을 조심하세요.";
			case "눈" -> "눈이 내리고 있습니다. 우산보다는 방수 기능이 있는 모자와 아우터를 준비하고, 미끄럼 방지 신발을 착용하세요.";
			case "태풍" -> "태풍이 예상됩니다. 가급적 외출을 자제하고, 외출시에는 우산 대신 방수 의류를 착용하세요.";
			default -> {
				String advice;
				if (precipitation >= 70) {
					advice = "강수 확률이 " + precipitation + "%로 매우 높습니다. 우산을 반드시 챙기고, 방수가 되는 신발을 착용하는 것이 좋습니다.";
				} else if (precipitation >= 50) {
					advice = "강수 확률이 " + precipitation + "%로 높은 편입니다. 우산을 챙기는 것이 좋겠습니다.";
				} else if (precipitation >= 30) {
					advice = "강수 확률이 " + precipitation + "%입니다. 접이식 우산을 가방에 넣어두는 것을 권장합니다.";
				} else if (precipitation >= 20 && (weatherStatus.equals("흐림") || weatherStatus.equals("약간 흐림"))) {
					advice = "하늘이 흐리고 약간의 비 가능성이 있습니다. 혹시 모르니 작은 우산을 준비해두세요.";
				} else {
					advice = "오늘은 우산이 필요 없을 것 같습니다. 편안한 하루 되세요!";
				}
				yield advice;
			}
		};
	}
}