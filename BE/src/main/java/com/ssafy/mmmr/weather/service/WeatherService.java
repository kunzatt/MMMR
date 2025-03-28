package com.ssafy.mmmr.weather.service;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.ssafy.mmmr.global.error.code.ErrorCode;
import com.ssafy.mmmr.global.error.exception.GeocoderException;
import com.ssafy.mmmr.global.error.exception.WeatherException;
import com.ssafy.mmmr.weather.client.GeocoderClient;
import com.ssafy.mmmr.weather.client.WeatherClient;
import com.ssafy.mmmr.weather.dto.WeatherResponseDto;
import com.ssafy.mmmr.weather.util.CoordinateConverter;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.stereotype.Service;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;
import org.xml.sax.InputSource;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import java.io.StringReader;
import java.time.LocalDate;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.HashMap;
import java.util.Map;

@Slf4j
@Service
@RequiredArgsConstructor
public class WeatherService {

	private final WeatherClient weatherClient;
	private final GeocoderClient geocoderClient;
	private final CoordinateConverter coordinateConverter;
	private final ObjectMapper objectMapper;

	/**
	 * 종합 날씨 정보를 제공하는 메서드
	 * @param address 날씨를 요청할 주소
	 * @return 날씨 정보를 포함한 응답 DTO
	 */
	public WeatherResponseDto getComprehensiveWeather(String address) {
		try {
			// 1. 주소를 좌표로 변환
			double[] coordinates = getCoordinatesFromAddress(address);
			double latitude = coordinates[0];
			double longitude = coordinates[1];

			// 2. 기상청 좌표로 변환
			int[] xy = coordinateConverter.convertToXY(latitude, longitude);
			int nx = xy[0];
			int ny = xy[1];

			// 3. 날씨 정보 조회
			Map<String, Object> weatherData = getWeatherData(nx, ny);

			// 4. 특보 정보 조회 (주소의 행정구역 코드를 사용해야 하나, 현재는 서울 코드로 대체)
			String areaCode = "108"; // 기본값: 서울
			boolean hasAlert = false;
			String alertMessage = "";

			try {
				Map<String, Object> alertData = getAlertData(areaCode);
				hasAlert = (boolean) alertData.get("hasAlert");
				alertMessage = (String) alertData.get("message");
			} catch (Exception e) {
				log.error("특보 정보 조회 실패", e);
				// 특보 정보 조회 실패해도 다른 정보는 계속 제공
			}

			// 5. 태풍 정보 조회
			boolean hasTyphoon = false;
			String typhoonMessage = "";

			try {
				Map<String, Object> typhoonData = getTyphoonData();
				hasTyphoon = (boolean) typhoonData.get("hasTyphoon");
				typhoonMessage = (String) typhoonData.get("message");
			} catch (Exception e) {
				log.error("태풍 정보 조회 실패", e);
				// 태풍 정보 조회 실패해도 다른 정보는 계속 제공
			}

			// 6. 종합 정보로 응답 구성
			double currentTemp = (double) weatherData.get("currentTemperature");
			double minTemp = (double) weatherData.get("minTemperature");
			double maxTemp = (double) weatherData.get("maxTemperature");
			String weatherStatus = (String) weatherData.get("weatherStatus");
			int humidity = (int) weatherData.get("humidity");
			int precipitation = (int) weatherData.get("precipitation");

			// 날씨 상태와 강수 확률 간의 일관성 확인 및 보정
			if (weatherStatus.contains("비") || weatherStatus.contains("소나기") || weatherStatus.contains("눈")) {
				// 비/눈 형태의 날씨인데 강수확률이 낮으면 보정
				if (precipitation < 30) {
					precipitation = Math.max(60, precipitation);
				}
			}

			// 7. 옷차림 추천 및 우산 필요 여부 결정
			String clothingAdvice = recommendClothing(currentTemp, minTemp, maxTemp, weatherStatus);
			String umbrellaAdvice = recommendUmbrella(weatherStatus, precipitation, hasAlert, hasTyphoon);

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
				.hasAlert(hasAlert)
				.alertMessage(alertMessage)
				.hasTyphoon(hasTyphoon)
				.typhoonMessage(typhoonMessage)
				.build();

		} catch (Exception e) {
			log.error("날씨 정보 조회 중 오류 발생", e);
			throw new WeatherException(ErrorCode.FAIL_GETTING_WEATHER);
		}
	}

	/**
	 * 주소를 위도/경도 좌표로 변환하는 메서드
	 */
	private double[] getCoordinatesFromAddress(String address) throws JsonProcessingException {
		String geocoderResponse = geocoderClient.geocodeAddress(address);
		JsonNode geocoderJson = objectMapper.readTree(geocoderResponse);

		// 응답 상태 확인
		if (!geocoderJson.path("response").path("status").asText().equals("OK")) {
			throw new GeocoderException(ErrorCode.FAIL_TO_CONVERT);
		}

		// 위도, 경도 추출
		JsonNode point = geocoderJson.path("response").path("result").path("point");
		double longitude = Double.parseDouble(point.path("x").asText());
		double latitude = Double.parseDouble(point.path("y").asText());

		return new double[]{latitude, longitude};
	}

	/**
	 * 기상청 API를 호출하여 날씨 데이터를 가져오는 메서드
	 */
	private Map<String, Object> getWeatherData(int nx, int ny) throws Exception {
		String weatherResponse = weatherClient.getUltraSrtFcst(nx, ny);

		// XML 파싱
		DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
		DocumentBuilder builder = factory.newDocumentBuilder();
		Document document = builder.parse(new InputSource(new StringReader(weatherResponse)));

		Map<String, Object> result = new HashMap<>();
		Map<String, String> rawData = new HashMap<>();

		// 기본값 설정
		rawData.put("SKY", "1");  // 하늘상태 (1:맑음, 3:구름많음, 4:흐림)
		rawData.put("PTY", "0");  // 강수형태 (0:없음, 1:비, 2:비/눈, 3:눈, 4:소나기)
		rawData.put("T1H", "0");  // 현재기온
		rawData.put("REH", "0");  // 습도
		rawData.put("POP", "0");  // 강수확률
		rawData.put("TMN", "0");  // 최저기온
		rawData.put("TMX", "0");  // 최고기온

		// 현재 날짜 시간
		LocalDateTime now = LocalDateTime.now();
		LocalDate today = now.toLocalDate();
		String formattedToday = today.format(DateTimeFormatter.ofPattern("yyyyMMdd"));

		// 최저/최고 기온을 위한 임시 변수
		int minTemp = Integer.MAX_VALUE;
		int maxTemp = Integer.MIN_VALUE;

		// 응답 상태 확인
		NodeList resultCodeNodes = document.getElementsByTagName("resultCode");
		if (resultCodeNodes.getLength() == 0 || !"00".equals(resultCodeNodes.item(0).getTextContent())) {
			log.error("기상청 API 오류 응답: {}", weatherResponse.substring(0, Math.min(weatherResponse.length(), 500)));
			throw new WeatherException(ErrorCode.FAIL_GETTING_WEATHER);
		}

		// XML에서 데이터 추출
		NodeList items = document.getElementsByTagName("item");
		for (int i = 0; i < items.getLength(); i++) {
			Element item = (Element) items.item(i);
			String category = getElementTextContent(item, "category");
			String fcstDate = getElementTextContent(item, "fcstDate");
			String fcstValue = getElementTextContent(item, "fcstValue");

			// 오늘 날짜의 데이터만 처리
			if (formattedToday.equals(fcstDate)) {
				switch (category) {
					case "SKY": rawData.put("SKY", fcstValue); break;
					case "PTY": rawData.put("PTY", fcstValue); break;
					case "T1H": rawData.put("T1H", fcstValue); break;
					case "REH": rawData.put("REH", fcstValue); break;
					case "POP": rawData.put("POP", fcstValue); break;
					case "TMN":
						try {
							int temp = Integer.parseInt(fcstValue);
							if (temp < minTemp) minTemp = temp;
						} catch (NumberFormatException e) {
							// 숫자 변환 실패 무시
						}
						break;
					case "TMX":
						try {
							int temp = Integer.parseInt(fcstValue);
							if (temp > maxTemp) maxTemp = temp;
						} catch (NumberFormatException e) {
							// 숫자 변환 실패 무시
						}
						break;
				}
			}
		}

		// 최저/최고 기온 처리
		if (minTemp != Integer.MAX_VALUE) {
			rawData.put("TMN", String.valueOf(minTemp));
		} else {
			// 데이터가 없으면 현재기온에서 5도 낮게 설정
			try {
				int currentTemp = Integer.parseInt(rawData.get("T1H"));
				rawData.put("TMN", String.valueOf(currentTemp - 5));
			} catch (NumberFormatException e) {
				rawData.put("TMN", "0");
			}
		}

		if (maxTemp != Integer.MIN_VALUE) {
			rawData.put("TMX", String.valueOf(maxTemp));
		} else {
			// 데이터가 없으면 현재기온에서 5도 높게 설정
			try {
				int currentTemp = Integer.parseInt(rawData.get("T1H"));
				rawData.put("TMX", String.valueOf(currentTemp + 5));
			} catch (NumberFormatException e) {
				rawData.put("TMX", "0");
			}
		}

		// 날씨 상태 결정
		String weatherStatus = determineWeatherStatus(rawData.get("SKY"), rawData.get("PTY"));

		// 결과 맵 구성
		result.put("weatherStatus", weatherStatus);
		result.put("currentTemperature", Double.parseDouble(rawData.get("T1H")));
		result.put("minTemperature", Double.parseDouble(rawData.get("TMN")));
		result.put("maxTemperature", Double.parseDouble(rawData.get("TMX")));
		result.put("humidity", Integer.parseInt(rawData.get("REH")));
		result.put("precipitation", Integer.parseInt(rawData.get("POP")));

		return result;
	}

	/**
	 * 특보 정보를 조회하는 메서드
	 */
	private Map<String, Object> getAlertData(String areaCode) throws Exception {
		String alertResponse = weatherClient.getWthrWrnList(areaCode);

		// XML 파싱
		DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
		DocumentBuilder builder = factory.newDocumentBuilder();
		Document document = builder.parse(new InputSource(new StringReader(alertResponse)));

		Map<String, Object> result = new HashMap<>();
		result.put("hasAlert", false);
		result.put("message", "");

		// 응답 상태 확인
		NodeList resultCodeNodes = document.getElementsByTagName("resultCode");
		if (resultCodeNodes.getLength() > 0 && "00".equals(resultCodeNodes.item(0).getTextContent())) {
			// 특보 데이터 추출
			NodeList items = document.getElementsByTagName("item");
			if (items.getLength() > 0) {
				// 가장 최근 특보 정보 사용
				Element latestAlert = (Element) items.item(0);
				String title = getElementTextContent(latestAlert, "title");

				// 유효한 특보가 있는 경우
				if (!title.isEmpty()) {
					result.put("hasAlert", true);
					result.put("message", title);
				}
			}
		}

		return result;
	}

	/**
	 * 태풍 정보를 조회하는 메서드
	 */
	private Map<String, Object> getTyphoonData() throws Exception {
		String typhoonResponse = weatherClient.getTyphoonInfo();

		// XML 파싱
		DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
		DocumentBuilder builder = factory.newDocumentBuilder();
		Document document = builder.parse(new InputSource(new StringReader(typhoonResponse)));

		Map<String, Object> result = new HashMap<>();
		result.put("hasTyphoon", false);
		result.put("message", "");

		// 응답 상태 확인
		NodeList resultCodeNodes = document.getElementsByTagName("resultCode");
		if (resultCodeNodes.getLength() > 0 && "00".equals(resultCodeNodes.item(0).getTextContent())) {
			// 태풍 데이터 추출
			NodeList items = document.getElementsByTagName("item");
			for (int i = 0; i < items.getLength(); i++) {
				Element typhoon = (Element) items.item(i);
				String typStatus = getElementTextContent(typhoon, "typStatus");

				// typStatus가 진행중(0)인 태풍만 고려
				if ("0".equals(typStatus)) {
					String typName = getElementTextContent(typhoon, "typName");
					String typLoc = getElementTextContent(typhoon, "typLoc");

					result.put("hasTyphoon", true);
					result.put("message", "현재 " + typName + " 태풍이 " + typLoc + "에 위치하고 있습니다.");
					break;
				}
			}
		}

		return result;
	}

	// XML 요소의 텍스트 내용을 가져오는 헬퍼 메서드
	private String getElementTextContent(Element parent, String tagName) {
		NodeList elements = parent.getElementsByTagName(tagName);
		if (elements.getLength() > 0) {
			return elements.item(0).getTextContent();
		}
		return "";
	}

	/**
	 * 하늘상태와 강수형태를 기반으로 날씨 상태를 결정하는 메서드
	 */
	private String determineWeatherStatus(String skyCode, String ptyCode) {
		// 강수형태가 있으면 우선 처리
		switch (ptyCode) {
			case "1": return "비";
			case "2": return "비/눈";
			case "3": return "눈";
			case "4": return "소나기";
		}

		// 강수가 없으면 하늘상태로 판단
		switch (skyCode) {
			case "1": return "맑음";
			case "3": return "구름많음";
			case "4": return "흐림";
			default: return "맑음";
		}
	}

	/**
	 * 기온 정보를 바탕으로 옷차림 추천을 제공하는 메서드
	 * 현재 기온, 일교차, 날씨 상태를 모두 고려합니다
	 */
	private String recommendClothing(double currentTemp, double minTemp, double maxTemp, String weatherStatus) {
		// 평균 기온 계산 (현재 기온에 가중치 부여)
		double averageTemp = (currentTemp * 2 + minTemp + maxTemp) / 4;

		// 일교차 계산
		double tempRange = maxTemp - minTemp;
		boolean isLargeTempRange = tempRange >= 8.0; // 일교차가 8도 이상인 경우

		String recommendation;

		if (averageTemp >= 28) {
			recommendation = "매우 더운 날씨입니다. 얇은 반팔, 반바지, 민소매 등 시원한 옷을 입으세요.";
			if (isLargeTempRange) {
				recommendation += " 저녁에는 체온 유지를 위해 얇은 긴팔을 준비하는 것이 좋습니다.";
			}
		} else if (averageTemp >= 23) {
			recommendation = "더운 날씨입니다. 반팔, 얇은 셔츠, 반바지나 얇은 면바지를 추천합니다.";
			if (isLargeTempRange) {
				recommendation += " 일교차가 큰 날씨이니 외출 시 얇은 가디건이나 셔츠를 챙기세요.";
			}
		} else if (averageTemp >= 20) {
			recommendation = "따뜻한 날씨입니다. 얇은 가디건이나 긴팔 티셔츠, 면바지를 추천합니다.";
			if (isLargeTempRange) {
				recommendation += " 아침저녁으로 쌀쌀할 수 있으니 얇은 겉옷을 챙기세요.";
			}
		} else if (averageTemp >= 17) {
			recommendation = "선선한 날씨입니다. 맨투맨, 가디건, 청바지, 얇은 니트를 추천합니다.";
			if (isLargeTempRange) {
				recommendation += " 일교차가 크니 겹쳐 입을 수 있는 옷을 준비하는 것이 좋습니다.";
			}
		} else if (averageTemp >= 12) {
			recommendation = "쌀쌀한 날씨입니다. 자켓, 가디건, 야상, 청바지, 스타킹을 추천합니다.";
			if (isLargeTempRange) {
				recommendation += " 실내외 온도차에 대비해 탈부착이 쉬운 옷을 선택하세요.";
			}
		} else if (averageTemp >= 9) {
			recommendation = "추운 날씨입니다. 코트, 야상, 니트, 기모 소재 옷을 추천합니다.";
			if (isLargeTempRange) {
				recommendation += " 체온 유지를 위해 히트텍이나 내복을 입는 것이 좋습니다.";
			}
		} else if (averageTemp >= 5) {
			recommendation = "꽤 추운 날씨입니다. 두꺼운 코트, 히트텍, 니트, 목도리를 추천합니다.";
			if (isLargeTempRange) {
				recommendation += " 장갑과 모자로 체온 손실을 막는 것이 좋습니다.";
			}
		} else {
			recommendation = "매우 추운 날씨입니다. 패딩, 두꺼운 코트, 목도리, 장갑, 기모제품을 추천합니다.";
			recommendation += " 외출 시 여러 겹 껴입기로 체온을 유지하세요.";
		}

		// 비나 눈이 올 경우 추가 권장사항
		if (weatherStatus.contains("비") || weatherStatus.contains("소나기")) {
			recommendation += " 비가 오니 방수 기능이 있는 아우터와 신발을 착용하고, 우산을 꼭 챙기세요.";
		} else if (weatherStatus.contains("비/눈")) {
			recommendation += " 비와 눈이 섞여 내리니 방수와 보온이 동시에 되는 옷차림을 준비하세요.";
		} else if (weatherStatus.contains("눈")) {
			recommendation += " 눈이 오니 방한용 방수 신발과 따뜻한 겨울용 아우터를 준비하세요.";
		} else if (weatherStatus.contains("흐림") && averageTemp <= 10) {
			recommendation += " 흐린 날씨는 체감온도가 더 낮을 수 있으니 평소보다 따뜻하게 입으세요.";
		}

		return recommendation;
	}

	/**
	 * 날씨 상태와 강수 확률을 기반으로 우산 권장사항과 안전 정보를 제공하는 메서드
	 */
	private String recommendUmbrella(String weatherStatus, int precipitation, boolean hasAlert, boolean hasTyphoon) {
		// 태풍 상황 (최우선 고려)
		if (hasTyphoon) {
			return "태풍이 접근하고 있습니다. 가능한 외출을 자제하고, 필수 외출 시에는 튼튼한 우산과 방수 의류를 준비하세요. 강풍에 날리는 물체와 간판에 주의하세요.";
		}

		// 기상 특보 상황
		if (hasAlert) {
			return "기상 특보가 발령되었습니다. 외출 전 최신 기상정보를 확인하고 안전에 유의하세요. 우산과 방수용품을 챙기세요.";
		}

		// 날씨 상태별 권고사항
		if (weatherStatus.contains("비") && precipitation >= 80) {
			return "강한 비가 예상됩니다. 우산만으로는 비를 피하기 어려울 수 있으니, 가능하면 외출을 자제하고 방수 의류와 신발을 착용하세요.";
		} else if (weatherStatus.contains("비") || weatherStatus.contains("소나기")) {
			return "비가 내리고 있습니다. 우산을 필수로 챙기고, 미끄러운 길을 조심하세요.";
		} else if (weatherStatus.contains("비/눈")) {
			return "비와 눈이 함께 내립니다. 우산과 함께 방수 기능이 있는 신발을 착용하고, 빙판길을 주의하세요.";
		} else if (weatherStatus.contains("눈")) {
			return "눈이 내리고 있습니다. 우산보다는 방수 기능이 있는 모자와 아우터를 준비하고, 미끄럼 방지 신발을 착용하세요.";
		}

		// 강수 확률별 권고사항
		if (precipitation >= 70) {
			return "강수 확률이 " + precipitation + "%로 매우 높습니다. 우산을 반드시 챙기고, 방수가 되는 신발을 착용하는 것이 좋습니다.";
		} else if (precipitation >= 50) {
			return "강수 확률이 " + precipitation + "%로 높은 편입니다. 우산을 챙기는 것이 좋겠습니다.";
		} else if (precipitation >= 30) {
			return "강수 확률이 " + precipitation + "%입니다. 접이식 우산을 가방에 넣어두는 것을 권장합니다.";
		} else if (precipitation >= 20 && weatherStatus.contains("흐림")) {
			return "하늘이 흐리고 약간의 비 가능성이 있습니다. 혹시 모르니 작은 우산을 준비해두세요.";
		} else {
			return "오늘은 우산이 필요 없을 것 같습니다. 편안한 하루 되세요!";
		}
	}
}