package com.ssafy.mmmr.transportation.client;

import java.io.StringReader;
import java.net.URI;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;

import org.springframework.beans.factory.annotation.Value;
import org.springframework.http.HttpEntity;
import org.springframework.http.HttpHeaders;
import org.springframework.http.HttpMethod;
import org.springframework.http.MediaType;
import org.springframework.http.ResponseEntity;
import org.springframework.http.converter.StringHttpMessageConverter;
import org.springframework.stereotype.Component;
import org.springframework.web.client.RestTemplate;
import org.springframework.web.util.UriComponentsBuilder;

import com.ssafy.mmmr.transportation.dto.TransportationResponseDto;

import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.InputSource;

@Component
public class MetroClient {
	private final RestTemplate restTemplate;

	@Value("${seoul.metro.api.key}")
	private String apiKey;

	private static final String BASE_URL = "http://swopenapi.seoul.go.kr/api/subway";
	private static final String ENDPOINT = "realtimeStationArrival";
	private static final Pattern NUMBER_MINUTE_PATTERN = Pattern.compile("(\\d+)[^\\d]+(\\d*)");
	private static final String SUCCESS_STATUS = "200";
	private static final String METRO_TYPE = "METRO";
	private static final String NO_INFO = "정보 없음";
	private static final String ARRIVAL_SOON = "곧 도착";
	private static final String TRAIN_APPROACHING = "열차 접근중";
	private static final String NO_ARRIVAL_INFO = "현재 도착 정보가 없습니다.";
	private static final String DEFAULT_LINE_ID = "1002";

	public MetroClient(RestTemplate restTemplate) {
		this.restTemplate = restTemplate;
		this.restTemplate.getMessageConverters()
			.add(0, new StringHttpMessageConverter(StandardCharsets.UTF_8));
	}

	public List<TransportationResponseDto> getMetroArrivals(String stationName, Long metroId) {
		List<TransportationResponseDto> arrivals = new ArrayList<>();
		String[] stationFormats = generateStationNameFormats(stationName);

		for (String candidateStation : stationFormats) {
			String responseBody = fetchMetroData(candidateStation);
			if (responseBody == null) continue;

			Document doc = parseXmlResponse(responseBody);
			if (doc == null) continue;

			if (!isSuccessResponse(doc)) continue;

			NodeList rowList = doc.getElementsByTagName("row");
			if (rowList.getLength() > 0) {
				String originStationName = normalizeStationName(stationName);
				processArrivals(rowList, originStationName, metroId, arrivals);
				break;
			}
		}

		if (arrivals.isEmpty()) {
			arrivals.add(createDefaultArrival(stationName, metroId));
		}

		return arrivals;
	}

	private String[] generateStationNameFormats(String stationName) {
		return new String[] {
			stationName.replace("역", ""),  // "역"을 제거한 이름 먼저 시도
			stationName,
			stationName + "역"
		};
	}

	private String normalizeStationName(String stationName) {
		return stationName.endsWith("역") ? stationName : stationName + "역";
	}

	private String fetchMetroData(String stationName) {
		URI uri = buildRequestUri(stationName);
		HttpEntity<?> entity = createHttpEntity();

		try {
			ResponseEntity<String> response = restTemplate.exchange(
				uri,
				HttpMethod.GET,
				entity,
				String.class
			);

			return response.getBody();
		} catch (Exception e) {
			// API 호출 실패 시 null 반환
			return null;
		}
	}

	private URI buildRequestUri(String stationName) {
		return UriComponentsBuilder.fromHttpUrl(BASE_URL)
			.pathSegment(apiKey, "xml", ENDPOINT, "1", "5", stationName)
			.build()
			.encode()
			.toUri();
	}

	private HttpEntity<?> createHttpEntity() {
		HttpHeaders headers = new HttpHeaders();
		headers.setAccept(List.of(MediaType.APPLICATION_XML));
		headers.set("Accept-Charset", StandardCharsets.UTF_8.name());
		return new HttpEntity<>(headers);
	}

	private Document parseXmlResponse(String responseBody) {
		if (responseBody == null || responseBody.isEmpty()) return null;

		try {
			DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
			DocumentBuilder builder = factory.newDocumentBuilder();
			InputSource is = new InputSource(new StringReader(responseBody));
			is.setEncoding("UTF-8");
			return builder.parse(is);
		} catch (Exception e) {
			// XML 파싱 실패 시 null 반환
			return null;
		}
	}

	private boolean isSuccessResponse(Document doc) {
		NodeList resultList = doc.getElementsByTagName("RESULT");
		if (resultList.getLength() > 0) {
			Element resultElement = (Element) resultList.item(0);
			String status = getElementTextContent(resultElement, "status");
			return SUCCESS_STATUS.equals(status);
		}
		return false;
	}

	private void processArrivals(NodeList rowList, String stationName, Long metroId, List<TransportationResponseDto> arrivals) {
		Map<String, List<String>> directionInfoMap = new HashMap<>();

		// 도착 정보 수집 및 방향별 그룹화
		for (int i = 0; i < rowList.getLength(); i++) {
			Node rowNode = rowList.item(i);
			if (rowNode.getNodeType() == Node.ELEMENT_NODE) {
				Element row = (Element) rowNode;
				collectArrivalInfo(row, directionInfoMap);
			}
		}

		// 방향별로 그룹화된 도착 정보를 응답 객체로 변환
		for (Map.Entry<String, List<String>> entry : directionInfoMap.entrySet()) {
			String direction = entry.getKey();
			List<String> infoList = entry.getValue();
			String combinedInfo = String.join(", ", infoList);

			String subwayId = getSubwayIdForDirection(rowList, direction);
			String lineNumber = convertSubwayIdToLineName(subwayId);

			TransportationResponseDto arrival = TransportationResponseDto.builder()
				.id(metroId)
				.type(METRO_TYPE)
				.number(lineNumber)
				.station(stationName)
				.information(combinedInfo)
				.direction(direction)
				.build();

			arrivals.add(arrival);
		}
	}

	private void collectArrivalInfo(Element row, Map<String, List<String>> directionInfoMap) {
		String trainLineNm = getElementTextContent(row, "trainLineNm");
		String arvlMsg2 = getElementTextContent(row, "arvlMsg2");

		String direction = processDirectionToDestination(trainLineNm);
		String information = processArrivalInfo(arvlMsg2);

		// 방향별로 도착 정보 그룹화
		if (!directionInfoMap.containsKey(direction)) {
			directionInfoMap.put(direction, new ArrayList<>());
		}
		directionInfoMap.get(direction).add(information);
	}

	private TransportationResponseDto createDefaultArrival(String stationName, Long metroId) {
		return TransportationResponseDto.builder()
			.id(metroId)
			.type(METRO_TYPE)
			.number(NO_INFO)
			.station(normalizeStationName(stationName))
			.direction(NO_INFO)
			.information(NO_ARRIVAL_INFO)
			.build();
	}

	private String getSubwayIdForDirection(NodeList rowList, String direction) {
		for (int i = 0; i < rowList.getLength(); i++) {
			Node rowNode = rowList.item(i);
			if (rowNode.getNodeType() == Node.ELEMENT_NODE) {
				Element row = (Element) rowNode;
				String trainLineNm = getElementTextContent(row, "trainLineNm");
				String processedDirection = processDirectionToDestination(trainLineNm);

				if (direction.equals(processedDirection)) {
					return getElementTextContent(row, "subwayId");
				}
			}
		}
		return DEFAULT_LINE_ID; // 기본값으로 2호선(ID 1002) 반환
	}

	private String convertSubwayIdToLineName(String subwayId) {
		return switch (subwayId) {
			case "1001" -> "1호선";
			case "1002" -> "2호선";
			case "1003" -> "3호선";
			case "1004" -> "4호선";
			case "1005" -> "5호선";
			case "1006" -> "6호선";
			case "1007" -> "7호선";
			case "1008" -> "8호선";
			case "1009" -> "9호선";
			case "1063" -> "경의중앙선";
			case "1065" -> "공항철도";
			case "1067" -> "경춘선";
			case "1075" -> "수인분당선";
			case "1077" -> "신분당선";
			case "1092" -> "우이신설선";
			default -> subwayId + "호선";
		};
	}

	private String processDirectionToDestination(String directionText) {
		if (directionText == null || directionText.isEmpty()) {
			return "미정방면";
		}

		// 1. '방면'이 이미 포함된 경우 그대로 사용
		if (directionText.contains("방면")) {
			return directionText;
		}

		// 2. '행'이 포함된 경우 '행' 앞의 텍스트 추출 후 '방면' 추가
		if (directionText.contains("행")) {
			String destination = directionText.split("행")[0].trim();
			return destination + "방면";
		}

		// 3. '-' 포함 시 마지막 부분 추출 (예: '성수행 - 신도림' -> '신도림방면')
		if (directionText.contains("-")) {
			String[] parts = directionText.split("-");
			String lastPart = parts[parts.length - 1].trim();

			// 마지막 부분에 '방면'이 없으면 추가
			if (!lastPart.contains("방면")) {
				lastPart += "방면";
			}
			return lastPart;
		}

		// 인코딩이 깨진 경우 또는 기타 패턴
		if (isEncodingCorrupted(directionText)) {
			return "열차 운행중";
		}

		// 그 외 모든 경우, 원본에 '방면' 추가
		return directionText + "방면";
	}

	private boolean isEncodingCorrupted(String text) {
		// 간단한 한글 식별 (UTF-8에서 한글은 3바이트 시퀀스)
		byte[] bytes = text.getBytes(StandardCharsets.UTF_8);
		return bytes.length > 0 && !isKoreanFirstByte(bytes[0]);
	}

	private String processArrivalInfo(String arrivalInfo) {
		if (arrivalInfo == null || arrivalInfo.isEmpty()) {
			return NO_INFO;
		}

		// 도착/진입 상태 처리
		if (arrivalInfo.contains("도착")) {
			return ARRIVAL_SOON;
		}

		if (arrivalInfo.contains("진입") || arrivalInfo.contains("지나")) {
			return TRAIN_APPROACHING;
		}

		// 분, 초 패턴 추출 (예: "4ë¶ 30ì´ í" -> "4분 30초 후")
		Matcher matcher = NUMBER_MINUTE_PATTERN.matcher(arrivalInfo);
		if (matcher.find()) {
			String minutes = matcher.group(1);
			String seconds = matcher.group(2);

			if (seconds != null && !seconds.isEmpty()) {
				return minutes + "분 " + seconds + "초 후";
			} else {
				return minutes + "분 후";
			}
		}

		// 인코딩 깨짐 감지 및 일반화된 패턴 기반 변환
		if (containsNonKoreanChars(arrivalInfo)) {
			// 숫자만 추출
			String numericPart = arrivalInfo.replaceAll("[^0-9]", "");
			if (!numericPart.isEmpty()) {
				return numericPart + "분 후";
			}
			return TRAIN_APPROACHING;
		}

		// 원본 반환
		return arrivalInfo;
	}

	// 한글 첫 바이트 체크 함수
	private boolean isKoreanFirstByte(byte b) {
		// UTF-8에서 한글 첫 바이트 범위 체크 (간단 버전)
		return (b & 0xE0) == 0xE0; // 한글은 보통 UTF-8에서 3바이트로 인코딩됨
	}

	// 비한글 문자 포함 여부 체크
	private boolean containsNonKoreanChars(String text) {
		// UTF-8로 인코딩했을 때 비정상적인 바이트 시퀀스 존재 여부 체크
		byte[] bytes = text.getBytes(StandardCharsets.UTF_8);
		for (int i = 0; i < bytes.length; i++) {
			// 기본적인 ASCII 및 한글 범위 체크 (간단 버전)
			if ((bytes[i] & 0x80) != 0 && (bytes[i] & 0xE0) != 0xE0) {
				return true;
			}
		}
		return false;
	}

	// XML 엘리먼트의 텍스트 콘텐츠 추출 헬퍼 메서드
	private String getElementTextContent(Element parent, String tagName) {
		NodeList nodeList = parent.getElementsByTagName(tagName);
		if (nodeList.getLength() > 0) {
			return nodeList.item(0).getTextContent();
		}
		return "";
	}
}