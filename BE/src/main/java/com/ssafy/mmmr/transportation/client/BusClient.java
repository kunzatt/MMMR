package com.ssafy.mmmr.transportation.client;

import java.io.StringReader;
import java.net.URI;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;
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

import lombok.extern.slf4j.Slf4j;

import org.w3c.dom.Document;
import org.w3c.dom.NodeList;
import org.w3c.dom.Element;
import org.xml.sax.InputSource;
import org.w3c.dom.Node;

@Component
@Slf4j
public class BusClient {
	private final RestTemplate restTemplate;

	@Value("${seoul.bus.api.key}")
	private String apiKey;

	private static final String BASE_URL = "http://ws.bus.go.kr/api/rest/arrive/getArrInfoByRouteList";
	private static final String SUCCESS_CODE = "0";
	private static final String BUS_TYPE = "BUS";
	private static final String NO_ARRIVAL_INFO = "현재 도착 정보가 없습니다.";
	private static final String NO_INFO = "정보 없음";
	private static final String DIRECTION_SUFFIX = " 방향";
	private static final String ARRIVING_SOON = "곧 도착";
	private static final String SERVICE_ENDED = "운행 종료";
	private static final String STOPS_AWAY_FORMAT = "%s정거장 전";

	public BusClient(RestTemplate restTemplate) {
		this.restTemplate = restTemplate;
		this.restTemplate.getMessageConverters()
			.add(0, new StringHttpMessageConverter(StandardCharsets.UTF_8));
	}

	public List<TransportationResponseDto> getBusArrivals(String routeId, String stationId, String busNumber, String stationName, String busDirection, Long busId) {
		List<TransportationResponseDto> arrivals = new ArrayList<>();
		List<String> infoList = new ArrayList<>();

		String responseBody = fetchBusData(routeId);
		if (responseBody == null) {
			return createDefaultResponse(busId, busNumber, stationName, busDirection);
		}

		Document doc = parseXmlResponse(responseBody);
		if (doc == null || !isSuccessResponse(doc)) {
			return createDefaultResponse(busId, busNumber, stationName, busDirection);
		}

		NodeList itemList = doc.getElementsByTagName("itemList");
		if (itemList.getLength() == 0) {
			log.warn("버스 노선 ID: {} 조회 결과 없음", routeId);
			return createDefaultResponse(busId, busNumber, stationName, busDirection);
		}

		log.info("버스 노선 ID: {}, 조회된 항목 수: {}", routeId, itemList.getLength());
		collectArrivalInfoForStation(itemList, stationId, infoList);

		if (!infoList.isEmpty()) {
			String formattedDirection = busDirection + DIRECTION_SUFFIX;
			String combinedInfo = String.join(", ", infoList);

			TransportationResponseDto arrival = TransportationResponseDto.builder()
				.id(busId)
				.type(BUS_TYPE)
				.number(busNumber)
				.station(stationName)
				.information(combinedInfo)
				.direction(formattedDirection)
				.build();

			arrivals.add(arrival);
		} else {
			arrivals = createDefaultResponse(busId, busNumber, stationName, busDirection);
		}

		return arrivals;
	}

	private String fetchBusData(String routeId) {
		try {
			URI uri = buildRequestUri(routeId);
			log.info("버스 API 호출 URI: {}", uri);

			HttpEntity<?> entity = createHttpEntity();
			ResponseEntity<String> response = restTemplate.exchange(
				uri,
				HttpMethod.GET,
				entity,
				String.class
			);

			return response.getBody();
		} catch (Exception e) {
			log.error("버스 노선 ID: {} 처리 중 오류 발생", routeId, e);
			return null;
		}
	}

	private URI buildRequestUri(String routeId) {
		return UriComponentsBuilder.fromHttpUrl(BASE_URL)
			.queryParam("serviceKey", apiKey)
			.queryParam("busRouteId", routeId)
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
		if (responseBody == null || responseBody.isEmpty()) {
			return null;
		}

		try {
			DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
			DocumentBuilder builder = factory.newDocumentBuilder();
			InputSource is = new InputSource(new StringReader(responseBody));
			is.setEncoding("UTF-8");
			return builder.parse(is);
		} catch (Exception e) {
			log.error("XML 파싱 오류", e);
			return null;
		}
	}

	private boolean isSuccessResponse(Document doc) {
		NodeList headerList = doc.getElementsByTagName("headerCd");
		if (headerList.getLength() > 0) {
			String headerCode = headerList.item(0).getTextContent();

			// 에러인 경우
			if (!SUCCESS_CODE.equals(headerCode)) {
				String headerMsg = doc.getElementsByTagName("headerMsg").item(0).getTextContent();
				log.warn("버스 API 오류: {} - {}", headerCode, headerMsg);
				return false;
			}
			return true;
		}
		return false;
	}

	private void collectArrivalInfoForStation(NodeList itemList, String stationId, List<String> infoList) {
		for (int i = 0; i < itemList.getLength(); i++) {
			Node itemNode = itemList.item(i);
			if (itemNode.getNodeType() == Node.ELEMENT_NODE) {
				Element item = (Element) itemNode;

				String arsId = getElementTextContent(item, "arsId");
				String stId = getElementTextContent(item, "stId");

				// stationId와 일치하는 정류소만 처리
				if (stationId.equals(stId) || stationId.equals(arsId)) {
					processArrivalInfo(item, infoList);
				}
			}
		}
	}

	private void processArrivalInfo(Element item, List<String> infoList) {
		String firstArrivalMsg = getElementTextContent(item, "arrmsg1");
		String secondArrivalMsg = getElementTextContent(item, "arrmsg2");

		// 도착 정보가 있는 경우만 처리
		if (!firstArrivalMsg.isEmpty()) {
			infoList.add(processArrivalMsg(firstArrivalMsg));
		}

		if (!secondArrivalMsg.isEmpty()) {
			infoList.add(processArrivalMsg(secondArrivalMsg));
		}
	}

	private List<TransportationResponseDto> createDefaultResponse(Long busId, String busNumber, String stationName, String busDirection) {
		List<TransportationResponseDto> arrivals = new ArrayList<>();
		String formattedDirection = busDirection + DIRECTION_SUFFIX;

		TransportationResponseDto defaultArrival = TransportationResponseDto.builder()
			.id(busId)
			.type(BUS_TYPE)
			.number(busNumber)
			.station(stationName)
			.direction(formattedDirection)
			.information(NO_ARRIVAL_INFO)
			.build();

		arrivals.add(defaultArrival);
		return arrivals;
	}

	// 버스 도착 메시지 가공
	private String processArrivalMsg(String arrivalMsg) {
		if (arrivalMsg == null || arrivalMsg.isEmpty()) {
			return NO_INFO;
		}

		// "[x번째 전]" 패턴 체크
		Pattern stopsAwayPattern = Pattern.compile("\\[(\\d+)번째 전\\]");
		Matcher stopsAwayMatcher = stopsAwayPattern.matcher(arrivalMsg);
		if (stopsAwayMatcher.find()) {
			return String.format(STOPS_AWAY_FORMAT, stopsAwayMatcher.group(1));
		}

		// "x분 x초 후 [x번째 전]" 패턴 체크
		Pattern timePattern = Pattern.compile("(\\d+)분 (\\d+)초 후");
		Matcher timeMatcher = timePattern.matcher(arrivalMsg);
		if (timeMatcher.find()) {
			String minutes = timeMatcher.group(1);
			String seconds = timeMatcher.group(2);
			return minutes + "분 " + seconds + "초 후";
		}

		// "곧 도착" 체크
		if (arrivalMsg.contains("곧 도착")) {
			return ARRIVING_SOON;
		}

		// "운행종료" 체크
		if (arrivalMsg.contains("운행종료")) {
			return SERVICE_ENDED;
		}

		// 그 외 기본 반환
		return arrivalMsg;
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