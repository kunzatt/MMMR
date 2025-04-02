package com.ssafy.mmmr.transportation.client;

import java.io.StringReader;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;

import org.springframework.beans.factory.annotation.Value;
import org.springframework.http.ResponseEntity;
import org.springframework.stereotype.Component;
import org.springframework.web.client.RestTemplate;
import org.springframework.web.util.UriComponentsBuilder;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;
import org.xml.sax.InputSource;

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

	public String getRealtimeArrivalInfo(String stationName) {
		try {
			String url = UriComponentsBuilder.fromHttpUrl(apiUrl)
				.path("/realtimeStationArrival/{apiKey}/xml/realtimeStationArrival/0/10/{stationName}")
				.buildAndExpand(apiKey, stationName)
				.toUriString();

			log.info("요청 URL: {}", url);

			ResponseEntity<String> response = restTemplate.getForEntity(url, String.class);
			String xmlResponse = response.getBody();

			log.debug("지하철 API 응답: {}", xmlResponse);

			return xmlResponse;
		} catch (Exception e) {
			log.error("지하철 API 호출 중 오류 발생: {}", e.getMessage(), e);
			return null;
		}
	}

	public List<Integer> getLineNumbers(String stationName) {
		Set<Integer> lineNumbers = new HashSet<>();

		try {
			String xmlResponse = getRealtimeArrivalInfo(stationName);

			if (xmlResponse == null || xmlResponse.isEmpty()) {
				return new ArrayList<>();
			}

			DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
			DocumentBuilder builder = factory.newDocumentBuilder();
			Document doc = builder.parse(new InputSource(new StringReader(xmlResponse)));

			// 결과 코드 확인
			NodeList resultCodeList = doc.getElementsByTagName("code");
			if (resultCodeList.getLength() > 0) {
				String resultCode = resultCodeList.item(0).getTextContent();
				if (!"INFO-000".equals(resultCode)) {
					log.warn("지하철 API 호출 결과 오류: {}", resultCode);
					return new ArrayList<>();
				}
			}

			// 지하철 ID 추출
			NodeList subwayIdList = doc.getElementsByTagName("subwayId");
			for (int i = 0; i < subwayIdList.getLength(); i++) {
				String subwayId = subwayIdList.item(i).getTextContent();
				try {
					int lineNo = Integer.parseInt(subwayId) % 1000;
					lineNumbers.add(lineNo);
				} catch (NumberFormatException e) {
					log.warn("지하철 노선 번호 파싱 실패: {}", subwayId);
				}
			}
		} catch (Exception e) {
			log.error("지하철 노선 정보 파싱 중 오류 발생: {}", e.getMessage(), e);
		}

		return new ArrayList<>(lineNumbers);
	}

	public boolean validateStation(String stationName) {
		try {
			String xmlResponse = getRealtimeArrivalInfo(stationName);

			if (xmlResponse == null || xmlResponse.isEmpty()) {
				return false;
			}

			// 정규식을 사용하여 row 태그의 개수를 확인
			Pattern pattern = Pattern.compile("<row>");
			Matcher matcher = pattern.matcher(xmlResponse);
			int count = 0;
			while (matcher.find()) {
				count++;
			}

			return count > 0;
		} catch (Exception e) {
			log.error("지하철역 검증 중 오류 발생: {}", e.getMessage(), e);
			return false;
		}
	}

	public Integer getLineNumber(String stationName) {
		List<Integer> lineNumbers = getLineNumbers(stationName);
		return lineNumbers.isEmpty() ? null : lineNumbers.get(0);
	}
}