package com.ssafy.mmmr.transportation.dto;

import java.util.List;

import com.fasterxml.jackson.dataformat.xml.annotation.JacksonXmlElementWrapper;
import com.fasterxml.jackson.dataformat.xml.annotation.JacksonXmlProperty;
import com.fasterxml.jackson.dataformat.xml.annotation.JacksonXmlRootElement;

import lombok.Getter;
import lombok.Setter;

@Getter
@Setter
@JacksonXmlRootElement(localName = "realtimeStationArrival")
public class MetroApiResponseDto {

	@JacksonXmlElementWrapper(useWrapping = false)
	@JacksonXmlProperty(localName = "row")
	private List<MetroArrivalInfo> rows;

	@Getter
	@Setter
	public static class MetroArrivalInfo {
		private String subwayId;      // 지하철 호선
		private String trainLineNm;   // 방향 정보 (종착지 방면)
		private String arvlMsg2;      // 도착 메시지
		private String statnNm;       // 역 이름
		private String updnLine;      // 상행/하행 또는 내선/외선 구분
		private String bstatnNm;      // 종착역
		private String arvlCd;        // 도착 코드
		private String btrainSttus;   // 열차 종류 (급행, 일반 등)
		private String btrainNo;      // 열차 번호
	}

}