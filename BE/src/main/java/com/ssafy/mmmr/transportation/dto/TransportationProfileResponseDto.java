package com.ssafy.mmmr.transportation.dto;

import java.util.List;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Getter
@Builder
@NoArgsConstructor
@AllArgsConstructor
public class TransportationProfileResponseDto {

	@Getter
	@Builder
	@NoArgsConstructor
	@AllArgsConstructor
	public static class BusInfo {
		private Long id;
		private String type;
		private String route;
		private String station;
		private String routeId;
		private String stationId;
		private String direction;
	}

	@Getter
	@Builder
	@NoArgsConstructor
	@AllArgsConstructor
	public static class MetroInfo {
		private Long id;
		private String type;
		private Integer line;
		private String station;
	}

	private List<BusInfo> buses;
	private List<MetroInfo> metros;
	private int totalCount;
}