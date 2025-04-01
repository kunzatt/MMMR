package com.ssafy.mmmr.transportation.dto;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Getter
@NoArgsConstructor
@AllArgsConstructor
@Builder
public class BusRequestDto {
	private Long profileId;
	private Integer routeId;
	private String route;
	private Integer stationId;
	private String station;
	private String direction;
}
