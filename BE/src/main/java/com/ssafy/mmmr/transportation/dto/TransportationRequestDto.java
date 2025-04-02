package com.ssafy.mmmr.transportation.dto;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Getter
@NoArgsConstructor
@AllArgsConstructor
@Builder
public class TransportationRequestDto {

	private Long profileId;

	private String type;

	private String number;

	private String station;

	private Integer routeId;

	private Integer stationId;
}