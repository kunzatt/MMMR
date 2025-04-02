package com.ssafy.mmmr.transportation.dto;

import lombok.Builder;
import lombok.Getter;

@Builder
@Getter
public class TransportationResponseDto {

	private Long id;

	private String type;

	private String number;

	private String station;

	private String direction;

	private String information;

}
