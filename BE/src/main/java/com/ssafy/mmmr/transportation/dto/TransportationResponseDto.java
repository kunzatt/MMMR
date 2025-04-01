package com.ssafy.mmmr.transportation.dto;

import lombok.Builder;

@Builder
public class TransportationResponseDto {

	private String type;

	private String station;

	private String sequence;

	private String number;

	private String information;

}
