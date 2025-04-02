package com.ssafy.mmmr.transportation.dto;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Getter
@NoArgsConstructor
@AllArgsConstructor
@Builder
public class TransportationSearchResponseDto {

	private String type;

	private String station;

	private String sequence;

	private String number;

	private String information;

}
