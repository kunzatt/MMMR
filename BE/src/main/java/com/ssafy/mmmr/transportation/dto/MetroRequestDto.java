package com.ssafy.mmmr.transportation.dto;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Getter
@NoArgsConstructor
@AllArgsConstructor
@Builder
public class MetroRequestDto {
	private Long profileId;
	private Integer line;
	private String station;
	private String direction;
}