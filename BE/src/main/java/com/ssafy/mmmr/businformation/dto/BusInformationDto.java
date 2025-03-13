package com.ssafy.mmmr.businformation.dto;

import com.ssafy.mmmr.businformation.entity.BusInformationEntity;

import jakarta.persistence.Column;
import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Builder
@Getter
@NoArgsConstructor
@AllArgsConstructor
public class BusInformationDto {

	private Integer routeId;

	private String route;

	private Integer sequence;

	private Integer stationId;

	private String station;

	public BusInformationEntity toEntity() {
		return BusInformationEntity.builder()
			.routeId(this.routeId)
			.route(this.route)
			.sequence(this.sequence)
			.stationId(this.stationId)
			.station(this.station)
			.build();
	}
}
