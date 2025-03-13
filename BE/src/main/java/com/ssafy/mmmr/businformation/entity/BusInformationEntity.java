package com.ssafy.mmmr.businformation.entity;

import jakarta.persistence.Column;
import jakarta.persistence.Entity;
import jakarta.persistence.GeneratedValue;
import jakarta.persistence.GenerationType;
import jakarta.persistence.Id;
import lombok.AccessLevel;
import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Entity
@Getter
@NoArgsConstructor(access = AccessLevel.PROTECTED)
public class BusInformationEntity {

	@Id
	@GeneratedValue(strategy = GenerationType.AUTO)
	private Integer id;

	@Column(nullable = false)
	private Integer routeId;

	@Column(nullable = false, length = 100)
	private String route;

	@Column(nullable = false)
	private Integer sequence;

	@Column(nullable = false)
	private Integer stationId;

	@Column(nullable = false, length = 100)
	private String station;

	@Builder
	public BusInformationEntity(Integer routeId, String route, Integer sequence, Integer stationId, String station) {
		this.routeId = routeId;
		this.route = route;
		this.sequence = sequence;
		this.stationId = stationId;
		this.station = station;
	}
}
