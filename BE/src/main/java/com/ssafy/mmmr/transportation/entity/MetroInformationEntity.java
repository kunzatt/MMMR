package com.ssafy.mmmr.transportation.entity;

import jakarta.persistence.Column;
import jakarta.persistence.Entity;
import jakarta.persistence.GeneratedValue;
import jakarta.persistence.GenerationType;
import jakarta.persistence.Id;
import jakarta.persistence.Table;
import lombok.AccessLevel;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Entity
@Table(name = "metro_informations")
@Getter
@NoArgsConstructor(access = AccessLevel.PROTECTED)
public class MetroInformationEntity {

	@Id
	@GeneratedValue(strategy = GenerationType.IDENTITY)
	private Integer id;

	@Column(name = "line_number", nullable = false)
	private String lineNumber;

	@Column(name = "station_name", nullable = false)
	private String stationName;
}
