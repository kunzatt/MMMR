package com.ssafy.mmmr.transportation.repository;

import java.util.List;
import java.util.Optional;

import com.ssafy.mmmr.transportation.entity.MetroInformationEntity;

public interface MetroInformationRepositoryCustom {
	List<MetroInformationEntity> searchByKeyword(String keyword);
	Optional<MetroInformationEntity> findByFlexibleLineNumberAndStationName(String lineNumber, String stationName);
}
