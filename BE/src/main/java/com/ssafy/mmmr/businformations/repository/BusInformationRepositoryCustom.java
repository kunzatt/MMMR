package com.ssafy.mmmr.businformations.repository;

import java.util.List;
import java.util.Optional;

import com.ssafy.mmmr.businformations.entity.BusInformationEntity;

public interface BusInformationRepositoryCustom {
	List<BusInformationEntity> searchByKeyword(String keyword);

	Optional<BusInformationEntity> findByRouteIdAndStationIdAndRoute(Integer routeId, Integer stationId, String route);
	Optional<BusInformationEntity> findByRouteIdAndSequenceAndRoute(Integer routeId, Integer sequence, String route);

}
