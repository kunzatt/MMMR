package com.ssafy.mmmr.businformations.repository;

import java.util.Optional;

import org.springframework.data.jpa.repository.JpaRepository;

import com.ssafy.mmmr.businformations.entity.BusInformationEntity;
import com.ssafy.mmmr.businformations.repository.BusInformationRepositoryCustom;

public interface BusInformationRepository extends JpaRepository<BusInformationEntity, Integer>,
	BusInformationRepositoryCustom {
	// 버스 route_id, station_id, route로 조회
	Optional<BusInformationEntity> findByRouteIdAndStationId(Integer routeId, Integer stationId);
	// 버스 route_id, sequence, route로 다음 정거장 조회
	Optional<BusInformationEntity> findByRouteIdAndSequence(Integer routeId, Integer sequence);
}