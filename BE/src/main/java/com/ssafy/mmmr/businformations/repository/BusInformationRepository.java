package com.ssafy.mmmr.businformations.repository;

import java.util.Optional;

import org.springframework.data.jpa.repository.JpaRepository;

import com.ssafy.mmmr.businformations.entity.BusInformationEntity;
import com.ssafy.mmmr.businformations.repository.BusInformationRepositoryCustom;

public interface BusInformationRepository extends JpaRepository<BusInformationEntity, Integer>, BusInformationRepositoryCustom {

	Optional<BusInformationEntity> findByRouteIdAndStationId(Integer routeId, Integer stationId);

	Optional<BusInformationEntity> findByRouteIdAndSequence(Integer routeId, Integer sequence);
}