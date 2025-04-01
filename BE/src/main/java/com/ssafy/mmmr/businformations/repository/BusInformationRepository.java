package com.ssafy.mmmr.businformations.repository;

import org.springframework.data.jpa.repository.JpaRepository;

import com.ssafy.mmmr.businformations.entity.BusInformationEntity;

public interface BusInformationRepository extends JpaRepository<BusInformationEntity, Integer> {
}
