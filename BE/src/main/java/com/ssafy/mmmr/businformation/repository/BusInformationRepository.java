package com.ssafy.mmmr.businformation.repository;

import org.springframework.data.jpa.repository.JpaRepository;

import com.ssafy.mmmr.businformation.entity.BusInformationEntity;

public interface BusInformationRepository extends JpaRepository<BusInformationEntity, Integer> {
}
