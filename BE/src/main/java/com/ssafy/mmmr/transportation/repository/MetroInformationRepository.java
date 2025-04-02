package com.ssafy.mmmr.transportation.repository;

import org.springframework.data.jpa.repository.JpaRepository;

import com.ssafy.mmmr.transportation.entity.MetroInformationEntity;

public interface MetroInformationRepository extends JpaRepository<MetroInformationEntity, Integer> {
}
